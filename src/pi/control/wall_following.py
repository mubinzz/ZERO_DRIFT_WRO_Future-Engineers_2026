"""
============================================================
wall_following.py — দেয়াল অনুসরণ করে স্টিয়ারিং কমান্ড বের করা
============================================================
কৌশল: LIDAR-এর left/right সেক্টর দূরত্ব থেকে একটা "error" বের করে
PID-কে দিয়ে steering angle বের করা হয়।

⚠️ Sign convention (এই মডিউলের সবচেয়ে গুরুত্বপূর্ণ অংশ, তাই এখানে
প্রতিটা ধাপ যুক্তি দিয়ে যাচাই করা আছে -- camera-র RGB/BGR ভুলের
পুনরাবৃত্তি এড়াতে):

  lidar_processor.py-এর কনভেনশন: angle 0°=সামনে, +90°=ডান -- তাই
  servo-ও একই কনভেনশনে থাকা দরকার: angle বাড়া (center-এর উপরে) =
  চাকা ডানে ঘোরা (calibrate_servo.py-তে servo_right_max > servo_center
  এভাবেই মাপা হয়েছিল)।

  correction-এর সংজ্ঞা এই মডিউলে: correction > 0 মানে ডানে স্টিয়ার
  করা (servo angle বাড়বে), correction < 0 মানে বামে।

  দুই পাশে দেয়াল থাকলে (double-wall case):
    error = right_distance - left_distance
    যুক্তি: ডান দেয়াল কাছে চলে এলে (right ছোট হয়ে যায়), error
    ঋণাত্মক হয়ে যাবে -> correction ঋণাত্মক -> বামে স্টিয়ার (ডান
    দেয়াল থেকে দূরে সরা) -- এটাই সঠিক আচরণ। উল্টোটাও (left ছোট
    হলে ডানে স্টিয়ার) একইভাবে সত্যি।

  শুধু বাম দেয়াল দেখা গেলে (single-wall, left):
    error = target_gap_m - left_distance
    যুক্তি: left_distance কমে গেলে (দেয়ালের খুব কাছে চলে এসেছে),
    error ধনাত্মক হবে -> correction ধনাত্মক -> ডানে স্টিয়ার
    (দেয়াল থেকে দূরে সরা) -- সঠিক।

  শুধু ডান দেয়াল দেখা গেলে (single-wall, right):
    error = right_distance - target_gap_m
    যুক্তি: right_distance কমে গেলে, error ঋণাত্মক -> correction
    ঋণাত্মক -> বামে স্টিয়ার (দেয়াল থেকে দূরে সরা) -- সঠিক।

⚠️ CW/CCW (round driving direction) সম্পর্কে এই মডিউল **কিছুই জানে
না বা জানারও দরকার নেই** -- এটা সম্পূর্ণ reactive/local (শুধু এই
মুহূর্তে left/right এ কী দেখা যাচ্ছে সেটার উপর ভিত্তি করে কাজ করে),
তাই কোনো preset direction ছাড়াই দুই দিকেই সমানভাবে কাজ করবে।

⚠️ Rate-limiting সময়-ভিত্তিক কেন (max_angle_change_per_sec, per-step না):
আগে max_angle_change_per_step ছিল -- servo angle প্রতি loop iteration-এ
(প্রতি compute_steering_angle() কলে) সর্বোচ্চ কত ডিগ্রি বদলাবে, বাস্তব
সময়ের সাথে কোনো সম্পর্ক ছাড়াই। সমস্যা: বিভিন্ন script-এর control loop
আলাদা গতিতে চলে (যেমন tune_open_challenge_gui.py প্রতি iteration-এ
live status দেখানোর জন্য get_sector_distances() আরেকবার আলাদাভাবে কল
করে -- extra কাজ, loop স্বাভাবিকভাবেই ধীর হয়), তাই একই calibrated
মান (যেমন ৮ ডিগ্রি/step) ভিন্ন script-এ ভিন্ন বাস্তব গতি (ডিগ্রি/সেকেন্ড)
তৈরি করত -- GUI-তে টিউন করে "smooth" মনে হলেও অন্য (দ্রুত loop চলা)
script-এ servo বাস্তবে দ্রুত ঘুরে "একাবাকা" লাগত। সমাধান: PIDController-
এর derivative term যেভাবে dt (প্রকৃত অতিক্রান্ত সময়) মাপে, রেট-লিমিটও
সেভাবে dt দিয়ে হিসাব হয় -- max_angle_change_per_sec × dt = এই কলে
সর্বোচ্চ কত ডিগ্রি বদলানো যাবে। এতে servo-র বাস্তব গতি সবসময় একই
থাকে, loop যত দ্রুতই চলুক না কেন।

⚠️ Gain scale (কেন KP=80/KD=15 ভুল ছিল, "fast wobble"-এর আসল কারণ):
error হিসাব হয় **মিটারে** (যেমন একটা 80-100cm করিডোরে error সাধারণত
±0.5m-এর মধ্যে, প্রায়ই মাত্র ০.০১-০.০৫m)। কিন্তু PID output clamp
করা -1.0..+1.0 রেঞ্জে। KP=80 দিলে মাত্র ১.২৫ সেমি (0.0125m) off-center
হলেই correction = 80×0.0125 = 1.0 (সম্পূর্ণ saturate)! LIDAR distance
এর স্বাভাবিক ১-২ সেমি frame-to-frame noise-ও তখন servo-কে full-left/
full-right এ ছুঁড়ে দেয় -- এটাই "fast wobble"। সমাধান: gain-কে error-এর
আসল স্কেলের (মিটার, ছোট দশমিক) সাথে মিলিয়ে অনেক ছোট রাখা, এবং
error smoothing + servo rate-limiting দিয়ে অতিরিক্ত সুরক্ষা।
============================================================
"""

import csv
import os
import time

from perception.lidar_processor import get_sector_distances
from control.pid_controller import PIDController

# straight-line চলার সময়কার error/servo-angle এখানে লগ হয় -- PID (KP/KD)
# টিউন করার সময় অনুমান না করে আসল oscillation প্যাটার্ন (amplitude,
# period, overshoot) দেখে সিদ্ধান্ত নেওয়ার জন্য। corner_turn_log.csv এর
# সাথে গুলিয়ে ফেলবেন না -- ওটা শুধু turn event, এটা প্রতিটা scan এর
# steering সিদ্ধান্ত।
LOG_FILE = os.path.join(os.path.dirname(__file__), "..", "logs", "wall_following_log.csv")
_LOG_HEADER = ["time", "left_m", "right_m", "raw_error", "filtered_error", "correction", "angle"]
_LOG_THROTTLE_SECONDS = 0.15  # প্রতি scan এ লিখলে SD card I/O কন্ট্রোল লুপ ধীর করে দিতে পারে, তাই থ্রটল
_last_log_time = 0.0


def _log_event(row: dict):
    global _last_log_time
    now = time.time()
    if now - _last_log_time < _LOG_THROTTLE_SECONDS:
        return
    _last_log_time = now
    try:
        os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
        file_exists = os.path.exists(LOG_FILE)
        with open(LOG_FILE, "a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=_LOG_HEADER)
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)
    except Exception:
        pass  # লগিং কখনো robot এর মূল কাজ থামাবে না


class WallFollower:
    def __init__(self, servo_center, servo_left_max, servo_right_max,
                 kp=3.0, ki=0.0, kd=0.3, target_gap_m=0.45,
                 smoothing_alpha=0.4, max_angle_change_per_sec=80,
                 max_wall_distance_m=1.2, max_error_m=0.8):
        """
        servo_center/left_max/right_max : calibration.json থেকে আসা
            আসল ক্যালিব্রেট করা ভ্যালু (এই ক্লাসে হার্ডকোড করা নেই,
            কারণ robot-ভেদে আলাদা)
        kp/ki/kd : ⚠️ শুরুর অনুমান, কিন্তু এবার error-এর আসল স্কেল
            (মিটার) মাথায় রেখে হিসাব করা -- ০.০৫m (৫ সেমি, মোটামুটি
            স্বাভাবিক বিচ্যুতি) error-এ correction ≈0.15 (মৃদু) হওয়ার
            লক্ষ্যে KP≈3 বসানো হয়েছে। তাও বাস্তব ট্র্যাকে টিউন করা লাগবে।
        target_gap_m : single-wall মোডে দেয়াল থেকে কাঙ্ক্ষিত দূরত্ব
            (মিটার) -- 80-100cm করিডোরের প্রায় অর্ধেক ধরে 0.45m
        smoothing_alpha : 0-1, error smoothing filter এর তীব্রতা।
            ছোট মান (যেমন 0.2) = বেশি smoothing কিন্তু ধীর সাড়া,
            বড় মান (যেমন 0.8) = কম smoothing, raw সংকেতের কাছাকাছি
        max_angle_change_per_sec : servo angle প্রতি সেকেন্ডে সর্বোচ্চ
            কত ডিগ্রি বদলাতে পারবে (rate limiting, হঠাৎ ঝাঁকি এড়াতে)।
            ⚠️ ডিফল্ট ৮০ শুরুর অনুমান মাত্র (আগের max_angle_change_per_step=8
            আর ~10Hz loop ধরে সমতুল্য) -- বাস্তব ট্র্যাকে GUI দিয়ে
            টিউন করা লাগবে।
        max_wall_distance_m : একটা সেক্টরে এর চেয়ে বেশি দূরত্ব পাওয়া
            গেলে সেটাকে "আসল পাশের দেয়াল" হিসেবে বিশ্বাস করা হবে না,
            None (কোনো দেয়াল নেই) হিসেবে ধরা হবে। কারণ: বাড়িতে বানানো
            করিডোরে ফাঁক/ফুটো থাকলে LIDAR সেই ফাঁক দিয়ে দূরের কোনো
            জিনিস (ঘরের আসল দেয়াল) দেখে ফেলতে পারে, যেটাকে "পাশের
            দেয়াল অনেএএক দূরে" ভুল বুঝে PID সেদিকে সজোরে স্টিয়ার
            করতে পারে -- বাইরে থেকে এটা হঠাৎ "U-turn" এর মতো দেখায়।
        max_error_m : smoothing-এর আগেও raw error-কে এই সীমায় clamp
            করা হয় -- একটা বিচ্ছিন্ন খারাপ frame যেন একবারেই বিশাল
            correction তৈরি করতে না পারে (defense in depth, smoothing
            filter-এর পরিপূরক)।
        """
        # correction রেঞ্জ -1.0 (পূর্ণ বামে) থেকে +1.0 (পূর্ণ ডানে)
        self.pid = PIDController(kp, ki, kd, output_min=-1.0, output_max=1.0)
        self.servo_center = servo_center
        self.servo_left_max = servo_left_max
        self.servo_right_max = servo_right_max
        self.target_gap_m = target_gap_m
        self.max_wall_distance_m = max_wall_distance_m
        self.max_error_m = max_error_m

        self.smoothing_alpha = smoothing_alpha
        self._filtered_error = None

        self.max_angle_change_per_sec = max_angle_change_per_sec
        self._last_angle = servo_center
        self._last_rate_limit_time = None  # dt মাপার জন্য, reset()-এও মুছতে হবে

    def reset(self):
        self.pid.reset()
        self._filtered_error = None
        self._last_angle = self.servo_center
        self._last_rate_limit_time = None

    def _smooth(self, raw_error):
        """low-pass filter (exponential moving average) -- LIDAR
        distance-এর ছোট noise PID-তে পৌঁছানোর আগেই মসৃণ করে দেয়।"""
        if self._filtered_error is None:
            self._filtered_error = raw_error
        else:
            self._filtered_error = (
                self.smoothing_alpha * raw_error
                + (1 - self.smoothing_alpha) * self._filtered_error
            )
        return self._filtered_error

    def compute_steering_angle(self, scan):
        """একটা LIDAR scan থেকে servo angle (int, 0-180) হিসাব করে।"""
        sectors = get_sector_distances(scan)
        left = sectors["left"]
        right = sectors["right"]

        # খুব বেশি দূরত্ব (করিডোরের প্রস্থের তুলনায় অস্বাভাবিক) মানে
        # সম্ভবত ফাঁক দিয়ে দূরের কিছু দেখা যাচ্ছে, আসল পাশের দেয়াল না --
        # সেটাকে "দেয়াল নেই" হিসেবে ধরা হচ্ছে, নাহলে PID ভুল করে
        # সেদিকে সজোরে স্টিয়ার করবে (এটাই হঠাৎ U-turn এর কারণ ছিল)
        if left is not None and left > self.max_wall_distance_m:
            left = None
        if right is not None and right > self.max_wall_distance_m:
            right = None

        if left is not None and right is not None:
            raw_error = right - left
        elif left is not None:
            raw_error = self.target_gap_m - left
        elif right is not None:
            raw_error = right - self.target_gap_m
        else:
            # কোনো দেয়ালই দেখা যাচ্ছে না (খুব খোলা জায়গা/corner) --
            # সোজা চলবে, PID-কে error=0 দেওয়া হচ্ছে যাতে ধীরে ধীরে
            # center-এ ফিরে আসে, হঠাৎ কোনো correction না দেয়
            raw_error = 0.0

        return self._steer_from_error(raw_error, left, right)

    def _steer_from_error(self, raw_error, left=None, right=None):
        """error (মিটার, ধনাত্মক=ডানে) থেকে clamp -> smooth -> PID ->
        servo-mapping -> rate-limit -> log -- এই পুরো pipeline চালিয়ে
        servo angle (int) রিটার্ন করে।

        ⚠️ এটা আলাদা করে বের করা হয়েছে যাতে obstacle_navigator.py-র
        pillar-avoidance ঠিক এই একই (ভালোভাবে টেস্ট করা) PID/smoothing/
        rate-limiting pipeline reuse করতে পারে, নিজে থেকে নতুন করে না
        লিখে -- শুধু raw_error-টা ভিন্নভাবে (pillar-এর অবস্থান থেকে)
        হিসাব করে এখানে পাঠাবে। left/right শুধু logging-এর জন্য
        (ঐচ্ছিক, pillar-avoidance এর সময় None থাকবে)।
        """
        # একটা বিচ্ছিন্ন খারাপ frame-ও যেন বিশাল correction তৈরি করতে
        # না পারে -- smoothing filter-এ পৌঁছানোর আগেই সীমা বেঁধে দেওয়া
        raw_error = max(-self.max_error_m, min(self.max_error_m, raw_error))

        error = self._smooth(raw_error)
        correction = self.pid.update(error)

        if correction >= 0:
            angle = self.servo_center + correction * (self.servo_right_max - self.servo_center)
        else:
            angle = self.servo_center + correction * (self.servo_center - self.servo_left_max)

        angle = max(self.servo_left_max, min(self.servo_right_max, angle))

        # rate limiting: dt (প্রকৃত অতিক্রান্ত সময়) দিয়ে হিসাব করা হচ্ছে,
        # যাতে servo-র বাস্তব গতি (ডিগ্রি/সেকেন্ড) loop-এর গতির উপর
        # নির্ভর না করে -- ধীর loop-এও একই বাস্তব গতি বজায় থাকবে।
        now = time.time()
        dt = 0.0 if self._last_rate_limit_time is None else (now - self._last_rate_limit_time)
        self._last_rate_limit_time = now

        if dt > 0:
            max_delta = self.max_angle_change_per_sec * dt
            delta = angle - self._last_angle
            if delta > max_delta:
                angle = self._last_angle + max_delta
            elif delta < -max_delta:
                angle = self._last_angle - max_delta

        self._last_angle = angle

        _log_event({
            "time": time.strftime("%H:%M:%S") + f".{int((time.time() % 1) * 1000):03d}",
            "left_m": f"{left:.3f}" if left is not None else "",
            "right_m": f"{right:.3f}" if right is not None else "",
            "raw_error": f"{raw_error:.3f}",
            "filtered_error": f"{error:.3f}",
            "correction": f"{correction:.3f}",
            "angle": int(round(angle)),
        })

        return int(round(angle))
