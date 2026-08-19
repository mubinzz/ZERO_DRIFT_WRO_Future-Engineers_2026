"""
============================================================
corner_turn.py — কোণ (corner) চেনা ও ৯০° ঘোরার লজিক
============================================================
কৌশল (সম্পূর্ণ reactive/local, KMIDS-এর getTurnDirection() এর
দর্শন থেকে অনুপ্রাণিত -- কোনো preset CW/CCW direction জানার দরকার
নেই, প্রতি মুহূর্তে যা দেখা যাচ্ছে তার উপর ভিত্তি করে সিদ্ধান্ত):

  ১. সামনের (front) দূরত্ব একটা threshold-এর নিচে নামলে ধরে নেওয়া
     হয় একটা কোণ কাছে চলে এসেছে (should_start_turn)।

  ২. সেই মুহূর্তে left/right সেক্টর তুলনা করে -- যেদিকে বেশি জায়গা
     (দূরত্ব বেশি, বা কোনো wall data-ই নেই মানে সেদিকটা খোলা)
     সেদিকেই ঘুরতে হবে -- এটাই ট্র্যাকের "খোলা" দিক, যেদিক দিয়ে
     পরের straight section শুরু হয় (start_turn)।

  ৩. turn শুরুর মুহূর্তের yaw (gyro) নোট করে রাখা হয়। turn চলতে
     থাকে যতক্ষণ না yaw প্রায় ৮৫° বদলে যায় (check_turn_complete)।
     encoder না, gyro ব্যবহার করা হচ্ছে কারণ turn করার সময় চাকা
     পিছলে (wheel slip) গেলেও gyro নির্ভুল থাকে।

⚠️ yaw sign convention (ডানে ঘুরলে yaw বাড়ে না কমে) এখনো যাচাই
করা হয়নি -- এটা MPU6050 চিপ কীভাবে বসানো তার উপর নির্ভর করে,
robot-ভেদে আলাদা হতে পারে। ব্যবহারের আগে test_serial_link.py দিয়ে
robot হাতে ৯০° ডানে ঘুরিয়ে yaw বাড়ে না কমে সেটা যাচাই করে,
caller script-এ YAW_INCREASES_CLOCKWISE ঠিক মান বসান।

⚠️ দিক-নির্ধারণে robustness: শুধু trigger হওয়ার মুহূর্তের একটামাত্র
scan দেখে দিক ঠিক করলে সেই একটা frame noisy হলে ভুল দিক বেছে
নেওয়া হতে পারে (এবং সেই ভুল পুরো turn জুড়ে বহাল থাকে, যেহেতু দিক
একবারই ঠিক হয়)। তাই should_start_turn() যতবার কল হয় (turn শুরুর
আগ পর্যন্ত প্রতিটা scan এ) ততবারই left/right এর মান একটা ছোট
history-তে জমা রাখা হয়, আর start_turn() এ সেই history-র **গড়**
ব্যবহার করে দিক ঠিক করা হয় -- একটা বিচ্ছিন্ন bad frame এখন আর
সিদ্ধান্ত পাল্টাতে পারবে না।

⚠️ trigger নেওয়াতেও একই ধরনের robustness দরকার: "front" সেক্টর
মাত্র ±20° সরু cone, আর একটামাত্র কাছের পয়েন্টই (min distance)
যথেষ্ট সেটাকে "কাছে" দেখানোর জন্য। সরু করিডোরে PID wobble-এর সময়
robot-এর heading সামান্য বেঁকে গেলে পাশের দেয়াল এক মুহূর্তের জন্য
এই cone-এর ভেতর ঢুকে যেতে পারে -- robot তখনো আসল কোণার কাছে না
থাকলেও একটা bad frame-এই turn শুরু হয়ে যেত। তাই front < front_trigger_m
পরপর কতবার (front_confirm_frames) সত্যি হলো তার একটা কাউন্টার
রাখা হয়, আর সেই কাউন্টার নির্দিষ্ট মানে পৌঁছালে তবেই turn সত্যিই
শুরু হয় -- একটা বিচ্ছিন্ন কাছের reading আর মাঝ-করিডোরে ভুল turn
শুরু করতে পারবে না।

⚠️⚠️ Obstacle Challenge-এর লগ (corner_turn_log.csv) থেকে ধরা পড়া
তিনটা বাস্তব ব্যর্থতা ও তাদের সমাধান (KMIDS-এর open_challenge
state machine থেকে সরাসরি নেওয়া তিনটা ধারণা):

  ১. "cascade" -- একটা আসল corner শেষ হওয়ার ২-৩ সেকেন্ডের মধ্যেই
     পরপর আরও দুটো ভুয়া turn শুরু হয়ে যেত, ফলে ১০ সেকেন্ডে robot
     ~২০০° ঘুরে যেত ("গোল হয়ে ঘোরা")। কারণ: turn ৯০°-এর বদলে
     ~৬৫° হয়ে থামে, তাই corner শেষে robot দেয়ালের দিকে তেরছা হয়ে
     থাকে, সেদিকে এগিয়ে front দূরত্ব আবার trigger-এর নিচে নেমে যায়।
     সমাধান: turn_cooldown_seconds -- turn শেষ হওয়ার পর এই সময়টুকু
     নতুন কোনো turn শুরু হতে পারবে না (KMIDS-এর PRE_TURN_COOLDOWN)।

  ২. করিডোরের মাঝখানে ভুয়া trigger (লগে avg_left ~০.৫-০.৮মি, অথচ
     আসল corner-এ সবসময় ≥১.০৫মি -- pillar বা wobble front cone-এ
     ঢুকে পড়ায়)। সমাধান: min_open_side_m -- আসল corner-এ অন্তত
     একটা পাশ সত্যিই খোলা থাকে (কারণ ভেতরের দেয়াল ওখানে শেষ হয়ে
     যায়), তাই দুই পাশই এর চেয়ে কাছে থাকলে সেটা corner ধরা হবে না।

  ৩. দিক নির্ধারণ noise-এর উপর চলছিল -- লগে একবার avg_left=0.560
     vs avg_right=0.565 (মাত্র ৫ মিলিমিটার ব্যবধান) দেখে "right"
     বেছে নিয়েছিল, অথচ কোর্সটা CCW, বাকি সব turn ছিল "left"। আরও
     খারাপ: robot দেয়ালে নাক ঠেকে আটকে গেলে right সেক্টর ফাঁকা
     (None) হয়ে যায়, আর তখন কোড ডিফল্ট "right" ধরে উল্টো দিকে
     ঘুরতে শুরু করত। সমাধান: দিক **একবারই** ঠিক হবে (প্রথম corner-এ,
     যেখানে জ্যামিতি স্পষ্ট -- একদিকে ১.৫মি, অন্যদিকে ০.৩৫মি), তারপর
     পুরো round জুড়ে সেই একই দিক (KMIDS-এর `if (!turnDirection_)`)।
     ট্র্যাকে driving direction একটা round-এ বদলায় না, তাই প্রতি
     corner-এ নতুন করে সিদ্ধান্ত নেওয়ার কোনো দরকারই ছিল না।
============================================================
"""

import csv
import os
import time
from collections import deque

from perception.lidar_processor import get_sector_distances

# turn শুরু/শেষের প্রতিটা সিদ্ধান্ত এখানে লগ হয় -- পরে কোনো turn ভুল
# দিকে গেলে বা দেয়ালে লাগলে, ঠিক সেই মুহূর্তে robot কী দেখেছিল আর
# কী সিদ্ধান্ত নিয়েছিল সেটা অনুমান না করে সরাসরি ফাইল থেকে পড়া যায়।
LOG_FILE = os.path.join(os.path.dirname(__file__), "..", "logs", "corner_turn_log.csv")
_LOG_HEADER = ["time", "event", "direction", "avg_left_m", "avg_right_m",
               "front_m", "start_yaw", "end_yaw", "yaw_change", "timed_out"]


def _log_event(row: dict):
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


class CornerTurnController:
    def __init__(self, servo_center, servo_left_max, servo_right_max,
                 front_trigger_m=0.40, target_yaw_change_deg=85,
                 max_turn_seconds=4.0, direction_history_size=5,
                 front_confirm_frames=3, turn_cooldown_seconds=1.5,
                 min_open_side_m=0.9):
        """
        front_trigger_m : সামনের দূরত্ব (মিটার) এর নিচে নামলে কোণ ধরা
            হবে। ⚠️ শুরুর অনুমান -- arena-র আকার ও robot-এর গতির
            উপর নির্ভর করে টিউন করা লাগবে (খুব ছোট হলে দেরিতে turn
            শুরু হয়ে দেয়ালে লাগতে পারে, খুব বড় হলে অকারণে সোজা
            অংশেই turn শুরু হয়ে যেতে পারে)।
        target_yaw_change_deg : turn সম্পন্ন ধরার জন্য yaw কত ডিগ্রি
            বদলাতে হবে (সঠিক ৯০° এর বদলে ৮৫° রাখা হয়েছে, যাতে সামান্য
            কম ঘুরলেও wall-following আবার নিয়ন্ত্রণ নিতে পারে, পুরো
            নির্ভরতা turn logic-এর নিখুঁততার উপর না থাকে)
        max_turn_seconds : নিরাপত্তা timeout। এর বেশি সময় ধরে turn
            চলতে থাকলে জোর করে turn শেষ ধরা হবে।
            ⚠️ কেন দরকার: আগে turn শেষ হওয়ার একমাত্র শর্ত ছিল yaw
            নির্দিষ্ট পরিমাণ বদলানো। robot যদি turn করতে গিয়ে দেয়ালে
            আটকে যায় বা চাকা পিছলে যায়, yaw আর বদলাতো না -- ফলে
            is_turning() চিরকাল True থেকে servo এক প্রান্তে স্থায়ীভাবে
            আটকে যেত ("servo freeze")। এই timeout সেই অবস্থা থেকে
            বেরিয়ে এসে আবার wall-following এ ফিরে যেতে দেয়।
        direction_history_size : দিক ঠিক করার আগে সাম্প্রতিক কতগুলো
            scan এর left/right গড় করা হবে (noise-resistance এর জন্য)।
        front_confirm_frames : front < front_trigger_m শর্তটা পরপর
            কতবার (scan) সত্যি হলে তবেই আসলে turn শুরু হবে। ১ রাখলে
            আগের মতোই single-frame আচরণ (কোনো protection নেই)।
            ⚠️ কেন দরকার: একটামাত্র bad/wobble frame-এ মাঝ-করিডোরেই
            ভুল করে turn শুরু হয়ে যাওয়া ঠেকাতে।
        turn_cooldown_seconds : একটা turn শেষ হওয়ার পর এই সময়টুকু
            নতুন কোনো turn শুরু হতে দেওয়া হবে না। ⚠️ কেন দরকার:
            উপরের ডকস্ট্রিং-এর "cascade" সমস্যা -- turn ৯০°-এর কম
            হয়ে থামায় corner শেষে robot তেরছা থাকে, front দূরত্ব
            আবার নেমে যায়, আর পরপর turn চলতেই থাকে। ১.৫ সেকেন্ড
            wall-following কে সোজা হওয়ার সুযোগ দেয় (লগে আসল corner
            গুলো পরস্পর ~৪ সেকেন্ড দূরে, তাই আসল corner কখনো
            cooldown-এ আটকাবে না)।
        min_open_side_m : corner ধরার জন্য left/right এর অন্তত একটা
            পাশ এই দূরত্বের চেয়ে দূরে (বা পুরো ফাঁকা) থাকতে হবে।
            ⚠️ কেন দরকার: আসল corner-এ ভেতরের দেয়াল শেষ হয়ে যায়,
            তাই একটা পাশ সত্যিই খোলা থাকে। দুই লগ মিলিয়ে মাপা:
            আসল corner-এ খোলা পাশ ছিল ১.০৫-১.৬৫মি, আর ভুয়া
            trigger-এ সর্বোচ্চ ০.৮৩মি -- তাই ০.৯মি ১০০% আলাদা করে।
            None (কোনো wall data নেই) মানে সেই পাশ max_wall_distance_m
            এর চেয়েও দূরে, অর্থাৎ খোলা -- তাই শর্ত পূর্ণ ধরা হয়।
        """
        self.servo_center = servo_center
        self.servo_left_max = servo_left_max
        self.servo_right_max = servo_right_max
        self.front_trigger_m = front_trigger_m
        self.target_yaw_change_deg = target_yaw_change_deg
        self.max_turn_seconds = max_turn_seconds
        self.front_confirm_frames = front_confirm_frames
        self.turn_cooldown_seconds = turn_cooldown_seconds
        self.min_open_side_m = min_open_side_m

        self._turning = False
        self._turn_direction = None  # "left" অথবা "right"
        self._start_yaw = None
        self._turn_start_time = None
        self.last_turn_timed_out = False  # ডিবাগিং/স্ট্যাটাসে দেখানোর জন্য

        # ⚠️ পুরো round-এ একবারই ঠিক হবে, তারপর আর বদলাবে না
        self._locked_direction = None

        self._left_history = deque(maxlen=direction_history_size)
        self._right_history = deque(maxlen=direction_history_size)
        self._last_front_m = None  # শুধু লগের জন্য মনে রাখা
        self._front_below_streak = 0  # front < trigger পরপর কতবার সত্যি হয়েছে
        self._last_turn_end_time = None  # cooldown মাপার জন্য
        self._last_block_log_time = 0.0  # blocked ইভেন্ট লগ থ্রটল করার জন্য

    def is_turning(self):
        return self._turning

    def should_start_turn(self, scan):
        """সামনের দূরত্ব দেখে কোণ কাছে এসেছে কিনা বলে। turn চলাকালীন
        এটা caller-এর আবার চেক করার দরকার নেই (is_turning() ব্যবহার করুন)।

        পাশাপাশি এখানেই left/right এর সাম্প্রতিক history আপডেট হয়,
        যাতে start_turn() এ single-frame noise এর বদলে গড় মান ব্যবহার
        করা যায়। front < front_trigger_m পরপর কতবার সত্যি হলো সেটাও
        গোনা হয় -- front_confirm_frames এ না পৌঁছালে এখনো turn শুরুর
        সংকেত দেওয়া হয় না (একটা বিচ্ছিন্ন কাছের reading যথেষ্ট না)।

        streak পূর্ণ হওয়ার পরেও দুটো gate পার হতে হয় -- cooldown
        (আগের turn শেষ হওয়ার পর যথেষ্ট সময় গেছে কিনা) আর geometry
        guard (অন্তত একটা পাশ সত্যিই খোলা কিনা)। কোনো gate আটকালে
        সেটা "blocked_*" ইভেন্ট হিসেবে লগে যায় (থ্রটল করা), যাতে
        পরে দেখা যায় guard আসল corner ভুল করে আটকাচ্ছে কিনা।"""
        sectors = get_sector_distances(scan)
        front = sectors["front"]
        left = sectors["left"]
        right = sectors["right"]
        self._last_front_m = front

        if left is not None:
            self._left_history.append(left)
        if right is not None:
            self._right_history.append(right)

        if front is not None and front < self.front_trigger_m:
            self._front_below_streak += 1
        else:
            self._front_below_streak = 0

        if self._front_below_streak < self.front_confirm_frames:
            return False

        # --- gate ১: cooldown (cascade ঠেকানো) ---
        if self._last_turn_end_time is not None:
            since_last = time.time() - self._last_turn_end_time
            if since_last < self.turn_cooldown_seconds:
                self._log_blocked("blocked_cooldown", left, right)
                return False

        # --- gate ২: geometry guard (করিডোরের মাঝে ভুয়া corner ঠেকানো) ---
        # None = সেই পাশে max_wall_distance_m এর ভেতরে কোনো দেয়াল নেই,
        # অর্থাৎ পাশটা খোলা -- শর্ত এমনিতেই পূর্ণ
        open_side = (left is None or right is None
                     or max(left, right) >= self.min_open_side_m)
        if not open_side:
            self._log_blocked("blocked_no_open_side", left, right)
            return False

        return True

    def _log_blocked(self, event, left, right):
        """gate আটকে দেওয়ার ঘটনা লগ করে -- প্রতি scan এ না লিখে
        সেকেন্ডে একবার (থ্রটল), নাহলে দেয়ালের সামনে দাঁড়িয়ে থাকলে
        ফাইল ভরে যেত।"""
        now = time.time()
        if now - self._last_block_log_time < 1.0:
            return
        self._last_block_log_time = now
        _log_event({
            "time": time.strftime("%H:%M:%S"),
            "event": event,
            "direction": self._locked_direction or "",
            "avg_left_m": f"{left:.3f}" if left is not None else "",
            "avg_right_m": f"{right:.3f}" if right is not None else "",
            "front_m": f"{self._last_front_m:.3f}" if self._last_front_m is not None else "",
            "start_yaw": "", "end_yaw": "", "yaw_change": "", "timed_out": "",
        })

    def start_turn(self, scan, current_yaw):
        """turn শুরু করা -- প্রথম corner-এ কোন দিকে ঘুরতে হবে সেটা
        সাম্প্রতিক কয়েকটা scan এর left/right গড় তুলনা করে ঠিক করা হয়
        (একটা বিচ্ছিন্ন noisy frame যেন ভুল দিক বেছে না নেয়), আর সেই
        দিকটাই পুরো round-এর জন্য lock হয়ে যায়। তারপর current_yaw কে
        রেফারেন্স হিসেবে সেভ করা হয়।

        ⚠️ দিক lock করা কেন: ট্র্যাকে driving direction (CW/CCW) একটা
        round-এ বদলায় না, তাই প্রতি corner-এ নতুন করে সিদ্ধান্ত নেওয়ার
        দরকার নেই -- উল্টো প্রতিবার সিদ্ধান্ত নেওয়া মানে প্রতিবার ভুল
        করার একটা নতুন সুযোগ। বাস্তব লগে দুটো ভুল ধরা পড়েছে: (ক) মাত্র
        ৫ মিলিমিটার ব্যবধানে (0.560 vs 0.565) উল্টো দিক বেছে নেওয়া,
        (খ) robot দেয়ালে আটকে গেলে এক পাশের sector ফাঁকা হয়ে যাওয়ায়
        ডিফল্ট "right" ধরে উল্টো ঘোরা। প্রথম corner-এ জ্যামিতি সবচেয়ে
        স্পষ্ট থাকে (লগে একদিকে ~১.৫মি, অন্যদিকে ~০.৩৫মি), আর
        min_open_side_m guard নিশ্চিত করে যে প্রথম সিদ্ধান্তটা সত্যিকারের
        corner-এই নেওয়া হচ্ছে -- তাই সেটাই সবচেয়ে বিশ্বাসযোগ্য।"""
        avg_left = (sum(self._left_history) / len(self._left_history)
                    if self._left_history else None)
        avg_right = (sum(self._right_history) / len(self._right_history)
                     if self._right_history else None)

        if self._locked_direction is not None:
            direction = self._locked_direction
        else:
            if avg_left is None and avg_right is None:
                # কোনো দিকেই wall data নেই -- অনির্ধারিত অবস্থা, ডিফল্ট
                # হিসেবে ডানে (WRO rule-এও অনির্ধারিত হলে clockwise ডিফল্ট,
                # KMIDS-এর কোডেও একই ডিফল্ট আছে)
                direction = "right"
            elif avg_left is None:
                # বামে কোনো wall data নেই মানে সেদিকটা খোলা -- সেদিকে ঘোরা
                direction = "left"
            elif avg_right is None:
                direction = "right"
            else:
                # দুই দিকেই wall আছে -- যেদিকে বেশি জায়গা (দূরত্ব বেশি) সেদিকে
                direction = "left" if avg_left > avg_right else "right"
            self._locked_direction = direction

        self._turning = True
        self._turn_direction = direction
        self._start_yaw = current_yaw
        self._turn_start_time = time.time()
        self.last_turn_timed_out = False

        _log_event({
            "time": time.strftime("%H:%M:%S"),
            "event": "start",
            "direction": direction,
            "avg_left_m": f"{avg_left:.3f}" if avg_left is not None else "",
            "avg_right_m": f"{avg_right:.3f}" if avg_right is not None else "",
            "front_m": f"{self._last_front_m:.3f}" if self._last_front_m is not None else "",
            "start_yaw": f"{current_yaw:.1f}",
            "end_yaw": "", "yaw_change": "", "timed_out": "",
        })
        # history পরিষ্কার করা হচ্ছে -- এই turn-এর পুরনো (pre-turn) ডেটা
        # যেন পরের কোণের সিদ্ধান্তে ভুলবশত মিশে না যায়
        self._left_history.clear()
        self._right_history.clear()
        self._front_below_streak = 0

    def get_turn_angle(self):
        """turn চলাকালীন servo angle -- নির্বাচিত দিকে সর্বোচ্চ ঘোরানো।"""
        if self._turn_direction == "left":
            return self.servo_left_max
        return self.servo_right_max

    def check_turn_complete(self, current_yaw):
        """turn শেষ হয়েছে কিনা যাচাই করে -- হয় gyro yaw যথেষ্ট বদলেছে,
        নয়তো নিরাপত্তা timeout পার হয়েছে। শেষ হলে True রিটার্ন করে এবং
        internal state রিসেট করে (পরের বার should_start_turn() আবার
        নতুন করে কাজ করবে)।"""
        if self._start_yaw is None:
            return True

        yaw_change = current_yaw - self._start_yaw
        # -180..180 রেঞ্জে normalize করা (safety, yaw wrap করলেও ঠিক থাকার জন্য)
        yaw_change = (yaw_change + 180) % 360 - 180

        if self._turn_direction == "left":
            done = yaw_change <= -self.target_yaw_change_deg
        else:
            done = yaw_change >= self.target_yaw_change_deg

        # নিরাপত্তা timeout: yaw যথেষ্ট না বদলালেও (robot আটকে গেছে/চাকা
        # পিছলাচ্ছে) নির্দিষ্ট সময় পর turn ছেড়ে wall-following এ ফেরা
        timed_out = False
        if not done and self._turn_start_time is not None:
            if time.time() - self._turn_start_time > self.max_turn_seconds:
                timed_out = True
                done = True

        if done:
            self.last_turn_timed_out = timed_out
            _log_event({
                "time": time.strftime("%H:%M:%S"),
                "event": "complete",
                "direction": self._turn_direction,
                "avg_left_m": "", "avg_right_m": "", "front_m": "",
                "start_yaw": f"{self._start_yaw:.1f}",
                "end_yaw": f"{current_yaw:.1f}",
                "yaw_change": f"{yaw_change:.1f}",
                "timed_out": timed_out,
            })
            self._turning = False
            self._turn_direction = None
            self._start_yaw = None
            self._turn_start_time = None
            # cooldown এখান থেকে গোনা শুরু -- পরের turn অন্তত
            # turn_cooldown_seconds পরে শুরু হতে পারবে
            self._last_turn_end_time = time.time()
            self._front_below_streak = 0

        return done
