"""
============================================================
pillar_avoider.py — pillar-এর সঠিক পাশ দিয়ে পার হওয়ার লজিক
============================================================
রুল (WRO 9.19): লাল pillar ডান দিক দিয়ে, সবুজ pillar বাম দিক দিয়ে
পার হতে হবে।

কৌশল (KMIDS-এর ধারণা থেকে অনুপ্রাণিত, কিন্তু সরলীকৃত): আলাদা কোনো
"avoidance maneuver" বানানো হয়নি। বরং pillar-এর robot-relative
(x, y) অবস্থান থেকে একটা "লক্ষ্য x অবস্থান" (aim_x) বের করা হয় --
pillar-এর ঠিক ডানে (লাল) বা বামে (সবুজ) একটা নিরাপদ মার্জিন রেখে --
আর সেটাকে wall_follower-এর ঠিক সেই একই, আগে থেকে টিউন করা PID/
smoothing/rate-limit pipeline-এ error হিসেবে পাঠানো হয়
(wall_following.py-র _steer_from_error())।

⚠️ Persistent voting: একবার scan-এ pillar দেখেই react করা হয় না --
পরপর confirm_frames বার একই রঙের pillar কাছাকাছি থাকলে তবেই
"active" ধরে lock করা হয়।

⚠️⚠️ Lock হওয়ার পর camera-র উপর নির্ভরতা বন্ধ (এটাই সবচেয়ে গুরুত্বপূর্ণ
অংশ, বাস্তব টেস্টে ধরা পড়া একটা bug-এর সমাধান): camera-র field of
view সরু (~৬৯° -- calibrate_camera_hfov.py দিয়ে মাপা), কিন্তু pillar
এড়ানোর সময় robot নিজেই বেঁকে যায় বলে pillar-এর robot-relative angle
দ্রুত বদলে যায় -- ফলে robot এখনো physically pillar-এর পাশ কাটায়নি,
কিন্তু pillar camera-র ফ্রেম থেকে বেরিয়ে গেছে। camera-নির্ভর হলে এটাকে
ভুল করে "পার হয়ে গেছে" ধরে avoidance বন্ধ হয়ে যেত, আর robot ঠিক
সবচেয়ে জরুরি মুহূর্তেই সোজা হয়ে pillar-এ ধাক্কা খেত।

সমাধান: একবার রঙ confirm হয়ে lock হওয়ার পর, প্রতি scan-এ camera দিয়ে
আবার confirm করার বদলে LIDAR-এর (৩৬০° দেখে, camera-র সরু FOV-এর
সীমাবদ্ধতা নেই) raw cluster-গুলোর মধ্যে থেকে শেষ জানা অবস্থানের
সবচেয়ে কাছেরটা খুঁজে (frame-to-frame nearest-neighbor tracking)
সেটাকেই pillar ধরে avoidance চালিয়ে যাওয়া হয়। "পার হয়ে গেছে" তখন
বোঝা যায় LIDAR-এর y (সামনের দূরত্ব) শূন্য/ঋণাত্মক হয়ে গেলে (pillar
এখন robot-এর পাশে/পেছনে) -- camera কিছু দেখছে কিনা তার উপর নির্ভর
করে না।
============================================================
"""

import csv
import math
import os
import time
from collections import deque

from perception.lidar_processor import scan_to_robot_points, cluster_points, get_sector_distances
from perception.pillar_detector import detect_pillars, MAX_PILLAR_EXTENT_M

# ⚠️ ডায়াগনস্টিক লগ -- green pillar কেন মাঝে মাঝে "একবার দেখেই ছেড়ে
# দেয়" (red এর মতো স্থিরভাবে lock/track হয় না) সেটা বোঝার জন্য।
# প্রতিটা scan এ camera যা যা pillar দেখেছে (filter করার আগে, তাই
# trigger_distance_m/max_side_angle_deg এ বাদ পড়া pillar ও দেখা যাবে),
# history/lock/release অবস্থা -- সবকিছু row আকারে লেখা হয়। কোনো
# throttle রাখা হয়নি (ইচ্ছাকৃত) -- যাতে ঠিক কোন frame এ green হারিয়ে
# যাচ্ছে সেটা মিস না হয় (এই log শুধু ডিবাগের জন্য অল্প সময় চালানো
# হবে, তাই SD card I/O নিয়ে চিন্তা না করলেও চলবে)।
LOG_FILE = os.path.join(os.path.dirname(__file__), "..", "logs", "pillar_avoider_log.csv")
_LOG_HEADER = [
    "time", "state", "event",
    "detected_pillars",      # detect_pillars() এর raw ফলাফল (filter করার আগে): "color:y:angle,..."
    "candidate_color", "candidate_y", "candidate_angle",
    "history", "active_color", "locked_x", "locked_y", "note",
]


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


class PillarAvoider:
    def __init__(self, margin_m=0.20, trigger_distance_m=0.65,
                 confirm_frames=3, max_side_angle_deg=55,
                 track_jump_limit_m=0.20, wall_clearance_m=0.09):
        """
        margin_m : pillar-এর কেন্দ্র থেকে কতটা পাশ দিয়ে (ডানে লাল/বামে
            সবুজ) পার হবে -- robot-এর অর্ধেক প্রস্থ + নিরাপত্তা মার্জিন
        trigger_distance_m : pillar এই দূরত্বের মধ্যে (সামনে) এলে
            avoidance সক্রিয় হবে। ⚠️ RPLIDAR ছোট বস্তু (pillar) দূরে
            গেলে নির্ভরযোগ্যভাবে দেখতে পায় না (triangulation LIDAR-এর
            স্বাভাবিক সীমাবদ্ধতা -- ছোট বস্তুর প্রতিফলিত আলো দূরত্বের
            সাথে দ্রুত দুর্বল হয়ে যায়, বড় দেয়ালের মতো না) -- তাই এই
            মান বাস্তবে LIDAR যতদূর pillar দেখতে পায় তার বেশি রাখলে
            কোনো লাভ নেই।
        confirm_frames : প্রাথমিক lock করার আগে পরপর কতবার একই রঙের
            pillar দেখলে "নিশ্চিত" ধরা হবে
        max_side_angle_deg : lock করার আগে এই কোণের বাইরে থাকা pillar
            উপেক্ষা করা হবে
        track_jump_limit_m : lock হওয়ার পর, শেষ জানা অবস্থান থেকে
            একটা LIDAR cluster এর বেশি দূরে হলে সেটাকে "একই pillar"
            ধরা হবে না (ভুল বস্তুতে lock না হয়ে যাওয়ার সুরক্ষা)
        wall_clearance_m : ⚠️ pillar-এর margin ধরে aim_x হিসাব করার
            সময় বিপরীত/একই দিকের দেয়ালের কথা মাথায় রাখা হতো না --
            pillar দেয়ালের কাছে থাকলে বা margin বড় হলে aim_x দেয়ালের
            ওপারে চলে যেতে পারত, robot দেয়ালে ধাক্কা খেত। তাই এখন
            aim_x-কে বর্তমান LIDAR left/right সেক্টর দূরত্ব দিয়ে বাঁধা
            (clamp) হয় -- দেয়াল থেকে অন্তত এই দূরত্ব (robot-এর অর্ধেক
            প্রস্থ + নিরাপত্তা মার্জিন) রেখে।
        """
        self.margin_m = margin_m
        self.trigger_distance_m = trigger_distance_m
        self.confirm_frames = confirm_frames
        self.max_side_angle_deg = max_side_angle_deg
        self.track_jump_limit_m = track_jump_limit_m
        self.wall_clearance_m = wall_clearance_m

        self._history = deque(maxlen=confirm_frames)
        self._active_color = None
        self._locked_position = None  # (x, y) -- LIDAR দিয়ে track করা শেষ জানা অবস্থান
        self.last_pillar_color = None  # ডিবাগ/স্ট্যাটাসের জন্য

    def reset(self):
        self._history.clear()
        self._active_color = None
        self._locked_position = None

    def is_active(self):
        return self._active_color is not None

    def compute_steering_angle(self, scan, frame, calibration, wall_follower):
        """pillar avoidance সক্রিয় থাকলে servo angle (int) রিটার্ন
        করে, নাহলে None (caller তখন normal wall-following ব্যবহার করবে)।
        """
        if self._active_color is not None:
            return self._continue_locked(scan, wall_follower)
        return self._search_and_lock(scan, frame, calibration, wall_follower)

    def _continue_locked(self, scan, wall_follower):
        """ইতিমধ্যে lock করা pillar-কে LIDAR দিয়েই (camera ছাড়া) খুঁজে
        avoidance চালিয়ে যায়। pillar না পাওয়া গেলে বা পার হয়ে গেলে
        (y<=0) lock ছেড়ে দিয়ে None রিটার্ন করে।"""
        points = scan_to_robot_points(scan)
        clusters = [(x, y) for x, y, extent_m in cluster_points(points)
                    if extent_m <= MAX_PILLAR_EXTENT_M]

        target = self._nearest_to_locked(clusters)
        if target is not None:
            x, y = target
            if y > 0:  # এখনো robot-এর সামনে -- পার হয়নি
                self._locked_position = (x, y)
                _log_event({
                    "time": f"{time.time():.3f}", "state": "locked", "event": "track",
                    "detected_pillars": "", "candidate_color": "", "candidate_y": "",
                    "candidate_angle": "", "history": "",
                    "active_color": self._active_color,
                    "locked_x": f"{x:.3f}", "locked_y": f"{y:.3f}", "note": "",
                })
                return self._steer_toward_xy(x, self._active_color, wall_follower, scan)

        # cluster হারিয়ে গেছে অথবা y<=0 (pillar পাশে/পেছনে চলে গেছে) --
        # পার হয়ে গেছে ধরে স্বাভাবিক wall-following এ ফেরা
        lost_x, lost_y = self._locked_position if self._locked_position else (None, None)
        note = "cluster হারিয়ে গেছে" if target is None else "y<=0 (পার হয়ে গেছে)"
        _log_event({
            "time": f"{time.time():.3f}", "state": "locked", "event": "release",
            "detected_pillars": "", "candidate_color": "", "candidate_y": "",
            "candidate_angle": "", "history": "",
            "active_color": self._active_color,
            "locked_x": f"{lost_x:.3f}" if lost_x is not None else "",
            "locked_y": f"{lost_y:.3f}" if lost_y is not None else "",
            "note": note,
        })
        self._active_color = None
        self._locked_position = None
        self._history.clear()
        return None

    def _nearest_to_locked(self, clusters):
        if not clusters or self._locked_position is None:
            return None
        lx, ly = self._locked_position
        best = None
        best_dist = self.track_jump_limit_m
        for x, y in clusters:
            d = math.hypot(x - lx, y - ly)
            if d < best_dist:
                best_dist = d
                best = (x, y)
        return best

    def _search_and_lock(self, scan, frame, calibration, wall_follower):
        """এখনো কোনো pillar lock করা হয়নি -- camera+LIDAR ফিউজড
        detection দিয়ে (রঙ চেনার জন্য camera লাগবেই) নতুন pillar
        খুঁজে, persistent voting দিয়ে নিশ্চিত হয়ে lock করে।"""
        try:
            pillars = detect_pillars(scan, frame, calibration)
        except ValueError:
            _log_event({
                "time": f"{time.time():.3f}", "state": "search", "event": "no_calibration",
                "detected_pillars": "", "candidate_color": "", "candidate_y": "",
                "candidate_angle": "", "history": ",".join(self._history),
                "active_color": "", "locked_x": "", "locked_y": "",
                "note": "camera calibration নেই (ValueError)",
            })
            return None  # camera calibration নেই -- avoidance বন্ধ, wall-following এ ফলব্যাক

        detected_str = ",".join(
            f"{p.color}:{p.y:.2f}:{p.angle_deg:.1f}" for p in pillars
        )

        candidates = [
            p for p in pillars
            if 0 < p.y <= self.trigger_distance_m and abs(p.angle_deg) <= self.max_side_angle_deg
        ]

        if candidates:
            nearest = min(candidates, key=lambda p: p.y)
            self._history.append(nearest.color)
            self.last_pillar_color = nearest.color
        else:
            self._history.clear()

        _log_event({
            "time": f"{time.time():.3f}", "state": "search",
            "event": "candidate" if candidates else "no_candidate",
            "detected_pillars": detected_str,
            "candidate_color": candidates and nearest.color or "",
            "candidate_y": f"{nearest.y:.3f}" if candidates else "",
            "candidate_angle": f"{nearest.angle_deg:.1f}" if candidates else "",
            "history": ",".join(self._history), "active_color": "",
            "locked_x": "", "locked_y": "", "note": "",
        })

        if len(self._history) == self.confirm_frames and len(set(self._history)) == 1:
            self._active_color = self._history[0]
            self._history.clear()
            same_color = [p for p in candidates if p.color == self._active_color]
            if same_color:
                target = min(same_color, key=lambda p: p.y)
                self._locked_position = (target.x, target.y)
                _log_event({
                    "time": f"{time.time():.3f}", "state": "search", "event": "lock",
                    "detected_pillars": detected_str, "candidate_color": "",
                    "candidate_y": "", "candidate_angle": "", "history": "",
                    "active_color": self._active_color,
                    "locked_x": f"{target.x:.3f}", "locked_y": f"{target.y:.3f}",
                    "note": "",
                })
                return self._steer_toward_xy(target.x, self._active_color, wall_follower, scan)

        return None

    def _steer_toward_xy(self, pillar_x, color, wall_follower, scan):
        # লাল -> pillar-এর ডানে পার হওয়া -> aim_x = pillar.x + margin
        # সবুজ -> pillar-এর বামে পার হওয়া -> aim_x = pillar.x - margin
        sign = 1 if color == "red" else -1
        aim_x = pillar_x + sign * self.margin_m

        # ⚠️ বিপরীত/একই দিকের দেয়ালে ধাক্কা না খাওয়ার সুরক্ষা -- aim_x
        # কে বর্তমান LIDAR left/right সেক্টর দূরত্ব দিয়ে বাঁধা হচ্ছে
        sectors = get_sector_distances(scan)
        right = sectors["right"]
        left = sectors["left"]
        if right is not None:
            aim_x = min(aim_x, right - self.wall_clearance_m)
        if left is not None:
            aim_x = max(aim_x, -(left - self.wall_clearance_m))

        # wall_following.py-র error convention: ধনাত্মক error = ডানে
        # স্টিয়ার। aim_x-ই সরাসরি সেই error (robot-relative x, ডানে
        # ধনাত্মক -- lidar_processor.py-র কনভেনশনের সাথেই মেলে)।
        return wall_follower._steer_from_error(aim_x)
