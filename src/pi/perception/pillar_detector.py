"""
============================================================
pillar_detector.py — LIDAR cluster + camera রঙ ফিউজ করে pillar খোঁজা
============================================================
কৌশল (KMIDS-এর combineTrafficLightInfo()-এর ray-casting ধারণা থেকে):
LIDAR দিয়ে আশেপাশের সব কাছের বস্তুর (wall + pillar মিশ্রিত) নিখুঁত
(x, y) অবস্থান পাওয়া যায়, কিন্তু LIDAR নিজে রঙ বোঝে না। camera রঙ
বোঝে, কিন্তু camera আর LIDAR robot-এর ভিন্ন ভিন্ন বিন্দুতে বসানো
(parallax) -- তাই শুধু angle মিলিয়ে ফিউজ করলে কাছের বস্তুর জন্য ভুল
হতে পারে।

তাই প্রতিটা camera-detected রঙিন ব্লকের angle থেকে camera-র **আসল
ভৌত অবস্থান** (camera_offset_m, robot-relative, LIDAR-এর কেন্দ্র
থেকে ruler দিয়ে মাপা) থেকে একটা রশ্মি (ray) ছোঁড়া হয়, আর সেই রশ্মি
কোন LIDAR cluster-কে (PILLAR_MATCH_RADIUS_M ব্যাসার্ধের একটা বৃত্ত
ধরে) সবচেয়ে কাছ থেকে ছেদ করছে সেটা জ্যামিতিকভাবে বের করা হয় --
সেই cluster-এর নিখুঁত LIDAR (x,y)/দূরত্ব, আর camera ব্লকের রঙ মিলিয়ে
চূড়ান্ত PillarDetection বানানো হয়।

⚠️ একই cluster-কে দুই রঙের রশ্মি একসাথে hit করলে (দ্বন্দ্ব) সেটা বাদ
দেওয়া হয় -- KMIDS-এর conflict resolution এর মতোই, একটা বস্তুর তো
একটাই আসল রঙ হতে পারে, দুটো মিললে বোঝা যায় কোনো একটা ray ভুল বস্তুতে
লেগেছে।

⚠️ আলাদা করে "এটা pillar নাকি wall" ফিল্টার করার দরকার নেই কেন: wall-এর
রঙ কালো (rule 13.6), তাই কখনোই লাল/সবুজ HSV রেঞ্জের সাথে ম্যাচ করবে
না -- camera color detection নিজেই এই ফিল্টারের কাজ করে দেয় (রঙ না
মিললে সেই দিকে কোনো ray-ই ছোঁড়া হয় না, wall কখনো candidate হিসেবে
আসে না)।
============================================================
"""

import math

from perception.lidar_processor import scan_to_robot_points, cluster_points
from perception.camera_processor import detect_blocks, pixel_to_robot_angle

# pillar-এর প্রকৃত আকার ৫০x৫০মিমি (rule 13.19), কর্ণ ~৭১মিমি -- LIDAR
# noise/একাধিক beam এর জন্য কিছুটা মার্জিন রেখে এর চেয়ে বড় cluster
# (দেয়াল/আসবাবের অংশ) বাদ দেওয়া হয়।
MAX_PILLAR_EXTENT_M = 0.12

# ray-circle intersection এ pillar কে ধরার জন্য বৃত্তের ব্যাসার্ধ --
# KMIDS-এর trafficLightRadius (0.08m) এর মতোই, pillar-এর আসল আকারের
# (অর্ধ-কর্ণ ~0.035m) চেয়ে বড় মার্জিন রেখে যাতে camera angle-এর সামান্য
# ভুলেও (linear approximation) ray মিস না হয়।
PILLAR_MATCH_RADIUS_M = 0.08


class PillarDetection:
    """একটা ফিউজড detection -- LIDAR-এর নিখুঁত (x, y)/দূরত্ব, camera-র রঙ।"""

    def __init__(self, x, y, distance_m, angle_deg, color):
        self.x = x
        self.y = y
        self.distance_m = distance_m
        self.angle_deg = angle_deg
        self.color = color  # "red" / "green" (এখানে None আসে না -- রঙ না মিললে detection-ই তৈরি হয় না)

    def __repr__(self):
        return (f"PillarDetection({self.color}, dist={self.distance_m:.2f}m, "
                f"angle={self.angle_deg:+.1f}°)")


def _ray_circle_intersect(origin, direction, center, radius):
    """origin থেকে direction (unit vector) দিকে ছোঁড়া রশ্মি center-কেন্দ্রিক
    radius ব্যাসার্ধের বৃত্তকে ছেদ করে কিনা, করলে সবচেয়ে কাছের (ধনাত্মক)
    ছেদ-দূরত্ব (t) রিটার্ন করে, না করলে None। KMIDS-এর
    rayCircleIntersect() এর মতোই আদর্শ ray-circle intersection formula।"""
    ox, oy = origin
    dx, dy = direction
    cx, cy = center
    ocx, ocy = ox - cx, oy - cy

    b = 2.0 * (ocx * dx + ocy * dy)
    c = ocx * ocx + ocy * ocy - radius * radius
    disc = b * b - 4 * c
    if disc < 0:
        return None

    sqrt_disc = math.sqrt(disc)
    t1 = (-b - sqrt_disc) * 0.5
    t2 = (-b + sqrt_disc) * 0.5

    if t1 >= 0:
        return t1
    if t2 >= 0:
        return t2
    return None


def detect_pillars(scan, frame, calibration):
    """একটা LIDAR scan + camera frame (BGR) থেকে ray-casting দিয়ে ফিউজড
    pillar detection-এর লিস্ট রিটার্ন করে -- শুধু camera-তে রঙ পাওয়া
    গেছে আর সেই রশ্মি pillar-আকারের কোনো LIDAR cluster-কে ছেদ করেছে,
    এমন detection-ই ফিরে আসে।

    calibration : config.load_calibration() থেকে পাওয়া dict, এতে
        camera_hfov_deg, camera_angle_offset_deg, camera_offset_m
        ক্যালিব্রেট করা থাকতে হবে (calibrate_camera_hfov.py)
    """
    hfov = calibration.get("camera_hfov_deg")
    angle_offset = calibration.get("camera_angle_offset_deg")
    cam_offset = calibration.get("camera_offset_m", {})
    if hfov is None or angle_offset is None or cam_offset.get("x") is None or cam_offset.get("y") is None:
        raise ValueError("camera_hfov_deg/camera_angle_offset_deg/camera_offset_m ক্যালিব্রেট করা নেই -- "
                          "আগে calibrate_camera_hfov.py চালান।")

    origin = (cam_offset["x"], cam_offset["y"])
    image_width = frame.shape[1]

    points = scan_to_robot_points(scan)
    clusters = [(x, y) for x, y, extent_m in cluster_points(points) if extent_m <= MAX_PILLAR_EXTENT_M]

    blocks = detect_blocks(frame, calibration)

    # প্রতিটা cluster ইনডেক্সে কোন কোন রঙ hit করেছে সেটা জমা রাখা হয়,
    # যাতে দ্বন্দ্ব (একই cluster-এ দুই রঙ) ধরা যায়
    cluster_hits = {}  # cluster_index -> set of colors
    for color in ("red", "green"):
        for block in blocks[color]:
            angle_deg = pixel_to_robot_angle(block.center_x, image_width, hfov, angle_offset)
            angle_rad = math.radians(angle_deg)
            direction = (math.sin(angle_rad), math.cos(angle_rad))

            best_index = None
            best_t = None
            for i, (cx, cy) in enumerate(clusters):
                t = _ray_circle_intersect(origin, direction, (cx, cy), PILLAR_MATCH_RADIUS_M)
                if t is not None and (best_t is None or t < best_t):
                    best_t = t
                    best_index = i

            if best_index is not None:
                cluster_hits.setdefault(best_index, set()).add(color)

    results = []
    for index, colors in cluster_hits.items():
        if len(colors) > 1:
            continue  # একাধিক রঙ একই বস্তুতে hit করেছে -- দ্বন্দ্বপূর্ণ, বাদ
        x, y = clusters[index]
        distance_m = math.hypot(x, y)
        angle_deg = math.degrees(math.atan2(x, y))
        results.append(PillarDetection(x, y, distance_m, angle_deg, next(iter(colors))))

    return results
