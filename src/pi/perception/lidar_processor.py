"""
============================================================
lidar_processor.py — RPLIDAR raw ডেটা থেকে robot-relative তথ্য
(সেক্টর দূরত্ব, pillar candidate cluster) বের করা
============================================================
এই মডিউলের কো-অর্ডিনেট রূপান্তর তিনটা **হাতে-মাপা** ভ্যালুর উপর
ভিত্তি করে তৈরি (অনুমান করা হয়নি, test_lidar_orientation.py আর
test_lidar_blocked_zone.py দিয়ে পরিমাপ করা হয়েছে):

  1. FRONT_OFFSET_DEG = 269.5°
     robot-এর ঠিক সামনে বস্তু রেখে raw angle বারবার মেপে (268.6°-
     270.7° রেঞ্জে, গড় ≈269.5°) বের করা হয়েছে।

  2. ROTATION SENSE: raw angle বৃদ্ধি = robot-এর সাপেক্ষে ডানদিকে
     (ঘড়ির কাঁটার দিকে)। বস্তু সামনে (269.5°) থেকে ডানে সরালে angle
     বেড়ে 308.2° হয়েছে -- তাই নিশ্চিত।

  3. BLOCKED_ZONE_RAW = 45°-125° (battery-blocked, robot-এর পেছনে
     battery বসানো থাকায় এই রেঞ্জে RPLIDAR কোনো valid point দেয় না)।
     robot-relative এ এটা ~135°-215° এ পড়ে (প্রায় সরাসরি পেছনে,
     যেটা physically ঠিক ধারণার সাথে মেলে -- sanity check পাস)।

কো-অর্ডিনেট কনভেনশন (robot-relative, KMIDS এর lidarToCartesian
এর মতোই):
  - angle: 0° = সামনে, ধনাত্মক = ডানে (CW), 90°=ডান, 180°=পেছন,
    270°(=-90°)=বাম
  - x, y (মিটার): x = distance·sin(angle), y = distance·cos(angle)
    -> +y = সামনে, +x = ডানে

⚠️ robot-এর mechanical পরিবর্তন হলে (LIDAR খুলে আবার লাগালে ইত্যাদি)
এই তিনটা ভ্যালু আবার মেপে নিশ্চিত হওয়া উচিত, না হলে পুরনো ভুল
ভ্যালুর উপর ভিত্তি করে wall-following কাজ করবে।
============================================================
"""

import math

FRONT_OFFSET_DEG = 269.5
BLOCKED_ZONE_RAW = (45.0, 125.0)  # raw angle রেঞ্জ, battery-blocked

MIN_VALID_DISTANCE_MM = 50     # এর কম দূরত্ব সাধারণত সেন্সর নয়েজ/robot-এর নিজের অংশ
MAX_VALID_DISTANCE_MM = 4000   # WRO ফিল্ড 3x3 মিটার, এর বেশি দূরত্ব প্রাসঙ্গিক না


def raw_to_robot_angle(raw_angle_deg):
    """raw LIDAR angle -> robot-relative angle (0-360, 0=সামনে, +=ডানে/CW)।"""
    return (raw_angle_deg - FRONT_OFFSET_DEG) % 360


def is_blocked_angle(raw_angle_deg):
    """এই raw angle-টা battery-blocked জোনে পড়ে কিনা।"""
    lo, hi = BLOCKED_ZONE_RAW
    return lo <= raw_angle_deg <= hi


def scan_to_robot_points(scan):
    """rplidar এর iter_scans() থেকে পাওয়া একটা scan (quality, angle,
    distance টুপলের লিস্ট) কে robot-relative (x, y, robot_angle,
    distance_m) টুপলের লিস্টে রূপান্তর করে -- battery-blocked এবং
    অবৈধ দূরত্বের পয়েন্ট বাদ দিয়ে।

    x, y মিটারে, LIDAR-এর অবস্থান থেকে (robot-এর যান্ত্রিক কেন্দ্র
    থেকে LIDAR-এর সামান্য অফসেট আপাতত উপেক্ষা করা হচ্ছে -- Phase 5
    এ wall-following টিউন করার সময় দরকার হলে যোগ করা হবে)।
    """
    points = []
    for quality, raw_angle, distance_mm in scan:
        if distance_mm < MIN_VALID_DISTANCE_MM or distance_mm > MAX_VALID_DISTANCE_MM:
            continue
        if is_blocked_angle(raw_angle):
            continue

        robot_angle = raw_to_robot_angle(raw_angle)
        distance_m = distance_mm / 1000.0
        rad = math.radians(robot_angle)
        x = distance_m * math.sin(rad)
        y = distance_m * math.cos(rad)
        points.append((x, y, robot_angle, distance_m))
    return points


# ---- সেক্টর-ভিত্তিক দূরত্ব (wall-following এর জন্য) ----
# angle robot-relative, -180..180 রেঞ্জে (0=সামনে, ধনাত্মক=ডানে)।
# লক্ষ্য করুন: সরাসরি "back" সেক্টর ইচ্ছাকৃতভাবে বাদ দেওয়া হয়েছে,
# কারণ ওটা পুরোপুরি battery-blocked জোনের (~135°-215°) সাথে মিলে
# যায় -- ওখানে কখনোই ভরসাযোগ্য ডেটা পাওয়া যাবে না।
DEFAULT_SECTORS = {
    "front":       (-20, 20),
    "front_right": (20, 70),
    "right":       (70, 110),
    "back_right":  (110, 160),
    "back_left":   (-160, -110),
    "left":        (-110, -70),
    "front_left":  (-70, -20),
}


def get_sector_distances(scan, sectors=None):
    """প্রতিটা সেক্টরে সবচেয়ে কাছের বস্তুর দূরত্ব (মিটার) বের করে।

    sectors: {নাম: (min_angle, max_angle)} dict; না দিলে DEFAULT_SECTORS
    রিটার্ন: {নাম: distance_m অথবা None (ঐ সেক্টরে কোনো বৈধ পয়েন্ট না থাকলে)}
    """
    if sectors is None:
        sectors = DEFAULT_SECTORS

    points = scan_to_robot_points(scan)
    result = {name: None for name in sectors}

    for x, y, robot_angle, distance_m in points:
        # robot_angle 0-360 রেঞ্জে থাকে, সেক্টর ডেফিনিশন -180..180 তে --
        # তুলনা করার আগে normalize করা হচ্ছে
        signed_angle = robot_angle if robot_angle <= 180 else robot_angle - 360

        for name, (a_min, a_max) in sectors.items():
            if a_min <= signed_angle <= a_max:
                if result[name] is None or distance_m < result[name]:
                    result[name] = distance_m

    return result


# ---- পয়েন্ট ক্লাস্টারিং (pillar candidate detection) ----

def cluster_points(points, distance_threshold_m=0.05):
    """কাছাকাছি (x,y) পয়েন্টগুলোকে একসাথে জোড়া দিয়ে ক্লাস্টার বানায়
    (KMIDS-এর getTrafficLightPoints() এর ক্লাস্টারিং ধারণা থেকে) --
    প্রতিটা ক্লাস্টারের গড় (x, y) রিটার্ন করে, যেটা সম্ভাব্য একটা
    বস্তুর (pillar) কেন্দ্রবিন্দু।

    points : scan_to_robot_points() এর রিটার্ন করা লিস্ট
    distance_threshold_m : এর চেয়ে কাছাকাছি (ধারাবাহিক) দুটো পয়েন্ট
                            একই বস্তুর অংশ ধরা হবে
    রিটার্ন: [(x, y), ...] প্রতিটা ক্লাস্টারের কেন্দ্র
    """
    if not points:
        return []

    # robot_angle অনুযায়ী সাজানো, যাতে একই বস্তুর ধারাবাহিক পয়েন্ট
    # পাশাপাশি ইনডেক্সে থাকে -- ক্লাস্টারিং সহজ ও দ্রুত হয়
    sorted_points = sorted(points, key=lambda p: p[2])

    clusters = []
    current_cluster = [sorted_points[0]]

    for p in sorted_points[1:]:
        last = current_cluster[-1]
        dist = math.hypot(p[0] - last[0], p[1] - last[1])
        if dist < distance_threshold_m:
            current_cluster.append(p)
        else:
            clusters.append(current_cluster)
            current_cluster = [p]
    clusters.append(current_cluster)

    centers = []
    for cluster in clusters:
        avg_x = sum(p[0] for p in cluster) / len(cluster)
        avg_y = sum(p[1] for p in cluster) / len(cluster)
        centers.append((avg_x, avg_y))

    return centers
