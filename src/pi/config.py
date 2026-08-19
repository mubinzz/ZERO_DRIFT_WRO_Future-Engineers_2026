"""
============================================================
config.py — ক্যালিব্রেশন ভ্যালু সেভ/লোড করার shared মডিউল
============================================================
calibration/ ফোল্ডারের স্ক্রিপ্টগুলো এখানে (config/calibration.json)
ফলাফল সেভ করবে। পরে perception/control/challenges কোড এখান থেকেই
লোড করবে -- এতে magic number গুলো কোডের এখানে-ওখানে ছড়িয়ে না থেকে
একটা জায়গায় গোছানো থাকবে, আর re-calibrate করলে শুধু এই একটা ফাইল
বদলালেই সব জায়গায় নতুন ভ্যালু কার্যকর হয়ে যাবে।
============================================================
"""

import json
import os

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "config")
CALIBRATION_FILE = os.path.join(CONFIG_DIR, "calibration.json")

# calibration.json এ কিছু না থাকলে (এখনো ক্যালিব্রেট করা হয়নি) এই
# ডিফল্ট ভ্যালুগুলো ব্যবহার হবে -- এগুলো শুধু আন্দাজ, আসল রোবটে
# কাজ নাও করতে পারে, তাই calibration script চালিয়ে আসল ভ্যালু
# বের করাটা জরুরি।
DEFAULTS = {
    "servo_center": 90,
    "servo_left_max": 60,
    "servo_right_max": 120,
    "motor_deadzone": 80,
    # calibrate_encoder_distance.py দিয়ে মাপা -- ১ encoder tick কত মিমি
    # দূরত্বের সমান। Open Challenge-এর ফাইনাল রাউন্ডে ৩ lap শেষে finish
    # section-এ সঠিক জায়গায় থামার জন্য দরকার (সময়/PWM দিয়ে দূরত্ব
    # আন্দাজ করলে speed একটু কমবেশি হলেই ভুল হয়ে যেত)।
    "encoder_mm_per_tick": None,
    "camera_red_hsv": {"lower": [0, 100, 100], "upper": [10, 255, 255]},
    "camera_green_hsv": {"lower": [40, 100, 100], "upper": [80, 255, 255]},
    # calibrate_camera_hfov.py দিয়ে মাপা (KMIDS-এর pixelToAngle()/
    # combineTrafficLightInfo() পদ্ধতি থেকে অনুপ্রাণিত)। ক্যামেরার
    # pixel x-position থেকে robot-relative angle বের করার জন্য
    # camera_hfov_deg/camera_angle_offset_deg (সরল রৈখিক মডেল, ruler
    # দিয়ে জ্যামিতিকভাবে মাপা পিলার-অবস্থান থেকে fit করা, LIDAR লাগে
    # না -- তাই ঘরে LIDAR-এ ভুল বস্তু ধরা পড়ার সমস্যা এড়ানো যায়)।
    # camera_offset_m: camera-র lens LIDAR-এর কেন্দ্র থেকে ঠিক কতটা
    # সামনে/পাশে বসানো (ruler দিয়ে সরাসরি মাপা) -- Obstacle Challenge-এ
    # camera রঙ আর LIDAR দূরত্ব ray-casting দিয়ে ফিউজ করার জন্য দরকার
    # (camera আর LIDAR ভিন্ন বিন্দুতে বসানো, parallax হিসাবে ধরতে হয়)।
    "camera_hfov_deg": None,
    "camera_angle_offset_deg": None,
    "camera_offset_m": {
        "x": None,  # ধনাত্মক = LIDAR-এর কেন্দ্র থেকে ডানে
        "y": None,  # ধনাত্মক = LIDAR-এর কেন্দ্র থেকে সামনে
    },
    # tune_open_challenge_live.py দিয়ে টিউন করা wall-following/corner-turn
    # প্যারামিটার -- এখানে শুধু শুরুর অনুমান, লাইভ টিউনিং টুল দিয়ে
    # বাস্তব ট্র্যাকে ঠিক করে 's' চেপে এখানে সেভ হবে
    "open_challenge": {
        "speed": 170,
        "kp": 3.0,
        "kd": 0.3,
        "smoothing_alpha": 0.4,
        "max_angle_change_per_sec": 80,
        "front_trigger_m": 0.40,
        "front_confirm_frames": 3,
        "target_yaw_change_deg": 85,
        "target_gap_m": 0.45,
        "max_wall_distance_m": 1.2,
        "max_error_m": 0.8,
        "max_turn_seconds": 4.0,
        # একটা turn শেষ হওয়ার পর এই সময়টুকু নতুন turn শুরু হতে পারবে না।
        # ⚠️ বাস্তব লগে ধরা পড়া "cascade" ঠেকানোর জন্য -- একটা আসল corner
        # শেষ হওয়ার ২-৩ সেকেন্ডের মধ্যে পরপর দুটো ভুয়া turn শুরু হয়ে
        # robot ~২০০° ঘুরে যাচ্ছিল। আসল corner গুলো পরস্পর ~৪ সেকেন্ড
        # দূরে, তাই ১.৫ সেকেন্ড কোনো আসল corner আটকাবে না।
        "turn_cooldown_seconds": 1.5,
        # corner ধরার জন্য left/right এর অন্তত একটা পাশ এর চেয়ে দূরে
        # (বা পুরো ফাঁকা) থাকতে হবে -- আসল corner-এ ভেতরের দেয়াল শেষ
        # হয়ে যায় বলে একটা পাশ সত্যিই খোলা থাকে। দুই লগে মাপা: আসল
        # corner-এ খোলা পাশ ১.০৫-১.৬৫মি, ভুয়া trigger-এ সর্বোচ্চ ০.৮৩মি।
        "min_open_side_m": 0.9,
        # ৩য় lap-এর শেষ (১২তম) corner পার হওয়ার পর finish section-এ
        # (যেখান থেকে round শুরু হয়েছিল) কতটা এগিয়ে থেমে যেতে হবে,
        # মিমিতে। ⚠️ WRO rule 9.24.2: পুরো robot section-এর ভেতরে
        # সম্পূর্ণভাবে থামতে হবে (আগের/পরের section-এ না গিয়ে) --
        # section length round-ভেদে 600-1000mm হতে পারে, তাই মাঝামাঝি
        # নিরাপদ মান হিসেবে ডিফল্ট 250mm রাখা হলো। বাস্তব ট্র্যাকে
        # section-এর দৈর্ঘ্য মেপে টিউন করা লাগবে।
        "finish_stop_distance_mm": 250,
        # নিরাপত্তা fallback -- WRO rule 9.1 অনুযায়ী round ৩ মিনিটের,
        # judge নিজেই সময় ধরেন, কিন্তু robot নিজে থেকেও এর বেশি সময়
        # ধরে চলতে থাকা উচিত না (কিছুটা মার্জিন রেখে 175s)।
        "round_timeout_seconds": 175,
    },
    # pillar_avoider.py-র প্যারামিটার -- Obstacle Challenge-এ
    # wall-following/corner-turn এর জন্য open_challenge-এর মানই reuse
    # হয় (একই robot, একই physics), এখানে শুধু pillar-avoidance-এর
    # নতুন প্যারামিটার
    "pillar_avoider": {
        "margin_m": 0.1,
        # ⚠️ RPLIDAR ছোট বস্তু (pillar) দূরে গেলে নির্ভরযোগ্যভাবে দেখতে
        # পায় না (~0.7m এর বেশি দূরে অনির্ভরযোগ্য, বাস্তব টেস্টে মাপা) --
        # তাই এর বেশি রাখলে কোনো লাভ নেই, নিরাপত্তা মার্জিন সহ 0.65
        "trigger_distance_m": 0.75,
        "confirm_frames": 3,
        "max_side_angle_deg": 55,
        # lock হওয়ার পর camera ছাড়াই LIDAR দিয়ে pillar track করার সময়
        # (frame-to-frame) শেষ জানা অবস্থান থেকে এর বেশি দূরে কোনো
        # cluster পেলে সেটাকে আর "একই pillar" ধরা হবে না
        "track_jump_limit_m": 0.20,
        # aim_x (pillar এড়ানোর লক্ষ্য অবস্থান) দেয়াল থেকে অন্তত এই
        # দূরত্ব দূরে রাখা হবে (robot-এর অর্ধেক প্রস্থ ~0.07m +
        # নিরাপত্তা মার্জিন) -- নাহলে বড় margin_m বা দেয়ালের কাছের
        # pillar-এর ক্ষেত্রে aim_x দেয়ালের ওপারে চলে যেতে পারে
        "wall_clearance_m": 0.09,
    },
    # ⚠️ এখনো মাপা হয়নি -- ফিতা/স্কেল দিয়ে সরাসরি মেপে বসান। WRO
    # নিয়ম মানা যাচাই (300x200x300mm) আর Phase 7 (parking) এ সঠিক
    # জ্যামিতির জন্য দরকার হবে। wall-following/corner-turn এর
    # front_trigger_m ইত্যাদি এখনই লাইভ টিউনিং দিয়ে empirically ঠিক
    # হচ্ছে বলে এই মাপ না থাকলেও সেগুলো কাজ করছে -- কিন্তু ভবিষ্যতে
    # লাগবে, তাই কাঠামোটা এখনই রাখা হলো।
    "robot_dimensions": {
        "length_m": None,             # সামনের বাম্পার থেকে পেছনের বাম্পার
        "width_m": None,
        "lidar_offset_front_m": None, # LIDAR এর কেন্দ্র থেকে robot এর সামনের প্রান্ত পর্যন্ত দূরত্ব
        "lidar_offset_rear_m": None,  # LIDAR এর কেন্দ্র থেকে robot এর পেছনের প্রান্ত পর্যন্ত দূরত্ব
    },
}


def load_calibration():
    """calibration.json থেকে সব ভ্যালু লোড করে, ডিফল্টের সাথে মিলিয়ে
    (ফাইলে যা নেই সেটার জন্য ডিফল্ট ব্যবহার হবে)।

    ⚠️ এক-স্তর deep merge (শুধু dict.update() না): "open_challenge"-এর
    মতো nested dict key-গুলোর জন্য শুধু top-level replace করলে সমস্যা
    হতো -- calibration.json-এ যদি "open_challenge" আগে থেকেই সেভ করা
    থাকে (GUI থেকে SAVE VALUES চাপার পর), আর পরে DEFAULTS-এ নতুন কোনো
    sub-key (যেমন round_timeout_seconds) যোগ করা হয়, shallow merge এ
    পুরনো "open_challenge" dict-টা DEFAULTS-এর নতুন key সহ পুরো dict-কে
    প্রতিস্থাপন করে ফেলত -- নতুন key কখনোই পাওয়া যেত না (KeyError)।
    তাই nested dict এর ক্ষেত্রে key-by-key মিশিয়ে (deep merge) merge
    করা হচ্ছে, শুধু top-level primitive value replace করা হচ্ছে সরাসরি।
    """
    if not os.path.exists(CALIBRATION_FILE):
        return dict(DEFAULTS)
    with open(CALIBRATION_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)
    merged = dict(DEFAULTS)
    for key, value in data.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            nested = dict(merged[key])
            nested.update(value)
            merged[key] = nested
        else:
            merged[key] = value
    return merged


def save_calibration(updates: dict):
    """updates dict এর key-value গুলো বর্তমান calibration.json এর
    সাথে merge করে সেভ করে (পুরনো ফাইলটা পুরোপুরি মুছে দেয় না,
    শুধু নতুন key গুলো যোগ/আপডেট করে)। load_calibration()-এর মতোই
    nested dict (যেমন "open_challenge") এর জন্য deep merge -- partial
    update (শুধু কয়েকটা sub-key) দিলেও বাকি sub-key গুলো মুছে যাবে না।
    """
    os.makedirs(CONFIG_DIR, exist_ok=True)
    current = load_calibration()
    for key, value in updates.items():
        if isinstance(value, dict) and isinstance(current.get(key), dict):
            nested = dict(current[key])
            nested.update(value)
            current[key] = nested
        else:
            current[key] = value
    with open(CALIBRATION_FILE, "w", encoding="utf-8") as f:
        json.dump(current, f, indent=2, ensure_ascii=False)
    return current
