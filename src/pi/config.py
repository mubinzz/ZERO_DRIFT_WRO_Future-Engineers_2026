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
    "camera_red_hsv": {"lower": [0, 100, 100], "upper": [10, 255, 255]},
    "camera_green_hsv": {"lower": [40, 100, 100], "upper": [80, 255, 255]},
}


def load_calibration():
    """calibration.json থেকে সব ভ্যালু লোড করে, ডিফল্টের সাথে মিলিয়ে
    (ফাইলে যা নেই সেটার জন্য ডিফল্ট ব্যবহার হবে)।"""
    if not os.path.exists(CALIBRATION_FILE):
        return dict(DEFAULTS)
    with open(CALIBRATION_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)
    merged = dict(DEFAULTS)
    merged.update(data)
    return merged


def save_calibration(updates: dict):
    """updates dict এর key-value গুলো বর্তমান calibration.json এর
    সাথে merge করে সেভ করে (পুরনো ফাইলটা পুরোপুরি মুছে দেয় না,
    শুধু নতুন key গুলো যোগ/আপডেট করে)।"""
    os.makedirs(CONFIG_DIR, exist_ok=True)
    current = load_calibration()
    current.update(updates)
    with open(CALIBRATION_FILE, "w", encoding="utf-8") as f:
        json.dump(current, f, indent=2, ensure_ascii=False)
    return current
