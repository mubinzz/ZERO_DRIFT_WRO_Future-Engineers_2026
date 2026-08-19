"""
============================================================
camera_link.py — Picamera2 কানেক্ট করার shared helper
============================================================
robot-এ camera উল্টো (১৮০°) করে মাউন্ট করা আছে -- ছবি স্বাভাবিকভাবে
capture করলে upside-down আসে। এখানে libcamera-র Transform দিয়ে
driver/hardware লেভেলেই এটা ঠিক করে দেওয়া হয় (প্রতিটা frame আলাদাভাবে
cv2 দিয়ে ঘোরানোর বদলে), যাতে যেকোনো script এই একটা জায়গা থেকে camera
খুললেই সবসময় সোজা ছবি পায় -- প্রতিটা script-এ আলাদা করে ঘোরানোর কোড
লেখা/মনে রাখার দরকার নেই।

⚠️ camera সোজা করে বসালে (বা ভবিষ্যতে দিক বদলালে) নিচের
CAMERA_UPSIDE_DOWN বদলে দিন -- এই একটা জায়গা বদলালেই সব script-এ
প্রভাব পড়বে।

⚠️ ১৮০° ঘোরানো মানে hflip + vflip দুটোই -- শুধু ছবিটাকে "সোজা" করাই
যথেষ্ট না, upside-down camera-তে left/right ও উল্টে থাকে (ছবির উপর
থেকে নিচে উল্টে গেলে বাম-ডানও উল্টে যায়)। তাই দুটোই ঠিক করা হচ্ছে,
নাহলে pixel_to_robot_angle()-এর বাম/ডান হিসাব উল্টো হয়ে যেত।
============================================================
"""

import time

from libcamera import Transform
from picamera2 import Picamera2

CAMERA_UPSIDE_DOWN = True  # ⚠️ camera physically উল্টো করে মাউন্ট করা আছে


def create_camera(width=640, height=480, warmup_seconds=2.0):
    """Picamera2 কানেক্ট করে, প্রয়োজনে ১৮০° ঘুরিয়ে, চালু করে রিটার্ন
    করে। ব্যবহারের পর caller-কে picam2.stop() দিয়ে বন্ধ করতে হবে।"""
    picam2 = Picamera2()
    transform = Transform(hflip=1, vflip=1) if CAMERA_UPSIDE_DOWN else Transform()
    config = picam2.create_preview_configuration(
        main={"size": (width, height), "format": "RGB888"},
        transform=transform,
    )
    picam2.configure(config)
    picam2.start()
    if warmup_seconds > 0:
        time.sleep(warmup_seconds)
    return picam2
