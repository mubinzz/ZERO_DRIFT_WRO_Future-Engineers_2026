"""
============================================================
CALIBRATION 7: Camera HFOV + camera-LIDAR অফসেট (KMIDS-স্টাইল)
============================================================
⚠️ এটা আগের calibrate_camera_fov.py-কে প্রতিস্থাপন করেছে। সেই
পদ্ধতিতে (LIDAR-কে রেফারেন্স ধরে pillar-এর angle মাপা) ছোট রুমে
বারবার ভুল বস্তু ধরা পড়ছিল। KMIDS repo (raspberry-pi-5/src/processors/
camera/camera_processor.cpp) বিশ্লেষণ করে দেখা গেছে তারা LIDAR-ভিত্তিক
co-calibration করেই না -- বরং:
  1. pixel-থেকে-angle একটা সরল রৈখিক সূত্র (pixelToAngle) দিয়ে,
     HFOV একটা জানা/ধরে-নেওয়া মান হিসেবে
  2. camera আর LIDAR-এর ভৌত অবস্থান আলাদা জেনে (ruler দিয়ে মাপা
     অফসেট), fusion-এর সময় ray-casting দিয়ে parallax ঠিক করে

এই script সেই একই পদ্ধতি অনুসরণ করে, কিন্তু "HFOV এমনি ধরে নেওয়া"-র
বদলে বাস্তব ডেটা দিয়ে fit করে (datasheet-এর সংখ্যা অন্ধভাবে বিশ্বাস
না করে) -- pillar-কে **ruler দিয়ে জ্যামিতিকভাবে মাপা নির্দিষ্ট
কোণে** রেখে (LIDAR দিয়ে "অনুমান" করে না), তাই ঘরে অন্য কোনো বস্তু
থাকলেও কোনো সমস্যা হয় না -- LIDAR এখানে ব্যবহারই হচ্ছে না।

কৌশল: robot-এর সামনে D মিটার দূরে, S মিটার পাশে (ডানে ধনাত্মক) একটা
লাল/সবুজ pillar রাখুন (টেপ মাপ দিয়ে মেপে) -- প্রকৃত angle তখন
atan2(S, D)। কয়েকটা ভিন্ন ভিন্ন (D, S) এ রেখে প্রতিবার ছবি তুলে
pillar-এর pixel x-position বের করা হয়। তারপর pixel (normalize করা)
বনাম angle -- এই জোড়াগুলো দিয়ে HFOV আর angle_offset (camera মাউন্ট
করার সময় সামান্য বাঁকা থাকলে সেটাও ধরার জন্য) fit করা হয়।

⚠️ চালানোর আগে camera_red_hsv/camera_green_hsv (tune_hsv_live.py)
আগে থেকেই ক্যালিব্রেট করা থাকতে হবে।

চালানোর নিয়ম:
    python3 calibrate_camera_hfov.py
============================================================
"""

import sys
import os
import math

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import load_calibration, save_calibration
from camera_link import create_camera
from perception.camera_processor import detect_blocks

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

TARGET_SAMPLES = 6


def list_camera_candidates(picam2, calibration):
    """একটা ছবিতে detect হওয়া সব লাল/সবুজ ব্লকের (color, area, center_x)
    লিস্ট রিটার্ন করে, area অনুযায়ী বড় থেকে ছোট সাজানো।"""
    frame = picam2.capture_array()
    blocks = detect_blocks(frame, calibration)
    all_blocks = blocks["red"] + blocks["green"]
    all_blocks.sort(key=lambda b: b.area, reverse=True)
    return [(b.color, b.area, b.center_x) for b in all_blocks]


def read_float(prompt):
    while True:
        try:
            return float(input(prompt).strip())
        except ValueError:
            print("সংখ্যা দিতে হবে, আবার চেষ্টা করুন।")


def main():
    cal = load_calibration()

    print("\n" + "=" * 60)
    print("ধাপ ১: camera-LIDAR অফসেট (ruler দিয়ে মেপে বসান)")
    print("=" * 60)
    print("LIDAR-এর কেন্দ্র থেকে camera-র lens ঠিক কতটা সামনে/পাশে")
    print("বসানো সেটা মেপে বলুন (মিটারে, যেমন 5cm = 0.05)।")
    offset_x = read_float("  ডানে/বামে অফসেট (ধনাত্মক=ডানে, বামে হলে ঋণাত্মক): ")
    offset_y = read_float("  সামনে/পেছনে অফসেট (ধনাত্মক=সামনে, পেছনে হলে ঋণাত্মক): ")

    print("\nক্যামেরা চালু হচ্ছে...")
    picam2 = create_camera(CAMERA_WIDTH, CAMERA_HEIGHT)

    print("\n" + "=" * 60)
    print("ধাপ ২: HFOV ক্যালিব্রেশন (LIDAR লাগবে না)")
    print("একটা লাল/সবুজ pillar আর একটা টেপ মাপ ব্যবহার করুন।")
    print("প্রতিটা sample এ pillar-কে robot-এর সামনে একটা মাপা")
    print("দূরত্বে (D, সামনে) আর মাপা পাশ-অফসেটে (S, ডানে ধনাত্মক)")
    print("রাখুন -- যত বেশি বিভিন্ন কোণে (কিছু বামে, কিছু ডানে,")
    print("কিছু মাঝে) ততই ভালো fit পাওয়া যাবে।")
    print("=" * 60)

    samples = []  # (normalized_px, true_angle_deg)

    try:
        while len(samples) < TARGET_SAMPLES:
            print(f"\n[{len(samples) + 1}/{TARGET_SAMPLES}]")
            d = read_float("  pillar robot-এর সামনে কত মিটার দূরে (D): ")
            s = read_float("  pillar কেন্দ্র থেকে কত মিটার পাশে (ডানে ধনাত্মক, S): ")
            true_angle = math.degrees(math.atan2(s, d))
            print(f"    হিসাব করা প্রকৃত angle = {true_angle:+.1f}°")

            cam_candidates = list_camera_candidates(picam2, cal)
            if not cam_candidates:
                print("!!! ক্যামেরা কোনো লাল/সবুজ ব্লক খুঁজে পায়নি "
                      "(HSV calibration ঠিক আছে তো?) -- আবার চেষ্টা করুন।")
                continue

            print(f"    ক্যামেরায় {len(cam_candidates)}টা লাল/সবুজ ব্লক পাওয়া গেছে:")
            for i, (cand_color, cand_area, cand_px) in enumerate(cam_candidates):
                print(f"      [{i}] রঙ={cand_color}  area={cand_area:.0f}  pixel_x={cand_px}")
            choice = input("    কোনটা আসল pillar? নম্বর লিখুন (কোনোটাই না হলে খালি রেখে Enter): ").strip()
            if not choice.isdigit() or int(choice) >= len(cam_candidates):
                print("    বাতিল -- আবার চেষ্টা করুন।")
                continue

            _, _, pixel_x = cam_candidates[int(choice)]
            normalized_px = (pixel_x / (CAMERA_WIDTH - 1)) - 0.5
            print(f"    বেছে নেওয়া হলো: pixel_x = {pixel_x}")
            samples.append((normalized_px, true_angle))

        # angle = normalized_px * hfov + angle_offset -- normalized_px এর
        # সাপেক্ষে angle এর linear সমীকরণ, তাই np.polyfit(degree=1) দিয়ে
        # fit করা যায় (slope=hfov, intercept=angle_offset)
        norm_pxs = np.array([n for n, _ in samples])
        angles = np.array([a for _, a in samples])
        hfov, angle_offset = np.polyfit(norm_pxs, angles, 1)

        print("\n" + "=" * 60)
        print(f"ফলাফল: HFOV = {hfov:.1f}°   angle_offset = {angle_offset:+.1f}°")
        print("=" * 60)

        print("\nযাচাই (fit করা model দিয়ে angle আবার হিসাব করে হাতে-মাপা")
        print("angle এর সাথে তুলনা -- পার্থক্য কয়েক ডিগ্রির বেশি হলে সন্দেহজনক):")
        for normalized_px, true_angle in samples:
            predicted = normalized_px * hfov + angle_offset
            print(f"  normalized_px={normalized_px:+.3f}  মাপা={true_angle:+6.1f}°  "
                  f"fit={predicted:+6.1f}°  পার্থক্য={abs(predicted - true_angle):.1f}°")

        answer = input("\nএই ভ্যালু সেভ করবেন? (y/n): ").strip().lower()
        if answer == "y":
            updated = save_calibration({
                "camera_hfov_deg": hfov,
                "camera_angle_offset_deg": angle_offset,
                "camera_offset_m": {"x": offset_x, "y": offset_y},
            })
            print(f"সেভ হয়েছে: camera_hfov_deg={updated['camera_hfov_deg']}, "
                  f"camera_angle_offset_deg={updated['camera_angle_offset_deg']}, "
                  f"camera_offset_m={updated['camera_offset_m']}")

    except KeyboardInterrupt:
        print("\nথামানো হচ্ছে...")

    finally:
        picam2.stop()
        print("Camera নিরাপদে বন্ধ করা হয়েছে।")


if __name__ == "__main__":
    main()
