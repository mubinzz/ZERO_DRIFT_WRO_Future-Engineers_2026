"""
============================================================
CALIBRATION 1: সার্ভো সেন্টার ও নিরাপদ লিমিট
============================================================
উদ্দেশ্য:
- "সোজা" (center) angle ঠিক কত, সেটা বের করা (মেকানিক্যাল লিংকেজের
  কারণে সবসময় 90 না-ও হতে পারে)
- বাম আর ডান দিকে যান্ত্রিকভাবে নিরাপদে (চাকা/লিংকেজ কোথাও না
  আটকে) সর্বোচ্চ কত angle পর্যন্ত ঘোরানো যায়, সেটা বের করা
এই তিনটা ভ্যালু config/calibration.json এ সেভ হবে, পরে Ackermann
স্টিয়ারিং কন্ট্রোলে (PID) এই রেঞ্জের বাইরে কখনো কমান্ড পাঠানো হবে
না -- নাহলে সার্ভো/লিংকেজ ভেঙে যেতে পারে।

⚠️ নিরাপত্তা: robot হাতে ধরে রাখুন বা এমন জায়গায় রাখুন যাতে
স্টিয়ারিং ঘুরানোর সময় লিংকেজ কোথাও জোরে আটকে যাওয়ার আগেই আপনি
থামাতে পারেন।

চালানোর নিয়ম:
    python3 calibrate_servo.py

কমান্ড:
    <সংখ্যা 0-180>  -> সার্ভোকে সেই angle এ নিয়ে যাবে (চোখে দেখে
                        যাচাই করুন চাকা কতটা ঘুরলো, লিংকেজ আটকাচ্ছে
                        কিনা)
    c               -> বর্তমান angle কে "center/সোজা" হিসেবে চিহ্নিত
    l               -> বর্তমান angle কে "left max" (নিরাপদ সর্বোচ্চ
                        বাম) হিসেবে চিহ্নিত
    r               -> বর্তমান angle কে "right max" হিসেবে চিহ্নিত
    s               -> এখন পর্যন্ত চিহ্নিত সবকিছু ফাইলে সেভ করে বন্ধ
    q               -> সেভ না করে বন্ধ

পরামর্শ: প্রথমে ১ ডিগ্রি করে না বদলে বড় ধাপে (৯০, ৭০, ৫০...)
মোটামুটি রেঞ্জ বের করুন, তারপর কাছাকাছি এসে ছোট ধাপে (২-৩ ডিগ্রি)
সূক্ষ্মভাবে ঠিক করুন।
============================================================
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from serial_link import SerialLink
from config import load_calibration, save_calibration

# ---- CONFIG (আপনার actual পোর্ট, `ls -l /dev/serial/by-id/` দিয়ে যাচাই করুন) ----
ESP32_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"


def main():
    print(f"ESP32 এর সাথে কানেক্ট হচ্ছে: {ESP32_PORT} ...")
    link = SerialLink(ESP32_PORT)
    time.sleep(1)

    cal = load_calibration()
    current_angle = cal["servo_center"]
    link.send_command(0, current_angle)  # speed সবসময় 0 -- এই ক্যালিব্রেশনে motor লাগবে না

    found = {}

    print("\nনির্দেশনা:")
    print("  <0-180>  -> সার্ভো ঐ angle এ যাবে")
    print("  c        -> বর্তমান angle = center (সোজা)")
    print("  l        -> বর্তমান angle = left max (নিরাপদ সর্বোচ্চ বাম)")
    print("  r        -> বর্তমান angle = right max (নিরাপদ সর্বোচ্চ ডান)")
    print("  s        -> সেভ করে বন্ধ")
    print("  q        -> সেভ না করে বন্ধ\n")

    try:
        while True:
            print(f"বর্তমান angle: {current_angle}  |  এখন পর্যন্ত চিহ্নিত: {found}")
            cmd = input("> ").strip().lower()

            if cmd == "c":
                found["servo_center"] = current_angle
                print(f"  -> center = {current_angle}")
            elif cmd == "l":
                found["servo_left_max"] = current_angle
                print(f"  -> left max = {current_angle}")
            elif cmd == "r":
                found["servo_right_max"] = current_angle
                print(f"  -> right max = {current_angle}")
            elif cmd == "s":
                if "servo_center" not in found:
                    print("এখনো center চিহ্নিত করেননি! অন্তত 'c' একবার দিন।")
                    continue
                updated = save_calibration(found)
                print(f"\nসেভ হয়েছে: {updated}")
                break
            elif cmd == "q":
                print("সেভ না করেই বন্ধ করা হচ্ছে।")
                break
            else:
                try:
                    angle = max(0, min(180, int(cmd)))
                    current_angle = angle
                    link.send_command(0, current_angle)
                except ValueError:
                    print("ভুল ইনপুট। সংখ্যা (0-180) অথবা c/l/r/s/q দিন।")

    except KeyboardInterrupt:
        print("\nথামানো হচ্ছে...")
    finally:
        link.close()  # এটা নিজে থেকেই servo কে center এ ফেরত পাঠায় (serial_link.py দেখুন)


if __name__ == "__main__":
    main()
