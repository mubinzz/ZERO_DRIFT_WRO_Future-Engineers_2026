"""
============================================================
CALIBRATION 6: Encoder ticks -> আসল দূরত্ব (মিমি) ক্যালিব্রেশন
============================================================
উদ্দেশ্য: encoder এর একটা tick আসলে কত মিলিমিটার দূরত্বের সমান, সেটা
বের করা। এটা লাগবে Open Challenge-এর ফাইনাল রাউন্ডে "৩ lap শেষে
finish section-এ ঠিক জায়গায় থেমে যাওয়া" ফিচারের জন্য -- সময়/PWM
দিয়ে দূরত্ব আন্দাজ করলে speed একটু কমবেশি হলেই ভুল হয়ে যাবে, কিন্তু
encoder tick থেকে বের করা দূরত্ব motor speed-এর উপর নির্ভর করে না।

⚠️ motor দিয়ে না, হাতে ঠেলে মাপা হচ্ছে কেন: motor চালিয়ে একটা নির্দিষ্ট
বিন্দুতে থামানো নিজেই এই calibration-এর উপর নির্ভরশীল (chicken-and-egg
সমস্যা) -- হাতে টেপ মেপে ঠেলে দিলে ভেরিয়েবল একটাই থাকে (encoder ঠিকমতো
tick গুনছে কিনা), motor speed/deadzone এর কোনো প্রভাব থাকে না।

চালানোর নিয়ম:
    python3 calibrate_encoder_distance.py

কী লাগবে: একটা মাপার টেপ/স্কেল আর মেঝেতে একটা সরলরেখা (যেমন মেঝের
টাইলের সীমানা বা টেপ দিয়ে টানা দাগ)।
============================================================
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from serial_link import SerialLink
from config import save_calibration

ESP32_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"


def main():
    print(f"ESP32 এর সাথে কানেক্ট হচ্ছে: {ESP32_PORT} ...")
    link = SerialLink(ESP32_PORT)

    print("\n" + "=" * 60)
    print("ধাপ ১: robot কে মেঝেতে একটা শুরুর দাগের ঠিক পেছনের চাকা")
    print("       বরাবর রাখুন (সোজা সামনের দিকে মুখ করে)।")
    print("=" * 60)
    input("প্রস্তুত হলে Enter চাপুন... ")
    start_encoder = link.get_telemetry()["encoder"]
    print(f"শুরুর encoder count: {start_encoder}")

    print("\n" + "=" * 60)
    print("ধাপ ২: robot কে হাতে ধরে সোজা সামনের দিকে ঠেলে নিয়ে যান")
    print("       (মোটর চালাবেন না) -- যত দূর ইচ্ছা, কম করে ১ মিটার")
    print("       হলে ভালো (দূরত্ব বেশি হলে হিসাব বেশি নির্ভুল হবে)।")
    print("       তারপর টেপ দিয়ে ঠিক কতদূর সরেছে সেটা মেপে রাখুন।")
    print("=" * 60)
    input("ঠেলা শেষ হলে Enter চাপুন... ")
    end_encoder = link.get_telemetry()["encoder"]
    print(f"শেষের encoder count: {end_encoder}")

    delta_ticks = abs(end_encoder - start_encoder)
    print(f"\nমোট {delta_ticks} টা encoder tick গোনা হয়েছে।")

    if delta_ticks == 0:
        print("!!! কোনো tick গোনা যায়নি -- encoder ঠিকমতো কানেক্ট আছে কিনা,")
        print("!!! বা robot আসলেই সরানো হয়েছিল কিনা যাচাই করুন। বাতিল করা হচ্ছে।")
        link.close()
        return

    try:
        distance_mm = float(input("\nআসলে কত মিলিমিটার সরেছে (যেমন 1000 মানে ১ মিটার): ").strip())
    except ValueError:
        print("সংখ্যা দিতে হবে। বাতিল করা হচ্ছে।")
        link.close()
        return

    if distance_mm <= 0:
        print("দূরত্ব ধনাত্মক হতে হবে। বাতিল করা হচ্ছে।")
        link.close()
        return

    mm_per_tick = distance_mm / delta_ticks
    print("\n" + "=" * 60)
    print(f"ফলাফল: প্রতি ১ tick = {mm_per_tick:.4f} মিমি")
    print("=" * 60)

    answer = input("\nএই ভ্যালু সেভ করবেন? (y/n): ").strip().lower()
    if answer == "y":
        updated = save_calibration({"encoder_mm_per_tick": mm_per_tick})
        print(f"সেভ হয়েছে: encoder_mm_per_tick = {mm_per_tick:.4f}")
        print(f"পুরো calibration.json: {updated}")

    link.close()


if __name__ == "__main__":
    main()
