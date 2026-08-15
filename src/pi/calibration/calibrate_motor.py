"""
============================================================
CALIBRATION 2: মোটর deadzone + PWM-to-speed কার্ভ
============================================================
উদ্দেশ্য:
- "deadzone" বের করা: PWM কত হলে (0-255 স্কেলে) চাকা আসলে ঘুরতে
  শুরু করে (স্থির ঘর্ষণ/stall কাটিয়ে)। এর নিচের যেকোনো PWM ভ্যালু
  দিলে motor শুধু গরম হবে/গুনগুন করবে, চাকা ঘুরবে না -- তাই control
  কোডে speed ভ্যালুকে deadzone-এর নিচে না রাখাই ভালো।
- PWM বনাম আসল স্পিড (encoder pulses/sec) এর একটা টেবিল বানানো, যাতে
  পরে বোঝা যায় কোন PWM ভ্যালুতে robot মোটামুটি কত দ্রুত চলে।

⚠️⚠️ নিরাপত্তা (অবশ্যই মানুন): robot কে এমনভাবে রাখুন যেন চাকা মাটি
স্পর্শ না করে (স্ট্যান্ডে তুলে বা টেবিলের কিনারায় চাকা শূন্যে
ভাসিয়ে) -- এই টেস্টে motor স্বয়ংক্রিয়ভাবে ধাপে ধাপে speed বাড়াতে
থাকবে, robot নিয়ন্ত্রণহীনভাবে দৌড় দিতে পারে যদি চাকা মাটিতে থাকে।

চালানোর নিয়ম:
    python3 calibrate_motor.py
Enter চাপলে sweep শুরু হবে, প্রতিটা ধাপ শেষে ফলাফল দেখাবে, শেষে
deadzone সেভ করার জন্য জিজ্ঞেস করবে।
============================================================
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from serial_link import SerialLink
from config import save_calibration

ESP32_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

# ---- sweep কনফিগ ----
PWM_START = 30
PWM_END = 255
PWM_STEP = 15
HOLD_SECONDS = 1.0          # প্রতিটা PWM ধাপে কতক্ষণ ধরে রাখা হবে
COMMAND_RESEND_INTERVAL = 0.1  # watchdog satisfied রাখতে কত ঘনঘন কমান্ড পাঠানো হবে
MOVEMENT_THRESHOLD_PPS = 5  # এর বেশি pulses/sec হলে "নড়েছে" ধরা হবে


def hold_speed_and_measure(link, speed, seconds):
    """একটা নির্দিষ্ট speed-এ motor ধরে রেখে শুরু ও শেষের encoder
    count এর পার্থক্য থেকে pulses/sec হিসাব করে।"""
    start_encoder = link.get_telemetry()["encoder"]
    start_time = time.time()

    while time.time() - start_time < seconds:
        link.send_command(speed, 90)
        time.sleep(COMMAND_RESEND_INTERVAL)

    end_encoder = link.get_telemetry()["encoder"]
    elapsed = time.time() - start_time
    pulses = abs(end_encoder - start_encoder)
    pps = pulses / elapsed if elapsed > 0 else 0
    return pps


def main():
    print(f"ESP32 এর সাথে কানেক্ট হচ্ছে: {ESP32_PORT} ...")
    link = SerialLink(ESP32_PORT)
    time.sleep(1)

    print("\n" + "=" * 60)
    print("⚠️  নিশ্চিত করুন robot এর চাকা মাটি/টেবিল স্পর্শ করছে না!")
    print("=" * 60)
    ready = input("প্রস্তুত হলে Enter চাপুন (বাতিল করতে Ctrl+C)... ")

    results = []  # (pwm, pps) জোড়ার লিস্ট

    try:
        print(f"\nPWM sweep শুরু হচ্ছে: {PWM_START} থেকে {PWM_END}, ধাপ {PWM_STEP}\n")

        for pwm in range(PWM_START, PWM_END + 1, PWM_STEP):
            pps = hold_speed_and_measure(link, pwm, HOLD_SECONDS)
            moved = "✓ নড়েছে" if pps > MOVEMENT_THRESHOLD_PPS else "  (স্থির)"
            print(f"PWM={pwm:3d}  ->  {pps:6.1f} pulses/sec   {moved}")
            results.append((pwm, pps))

        # থামানো
        link.send_command(0, 90)

        # deadzone বের করা: প্রথম PWM যেখানে movement threshold পার হয়েছে
        deadzone = None
        for pwm, pps in results:
            if pps > MOVEMENT_THRESHOLD_PPS:
                deadzone = pwm
                break

        print("\n" + "=" * 60)
        if deadzone is not None:
            print(f"সনাক্ত হওয়া deadzone: PWM = {deadzone}")
            print("(এর মানে এই PWM এর নিচে দিলে চাকা ঘোরে না বলে ধরা হচ্ছে)")
        else:
            print("!!! কোনো PWM ভ্যালুতেই যথেষ্ট movement পাওয়া যায়নি।")
            print("!!! চাকা সত্যিই মাটি থেকে মুক্ত আছে কিনা, encoder ঠিকমতো")
            print("!!! কানেক্ট আছে কিনা আবার চেক করুন।")
        print("=" * 60)

        if deadzone is not None:
            answer = input("\nএই deadzone ভ্যালু সেভ করবেন? (y/n): ").strip().lower()
            if answer == "y":
                # একটু নিরাপত্তা মার্জিন যোগ করা হচ্ছে (সনাক্ত হওয়া
                # ভ্যালুর চেয়ে সামান্য বেশি রাখা ভালো, বর্ডারলাইন
                # ভ্যালুতে মাঝেমধ্যে নাও ঘুরতে পারে)
                safe_deadzone = min(255, deadzone + PWM_STEP // 2)
                updated = save_calibration({"motor_deadzone": safe_deadzone})
                print(f"সেভ হয়েছে (নিরাপত্তা মার্জিনসহ): motor_deadzone = {safe_deadzone}")
                print(f"পুরো calibration.json: {updated}")

    except KeyboardInterrupt:
        print("\nথামানো হচ্ছে...")
    finally:
        link.close()


if __name__ == "__main__":
    main()
