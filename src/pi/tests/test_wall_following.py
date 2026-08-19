"""
============================================================
TEST 13: Wall-following লাইভ টেস্ট (প্রথম autonomous driving টেস্ট!)
============================================================
⚠️⚠️⚠️ নিরাপত্তা -- এবার robot সত্যিই নিজে থেকে চলবে ⚠️⚠️⚠️
1. একটা "করিডোর" বানান -- দুই সারি বই/বাক্স দিয়ে, প্রায় ৬০-১০০ সেমি
   চওড়া, কমপক্ষে ২-৩ মিটার লম্বা। মেঝেতে যথেষ্ট খোলা জায়গা থাকা
   দরকার (সিঁড়ির কাছে/উঁচু জায়গায় টেস্ট করবেন না)।
2. প্রথমবার চালানোর সময় robot-এর পাশে দাঁড়িয়ে থাকুন, দরকার হলে
   হাতে ধরে থামানোর জন্য প্রস্তুত থাকুন।
3. স্পিড ইচ্ছাকৃতভাবে কম (নিচে TEST_SPEED) রাখা হয়েছে -- ভালোভাবে
   কাজ করলে পরে বাড়ানো যাবে।
4. স্বয়ংক্রিয় সময়সীমা (MAX_RUN_SECONDS) আছে, তারপর নিজে থেকেই
   থেমে যাবে -- তারপরও Ctrl+C সবসময় কাজ করবে।

উদ্দেশ্য: WallFollower ক্লাস ব্যবহার করে robot করিডোরের মাঝ বরাবর
চলছে কিনা, দেয়ালের দিকে হেলে গেলে নিজে থেকে সংশোধন করছে কিনা
যাচাই করা। এটা Open Challenge-এর মূল ভিত্তি (এখনো lap counting/
corner turning নেই, শুধু straight wall-following)।

⚠️ এই ফাইলটা এখন কার্যত `test_open_challenge_navigation.py` দিয়ে
superseded (ওটাতে এই একই wall-following + corner-turn দুটোই আছে)।
এটা রাখা হয়েছে শুধু corner-turn ছাড়া, বিচ্ছিন্নভাবে শুধু
wall-following-টুকু টেস্ট করতে চাইলে (একদম সোজা করিডোরে PID
আচরণ যাচাই করা সহজ হয় corner-turn এর জটিলতা ছাড়া)।

চালানোর নিয়ম:
    python3 test_wall_following.py
থামাতে: Ctrl+C (বা নিজে থেকেই MAX_RUN_SECONDS পর থামবে)

⚠️ এখন calibration.json-এর "open_challenge" থেকে সব মান পড়ে --
tune_open_challenge_live.py দিয়ে টিউন করা মান এখানেও ব্যবহার হবে।
============================================================
"""

import sys
import os
import time

from rplidar import RPLidar, RPLidarException
from serial import SerialException

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import load_calibration
from serial_link import SerialLink
from control.wall_following import WallFollower

LIDAR_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_19ad04198e596140a9e96e563d38c7ae-if00-port0"
LIDAR_BAUDRATE = 256000
ESP32_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

MAX_RUN_SECONDS = 20      # স্বয়ংক্রিয়ভাবে থেমে যাওয়ার সময়সীমা


def main():
    cal = load_calibration()
    oc = cal["open_challenge"]

    print("Robot serial link এর সাথে কানেক্ট হচ্ছে...")
    link = SerialLink(ESP32_PORT)
    time.sleep(1)

    print("LIDAR এর সাথে কানেক্ট হচ্ছে...")
    lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
    lidar.get_info()
    try:
        lidar.clean_input()
    except AttributeError:
        lidar._serial_port.reset_input_buffer()
    time.sleep(2)

    follower = WallFollower(
        servo_center=cal["servo_center"],
        servo_left_max=cal["servo_left_max"],
        servo_right_max=cal["servo_right_max"],
        kp=oc["kp"], ki=0.0, kd=oc["kd"],
        smoothing_alpha=oc["smoothing_alpha"],
        max_angle_change_per_sec=oc["max_angle_change_per_sec"],
        target_gap_m=oc["target_gap_m"],
        max_wall_distance_m=oc["max_wall_distance_m"],
        max_error_m=oc["max_error_m"],
    )

    print("\n" + "=" * 60)
    print("⚠️  ৩ সেকেন্ড পর robot চলা শুরু করবে -- সরে দাঁড়ান/প্রস্তুত থাকুন")
    print("=" * 60)
    time.sleep(3)

    start_time = time.time()
    scan_iterator = lidar.iter_scans()
    consecutive_bad = 0

    try:
        while time.time() - start_time < MAX_RUN_SECONDS:
            try:
                scan = next(scan_iterator)
            except RPLidarException:
                consecutive_bad += 1
                if consecutive_bad > 8:
                    print("!!! পরপর অনেক bad LIDAR frame -- নিরাপত্তার জন্য থামানো হচ্ছে।")
                    break
                scan_iterator = lidar.iter_scans()
                continue
            consecutive_bad = 0

            if not scan:
                continue

            angle = follower.compute_steering_angle(scan)
            link.send_command(oc["speed"], angle)

            telem = link.get_telemetry()
            print(f"servo_angle={angle:3d}  encoder={telem['encoder']:6d}  "
                  f"elapsed={time.time()-start_time:4.1f}s", end="\r")

        print("\n\nসময়সীমা শেষ, থামানো হচ্ছে।")

    except KeyboardInterrupt:
        print("\n\nইউজার থামিয়ে দিয়েছেন।")

    finally:
        link.send_command(0, cal["servo_center"])
        time.sleep(0.2)
        link.close()
        for step in (lidar.stop, lidar.stop_motor, lidar.disconnect):
            try:
                step()
            except Exception:
                pass
        print("Robot ও LIDAR নিরাপদে বন্ধ করা হয়েছে।")


if __name__ == "__main__":
    main()
