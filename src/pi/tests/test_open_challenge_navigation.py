"""
============================================================
TEST 14: Open Challenge navigation লাইভ টেস্ট (wall-following + corner turn)
============================================================
⚠️⚠️⚠️ নিরাপত্তা -- আগের মতোই ⚠️⚠️⚠️
robot-এর পাশে থাকুন, প্রয়োজনে হাতে ধরে থামানোর জন্য প্রস্তুত থাকুন।
স্বয়ংক্রিয় সময়সীমা (MAX_RUN_SECONDS) আছে, Ctrl+C সবসময় কাজ করবে।

⚠️⚠️ এই টেস্ট চালানোর আগে yaw sign convention যাচাই করা আবশ্যক ⚠️⚠️
নিচের YAW_INCREASES_CLOCKWISE ভুল হলে corner turn সম্পূর্ণ বিপরীত
দিকে ঘুরবে, বা কখনো "শেষ" হবে না (yaw বাড়া/কমার ভুল দিকে অপেক্ষা
করতে থাকবে) -- ফলাফল: robot গোল গোল ঘুরতেই থাকবে বা ভুল কোণে যাবে।

যাচাই করার উপায়:
  1. python3 test_serial_link.py চালান
  2. robot-টা হাতে ধরে সাবধানে ৯০° ডানে (উপর থেকে দেখলে ঘড়ির
     কাঁটার দিকে) ঘোরান
  3. YAW ভ্যালু বাড়লো নাকি কমলো লক্ষ্য করুন
  4. বাড়লে YAW_INCREASES_CLOCKWISE = True রাখুন
     কমলে YAW_INCREASES_CLOCKWISE = False করুন

উদ্দেশ্য: OpenChallengeNavigator (wall-following + corner-turn একসাথে)
robot-কে করিডোরে সোজা রাখতে আর কোণে সঠিক দিকে ৯০° ঘুরতে পারছে কিনা
যাচাই করা। এখনো lap counting নেই -- এটা শুধু navigation যাচাই করার
জন্য, MAX_RUN_SECONDS পর নিজে থেকেই থেমে যাবে।

চালানোর নিয়ম:
    python3 test_open_challenge_navigation.py

⚠️ এই স্ক্রিপ্ট এখন KP/KD/FRONT_TRIGGER_M ইত্যাদি হার্ডকোড করে না --
calibration.json এর "open_challenge" থেকে লোড করে। মান বদলাতে হলে
calibration/tune_open_challenge_live.py দিয়ে লাইভ টিউন করে 's'
চেপে সেভ করুন, তারপর এই স্ক্রিপ্ট আবার চালান -- নতুন মান নিজে থেকেই
ব্যবহার হবে।
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
from control.corner_turn import CornerTurnController
from control.open_challenge_navigator import OpenChallengeNavigator

LIDAR_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_19ad04198e596140a9e96e563d38c7ae-if00-port0"
LIDAR_BAUDRATE = 256000
ESP32_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

MAX_RUN_SECONDS = 30

# যাচাই করা হয়েছে: ডানে (clockwise) ঘোরালে yaw কমে
YAW_INCREASES_CLOCKWISE = False


def main():
    cal = load_calibration()
    oc = cal["open_challenge"]  # tune_open_challenge_live.py দিয়ে টিউন করা মান

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

    wall_follower = WallFollower(
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
    corner_turn = CornerTurnController(
        servo_center=cal["servo_center"],
        servo_left_max=cal["servo_left_max"],
        servo_right_max=cal["servo_right_max"],
        front_trigger_m=oc["front_trigger_m"],
        front_confirm_frames=oc["front_confirm_frames"],
        target_yaw_change_deg=oc["target_yaw_change_deg"],
        max_turn_seconds=oc["max_turn_seconds"],
    )
    navigator = OpenChallengeNavigator(wall_follower, corner_turn)

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
                    print("\n!!! পরপর অনেক bad LIDAR frame -- নিরাপত্তার জন্য থামানো হচ্ছে।")
                    break
                scan_iterator = lidar.iter_scans()
                continue
            consecutive_bad = 0

            if not scan:
                continue

            telem = link.get_telemetry()
            raw_yaw = telem["yaw"]
            yaw = raw_yaw if YAW_INCREASES_CLOCKWISE else -raw_yaw

            angle = navigator.compute_steering_angle(scan, yaw)
            link.send_command(oc["speed"], angle)

            state = "TURN " if corner_turn.is_turning() else "WALL "
            print(f"{state} servo={angle:3d}  yaw={yaw:7.1f}  "
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
