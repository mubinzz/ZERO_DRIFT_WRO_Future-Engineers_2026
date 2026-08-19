"""
============================================================
TEST 16: Obstacle Challenge navigation লাইভ টেস্ট (wall-following +
corner-turn + pillar avoidance)
============================================================
⚠️⚠️⚠️ নিরাপত্তা -- আগের মতোই ⚠️⚠️⚠️
robot-এর পাশে থাকুন, প্রয়োজনে হাতে ধরে থামানোর জন্য প্রস্তুত থাকুন।
স্বয়ংক্রিয় সময়সীমা (MAX_RUN_SECONDS) আছে, Ctrl+C সবসময় কাজ করবে।

⚠️⚠️ চালানোর আগে অবশ্যই ক্যালিব্রেট করা থাকতে হবে ⚠️⚠️
  - open_challenge (speed/KP/KD/front_trigger_m ইত্যাদি) -- reuse হয়
  - camera_red_hsv/camera_green_hsv
  - camera_hfov_deg/camera_angle_offset_deg/camera_offset_m
    (calibrate_camera_hfov.py)
  - yaw sign convention (YAW_INCREASES_CLOCKWISE নিচে ঠিক আছে কিনা)

উদ্দেশ্য: ObstacleChallengeNavigator (wall-following + corner-turn +
pillar-avoidance) robot-কে করিডোরে সোজা রাখতে, কোণে ঠিকমতো ঘুরতে,
আর pillar-এর সঠিক পাশ (লাল=ডানে, সবুজ=বামে) দিয়ে পার হতে পারছে কিনা
যাচাই করা। lap counting/finish-stop এখনো নেই -- এটা শুধু navigation
যাচাই করার জন্য।

চালানোর নিয়ম:
    python3 test_obstacle_challenge_navigation.py
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
from camera_link import create_camera
from control.wall_following import WallFollower
from control.corner_turn import CornerTurnController
from control.pillar_avoider import PillarAvoider
from control.obstacle_challenge_navigator import ObstacleChallengeNavigator

LIDAR_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_19ad04198e596140a9e96e563d38c7ae-if00-port0"
LIDAR_BAUDRATE = 256000
ESP32_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

MAX_RUN_SECONDS = 40

# যাচাই করা হয়েছে: ডানে (clockwise) ঘোরালে yaw কমে
YAW_INCREASES_CLOCKWISE = False


def main():
    cal = load_calibration()
    oc = cal["open_challenge"]
    pa = cal["pillar_avoider"]

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

    print("ক্যামেরা চালু হচ্ছে...")
    picam2 = create_camera()

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
        turn_cooldown_seconds=oc["turn_cooldown_seconds"],
        min_open_side_m=oc["min_open_side_m"],
    )
    pillar_avoider = PillarAvoider(
        margin_m=pa["margin_m"],
        trigger_distance_m=pa["trigger_distance_m"],
        confirm_frames=pa["confirm_frames"],
        max_side_angle_deg=pa["max_side_angle_deg"],
        track_jump_limit_m=pa["track_jump_limit_m"],
        wall_clearance_m=pa["wall_clearance_m"],
    )
    navigator = ObstacleChallengeNavigator(wall_follower, corner_turn, pillar_avoider)

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

            frame = picam2.capture_array()

            telem = link.get_telemetry()
            raw_yaw = telem["yaw"]
            yaw = raw_yaw if YAW_INCREASES_CLOCKWISE else -raw_yaw

            angle = navigator.compute_steering_angle(scan, frame, yaw, cal)
            link.send_command(oc["speed"], angle)

            if corner_turn.is_turning():
                state = "TURN  "
            elif pillar_avoider.is_active():
                state = f"PILLAR({pillar_avoider.last_pillar_color})"
            else:
                state = "WALL  "
            print(f"{state} servo={angle:3d}  yaw={yaw:7.1f}  "
                  f"elapsed={time.time()-start_time:4.1f}s", end="\r")

        print("\n\nসময়সীমা শেষ, থামানো হচ্ছে।")

    except KeyboardInterrupt:
        print("\n\nইউজার থামিয়ে দিয়েছেন।")

    finally:
        link.send_command(0, cal["servo_center"])
        time.sleep(0.2)
        link.close()
        picam2.stop()
        for step in (lidar.stop, lidar.stop_motor, lidar.disconnect):
            try:
                step()
            except Exception:
                pass
        print("Robot, ক্যামেরা ও LIDAR নিরাপদে বন্ধ করা হয়েছে।")


if __name__ == "__main__":
    main()
