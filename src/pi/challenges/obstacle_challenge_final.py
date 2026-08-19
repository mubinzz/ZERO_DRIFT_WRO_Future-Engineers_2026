"""
============================================================
obstacle_challenge_final.py — Obstacle Challenge-এর পূর্ণাঙ্গ
competition run (parking ছাড়া)
============================================================
open_challenge_final.py-এর সাথে প্রায় হুবহু একই কাঠামো (start button,
lap counting, finish-section stop) -- শুধু navigation-এ
ObstacleChallengeNavigator ব্যবহার হয়, যেটা wall-following/corner-turn
এর পাশাপাশি pillar_avoider.py দিয়ে pillar-এর সঠিক পাশ (লাল=ডানে,
সবুজ=বামে, rule 9.19) দিয়েও পার হয়।

⚠️ এই সংস্করণে parking (rule 8, ৩ lap শেষে parking lot খোঁজা) এখনো
নেই -- সময়ের অভাবে এখন শুধু ৩ lap সঠিকভাবে (pillar পাশ মেনে) সম্পন্ন
করে finish section-এ থামার উপর ফোকাস করা হয়েছে (rule 1.1+1.2+1.3+
1.6/1.7 পয়েন্ট, parking ছাড়াই)।

⚠️ finishing (১২তম corner-এর পর শেষ straight অংশ) চলাকালীন
pillar-avoidance বন্ধ থাকে (open_challenge_final.py-র মতোই শুধু
wall_follower সরাসরি ব্যবহার হয়) -- যদি finish section-এর ঠিক ওই
ছোট অংশে (finish_stop_distance_mm, ডিফল্ট ২৫০মিমি) কোনো pillar
থাকে, সেটা এড়ানো হবে না। বাস্তব round-এ finish section-এ সাধারণত
pillar থাকার কথা না (parking lot-এর জন্য pillar সরিয়ে নেওয়া হয়,
rule 8/Figure 8e), তাই এটা সাধারণত সমস্যা করার কথা না।

⚠️ চালানোর আগে অবশ্যই ক্যালিব্রেট করা থাকতে হবে:
  1. open_challenge (speed/KP/KD/front_trigger_m ইত্যাদি)
  2. pillar_avoider (margin_m/trigger_distance_m ইত্যাদি)
  3. camera_red_hsv/camera_green_hsv
  4. camera_hfov_deg/camera_angle_offset_deg/camera_offset_m
  5. encoder_mm_per_tick
  6. yaw sign convention (YAW_INCREASES_CLOCKWISE)
  7. test_serial_link.py দিয়ে BTN=1 বাটন চাপার মান কিনা যাচাই

চালানোর নিয়ম (প্রতিযোগিতার দিনে ঠিক এভাবেই):
    python3 obstacle_challenge_final.py
robot switched ON হওয়ার পর script কানেক্ট করবে, তারপর push button
চাপার জন্য অপেক্ষা করবে -- বাটন চাপলেই round শুরু হবে।
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

# যাচাই করা হয়েছে: ডানে (clockwise) ঘোরালে yaw কমে
YAW_INCREASES_CLOCKWISE = False

# ⚠️ test_serial_link.py দিয়ে যাচাই করুন বাটন চাপলে BTN কী মান দেয়
BUTTON_PRESSED_VALUE = 1

CORNERS_PER_LAP = 4
TOTAL_LAPS = 3
TOTAL_CORNERS = CORNERS_PER_LAP * TOTAL_LAPS  # 12


def wait_for_start_button(link):
    """robot switched-on অবস্থায় থেকে physical push button চাপার জন্য
    অপেক্ষা করে (rule 9.11) -- motor/servo এই সময় সম্পূর্ণ নিষ্ক্রিয়।
    বাটনের rising edge (আগে না-চাপা অবস্থা থেকে চাপা অবস্থায় যাওয়া)
    দেখে ট্রিগার হয়, যাতে বাটন ধরে রাখলে বারবার ট্রিগার না হয়।"""
    print("Push button চাপার জন্য অপেক্ষা করা হচ্ছে...")
    was_pressed = False
    while True:
        telem = link.get_telemetry()
        pressed = telem["button"] == BUTTON_PRESSED_VALUE
        if pressed and not was_pressed:
            return
        was_pressed = pressed
        time.sleep(0.02)


def main():
    cal = load_calibration()
    oc = cal["open_challenge"]
    pa = cal["pillar_avoider"]
    mm_per_tick = cal.get("encoder_mm_per_tick")

    if not mm_per_tick:
        print("!!! calibration.json এ encoder_mm_per_tick নেই বা 0/None।")
        print("!!! calibrate_encoder_distance.py চালিয়ে আগে মাপুন, নাহলে")
        print("!!! finish section এ থামার দূরত্ব হিসাব করা যাবে না।")
        return

    geo_ready = (cal.get("camera_hfov_deg") is not None
                 and cal.get("camera_angle_offset_deg") is not None
                 and cal.get("camera_offset_m", {}).get("x") is not None
                 and cal.get("camera_offset_m", {}).get("y") is not None)
    if not geo_ready:
        print("!!! camera_hfov_deg/camera_angle_offset_deg/camera_offset_m ক্যালিব্রেট করা নেই।")
        print("!!! calibrate_camera_hfov.py চালিয়ে আগে মাপুন।")
        return

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
        target_yaw_change_deg=oc["target_yaw_change_deg"],
        max_turn_seconds=oc["max_turn_seconds"],
        front_confirm_frames=oc["front_confirm_frames"],
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

    # round শুরুর আগে motor/servo নিষ্ক্রিয়, শুধু button-এর অপেক্ষা
    link.send_command(0, cal["servo_center"])
    wait_for_start_button(link)
    print("\nবাটন চাপা হয়েছে -- round শুরু!\n")

    start_time = time.time()
    corners_completed = 0
    finishing = False          # ১২তম corner শেষ হওয়ার পর True
    finish_start_encoder = None

    scan_iterator = lidar.iter_scans()
    consecutive_bad = 0

    try:
        while True:
            elapsed = time.time() - start_time
            if elapsed > oc["round_timeout_seconds"]:
                print("\n!!! round timeout (rule 9.2: সর্বোচ্চ ৩ মিনিট) -- থামানো হচ্ছে।")
                break

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

            if not finishing:
                was_turning = corner_turn.is_turning()
                angle = navigator.compute_steering_angle(scan, frame, yaw, cal)
                if was_turning and not corner_turn.is_turning():
                    corners_completed += 1
                    print(f"\n  Corner #{corners_completed} সম্পন্ন "
                          f"(lap {corners_completed // CORNERS_PER_LAP + 1})")
                    if corners_completed >= TOTAL_CORNERS:
                        # ৩ lap শেষ -- আর কোনো নতুন corner turn/pillar
                        # avoidance শুরু করা যাবে না (rule 9.24.3: finish
                        # section পার হয়ে পরের corner-এ গেলে বোনাস পয়েন্ট
                        # পাওয়া যাবে না)। এখন থেকে শুধু সোজা wall-following।
                        finishing = True
                        finish_start_encoder = telem["encoder"]
                        print("  ৩ lap সম্পন্ন! finish section-এ থামার জন্য এগোচ্ছে...")
            else:
                angle = wall_follower.compute_steering_angle(scan)
                traveled_mm = abs(telem["encoder"] - finish_start_encoder) * mm_per_tick
                if traveled_mm >= oc["finish_stop_distance_mm"]:
                    print(f"\n  finish section-এ পৌঁছানো হয়েছে "
                          f"({traveled_mm:.0f}mm এগিয়ে) -- থামছে।")
                    break

            link.send_command(oc["speed"], angle)

            if corner_turn.is_turning():
                state = "TURN  "
            elif finishing:
                state = "FINISH"
            elif pillar_avoider.is_active():
                state = f"PILLAR({pillar_avoider.last_pillar_color})"
            else:
                state = "WALL  "
            print(f"{state} corners={corners_completed:2d}/{TOTAL_CORNERS}  "
                  f"servo={angle:3d}  yaw={yaw:7.1f}  elapsed={elapsed:5.1f}s", end="\r")

    except KeyboardInterrupt:
        print("\n\nইউজার থামিয়ে দিয়েছেন।")

    finally:
        # সম্পূর্ণ থামা -- rule 9.24.2 নোট ২: থামার পর ১৫ সেকেন্ড আর
        # না নড়লে সেটাই autonomous stop হিসেবে গণ্য হবে, তাই এখানে
        # command পাঠানো বন্ধ করে দেওয়াই যথেষ্ট।
        link.send_command(0, cal["servo_center"])
        time.sleep(0.2)
        link.close()
        picam2.stop()
        for step in (lidar.stop, lidar.stop_motor, lidar.disconnect):
            try:
                step()
            except Exception:
                pass
        print("\nRobot, ক্যামেরা ও LIDAR নিরাপদে বন্ধ করা হয়েছে।")


if __name__ == "__main__":
    main()
