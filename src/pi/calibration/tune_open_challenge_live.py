"""
============================================================
CALIBRATION 4: Open Challenge লাইভ PID/parameter টিউনিং UI
============================================================
robot বাস্তবিকভাবে চলতে থাকা অবস্থায় trackbar দিয়ে PID gain, wall
distance, corner-trigger ইত্যাদি প্যারামিটার তাৎক্ষণিকভাবে বদলানো
যাবে -- ফাইল এডিট করে scp করে আবার চালানোর ধীর চক্র লাগবে না।

⚠️⚠️⚠️ নিরাপত্তা -- robot সত্যিই চলবে ⚠️⚠️⚠️
robot-এর পাশে থাকুন, প্রয়োজনে হাতে ধরে থামাতে প্রস্তুত থাকুন।

⚠️ চালানোর আগে yaw sign convention যাচাই করুন (test_serial_link.py
দিয়ে robot হাতে ৯০° ডানে ঘুরিয়ে YAW বাড়ে না কমে দেখুন), নিচে
YAW_INCREASES_CLOCKWISE ঠিক করে নিন।

ব্যবহার:
    python3 tune_open_challenge_live.py

Trackbar (একটা window-এ, VNC/GUI দরকার):
    Speed                -> মোটর স্পিড (0-255)
    KP x100               -> proportional gain (আসল KP = ভ্যালু/100)
    KD x100               -> derivative gain (আসল KD = ভ্যালু/100)
    Smoothing x100        -> error smoothing (আসল alpha = ভ্যালু/100)
    MaxAngleStep          -> প্রতি loop এ servo সর্বোচ্চ কত ডিগ্রি বদলাবে
    FrontTriggerCm        -> কত সেমি সামনে দেয়াল দেখলে turn শুরু হবে
    TargetYawDeg          -> turn সম্পন্ন ধরার জন্য yaw কত ডিগ্রি বদলাতে হবে

কীবোর্ড কমান্ড (window সিলেক্ট রেখে):
    s -> বর্তমান trackbar ভ্যালু calibration.json এর "open_challenge"
         এ সেভ করুন (পরে test_open_challenge_navigation.py এই
         ভ্যালুই ব্যবহার করবে)
    q -> থামিয়ে বন্ধ করুন (motor/servo নিরাপদে বন্ধ হবে)

টিউনিং কৌশল:
- robot চলতে চলতেই স্লাইডার নাড়ান, সাথে সাথে আচরণ বদলাবে
- দুলছে/wobble করছে -> KD কমান বা KP কমান, অথবা Smoothing কমান
  (মনে রাখবেন Smoothing trackbar-এ ছোট ভ্যালু = বেশি smoothing)
- দেয়ালের দিকে হেলে গিয়েও সংশোধন দেরিতে হচ্ছে -> KP বাড়ান
- turn দেরিতে শুরু হয়ে দেয়ালে লাগছে -> FrontTriggerCm বাড়ান
- turn অতিরিক্ত/কম ঘুরছে -> TargetYawDeg সামঞ্জস্য করুন
- ভালো লাগলেই সাথে সাথে 's' চাপুন -- ভুলে গেলে বন্ধ হওয়ার পর হারিয়ে যাবে
============================================================
"""

import sys
import os
import time
import cv2
import numpy as np

from rplidar import RPLidar, RPLidarException
from serial import SerialException

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import load_calibration, save_calibration
from serial_link import SerialLink
from control.wall_following import WallFollower
from control.corner_turn import CornerTurnController
from control.open_challenge_navigator import OpenChallengeNavigator

LIDAR_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_19ad04198e596140a9e96e563d38c7ae-if00-port0"
LIDAR_BAUDRATE = 256000
ESP32_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

# ⚠️⚠️ test_serial_link.py দিয়ে যাচাই করে ঠিক মান বসান ⚠️⚠️
YAW_INCREASES_CLOCKWISE = False  # যাচাই করা হয়েছে: ডানে (clockwise) ঘোরালে yaw কমে

WINDOW = "Open Challenge Tuning (s=save, q=quit)"


def nothing(x):
    pass


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

    wall_follower = WallFollower(
        servo_center=cal["servo_center"],
        servo_left_max=cal["servo_left_max"],
        servo_right_max=cal["servo_right_max"],
    )
    corner_turn = CornerTurnController(
        servo_center=cal["servo_center"],
        servo_left_max=cal["servo_left_max"],
        servo_right_max=cal["servo_right_max"],
    )
    navigator = OpenChallengeNavigator(wall_follower, corner_turn)

    cv2.namedWindow(WINDOW)
    cv2.createTrackbar("Speed", WINDOW, oc["speed"], 255, nothing)
    cv2.createTrackbar("KP x100", WINDOW, int(oc["kp"] * 100), 1000, nothing)
    cv2.createTrackbar("KD x100", WINDOW, int(oc["kd"] * 100), 1000, nothing)
    cv2.createTrackbar("Smoothing x100", WINDOW, int(oc["smoothing_alpha"] * 100), 100, nothing)
    cv2.createTrackbar("MaxAngleRate", WINDOW, oc["max_angle_change_per_sec"], 400, nothing)
    cv2.createTrackbar("FrontTriggerCm", WINDOW, int(oc["front_trigger_m"] * 100), 150, nothing)
    cv2.createTrackbar("FrontConfirmFrames", WINDOW, oc["front_confirm_frames"], 10, nothing)
    cv2.createTrackbar("TargetYawDeg", WINDOW, oc["target_yaw_change_deg"], 120, nothing)
    cv2.createTrackbar("TargetGapCm", WINDOW, int(oc["target_gap_m"] * 100), 100, nothing)
    cv2.createTrackbar("MaxWallDistCm", WINDOW, int(oc["max_wall_distance_m"] * 100), 300, nothing)
    cv2.createTrackbar("MaxErrorCm", WINDOW, int(oc["max_error_m"] * 100), 200, nothing)

    print("\n" + "=" * 60)
    print("⚠️  ৩ সেকেন্ড পর robot চলা শুরু করবে -- সরে দাঁড়ান/প্রস্তুত থাকুন")
    print("=" * 60)
    time.sleep(3)

    scan_iterator = lidar.iter_scans()
    consecutive_bad = 0

    try:
        while True:
            # ---- ট্র্যাকবার থেকে লাইভ ভ্যালু পড়া, প্যারামিটার আপডেট ----
            speed = cv2.getTrackbarPos("Speed", WINDOW)
            kp = cv2.getTrackbarPos("KP x100", WINDOW) / 100.0
            kd = cv2.getTrackbarPos("KD x100", WINDOW) / 100.0
            smoothing_alpha = cv2.getTrackbarPos("Smoothing x100", WINDOW) / 100.0
            max_angle_rate = max(1, cv2.getTrackbarPos("MaxAngleRate", WINDOW))
            front_trigger_m = cv2.getTrackbarPos("FrontTriggerCm", WINDOW) / 100.0
            front_confirm_frames = max(1, cv2.getTrackbarPos("FrontConfirmFrames", WINDOW))
            target_yaw_deg = cv2.getTrackbarPos("TargetYawDeg", WINDOW)
            target_gap_m = cv2.getTrackbarPos("TargetGapCm", WINDOW) / 100.0
            max_wall_distance_m = cv2.getTrackbarPos("MaxWallDistCm", WINDOW) / 100.0
            max_error_m = cv2.getTrackbarPos("MaxErrorCm", WINDOW) / 100.0

            wall_follower.pid.kp = kp
            wall_follower.pid.kd = kd
            wall_follower.smoothing_alpha = smoothing_alpha
            wall_follower.max_angle_change_per_sec = max_angle_rate
            wall_follower.target_gap_m = target_gap_m
            wall_follower.max_wall_distance_m = max_wall_distance_m
            wall_follower.max_error_m = max_error_m
            corner_turn.front_trigger_m = front_trigger_m
            corner_turn.front_confirm_frames = front_confirm_frames
            corner_turn.target_yaw_change_deg = target_yaw_deg
            corner_turn.max_turn_seconds = oc["max_turn_seconds"]

            # ---- LIDAR scan পড়া ----
            try:
                scan = next(scan_iterator)
            except RPLidarException:
                consecutive_bad += 1
                if consecutive_bad > 8:
                    print("\n!!! পরপর অনেক bad LIDAR frame -- থামানো হচ্ছে।")
                    break
                scan_iterator = lidar.iter_scans()
                continue
            consecutive_bad = 0

            if scan:
                telem = link.get_telemetry()
                raw_yaw = telem["yaw"]
                yaw = raw_yaw if YAW_INCREASES_CLOCKWISE else -raw_yaw

                angle = navigator.compute_steering_angle(scan, yaw)
                link.send_command(speed, angle)

                state = "TURN" if corner_turn.is_turning() else "WALL"

                # ---- স্ট্যাটাস প্যানেল আঁকা (কোনো ক্যামেরা ফিড নেই, শুধু টেক্সট) ----
                panel = np.zeros((290, 480, 3), dtype=np.uint8)
                lines = [
                    f"State: {state}",
                    f"Servo angle: {angle}",
                    f"Yaw: {yaw:.1f} deg",
                    f"Speed: {speed}",
                    f"KP={kp:.2f} KD={kd:.2f} Smooth={smoothing_alpha:.2f}",
                    f"MaxAngleRate={max_angle_rate} deg/sec",
                    f"FrontTrigger={front_trigger_m:.2f}m x{front_confirm_frames}frames TargetYaw={target_yaw_deg}deg",
                    f"TargetGap={target_gap_m:.2f}m MaxWallDist={max_wall_distance_m:.2f}m MaxErr={max_error_m:.2f}m",
                    "",
                    "'s' = save to calibration.json   'q' = quit",
                ]
                for i, line in enumerate(lines):
                    cv2.putText(panel, line, (10, 25 + i * 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
                cv2.imshow(WINDOW, panel)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                updated = save_calibration({
                    "open_challenge": {
                        "speed": speed,
                        "kp": kp,
                        "kd": kd,
                        "smoothing_alpha": smoothing_alpha,
                        "max_angle_change_per_sec": max_angle_rate,
                        "front_trigger_m": front_trigger_m,
                        "front_confirm_frames": front_confirm_frames,
                        "target_yaw_change_deg": target_yaw_deg,
                        "target_gap_m": target_gap_m,
                        "max_wall_distance_m": max_wall_distance_m,
                        "max_error_m": max_error_m,
                        "max_turn_seconds": oc["max_turn_seconds"],
                    }
                })
                print(f"\nসেভ হয়েছে: {updated['open_challenge']}")

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
        cv2.destroyAllWindows()
        print("Robot ও LIDAR নিরাপদে বন্ধ করা হয়েছে।")


if __name__ == "__main__":
    main()
