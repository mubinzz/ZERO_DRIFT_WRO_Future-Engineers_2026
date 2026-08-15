"""
============================================================
TEST 12: lidar_processor.py লাইভ যাচাই
============================================================
উদ্দেশ্য: সেক্টর দূরত্ব আর pillar cluster লাইভ টেক্সট আকারে দেখা,
যাতে lidar_processor.py আসলে ঠিকমতো কাজ করছে কিনা নিশ্চিত হওয়া
যায় (GUI দরকার নেই, শুধু টার্মিনাল)।

ব্যবহার:
    python3 test_lidar_processor.py
Ctrl+C চাপুন বন্ধ করতে।

যা যাচাই করবেন:
- robot-এর সামনে হাত/বই ধরলে শুধু "front" সেক্টরের distance কমা
  উচিত, বাকিগুলো প্রায় অপরিবর্তিত থাকা উচিত
- জিনিসটা ডানে/বামে সরালে ঠিক সেই সেক্টরটাই সাড়া দেওয়া উচিত (এটাই
  আসলে front offset/rotation direction ঠিক বসেছে কিনা তার প্রমাণ)
- ছোট বস্তু (বোতল/বাক্স, pillar-এর মতো আকারে) রাখলে "cluster"
  হিসেবে ধরা পড়া উচিত, আশেপাশে একাধিক ভুয়া cluster না আসা উচিত
============================================================
"""

import sys
import os
import time
import math

from rplidar import RPLidar, RPLidarException
from serial import SerialException

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from perception.lidar_processor import get_sector_distances, scan_to_robot_points, cluster_points

LIDAR_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_19ad04198e596140a9e96e563d38c7ae-if00-port0"
LIDAR_BAUDRATE = 256000


def main():
    print(f"LIDAR এর সাথে কানেক্ট হচ্ছে: {LIDAR_PORT} @ {LIDAR_BAUDRATE} baud ...")
    lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)

    try:
        lidar.get_info()
        try:
            lidar.clean_input()
        except AttributeError:
            lidar._serial_port.reset_input_buffer()
        print("Motor স্থির গতিতে পৌঁছানোর জন্য ২ সেকেন্ড অপেক্ষা করা হচ্ছে...")
        time.sleep(2)

        print("Ctrl+C চাপুন বন্ধ করতে।\n")
        time.sleep(1)

        scan_iterator = lidar.iter_scans()
        consecutive_bad = 0

        while True:
            try:
                scan = next(scan_iterator)
            except RPLidarException:
                consecutive_bad += 1
                if consecutive_bad > 8:
                    print("!!! পরপর অনেক bad frame -- cable/power চেক করুন।")
                    break
                scan_iterator = lidar.iter_scans()
                continue
            consecutive_bad = 0

            if not scan:
                continue

            sectors = get_sector_distances(scan)
            points = scan_to_robot_points(scan)
            clusters = cluster_points(points)

            print("\033[H\033[J", end="")  # টার্মিনাল ক্লিয়ার করে নতুন করে দেখানো

            print("সেক্টর দূরত্ব (মিটার, robot-relative -- 0°=সামনে, +=ডানে):")
            for name, dist in sectors.items():
                dist_str = f"{dist:.2f}m" if dist is not None else "  --  "
                print(f"  {name:12}: {dist_str}")

            print(f"\nমোট {len(points)}টা বৈধ পয়েন্ট, {len(clusters)}টা cluster:")
            for i, (x, y) in enumerate(clusters[:10]):
                dist = math.hypot(x, y)
                angle = math.degrees(math.atan2(x, y))
                print(f"  Cluster {i+1}: x={x:+.2f} y={y:+.2f}  (দূরত্ব={dist:.2f}m, angle={angle:+.1f}°)")

    except KeyboardInterrupt:
        print("\nথামানো হচ্ছে...")
    except SerialException as e:
        print(f"\n!!! USB সংযোগ বিচ্ছিন্ন: {e} (পাওয়ার সমস্যা হতে পারে)")
    finally:
        for step in (lidar.stop, lidar.stop_motor, lidar.disconnect):
            try:
                step()
            except Exception:
                pass
        print("LIDAR বন্ধ করার চেষ্টা করা হয়েছে।")


if __name__ == "__main__":
    main()
