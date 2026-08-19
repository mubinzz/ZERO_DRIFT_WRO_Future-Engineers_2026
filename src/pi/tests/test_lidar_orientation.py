"""
============================================================
TEST 10: LIDAR মাউন্ট অরিয়েন্টেশন যাচাই
============================================================
উদ্দেশ্য: robot-এর "সামনে" দিকটা RPLIDAR-এর raw angle স্কেলে ঠিক
কত ডিগ্রিতে পড়ে সেটা অনুমান না করে সরাসরি মেপে বের করা, এবং angle
বাড়া মানে robot-এর সাপেক্ষে ঘড়ির কাঁটার দিকে (ডানে) না উল্টো
দিকে (বামে) ঘোরা -- সেটাও নিশ্চিত হওয়া।

⚠️ এই দুইটা তথ্য (front offset + rotation direction) ছাড়া
wall-following কোড লেখা মানে অনুমানের উপর ভিত্তি করে লেখা --
ঠিক যে ভুলটা camera color detection-এ হয়েছিল (Picamera2-র ফরম্যাট
নাম না যাচাই করে ধরে নেওয়া)। তাই এখানে প্রথমে মেপে নিশ্চিত হচ্ছি।

পদ্ধতি:
1. robot-টা একটা খোলা জায়গায় রাখুন (আশেপাশে অন্য কোনো দেয়াল/বস্তু
   না থাকাই ভালো, যাতে "সবচেয়ে কাছের বস্তু" মানেই আপনার ধরা বাক্স
   হয়)।
2. একটা বাক্স/বই robot চ্যাসিসের ঠিক "সামনে" (robot যেদিকে এগোবে
   সেই দিকে) ধরুন, robot থেকে প্রায় ৩০-৫০ সেমি দূরে।
3. এই স্ক্রিপ্ট চালান -- এটা প্রতি মুহূর্তে সবচেয়ে কাছের বস্তুর
   angle+distance দেখাবে।
4. যে angle সংখ্যাটা বারবার (আপনার ধরা বাক্সের জন্য) দেখা যাচ্ছে,
   **সেটাই "front offset"** -- নোট করে রাখুন।
5. এবার বাক্সটা robot-এর ডানদিকে (হাতে ধরে) সরান, angle সংখ্যাটা
   বাড়ছে না কমছে লক্ষ্য করুন:
   - যদি বাক্স ডানদিকে গেলে angle বাড়ে -> angle বৃদ্ধি = ঘড়ির
     কাঁটার দিকে (ডানে) ঘোরা
   - যদি বাক্স ডানদিকে গেলে angle কমে -> angle বৃদ্ধি = ঘড়ির
     কাঁটার উল্টো দিকে (বামে) ঘোরা

দুটো তথ্যই (front offset ডিগ্রি, আর angle বৃদ্ধির দিক) আমাকে জানান --
এর উপর ভিত্তি করেই lidar_processor.py-এর কো-অর্ডিনেট রূপান্তর কোড
লিখব, কোনো অনুমান ছাড়া।

চালানোর নিয়ম:
    python3 test_lidar_orientation.py
বন্ধ করতে Ctrl+C।
============================================================
"""

from rplidar import RPLidar, RPLidarException
from serial import SerialException
import time

# আগে যাচাই করা স্থায়ী by-id পোর্ট (test_lidar.py এর মতোই)
LIDAR_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_19ad04198e596140a9e96e563d38c7ae-if00-port0"
LIDAR_BAUDRATE = 256000


def main():
    print(f"LIDAR এর সাথে কানেক্ট হচ্ছে: {LIDAR_PORT} @ {LIDAR_BAUDRATE} baud ...")
    lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)

    try:
        info = lidar.get_info()
        print("Device info:", info)

        try:
            lidar.clean_input()
        except AttributeError:
            lidar._serial_port.reset_input_buffer()
        print("Motor স্থির গতিতে পৌঁছানোর জন্য ২ সেকেন্ড অপেক্ষা করা হচ্ছে...")
        time.sleep(2)

        print("\nrobot-এর সামনে বাক্স ধরুন (৩০-৫০ সেমি দূরে)। Ctrl+C চাপুন থামাতে।\n")

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

            closest = min(scan, key=lambda p: p[2])
            quality, angle, distance = closest
            print(f"সবচেয়ে কাছের বস্তু: angle={angle:6.1f}°   distance={distance:6.0f}mm")

    except KeyboardInterrupt:
        print("\nথামানো হচ্ছে...")
    except SerialException as e:
        print(f"\n!!! USB/সিরিয়াল সংযোগ বিচ্ছিন্ন: {e} (পাওয়ার সমস্যা হতে পারে)")
    finally:
        for step in (lidar.stop, lidar.stop_motor, lidar.disconnect):
            try:
                step()
            except Exception:
                pass
        print("LIDAR বন্ধ করার চেষ্টা করা হয়েছে।")


if __name__ == "__main__":
    main()
