"""
============================================================
TEST 6: RPLIDAR A3M1 টেস্ট
============================================================
উদ্দেশ্য: LIDAR ঠিকমতো USB দিয়ে কানেক্ট হচ্ছে কিনা, মোটর ঘুরছে
কিনা, আর scan ডেটা (angle + distance জোড়া) আসছে কিনা যাচাই করা।

ইনস্টল করতে হবে:
    pip install --break-system-packages rplidar

⚠️ গুরুত্বপূর্ণ: RPLIDAR A3 এর ডিফল্ট baudrate 256000 (A1/A2 এর
115200 থেকে আলাদা!)। ভুল baudrate দিলে কানেক্ট হবে না বা garbage
ডেটা আসবে।

USB পোর্ট বের করার উপায় (Pi টার্মিনালে):
    ls /dev/ttyUSB*
সাধারণত /dev/ttyUSB0 হয়, কিন্তু ESP32ও USB সিরিয়াল হিসেবে দেখা
যেতে পারে, তাই দুইটা ডিভাইস প্লাগ থাকলে কোনটা কোনটা সেটা
`ls -l /dev/serial/by-id/` কমান্ড দিয়ে নিশ্চিত হওয়া ভালো (এখানে
প্রতিটা ডিভাইসের ইউনিক নাম দেখা যায়, প্লাগ খুললে-লাগালে পোর্ট
নাম্বার (ttyUSB0/1) বদলে যেতে পারে কিন্তু by-id নাম বদলায় না)।

চালানোর নিয়ম:
    python3 test_lidar.py

সমস্যা হলে:
- "Permission denied" এলে: sudo usermod -a -G dialout $USER চালিয়ে
  লগ-আউট/লগ-ইন করুন (অথবা sudo দিয়ে চালান, কিন্তু স্থায়ী সমাধান হলো
  dialout গ্রুপে যোগ হওয়া)।
- মোটর ঘুরছে না/স্পিন আপ হচ্ছে না: LIDAR এর পাওয়ার পিন (সাধারণত
  motor control আলাদা পিনে থাকে) ঠিকমতো কানেক্ট আছে কিনা দেখুন।
- scan এ পয়েন্ট সংখ্যা খুব কম (৫০ এর নিচে): এটা normal হতে পারে
  scan সবে শুরু হলে, কয়েক scan পরে স্থির হয়ে যাবে।
- get_info()/get_health() সফল হয় কিন্তু iter_scans() শুরু হতেই
  "device disconnected" / "Input/output error" আসে: এটা প্রায়
  সবসময় পাওয়ার সমস্যা, কোডের বাগ না। info/health ছোট command
  মাত্র, কিন্তু scan শুরু হলে motor স্পিন করে যেটা হঠাৎ বেশি
  কারেন্ট টানে -- সেই মুহূর্তে USB পাওয়ার ড্রপ করে ডিভাইস disconnect
  হয়ে যায়। এরর হলে সাথে সাথে আরেকটা টার্মিনালে `dmesg -T | tail -40`
  চালিয়ে দেখুন "usb disconnect"/"over-current" জাতীয় লাইন আছে কিনা।
  সমাধান: (ক) একই সময়ে অন্য USB ডিভাইস (যেমন ESP32) খুলে রেখে
  আলাদা টেস্ট করুন, (খ) সাময়িকভাবে Pi কে অফিসিয়াল ওয়াল অ্যাডাপ্টার
  দিয়ে চালিয়ে দেখুন কাজ করে কিনা (LiPo/buck converter বাদ দিয়ে)।
- "RPLidarException: Wrong body size" আসে: এটা পাওয়ার সমস্যা না,
  ডেটা ফ্রেম এলোমেলো (corrupted) আসার লক্ষণ। সাধারণত motor সবে
  স্পিন-আপ শুরু করেছে (এখনো স্থির গতিতে পৌঁছায়নি) এমন সময়ে scan
  request পাঠালে অথবা আগের command (get_info/get_health) এর কিছু
  বাইট serial বাফারে "আটকে" থেকে গেলে এটা হয়। নিচের কোডে এটার জন্য
  দুটো সুরক্ষা যোগ করা হয়েছে: (১) scan শুরুর আগে বাফার পরিষ্কার করা
  ও motor স্থির হওয়ার জন্য অপেক্ষা করা, (২) কোনো একটা scan frame
  এলোমেলো এলে পুরো প্রোগ্রাম বন্ধ না করে সেটা স্কিপ করে পরের frame
  থেকে চালিয়ে যাওয়া (বাস্তব রোবটেও সেন্সর মাঝেমধ্যে খারাপ ডেটা
  দেয়, তাই code-কে resilient/সহনশীল হতে হয়, একটা bad frame এ crash
  করলে চলবে না)।
============================================================
"""

from rplidar import RPLidar, RPLidarException
from serial import SerialException
import time

# ---- CONFIG ----
# /dev/ttyUSB0/1 এর বদলে স্থায়ী /dev/serial/by-id/ পাথ ব্যবহার করা
# হচ্ছে -- প্লাগ/রিবুট এর ক্রম অনুযায়ী ttyUSB নাম্বার উল্টে যেতে
# পারে (আমাদের সাথে ঘটেছিল), by-id নাম কখনো বদলায় না। `ls -l
# /dev/serial/by-id/` চালিয়ে LIDAR এর (CP2102/Silicon Labs) সঠিক
# নামটা মিলিয়ে নিন।
LIDAR_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_19ad04198e596140a9e96e563d38c7ae-if00-port0"
LIDAR_BAUDRATE = 256000  # RPLIDAR A3 এর জন্য

def main():
    print(f"LIDAR এর সাথে কানেক্ট হচ্ছে: {LIDAR_PORT} @ {LIDAR_BAUDRATE} baud ...")
    lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)

    try:
        info = lidar.get_info()
        print("Device info:", info)

        health = lidar.get_health()
        print("Health status:", health)  # ('Good', 0) হওয়া উচিত

        # scan শুরু করার আগে: (১) আগের command এর কোনো অবশিষ্ট বাইট
        # serial বাফারে থেকে গেলে সেটা পরিষ্কার করা, (২) motor স্থির
        # গতিতে পৌঁছানোর জন্য কিছুটা সময় দেওয়া। এই দুটো না করলে
        # প্রথম কয়েকটা scan frame এলোমেলো (corrupted) আসার সম্ভাবনা
        # বেশি থাকে।
        def clear_serial_buffer():
            try:
                lidar.clean_input()  # লাইব্রেরির পাবলিক মেথড, থাকলে এটাই ব্যবহার হবে
            except AttributeError:
                lidar._serial_port.reset_input_buffer()  # পুরনো ভার্সনে fallback

        clear_serial_buffer()
        print("Motor স্থির গতিতে পৌঁছানোর জন্য ২ সেকেন্ড অপেক্ষা করা হচ্ছে...")
        time.sleep(2)

        print("\nScan শুরু হচ্ছে... বন্ধ করতে Ctrl+C চাপুন।\n")

        scan_count = 0
        bad_frame_count = 0
        consecutive_bad_frames = 0
        MAX_CONSECUTIVE_BAD = 8  # এর বেশি পরপর bad frame এলে hardware সমস্যা ধরে নিয়ে থামব

        scan_iterator = lidar.iter_scans()
        while scan_count < 20:
            try:
                scan = next(scan_iterator)
            except RPLidarException as e:
                bad_frame_count += 1
                consecutive_bad_frames += 1
                print(f"  (একটা scan frame এলোমেলো এসেছে, স্কিপ করছি: {e})")

                if consecutive_bad_frames >= MAX_CONSECUTIVE_BAD:
                    print(
                        f"\n!!! পরপর {MAX_CONSECUTIVE_BAD} বার এলোমেলো frame এলো -- "
                        "এটা এখন আর সাধারণ warm-up glitch মনে হচ্ছে না, cable/connector/"
                        "পাওয়ার আবার চেক করুন।"
                    )
                    break

                # একটা corrupt frame এর পর generator টা "শেষ" হয়ে যায়,
                # তাই নতুন করে iterator বানিয়ে আবার শুরু করতে হয়
                clear_serial_buffer()
                scan_iterator = lidar.iter_scans()
                continue

            consecutive_bad_frames = 0  # ভালো frame পেলে counter রিসেট
            scan_count += 1
            # প্রতিটা scan হলো (quality, angle, distance_mm) এর লিস্ট
            num_points = len(scan)

            if num_points > 0:
                closest = min(scan, key=lambda p: p[2])
                print(
                    f"Scan #{scan_count}: {num_points} points | "
                    f"সবচেয়ে কাছের বস্তু: angle={closest[1]:.1f} deg, "
                    f"distance={closest[2]:.0f} mm"
                )
            else:
                print(f"Scan #{scan_count}: কোনো পয়েন্ট পাওয়া যায়নি")

        print(f"\n২০টা ভালো scan হয়ে গেছে (মাঝে {bad_frame_count}টা এলোমেলো frame স্কিপ করা হয়েছে), টেস্ট শেষ করা হচ্ছে।")

    except KeyboardInterrupt:
        print("\nইউজার থামিয়ে দিয়েছেন।")
    except SerialException as e:
        print(f"\n!!! USB/সিরিয়াল সংযোগ হঠাৎ বিচ্ছিন্ন হয়ে গেছে: {e}")
        print("!!! এটা প্রায় সবসময় পাওয়ার সাপ্লাই সমস্যা (উপরের 'সমস্যা হলে' অংশ দেখুন),")
        print("!!! কোডের বাগ না। motor স্পিন-আপের সময় কারেন্ট স্পাইকের কারণে এটা হয়।")
    finally:
        # LIDAR মোটর বন্ধ করা এবং কানেকশন পরিষ্কারভাবে ছাড়া — এটা না করলে
        # পরের বার চালাতে সমস্যা হতে পারে। ডিভাইস আগে থেকেই disconnected
        # থাকলে এই cleanup কলগুলোও fail করতে পারে, তাই প্রতিটা আলাদাভাবে
        # try/except দিয়ে ঘিরে রাখা হয়েছে -- যাতে আসল এরর ঢাকা না পড়ে।
        for cleanup_step in (lidar.stop, lidar.stop_motor, lidar.disconnect):
            try:
                cleanup_step()
            except Exception:
                pass
        print("LIDAR বন্ধ করার চেষ্টা করা হয়েছে (ডিভাইস আগে থেকেই disconnected থাকলে এই ধাপগুলো নীরবে skip হয়েছে)।")


if __name__ == "__main__":
    main()
