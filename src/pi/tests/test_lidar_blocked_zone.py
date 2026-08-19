"""
============================================================
TEST 11: LIDAR-এর "ব্লকড জোন" (battery-এ আটকানো অংশ) চিহ্নিত করা
============================================================
কেন এটা দরকার: robot-এর পেছনে battery বসানো, যেটা LIDAR-এর পেছনের
একটা অংশের দৃষ্টিসীমা ঢেকে রেখেছে। সেই দিকে LIDAR যে distance
রিপোর্ট করবে সেটা room-এর real distance না -- battery casing-এর
distance। এই ডেটাকে সত্যিকারের "wall" ভেবে ব্যবহার করলে robot ভুল
সিদ্ধান্ত নেবে (যেমন সবসময় মনে করবে ঠিক পেছনেই একটা দেয়াল লেগে
আছে, যেটা আসলে battery)। তাই এই ব্লকড angular range-টা মেপে বের করে
lidar_processor.py-তে বাদ দিতে হবে।

পদ্ধতি:
1. robot-টা একটা খোলা জায়গায় রাখুন -- চারপাশে (কমপক্ষে ১-২ মিটার)
   কোনো দেয়াল/বড় বস্তু না থাকাই ভালো।
2. এই স্ক্রিপ্ট চালান -- প্রতি ১০° কোণ-বাক্সে সবচেয়ে কাছের distance
   দেখাবে, বারবার (live) আপডেট হবে।
3. **স্বাভাবিক/খোলা দিকগুলোতে** distance বড় সংখ্যা দেখাবে (কারণ
   কাছে কিছু নেই) -- ঘরের block অনুযায়ী কম-বেশি ওঠানামাও করতে পারে
   যদি আপনি হাত নাড়ান/হাঁটেন।
4. **battery-blocked দিকে** distance থাকবে **ছোট এবং প্রায় স্থির**
   (protap বার প্রায় একই সংখ্যা, আপনি চারপাশে যাই করুন না কেন
   বদলাবে না) -- এটাই ব্লকড zone-এর আসল চিহ্ন। কোড এই ধরনের সন্দেহজনক
   (৩০ সেমি এর কম) সারিগুলোকে "<-- সন্দেহজনক" ফ্ল্যাগ দিয়ে দেখাবে,
   কিন্তু আসল সিদ্ধান্ত আপনাকেই নিতে হবে -- **অস্থির/বদলানো** সংখ্যা
   (আপনি হাত নাড়ালে বদলায়) মানে ওটা real world, ব্লকড না।
5. যেসব angle bucket-এ এই "স্থির-ছোট" প্যাটার্ন দেখছেন, সেই পুরো
   রেঞ্জটা (শুরু-শেষ ডিগ্রি) নোট করে জানান।

চালানোর নিয়ম:
    python3 test_lidar_blocked_zone.py
বন্ধ করতে Ctrl+C।
============================================================
"""

from rplidar import RPLidar, RPLidarException
from serial import SerialException
import time

# আগে যাচাই করা স্থায়ী by-id পোর্ট
LIDAR_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_19ad04198e596140a9e96e563d38c7ae-if00-port0"
LIDAR_BAUDRATE = 256000
BUCKET_SIZE_DEG = 10
SUSPICIOUS_DISTANCE_MM = 300  # শুধু চোখে পড়ার জন্য ফ্ল্যাগ, চূড়ান্ত সিদ্ধান্ত না


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

        print("চারপাশে খোলা জায়গা নিশ্চিত করুন। Ctrl+C চাপুন থামাতে।\n")
        time.sleep(1)

        scan_iterator = lidar.iter_scans()
        consecutive_bad = 0
        num_buckets = 360 // BUCKET_SIZE_DEG

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

            buckets = [None] * num_buckets
            for quality, angle, distance in scan:
                idx = int(angle // BUCKET_SIZE_DEG) % num_buckets
                if buckets[idx] is None or distance < buckets[idx]:
                    buckets[idx] = distance

            # টার্মিনাল স্ক্রিন ক্লিয়ার করে নতুন করে টেবিল আঁকা (live আপডেটের মতো দেখতে)
            print("\033[H\033[J", end="")
            print("Raw LIDAR angle (robot-relative না, সরাসরি সেন্সরের সংখ্যা) অনুযায়ী সবচেয়ে কাছের distance:\n")
            print(f"{'কোণ রেঞ্জ':>12} | {'Min Dist (mm)':>14} | নোট")
            print("-" * 55)
            for i, dist in enumerate(buckets):
                a0 = i * BUCKET_SIZE_DEG
                a1 = a0 + BUCKET_SIZE_DEG
                if dist is None:
                    print(f"{a0:>4}-{a1:<4}° | {'(কোনো পয়েন্ট নেই)':>14} |")
                else:
                    flag = "  <-- সন্দেহজনক (ছোট, স্থির কিনা লক্ষ্য করুন)" if dist < SUSPICIOUS_DISTANCE_MM else ""
                    print(f"{a0:>4}-{a1:<4}° | {dist:>14.0f} |{flag}")

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
