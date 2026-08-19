"""
============================================================
TEST 8: Pi <-> ESP32 সিরিয়াল লিংক ইন্টিগ্রেশন টেস্ট
============================================================
উদ্দেশ্য: main_controller.ino (ESP32) আর serial_link.py (Pi) একসাথে
ঠিকমতো কথা বলছে কিনা যাচাই করা। এই একটা টেস্টেই motor, servo,
encoder, IMU, button -- সবগুলোর সংযোগ একসাথে দেখা যাবে।

আগে যা করতে হবে:
1. esp32_firmware/main_controller/main_controller.ino Arduino
   IDE দিয়ে ESP32-তে আপলোড করুন।
2. ⚠️ robot টাকে এমন জায়গায় রাখুন যেখানে চাকা ঘুরলেও কোথাও ধাক্কা
   না লাগে (যেমন টেবিলের উপর চাকা শূন্যে ভাসিয়ে, বা স্ট্যান্ডে
   তুলে) -- কারণ এই টেস্টে সত্যিকারের speed দিয়ে মোটর চালানো হবে।

⚠️ পোর্ট বের করা: RPLIDAR আর ESP32 দুটোই CP2102 চিপ ব্যবহার করতে
পারে, তাই নাম দেখে আলাদা করা কঠিন হতে পারে। নিশ্চিত হওয়ার উপায়:
    ls -l /dev/serial/by-id/
এখানে যে সিরিয়াল নাম্বারটা RPLIDAR টেস্টে (test_lidar.py) দেখা
সিরিয়াল নাম্বারের সাথে মিলবে না, সেটাই ESP32।

চালানোর নিয়ম:
    python3 test_serial_link.py

তারপর "speed angle" ফরম্যাটে লিখুন, যেমন:
    100 90      -> সামনে speed 100, স্টিয়ারিং সোজা
    0 90        -> থামানো, সোজা
    -80 60      -> পেছনে speed 80, স্টিয়ারিং বামে

প্রতিবার আপনি একটা কমান্ড দেওয়ার পর সর্বশেষ telemetry দেখাবে --
    - ENC বাড়ছে/কমছে কিনা (চাকা সত্যিই ঘুরছে কিনা)
    - YAW বদলাচ্ছে কিনা (robot হাতে ঘোরালে)
    - BTN 1 হচ্ছে কিনা (বাটন চাপলে)

সমস্যা হলে:
- কোনো telemetry না এলে (ENC/YAW সবসময় 0, last_update বাড়ছে না):
  ESP32 এর সাথে সংযোগই হয়নি। পোর্ট নাম ভুল কিনা, ESP32 এ
  main_controller.ino আপলোড হয়েছে কিনা চেক করুন।
- মোটর/সার্ভো সাড়া না দিলেও telemetry ঠিকমতো আসলে: কমান্ড পাঠানো/
  পার্স করায় সমস্যা, ESP32 এর Serial Monitor (Arduino IDE বন্ধ
  রাখতে হবে যেন port conflict না হয়) দিয়ে "M:100,S:90" ম্যানুয়ালি
  পাঠিয়ে দেখুন কাজ করে কিনা।
- মোটর এক ঝলক ঘুরেই থেমে যায়, speed কম/বেশি পার্থক্য বোঝা যায় না:
  এটা bug না -- main_controller.ino তে একটা নিরাপত্তা watchdog আছে
  যেটা ৫০০ms এর মধ্যে নতুন কমান্ড না পেলে মোটর নিজে থেকে থামিয়ে
  দেয়। যেহেতু এখানে আপনি ম্যানুয়ালি টাইপ করে কমান্ড দিচ্ছেন (একবারই
  পাঠানো হয়, বারবার না), তাই নিচের কোডে একটা "repeater" থ্রেড যোগ
  করা হয়েছে যেটা background এ প্রতি ১৫০ms এ শেষ কমান্ডটাই বারবার
  পাঠাতে থাকে -- এতে আপনি নতুন কমান্ড না দেওয়া পর্যন্ত motor চালু
  থাকবে। (আসল challenge কোডেও এভাবেই continuous ভাবে কমান্ড পাঠানো
  হবে, তাই এই প্যাটার্নটা বাস্তব ব্যবহারেরই preview)
============================================================
"""

import sys
import os
import time
import threading

# serial_link.py প্যারেন্ট ফোল্ডারে (pi/) আছে, tests/ এ না -- তাই
# path এ যোগ করে নিচ্ছি
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from serial_link import SerialLink


class CommandRepeater:
    """শেষ যে কমান্ড দেওয়া হয়েছে সেটা ব্যাকগ্রাউন্ডে বারবার পাঠাতে থাকে,
    যাতে ESP32 এর ৫০০ms নিরাপত্তা watchdog ট্রিগার না হয়।"""

    def __init__(self, link, interval=0.15):
        self.link = link
        self.interval = interval
        self.speed = 0
        self.angle = 90
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def set_command(self, speed, angle):
        with self._lock:
            self.speed = speed
            self.angle = angle

    def _loop(self):
        while self._running:
            with self._lock:
                s, a = self.speed, self.angle
            self.link.send_command(s, a)
            time.sleep(self.interval)

    def stop(self):
        self._running = False
        self._thread.join(timeout=1.0)

# ---- CONFIG ----
# /dev/ttyUSB0 বা /dev/ttyUSB1 সরাসরি ব্যবহার করা হচ্ছে না, কারণ
# ডিভাইস প্লাগ করার ক্রম/রিবুটের উপর ভিত্তি করে এই নাম্বার উল্টে
# যেতে পারে (যা আমাদের সাথেই ঘটেছিল)। এর বদলে /dev/serial/by-id/
# এর স্থায়ী নাম ব্যবহার করা হচ্ছে -- এটা vendor/serial number থেকে
# আসে, তাই কখনো বদলায় না। `ls -l /dev/serial/by-id/` চালিয়ে আপনার
# ESP32 বোর্ডের (CH340/"1a86" চিপ) সঠিক নামটা মিলিয়ে নিন।
ESP32_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"


def format_telemetry(telem):
    if telem["last_update"] == 0.0:
        return "এখনো কোনো telemetry পাওয়া যায়নি"
    age = time.time() - telem["last_update"]
    return (
        f"ENC={telem['encoder']} YAW={telem['yaw']:.2f} BTN={telem['button']} "
        f"(শেষ আপডেট {age:.2f}s আগে)"
    )


def main():
    print(f"ESP32 এর সাথে কানেক্ট হচ্ছে: {ESP32_PORT} ...")
    link = SerialLink(ESP32_PORT)
    repeater = CommandRepeater(link)  # watchdog satisfied রাখতে ব্যাকগ্রাউন্ডে বারবার পাঠাবে

    print("ESP32 বুট + gyro calibration এর জন্য অপেক্ষা করা হচ্ছে...")
    time.sleep(1)

    print("\nকমান্ড ফরম্যাট: <speed> <angle>  (উদাহরণ: 100 90)")
    print("speed: -255..255 (0 = থামা) | angle: 0..180 (90 = সোজা)")
    print("দেওয়া কমান্ড নতুন কমান্ড না দেওয়া পর্যন্ত চালু থাকবে (ব্যাকগ্রাউন্ডে বারবার পাঠানো হচ্ছে)।")
    print("খালি রেখে Enter দিলে শুধু telemetry রিফ্রেশ হবে। বন্ধ করতে Ctrl+C।\n")

    try:
        while True:
            print(f"[টেলিমেট্রি] {format_telemetry(link.get_telemetry())}")
            user_input = input("speed angle > ").strip()

            if not user_input:
                continue

            parts = user_input.split()
            if len(parts) != 2:
                print("ভুল ফরম্যাট! এভাবে লিখুন, দুইটা সংখ্যা স্পেস দিয়ে আলাদা: 100 90")
                continue

            try:
                speed = int(parts[0])
                angle = int(parts[1])
            except ValueError:
                print("সংখ্যা দিতে হবে, যেমন: 100 90")
                continue

            repeater.set_command(speed, angle)
            time.sleep(0.1)  # ESP32 এর পরবর্তী telemetry আসার জন্য সামান্য অপেক্ষা

    except KeyboardInterrupt:
        print("\nথামানো হচ্ছে...")
    finally:
        repeater.set_command(0, 90)  # বন্ধ করার আগে নিরাপদ কমান্ডে সেট করা
        time.sleep(0.2)
        repeater.stop()
        link.close()
        print("নিরাপত্তার জন্য motor বন্ধ ও servo সোজা করে দেওয়া হয়েছে।")


if __name__ == "__main__":
    main()
