"""
============================================================
serial_link.py — Pi ↔ ESP32 কমিউনিকেশন মডিউল
============================================================
এটা ESP32-এর main_controller.ino ফার্মওয়্যারের সাথে কথা বলার
জন্য একটা reusable ক্লাস। পরে Open Challenge, Obstacle Challenge,
Parking -- সব challenge script এটাই ব্যবহার করবে, তাই এই মডিউল
একবার ঠিকমতো কাজ করলে বাকি সব জায়গায় নিশ্চিন্তে reuse করা যাবে।

ডিজাইন সিদ্ধান্ত: telemetry পড়ার জন্য একটা আলাদা ব্যাকগ্রাউন্ড
থ্রেড ব্যবহার করা হয়েছে। কারণ: main perception loop (LIDAR/camera
প্রসেস করা) কে সিরিয়াল read এর জন্য block/অপেক্ষা করতে হবে না --
দুটো কাজ সমান্তরালে (parallel) চলবে। থ্রেড ক্রমাগত নতুন telemetry
লাইন পড়ে "সর্বশেষ মান" আপডেট করতে থাকে, main script যেকোনো সময়
get_telemetry() কল করলে সর্বশেষ যা পাওয়া গেছে সেটাই সাথে সাথে
(instantly) পেয়ে যাবে।
============================================================
"""

import serial
import threading
import time


class SerialLink:
    def __init__(self, port, baudrate=115200, timeout=1.0):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

        # ESP32 বোর্ড serial port খোলার সময় সাধারণত auto-reset হয়ে যায়
        # (USB-UART চিপের DTR/RTS টগল হওয়ার কারণে, ঠিক Arduino Uno এর
        # মতো)। তাই বোর্ড বুট + setup() + gyro calibration শেষ হওয়ার
        # জন্য কিছুটা সময় দেওয়া দরকার, নাহলে শুরুর কমান্ড হারিয়ে যেতে
        # পারে।
        time.sleep(2.5)

        self._lock = threading.Lock()
        self._latest = {"encoder": 0, "yaw": 0.0, "button": 0, "last_update": 0.0}
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while self._running:
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                time.sleep(0.1)
                continue

            if not raw:
                continue  # timeout, নতুন কোনো ডেটা আসেনি

            line = raw.decode("utf-8", errors="ignore").strip()

            # ESP32 বুট/ডিবাগ মেসেজ "#" দিয়ে শুরু হয় (main_controller.ino
            # দেখুন), এগুলো telemetry না, উপেক্ষা করা হচ্ছে
            if not line or line.startswith("#"):
                continue

            self._parse_telemetry(line)

    def _parse_telemetry(self, line):
        # প্রত্যাশিত ফরম্যাট: "ENC:123,YAW:45.67,BTN:1"
        fields = {}
        for chunk in line.split(","):
            if ":" not in chunk:
                return  # ফরম্যাট মিলছে না, পুরো লাইনটাই বাদ
            key, _, value = chunk.partition(":")
            fields[key] = value

        if not {"ENC", "YAW", "BTN"} <= fields.keys():
            return  # কোনো ফিল্ড মিসিং, বাদ

        try:
            encoder = int(fields["ENC"])
            yaw = float(fields["YAW"])
            button = int(fields["BTN"])
        except ValueError:
            return  # সংখ্যায় রূপান্তর করা গেল না (আধা-লেখা লাইন হতে পারে), বাদ

        with self._lock:
            self._latest["encoder"] = encoder
            self._latest["yaw"] = yaw
            self._latest["button"] = button
            self._latest["last_update"] = time.time()

    def get_telemetry(self):
        """সর্বশেষ পাওয়া telemetry dict আকারে রিটার্ন করে।
        {'encoder': int, 'yaw': float, 'button': 0/1, 'last_update': unix_timestamp}
        last_update == 0.0 মানে এখনো কোনো ভ্যালিড telemetry পাওয়া যায়নি।"""
        with self._lock:
            return dict(self._latest)

    def send_command(self, speed, angle):
        """speed: -255..255, angle: 0..180 (90 = center)"""
        speed = max(-255, min(255, int(speed)))
        angle = max(0, min(180, int(angle)))
        cmd = f"M:{speed},S:{angle}\n"
        self.ser.write(cmd.encode("utf-8"))

    def close(self):
        # বন্ধ করার আগে নিরাপত্তার জন্য মোটর থামিয়ে, সোজা করে দেওয়া হচ্ছে
        try:
            self.send_command(0, 90)
            time.sleep(0.1)
        except Exception:
            pass
        self._running = False
        self._thread.join(timeout=1.0)
        self.ser.close()
