"""
============================================================
TEST 15: Arena live visualization (LIDAR wall + camera-fused pillar)
============================================================
উদ্দেশ্য: robot-এর চারপাশে কী দেখা যাচ্ছে সেটা চোখে দেখে যাচাই করা --
কোনটা wall (ধূসর বিন্দু), কোনটা pillar আর কোন রঙের (লাল/সবুজ বৃত্ত)।
pillar_detector.py-এর ray-casting fusion শুধু তখনই একটা pillar
রিটার্ন করে যখন camera-তে রঙ পাওয়া গেছে এবং সেই রশ্মি LIDAR-এর কোনো
pillar-আকারের cluster-কে ছুঁয়েছে -- তাই এখানে "অজানা রঙ" বলে কিছু
দেখানো হয় না, wall শুধু ধূসর বিন্দু হিসেবেই থাকে।

⚠️ robot নড়বে না -- এটা শুধু sensing (LIDAR + camera) যাচাই, motor
কমান্ড পাঠানো হয় না। robot টেবিলে/মেঝেতে যেকোনো জায়গায় রাখলেই চলবে।

⚠️ চালানোর আগে অবশ্যই লাগবে:
  - camera_red_hsv/camera_green_hsv (tune_hsv_live.py)
  - camera_hfov_deg/camera_angle_offset_deg/camera_offset_m
    (calibrate_camera_hfov.py) -- না থাকলে script শুরুতেই error
    দেখিয়ে থেমে যাবে

চালানোর নিয়ম:
    python3 test_arena_visualization.py
বন্ধ করতে QUIT বাটন বা window-এর X চাপুন।

দেখানো হয়:
  - robot (মাঝে, উপরের দিকে মুখ করা ত্রিভুজ -- 0°=সামনে)
  - ধূসর বিন্দু: LIDAR-এর raw point (wall/বাধা)
  - লাল/সবুজ বৃত্ত: ফিউজড pillar (camera রঙ + LIDAR দূরত্ব)
============================================================
"""

import sys
import os
import time
import threading
import tkinter as tk
from tkinter import ttk

from rplidar import RPLidar, RPLidarException

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import load_calibration
from camera_link import create_camera
from perception.lidar_processor import scan_to_robot_points
from perception.pillar_detector import detect_pillars

LIDAR_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_19ad04198e596140a9e96e563d38c7ae-if00-port0"
LIDAR_BAUDRATE = 256000

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

CANVAS_SIZE = 700
SCALE_PX_PER_M = 100  # ক্যানভাসে ১ মিটার = ১০০ পিক্সেল (robot মাঝে, তাই ~3.5m radius দেখা যাবে)


class ArenaVisualizer:
    def __init__(self, root, calibration):
        self.root = root
        self.root.title("Arena Live Visualization -- LIDAR + Camera Fusion")
        self.cal = calibration

        self.lock = threading.Lock()
        self.shared = {"points": [], "pillars": [], "status": "শুরু হচ্ছে..."}
        self.alive = True

        self.lidar = None
        self.picam2 = None

        canvas_frame = ttk.Frame(root)
        canvas_frame.grid(row=0, column=0, padx=8, pady=8)
        self.canvas = tk.Canvas(canvas_frame, width=CANVAS_SIZE, height=CANVAS_SIZE, bg="white")
        self.canvas.pack()

        status_frame = ttk.LabelFrame(root, text="অবস্থা")
        status_frame.grid(row=1, column=0, padx=8, pady=(0, 8), sticky="ew")
        self.status_label = tk.Label(status_frame, text="শুরু হচ্ছে...",
                                      font=("Courier", 10), justify="left", anchor="w")
        self.status_label.grid(row=0, column=0, padx=6, pady=6, sticky="w")

        tk.Button(root, text="QUIT", width=10, height=2, command=self.on_quit).grid(
            row=2, column=0, pady=(0, 8))

        self.root.protocol("WM_DELETE_WINDOW", self.on_quit)

        self.sensor_thread = threading.Thread(target=self._sensor_loop, daemon=True)
        self.sensor_thread.start()

        self._redraw()

    # ============================================================
    # সেন্সর থ্রেড (LIDAR + camera পড়া, GUI থ্রেড আটকাবে না)
    # ============================================================
    def _sensor_loop(self):
        try:
            with self.lock:
                self.shared["status"] = "LIDAR এর সাথে কানেক্ট হচ্ছে..."
            lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
            self.lidar = lidar
            lidar.get_info()
            try:
                lidar.clean_input()
            except AttributeError:
                lidar._serial_port.reset_input_buffer()
            time.sleep(2)

            with self.lock:
                self.shared["status"] = "ক্যামেরা চালু হচ্ছে..."
            picam2 = create_camera(CAMERA_WIDTH, CAMERA_HEIGHT)
            self.picam2 = picam2

            scan_iterator = lidar.iter_scans()
            consecutive_bad = 0

            while self.alive:
                try:
                    scan = next(scan_iterator)
                except RPLidarException:
                    consecutive_bad += 1
                    if consecutive_bad > 8:
                        with self.lock:
                            self.shared["status"] = "!!! LIDAR বারবার খারাপ ডেটা দিচ্ছে"
                        break
                    scan_iterator = lidar.iter_scans()
                    continue
                consecutive_bad = 0

                if not scan:
                    continue

                points = scan_to_robot_points(scan)
                frame = picam2.capture_array()
                try:
                    pillars = detect_pillars(scan, frame, self.cal)
                except ValueError as e:
                    with self.lock:
                        self.shared["status"] = f"!!! {e}"
                    break

                red_count = sum(1 for p in pillars if p.color == "red")
                green_count = sum(1 for p in pillars if p.color == "green")
                status = (f"মোট LIDAR পয়েন্ট: {len(points)}\n"
                          f"লাল pillar: {red_count}   সবুজ pillar: {green_count}")

                with self.lock:
                    self.shared["points"] = points
                    self.shared["pillars"] = pillars
                    self.shared["status"] = status

        except Exception as e:
            with self.lock:
                self.shared["status"] = f"!!! সমস্যা: {e}"

        finally:
            self._cleanup_hardware()

    def _cleanup_hardware(self):
        if self.picam2 is not None:
            try:
                self.picam2.stop()
            except Exception:
                pass
        if self.lidar is not None:
            for step in (self.lidar.stop, self.lidar.stop_motor, self.lidar.disconnect):
                try:
                    step()
                except Exception:
                    pass

    # ============================================================
    # ক্যানভাস আঁকা (GUI থ্রেড)
    # ============================================================
    def _redraw(self):
        with self.lock:
            points = list(self.shared["points"])
            pillars = list(self.shared["pillars"])
            status = self.shared["status"]

        self.canvas.delete("all")
        cx, cy = CANVAS_SIZE // 2, CANVAS_SIZE // 2

        # দূরত্ব-নির্দেশক বৃত্ত (প্রতি ১ মিটারে একটা)
        for r_m in range(1, 4):
            r_px = r_m * SCALE_PX_PER_M
            self.canvas.create_oval(cx - r_px, cy - r_px, cx + r_px, cy + r_px,
                                     outline="#dddddd")
            self.canvas.create_text(cx + r_px, cy - 4, text=f"{r_m}m", fill="#aaaaaa",
                                     font=("Arial", 8))

        # robot marker (ছোট ত্রিভুজ, উপরের দিকে মুখ করা = সামনে)
        self.canvas.create_polygon(cx, cy - 12, cx - 8, cy + 8, cx + 8, cy + 8,
                                    fill="#2e7d32", outline="black")

        # raw LIDAR পয়েন্ট (wall/বাধা) -- ধূসর বিন্দু
        for x, y, _, _ in points:
            px = cx + x * SCALE_PX_PER_M
            py = cy - y * SCALE_PX_PER_M
            self.canvas.create_oval(px - 1, py - 1, px + 1, py + 1, fill="#999999", outline="")

        # ফিউজড pillar detection (color সবসময় "red"/"green", None আসে না)
        for p in pillars:
            px = cx + p.x * SCALE_PX_PER_M
            py = cy - p.y * SCALE_PX_PER_M
            color = "#e02727" if p.color == "red" else "#22a022"
            self.canvas.create_oval(px - 7, py - 7, px + 7, py + 7,
                                     fill=color, outline="black", width=1)

        self.status_label.config(text=status)

        if self.alive:
            self.root.after(150, self._redraw)

    # ============================================================
    # নিরাপদ বন্ধ করা
    # ============================================================
    def on_quit(self):
        self.alive = False
        self.status_label.config(text="বন্ধ করা হচ্ছে...")
        self._wait_for_shutdown(0)

    def _wait_for_shutdown(self, attempts):
        if not self.sensor_thread.is_alive():
            self.root.destroy()
            return
        if attempts > 50:  # ~৫ সেকেন্ড
            self._cleanup_hardware()
            self.root.destroy()
            return
        self.root.after(100, lambda: self._wait_for_shutdown(attempts + 1))


def main():
    cal = load_calibration()
    cam_offset = cal.get("camera_offset_m", {})
    if (cal.get("camera_hfov_deg") is None or cal.get("camera_angle_offset_deg") is None
            or cam_offset.get("x") is None or cam_offset.get("y") is None):
        print("!!! calibration.json এ camera_hfov_deg/camera_angle_offset_deg/camera_offset_m নেই।")
        print("!!! calibrate_camera_hfov.py আগে চালান।")
        return

    root = tk.Tk()
    ArenaVisualizer(root, cal)
    root.mainloop()


if __name__ == "__main__":
    main()
