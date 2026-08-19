"""
============================================================
CALIBRATION 5: Open Challenge টিউনিং GUI (Tkinter -- VNC-friendly)
============================================================
কেন এটা tune_open_challenge_live.py (OpenCV ভার্সন) এর চেয়ে ভালো:
OpenCV এর cv2.imshow() প্রতি frame এ পুরো ছবি নতুন করে আঁকে, আর VNC
সেই পুরো ছবিটা নেটওয়ার্কে পাঠায় -- তাই স্লাইডার নাড়ানো ল্যাগ করে।
Tkinter native widget ব্যবহার করে, শুধু যেটুকু বদলায় সেটুকুই রিড্র
হয় -- VNC তে অনেক হালকা ও মসৃণ।

⚠️⚠️⚠️ নিরাপত্তা ⚠️⚠️⚠️
- START চাপার আগে robot নড়বে না। START চাপলেই চলা শুরু করবে।
- STOP চাপলে সাথে সাথে motor বন্ধ (কিন্তু LIDAR/servo চালু থাকবে,
  আবার START চাপলে চলবে)।
- window বন্ধ করলে (X) বা QUIT চাপলে motor বন্ধ হয়ে সব পরিষ্কারভাবে
  ছাড়া হবে।
- robot এর পাশে থাকুন, প্রয়োজনে হাতে ধরে থামাতে প্রস্তুত থাকুন।

চালানোর নিয়ম:
    cd ~/Downloads/WRO2026-MyRobot/pi/calibration
    python3 tune_open_challenge_gui.py

⚠️ yaw sign convention আগেই যাচাই করা হয়েছে (ডানে ঘোরালে yaw কমে),
তাই নিচে YAW_INCREASES_CLOCKWISE = False রাখা আছে। robot এ MPU6050
এর অবস্থান বদলালে এটা আবার যাচাই করতে হবে।

একটা কাজের ফিচার: STOP অবস্থাতেও (motor বন্ধ) LIDAR চালু থাকে আর
servo সাড়া দেয় -- তাই robot হাতে ধরে এদিক-ওদিক নাড়ালে steering
কেমন প্রতিক্রিয়া দেখায় সেটা robot না চালিয়েই পরীক্ষা করা যায়।
এটা KP/KD এর প্রাথমিক ধারণা পেতে খুব কাজে লাগে।
============================================================
"""

import sys
import os
import time
import threading
import tkinter as tk
from tkinter import ttk

from rplidar import RPLidar, RPLidarException
from serial import SerialException

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import load_calibration, save_calibration
from serial_link import SerialLink
from perception.lidar_processor import get_sector_distances
from control.wall_following import WallFollower
from control.corner_turn import CornerTurnController
from control.open_challenge_navigator import OpenChallengeNavigator

LIDAR_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_19ad04198e596140a9e96e563d38c7ae-if00-port0"
LIDAR_BAUDRATE = 256000
ESP32_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

# যাচাই করা হয়েছে: ডানে (clockwise) ঘোরালে yaw কমে
YAW_INCREASES_CLOCKWISE = False

# প্রতিটা প্যারামিটার: (calibration key, লেবেল, min, max, step, পূর্ণসংখ্যা কিনা)
PARAM_SPECS = [
    ("speed",                     "Speed (motor PWM)",        0,    255,  5,    True),
    ("kp",                        "KP (proportional)",        0.0,  20.0, 0.1,  False),
    ("kd",                        "KD (derivative)",          0.0,  5.0,  0.05, False),
    ("smoothing_alpha",           "Smoothing (ছোট=বেশি মসৃণ)", 0.05, 1.0,  0.05, False),
    ("max_angle_change_per_sec",  "Max angle rate (deg/sec)", 10,   400,  5,    True),
    ("front_trigger_m",           "Front trigger (m)",        0.05, 1.5,  0.01, False),
    ("front_confirm_frames",      "Front confirm (frames)",   1,    10,   1,    True),
    ("target_yaw_change_deg",     "Target yaw change (deg)",  30,   120,  1,    True),
    ("target_gap_m",              "Target gap (m)",           0.05, 1.0,  0.01, False),
    ("max_wall_distance_m",       "Max wall distance (m)",    0.2,  3.0,  0.05, False),
    ("max_error_m",               "Max error (m)",            0.1,  2.0,  0.05, False),
    ("max_turn_seconds",          "Turn timeout (sec)",       1.0,  10.0, 0.5,  False),
]


class TuningApp:
    def __init__(self, root):
        self.root = root
        self.root.title("WRO Open Challenge — Tuning")

        self.cal = load_calibration()
        oc = self.cal["open_challenge"]

        # ---- থ্রেড-শেয়ার্ড অবস্থা ----
        # Tkinter থ্রেড-নিরাপদ না, তাই GUI থ্রেড আর control থ্রেড সরাসরি
        # একে অপরের ডেটা ছোঁয় না। GUI থ্রেড প্রতি 100ms এ tk variable
        # গুলো পড়ে একটা সাধারণ dict এ কপি করে (lock দিয়ে সুরক্ষিত), আর
        # control থ্রেড সেই dict পড়ে। উল্টোদিকে control থ্রেড status
        # লিখে, GUI থ্রেড সেটা পড়ে লেবেলে দেখায়।
        self.lock = threading.Lock()
        self.shared_params = {}
        self.shared_status = {"text": "শুরু হচ্ছে..."}
        self.motor_enabled = False   # START/STOP এর অবস্থা
        self.alive = True
        self.save_message_until = 0.0  # SAVE নিশ্চিতকরণ বার্তা কতক্ষণ দেখানো হবে

        # হার্ডওয়্যার হ্যান্ডেল -- control থ্রেডে তৈরি হয়, কিন্তু জরুরি
        # অবস্থায় (control থ্রেড আটকে গেলে) main থ্রেড থেকেও বন্ধ করা
        # যায় বলে instance attribute হিসেবে রাখা হয়েছে
        self.link = None
        self.lidar = None

        self.vars = {}
        self.last_good = {}

        self._build_ui(oc)
        self._sync_params()  # প্রথমবার, তারপর নিজে থেকেই বারবার চলবে

        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_quit)
        self._refresh_status()

    # ============================================================
    # UI তৈরি
    # ============================================================
    def _build_ui(self, oc):
        param_frame = ttk.LabelFrame(self.root, text="প্যারামিটার (START এর আগে/পরে যেকোনো সময় বদলানো যায়)")
        param_frame.grid(row=0, column=0, padx=10, pady=10, sticky="ew")

        for i, (key, label, lo, hi, step, is_int) in enumerate(PARAM_SPECS):
            initial = oc.get(key, lo)
            var = tk.DoubleVar(value=float(initial))
            self.vars[key] = var
            self.last_good[key] = float(initial)

            ttk.Label(param_frame, text=label, width=26, anchor="w").grid(
                row=i, column=0, padx=4, pady=3, sticky="w")

            # ◀ বোতাম: এক step কমায়
            ttk.Button(param_frame, text="◀", width=3,
                       command=lambda k=key, s=step: self._nudge(k, -s)).grid(
                row=i, column=1, padx=2)

            scale = tk.Scale(param_frame, from_=lo, to=hi, resolution=step,
                             orient=tk.HORIZONTAL, variable=var,
                             showvalue=False, length=220)
            scale.grid(row=i, column=2, padx=4)

            # ▶ বোতাম: এক step বাড়ায়
            ttk.Button(param_frame, text="▶", width=3,
                       command=lambda k=key, s=step: self._nudge(k, s)).grid(
                row=i, column=3, padx=2)

            # সরাসরি সংখ্যা টাইপ করার ঘর -- টাইপ করে Enter চাপলে কার্যকর হবে
            entry = ttk.Entry(param_frame, textvariable=var, width=8, justify="right")
            entry.grid(row=i, column=4, padx=4)
            entry.bind("<Return>", lambda e, k=key: self._validate(k))
            entry.bind("<FocusOut>", lambda e, k=key: self._validate(k))

        # ---- নিয়ন্ত্রণ বোতাম ----
        btn_frame = ttk.Frame(self.root)
        btn_frame.grid(row=1, column=0, padx=10, pady=(0, 10), sticky="ew")

        self.start_btn = tk.Button(btn_frame, text="START", width=12, height=2,
                                   bg="#2e7d32", fg="white", font=("Arial", 11, "bold"),
                                   command=self.on_start)
        self.start_btn.grid(row=0, column=0, padx=5)

        self.stop_btn = tk.Button(btn_frame, text="STOP", width=12, height=2,
                                  bg="#c62828", fg="white", font=("Arial", 11, "bold"),
                                  command=self.on_stop, state=tk.DISABLED)
        self.stop_btn.grid(row=0, column=1, padx=5)

        tk.Button(btn_frame, text="SAVE VALUES", width=14, height=2,
                  command=self.on_save).grid(row=0, column=2, padx=5)

        tk.Button(btn_frame, text="QUIT", width=10, height=2,
                  command=self.on_quit).grid(row=0, column=3, padx=5)

        # ---- লাইভ অবস্থা ----
        status_frame = ttk.LabelFrame(self.root, text="লাইভ অবস্থা")
        status_frame.grid(row=2, column=0, padx=10, pady=(0, 10), sticky="ew")

        self.status_label = tk.Label(status_frame, text="শুরু হচ্ছে...",
                                     font=("Courier", 11), justify="left", anchor="w")
        self.status_label.grid(row=0, column=0, padx=8, pady=8, sticky="w")

    # ============================================================
    # প্যারামিটার হ্যান্ডলিং
    # ============================================================
    def _clamp(self, key, value):
        for k, _, lo, hi, _, is_int in PARAM_SPECS:
            if k == key:
                value = max(lo, min(hi, value))
                return round(value) if is_int else round(value, 3)
        return value

    def _nudge(self, key, delta):
        try:
            current = self.vars[key].get()
        except tk.TclError:
            current = self.last_good[key]
        self.vars[key].set(self._clamp(key, current + delta))

    def _validate(self, key):
        """Entry তে হাতে টাইপ করা মান যাচাই করে সীমার মধ্যে আনে।
        উল্টোপাল্টা কিছু টাইপ করলে আগের ভালো মানে ফিরে যায়।"""
        try:
            value = self.vars[key].get()
        except tk.TclError:
            value = self.last_good[key]
        self.vars[key].set(self._clamp(key, value))

    def _sync_params(self):
        """GUI থ্রেডে চলে: tk variable গুলো পড়ে সাধারণ dict এ কপি করে,
        যাতে control থ্রেড নিরাপদে পড়তে পারে।"""
        values = {}
        for key, _, _, _, _, is_int in PARAM_SPECS:
            try:
                v = self.vars[key].get()
            except tk.TclError:
                v = self.last_good[key]  # টাইপ করার মাঝপথে অবৈধ থাকলে
            v = self._clamp(key, v)
            self.last_good[key] = v
            values[key] = int(v) if is_int else float(v)

        with self.lock:
            self.shared_params = values

        if self.alive:
            self.root.after(100, self._sync_params)

    # ============================================================
    # বোতামের কাজ
    # ============================================================
    def on_start(self):
        with self.lock:
            self.motor_enabled = True
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)

    def on_stop(self):
        with self.lock:
            self.motor_enabled = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)

    def on_save(self):
        with self.lock:
            values = dict(self.shared_params)
        updated = save_calibration({"open_challenge": values})
        print(f"সেভ হয়েছে: {updated['open_challenge']}")
        # ৩ সেকেন্ডের জন্য নিশ্চিতকরণ বার্তা দেখানো হবে (নাহলে পরের
        # status refresh এ সাথে সাথেই মুছে যেত, চোখে পড়ত না)
        self.save_message_until = time.time() + 3.0

    def on_quit(self):
        """⚠️ এখানে সাথে সাথে window destroy করা যাবে না। control থ্রেড
        daemon=True, তাই main program শেষ হওয়ামাত্র সেটা মাঝপথে জোর করে
        মেরে ফেলা হয় -- তখন তার finally: ব্লকের lidar.stop_motor()
        কখনো চলার সুযোগই পায় না, ফলে LIDAR ঘুরতেই থাকে। তাই control
        থ্রেড নিজে থেকে cleanup শেষ করা পর্যন্ত অপেক্ষা করতে হবে।"""
        self.alive = False
        with self.lock:
            self.motor_enabled = False
        self.status_label.config(text="বন্ধ করা হচ্ছে...\nLIDAR থামানো হচ্ছে, একটু অপেক্ষা করুন।")
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)
        self._wait_for_shutdown(0)

    def _wait_for_shutdown(self, attempts):
        if not self.control_thread.is_alive():
            self.root.destroy()
            return

        if attempts > 50:  # ~৫ সেকেন্ড অপেক্ষার পরেও না থামলে
            # শেষ চেষ্টা: main থ্রেড থেকেই সরাসরি হার্ডওয়্যার বন্ধ করা
            print("!!! control থ্রেড সময়মতো থামেনি -- জোর করে হার্ডওয়্যার বন্ধ করা হচ্ছে")
            self._force_cleanup()
            self.root.destroy()
            return

        self.root.after(100, lambda: self._wait_for_shutdown(attempts + 1))

    def _force_cleanup(self):
        """control থ্রেড আটকে গেলে main থ্রেড থেকে হার্ডওয়্যার বন্ধ করার
        শেষ উপায়। দুইবার cleanup হলেও ক্ষতি নেই (সব try/except এ মোড়ানো)।"""
        if self.link is not None:
            try:
                self.link.send_command(0, self.cal["servo_center"])
                time.sleep(0.1)
                self.link.close()
            except Exception:
                pass
        if self.lidar is not None:
            for step in (self.lidar.stop, self.lidar.stop_motor, self.lidar.disconnect):
                try:
                    step()
                except Exception:
                    pass

    # ============================================================
    # লাইভ অবস্থা দেখানো (GUI থ্রেড)
    # ============================================================
    def _refresh_status(self):
        with self.lock:
            text = self.shared_status.get("text", "")
        if time.time() < self.save_message_until:
            text = "✓ calibration.json এ সেভ হয়েছে!\n\n" + text
        self.status_label.config(text=text)
        if self.alive:
            self.root.after(150, self._refresh_status)

    # ============================================================
    # নিয়ন্ত্রণ লুপ (আলাদা থ্রেড -- GUI যেন আটকে না যায়)
    # ============================================================
    def _control_loop(self):
        try:
            with self.lock:
                self.shared_status["text"] = "ESP32 এর সাথে কানেক্ট হচ্ছে..."
            link = SerialLink(ESP32_PORT)
            self.link = link
            time.sleep(1)

            with self.lock:
                self.shared_status["text"] = "LIDAR এর সাথে কানেক্ট হচ্ছে..."
            lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
            self.lidar = lidar
            lidar.get_info()
            try:
                lidar.clean_input()
            except AttributeError:
                lidar._serial_port.reset_input_buffer()
            time.sleep(2)

            wall_follower = WallFollower(
                servo_center=self.cal["servo_center"],
                servo_left_max=self.cal["servo_left_max"],
                servo_right_max=self.cal["servo_right_max"],
            )
            corner_turn = CornerTurnController(
                servo_center=self.cal["servo_center"],
                servo_left_max=self.cal["servo_left_max"],
                servo_right_max=self.cal["servo_right_max"],
            )
            navigator = OpenChallengeNavigator(wall_follower, corner_turn)

            scan_iterator = lidar.iter_scans()
            consecutive_bad = 0

            while self.alive:
                try:
                    scan = next(scan_iterator)
                except RPLidarException:
                    consecutive_bad += 1
                    if consecutive_bad > 8:
                        with self.lock:
                            self.shared_status["text"] = "!!! LIDAR বারবার খারাপ ডেটা দিচ্ছে -- cable/power চেক করুন"
                        break
                    scan_iterator = lidar.iter_scans()
                    continue
                consecutive_bad = 0

                if not scan:
                    continue

                with self.lock:
                    p = dict(self.shared_params)
                    motor_on = self.motor_enabled

                if not p:
                    continue

                # লাইভ প্যারামিটার প্রয়োগ
                wall_follower.pid.kp = p["kp"]
                wall_follower.pid.kd = p["kd"]
                wall_follower.smoothing_alpha = p["smoothing_alpha"]
                wall_follower.max_angle_change_per_sec = p["max_angle_change_per_sec"]
                wall_follower.target_gap_m = p["target_gap_m"]
                wall_follower.max_wall_distance_m = p["max_wall_distance_m"]
                wall_follower.max_error_m = p["max_error_m"]
                corner_turn.front_trigger_m = p["front_trigger_m"]
                corner_turn.front_confirm_frames = p["front_confirm_frames"]
                corner_turn.target_yaw_change_deg = p["target_yaw_change_deg"]
                corner_turn.max_turn_seconds = p["max_turn_seconds"]

                telem = link.get_telemetry()
                raw_yaw = telem["yaw"]
                yaw = raw_yaw if YAW_INCREASES_CLOCKWISE else -raw_yaw

                angle = navigator.compute_steering_angle(scan, yaw)

                # STOP অবস্থায় speed=0 পাঠানো হয় কিন্তু steering angle
                # ঠিকই পাঠানো হয় -- তাই robot না চালিয়েও হাতে নাড়িয়ে
                # steering প্রতিক্রিয়া দেখা যায়
                link.send_command(p["speed"] if motor_on else 0, angle)

                sectors = get_sector_distances(scan)

                def fmt(v):
                    return f"{v:.2f}m" if v is not None else "  --  "

                state = "TURN" if corner_turn.is_turning() else "WALL"
                run_state = "▶ চলছে" if motor_on else "■ থেমে আছে"

                timeout_note = ""
                if corner_turn.last_turn_timed_out:
                    timeout_note = "  ⚠ শেষ turn timeout এ থেমেছে (yaw যথেষ্ট বদলায়নি)"

                status_text = (
                    f"{run_state}    মোড: {state}{timeout_note}\n"
                    f"Servo angle : {angle}\n"
                    f"Yaw         : {yaw:.1f}°\n"
                    f"Front       : {fmt(sectors['front'])}\n"
                    f"Left        : {fmt(sectors['left'])}    "
                    f"Right : {fmt(sectors['right'])}\n"
                    f"Encoder     : {telem['encoder']}"
                )
                with self.lock:
                    self.shared_status["text"] = status_text

        except SerialException as e:
            with self.lock:
                self.shared_status["text"] = f"!!! সিরিয়াল সংযোগ সমস্যা: {e}\n(পাওয়ার/তার চেক করুন)"
        except Exception as e:
            with self.lock:
                self.shared_status["text"] = f"!!! সমস্যা: {e}"
        finally:
            # সবসময় নিরাপদে থামানো (_force_cleanup ব্যবহার করা হচ্ছে যাতে
            # একই cleanup কোড দুই জায়গায় ডুপ্লিকেট না হয়)
            self._force_cleanup()
            print("Robot ও LIDAR নিরাপদে বন্ধ করা হয়েছে।")


def main():
    root = tk.Tk()
    TuningApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
