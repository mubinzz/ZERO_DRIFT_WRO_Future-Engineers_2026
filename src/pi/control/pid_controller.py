"""
============================================================
pid_controller.py — সাধারণ, reusable PID কন্ট্রোলার
============================================================
PID (Proportional-Integral-Derivative) একটা "error" (লক্ষ্য মান
আর বর্তমান মানের পার্থক্য) নিয়ে সেটাকে শূন্যে আনার জন্য একটা
correction/output বের করে। তিনটা অংশ:

  - P (Proportional): error যত বড়, correction তত বড় -- সরাসরি
    আনুপাতিক প্রতিক্রিয়া। এটাই মূল অংশ।
  - I (Integral): error সময়ের সাথে জমতে থাকে -- persistent/স্থায়ী
    bias (যেমন robot-এর যান্ত্রিক অসামঞ্জস্যের কারণে সবসময় একটু
    বামে হেলে থাকা) ঠিক করতে সাহায্য করে।
  - D (Derivative): error কত দ্রুত বদলাচ্ছে -- হঠাৎ বড় correction
    দিয়ে overshoot (প্রয়োজনের চেয়ে বেশি ঘুরে যাওয়া) কমায়।

এই ক্লাসটা generic -- wall-following স্টিয়ারিং এ ব্যবহার হবে,
ভবিষ্যতে speed control বা অন্য কিছুতেও reuse করা যাবে।
============================================================
"""

import time


class PIDController:
    def __init__(self, kp, ki, kd, output_min=None, output_max=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None

    def reset(self):
        """নতুন রাউন্ড শুরুর আগে internal state মুছে ফেলা -- না করলে
        আগের রাউন্ডের জমে থাকা integral/error নতুন রাউন্ডে ভুল
        correction দিতে পারে।"""
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None

    def update(self, error):
        """error দিয়ে নতুন correction/output হিসাব করে।

        error : লক্ষ্য মান - বর্তমান মান (ধনাত্মক/ঋণাত্মক দুই দিকেই
                হতে পারে)
        রিটার্ন: correction (output_min/max দেওয়া থাকলে সেই রেঞ্জে
                 clamp করা হবে)
        """
        now = time.time()
        dt = 0.0 if self._last_time is None else (now - self._last_time)
        self._last_time = now

        self._integral += error * dt
        derivative = (error - self._last_error) / dt if dt > 0 else 0.0
        self._last_error = error

        output = (self.kp * error) + (self.ki * self._integral) + (self.kd * derivative)

        if self.output_min is not None:
            output = max(self.output_min, output)
        if self.output_max is not None:
            output = min(self.output_max, output)

        return output
