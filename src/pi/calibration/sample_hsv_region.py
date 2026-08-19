"""
============================================================
CALIBRATION 3b: ছবির একটা অংশ থেকে HSV রঙ থ্রেশহোল্ড বের করা
============================================================
উদ্দেশ্য: capture_sample.py দিয়ে তোলা ছবির মধ্যে pillar-এর একটা
ছোট আয়তক্ষেত্র (box) বেছে নিয়ে, সেই অংশের HSV মান বিশ্লেষণ করে
lower/upper threshold সাজেস্ট করা এবং calibration.json এ সেভ করা।

কীভাবে box কো-অর্ডিনেট বের করবেন (Pi তে GUI নেই বলে):
1. capture_sample.py দিয়ে তোলা red_sample.jpg/green_sample.jpg
   scp দিয়ে Windows এ আনুন।
2. MS Paint (বা যেকোনো ইমেজ এডিটর) দিয়ে খুলুন।
3. Paint এ cursor pillar এর উপর রাখলে নিচে status bar এ (x,y)
   পিক্সেল কো-অর্ডিনেট দেখা যায়। pillar এর ভেতরের (কিনারা থেকে
   একটু দূরে, যাতে background/ছায়া না ঢোকে) উপরের-বাম আর
   নিচের-ডান কোণার কো-অর্ডিনেট নোট করুন।

ব্যবহার:
    python3 sample_hsv_region.py <image.jpg> <x1> <y1> <x2> <y2> <red|green>

উদাহরণ:
    python3 sample_hsv_region.py red_sample.jpg 280 150 320 190 red

⚠️ লাল রঙ নিয়ে একটা বিশেষ ব্যাপার: HSV তে Hue (H) এর মান 0-179
এর একটা বৃত্তাকার (circular) স্কেল, আর লাল রঙ ঠিক এই স্কেলের ০/১৭৯
সীমানায় থাকে (H≈0 আর H≈179 দুটোই লাল)। তাই যদি নিচের আউটপুটে H
এর std (বিচ্যুতি) অস্বাভাবিক বড় দেখায়, সেটা এই "wraparound"
সমস্যার লক্ষণ হতে পারে -- তখন single lower/upper range যথেষ্ট
নাও হতে পারে (KMIDS এর কোডে তাই লাল রঙের জন্য দুটো আলাদা HSV
রেঞ্জ ব্যবহার হয়েছিল, ০ এর কাছে একটা আর ১৭৯ এর কাছে একটা)।
============================================================
"""

import sys
import os
import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import save_calibration


def main():
    if len(sys.argv) != 7:
        print("ব্যবহার: python3 sample_hsv_region.py <image.jpg> <x1> <y1> <x2> <y2> <red|green>")
        sys.exit(1)

    image_path = sys.argv[1]
    x1, y1, x2, y2 = (int(v) for v in sys.argv[2:6])
    color_name = sys.argv[6].lower()

    if color_name not in ("red", "green"):
        print("color name অবশ্যই 'red' অথবা 'green' হতে হবে")
        sys.exit(1)

    img = cv2.imread(image_path)
    if img is None:
        print(f"ছবি খোলা গেল না: {image_path} (path ঠিক আছে কিনা দেখুন)")
        sys.exit(1)

    h, w = img.shape[:2]
    x1, x2 = sorted((max(0, min(x1, w - 1)), max(0, min(x2, w - 1))))
    y1, y2 = sorted((max(0, min(y1, h - 1)), max(0, min(y2, h - 1))))

    region_bgr = img[y1:y2, x1:x2]
    if region_bgr.size == 0:
        print("খালি region পাওয়া গেল! x1,y1,x2,y2 কো-অর্ডিনেট আবার চেক করুন।")
        sys.exit(1)

    region_hsv = cv2.cvtColor(region_bgr, cv2.COLOR_BGR2HSV)
    h_vals = region_hsv[:, :, 0].astype(np.float32).flatten()
    s_vals = region_hsv[:, :, 1].astype(np.float32).flatten()
    v_vals = region_hsv[:, :, 2].astype(np.float32).flatten()

    print(f"নমুনা অঞ্চল: ({x1},{y1}) - ({x2},{y2}), মোট {h_vals.size} পিক্সেল\n")
    print(f"H: min={h_vals.min():.0f} max={h_vals.max():.0f} mean={h_vals.mean():.1f} std={h_vals.std():.1f}")
    print(f"S: min={s_vals.min():.0f} max={s_vals.max():.0f} mean={s_vals.mean():.1f} std={s_vals.std():.1f}")
    print(f"V: min={v_vals.min():.0f} max={v_vals.max():.0f} mean={v_vals.mean():.1f} std={v_vals.std():.1f}")

    if color_name == "red" and h_vals.std() > 40:
        print(
            "\n⚠️ H এর std অস্বাভাবিক বড় -- সম্ভবত hue wraparound সমস্যা "
            "(কিছু পিক্সেল H≈0 এর কাছে, কিছু H≈179 এর কাছে)। নিচের "
            "সাজেস্ট করা রেঞ্জটা ভুল/অতিরিক্ত চওড়া হতে পারে। docstring এ "
            "লেখা wraparound সমাধান বিবেচনা করুন।"
        )

    def suggest_bound(vals, low_ok, high_ok, margin=2.5):
        lo = int(max(low_ok, vals.mean() - margin * vals.std() - 5))
        hi = int(min(high_ok, vals.mean() + margin * vals.std() + 5))
        return lo, hi

    h_lo, h_hi = suggest_bound(h_vals, 0, 179)
    s_lo, s_hi = suggest_bound(s_vals, 0, 255)
    v_lo, v_hi = suggest_bound(v_vals, 0, 255)

    print(f"\nসাজেস্ট করা threshold:")
    print(f"  lower = [{h_lo}, {s_lo}, {v_lo}]")
    print(f"  upper = [{h_hi}, {s_hi}, {v_hi}]")

    key = f"camera_{color_name}_hsv"
    updated = save_calibration({key: {"lower": [h_lo, s_lo, v_lo], "upper": [h_hi, s_hi, v_hi]}})
    print(f"\ncalibration.json এ সেভ হয়েছে ({key}): {updated[key]}")


if __name__ == "__main__":
    main()
