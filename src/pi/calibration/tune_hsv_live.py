"""
============================================================
CALIBRATION 3c: লাইভ HSV ক্যালিব্রেশন (VNC/মনিটর দিয়ে GUI অ্যাক্সেস থাকলে)
============================================================
এটা capture_sample.py + sample_hsv_region.py এর চেয়ে অনেক দ্রুত ও
নির্ভুল পদ্ধতি -- কারণ ছবি তুলে বিশ্লেষণ করার বদলে সরাসরি live
camera feed দেখতে দেখতে trackbar দিয়ে থ্রেশহোল্ড টিউন করা যায়,
সাথে সাথে mask window-এ ফলাফল দেখা যায়।

⚠️ VNC-এর মাধ্যমে চালালে ছবি একটু ধীরে (laggy) আপডেট হতে পারে
(নেটওয়ার্কের মধ্য দিয়ে পুরো ছবি পাঠাতে হয়) -- ক্যালিব্রেশনের জন্য
এটা কোনো সমস্যা না, শুধু আসল robot এর real-time control loop এ এত
ধীরে চললে চলবে না (কিন্তু সেটা তো headless/direct চলবে, VNC দিয়ে না)।

ব্যবহার:
    python3 tune_hsv_live.py red
    python3 tune_hsv_live.py green

তিনটা window খুলবে:
    - "Original"          -> আসল ক্যামেরা ছবি
    - "Mask"               -> সাদা = detect হওয়া রঙ, কালো = বাকি সব
    - "HSV Tuning - <রঙ>"  -> trackbar + masked ফলাফল (শুধু detect
                              হওয়া অংশটুকু রঙিন, বাকি কালো)

কীবোর্ড কমান্ড (window সিলেক্ট থাকা অবস্থায়):
    s -> বর্তমান trackbar ভ্যালু calibration.json এ সেভ
    q -> বন্ধ

টিউনিং কৌশল:
- pillar-টা বাস্তব প্রতিযোগিতার দূরত্ব/কোণে রাখুন, সম্ভব হলে
  মাঠের কাছাকাছি আলোতে
- Mask window-এ pillar-এর জায়গাটা পরিষ্কার সাদা, বাকি সব (মেঝে,
  দেয়াল, ছায়া) কালো হওয়া পর্যন্त trackbar নাড়ান
- V min কমালে ছায়াও ধরা পড়তে পারে, বেশি বাড়ালে কম আলোয় pillar
  miss হতে পারে -- মাঝামাঝি একটা ভ্যালু খুঁজুন
- S min কম রাখলে ধূসর/সাদা জিনিসও ভুল করে "রঙিন" ধরতে পারে
- pillar কে একটু ঘুরিয়ে/দূরে-কাছে নিয়ে দেখুন mask স্থিতিশীল থাকে কিনা

⚠️ লাল রঙের জন্য বিশেষ সতর্কতা: HSV তে H (Hue) স্কেল বৃত্তাকার
(0-179), আর লাল রঙ ঠিক এই স্কেলের সীমানায় (0 এর কাছে ও 179 এর
কাছে দুটোতেই) থাকে। একটামাত্র H min/max রেঞ্জ দিয়ে পুরো লাল রঙ
নাও ধরা পড়তে পারে যদি আপনার ক্যামেরার লাল রঙ দুই প্রান্তেই ছড়িয়ে
থাকে। প্রথমে চেষ্টা করুন যেকোনো একটা প্রান্ত (0 এর কাছে, যেমন
H: 0-10) দিয়ে ঠিকমতো ধরা পড়ে কিনা -- সাধারণত এটাই যথেষ্ট হয়। না
হলে জানাবেন, তখন দুই-রেঞ্জ (dual-range) সাপোর্ট যোগ করে দেব।
============================================================
"""

import sys
import os
import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import load_calibration, save_calibration
from camera_link import create_camera


def nothing(x):
    pass


def main():
    if len(sys.argv) != 2 or sys.argv[1].lower() not in ("red", "green"):
        print("ব্যবহার: python3 tune_hsv_live.py <red|green>")
        sys.exit(1)

    color_name = sys.argv[1].lower()
    key = f"camera_{color_name}_hsv"

    cal = load_calibration()
    lower = cal[key]["lower"]
    upper = cal[key]["upper"]

    window = f"HSV Tuning - {color_name}"
    cv2.namedWindow(window)
    cv2.createTrackbar("H min", window, lower[0], 179, nothing)
    cv2.createTrackbar("H max", window, upper[0], 179, nothing)
    cv2.createTrackbar("S min", window, lower[1], 255, nothing)
    cv2.createTrackbar("S max", window, upper[1], 255, nothing)
    cv2.createTrackbar("V min", window, lower[2], 255, nothing)
    cv2.createTrackbar("V max", window, upper[2], 255, nothing)

    picam2 = create_camera(640, 480)

    print("Trackbar নাড়িয়ে 'Mask' window-এ pillar পরিষ্কার সাদা করুন।")
    print("সেভ করতে 's', বন্ধ করতে 'q' চাপুন (কোনো একটা window সিলেক্ট রেখে)।")

    try:
        while True:
            frame = picam2.capture_array()
            # Picamera2-এর "RGB888" নাম বিভ্রান্তিকর -- capture_array() আসলে
            # ইতিমধ্যেই BGR অর্ডারে দেয় (OpenCV compatibility এর জন্য), তাই
            # আবার cv2.cvtColor(...COLOR_RGB2BGR) করলে Red/Blue উল্টে যায়
            # (এই বাগের কারণেই আগে লাল রঙ বেগুনি/নীলচে দেখাচ্ছিল)
            bgr = frame
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

            h_min = cv2.getTrackbarPos("H min", window)
            h_max = cv2.getTrackbarPos("H max", window)
            s_min = cv2.getTrackbarPos("S min", window)
            s_max = cv2.getTrackbarPos("S max", window)
            v_min = cv2.getTrackbarPos("V min", window)
            v_max = cv2.getTrackbarPos("V max", window)

            # প্রতিটা trackbar (min, max) একে অপরের থেকে স্বাধীন -- তাই
            # স্লাইডার টানতে টানতে ভুলবশত max কে min এর চেয়ে ছোট করে
            # ফেলা সহজ। cv2.inRange() এ কোনো চ্যানেলের max < min হলে
            # কোনো error না দিয়েই mask পুরোপুরি কালো (খালি) হয়ে যায় --
            # তাই এখানে নিরাপদে min/max swap করে নেওয়া হচ্ছে
            h_lo, h_hi = min(h_min, h_max), max(h_min, h_max)
            s_lo, s_hi = min(s_min, s_max), max(s_min, s_max)
            v_lo, v_hi = min(v_min, v_max), max(v_min, v_max)

            lower_np = np.array([h_lo, s_lo, v_lo])
            upper_np = np.array([h_hi, s_hi, v_hi])
            mask = cv2.inRange(hsv, lower_np, upper_np)
            result = cv2.bitwise_and(bgr, bgr, mask=mask)

            cv2.imshow("Original", bgr)
            cv2.imshow("Mask", mask)
            cv2.imshow(window, result)

            k = cv2.waitKey(30) & 0xFF
            if k == ord('q'):
                break
            elif k == ord('s'):
                new_lower = [h_lo, s_lo, v_lo]
                new_upper = [h_hi, s_hi, v_hi]
                updated = save_calibration({key: {"lower": new_lower, "upper": new_upper}})
                print(f"সেভ হয়েছে ({key}): {updated[key]}")

    finally:
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
