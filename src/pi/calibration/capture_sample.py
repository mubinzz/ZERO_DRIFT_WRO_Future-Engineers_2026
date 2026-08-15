"""
============================================================
CALIBRATION 3a: ক্যামেরা স্যাম্পল ছবি তোলা (red/green pillar এর জন্য)
============================================================
উদ্দেশ্য: red/green pillar কে ক্যামেরার সামনে রেখে একটা ছবি তুলে
সেভ করা, যেটা পরে sample_hsv_region.py দিয়ে বিশ্লেষণ করা হবে।

ব্যবহার:
    python3 capture_sample.py red_sample.jpg
    python3 capture_sample.py green_sample.jpg

পরামর্শ: প্রতিযোগিতার মাঠের মতোই আলো (lighting) থাকা অবস্থায় ছবি
তুলুন -- আলো বদলালে রঙ কেমন দেখায় সেটাও বদলে যায়, তাই বাসার আলোতে
ক্যালিব্রেট করলে মাঠে গিয়ে ভুল করতে পারে। pillar-টাকে robot থেকে
মোটামুটি সেই দূরত্বে রাখুন যে দূরত্ব থেকে আসল খেলায় শনাক্ত করতে হবে।
============================================================
"""

import sys
import time
from picamera2 import Picamera2
import cv2


def main():
    if len(sys.argv) != 2:
        print("ব্যবহার: python3 capture_sample.py <output_filename.jpg>")
        sys.exit(1)

    filename = sys.argv[1]

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()

    print("ক্যামেরা warm-up হচ্ছে (২ সেকেন্ড)...")
    time.sleep(2)

    frame = picam2.capture_array()
    # Picamera2-এর "RGB888" নাম বিভ্রান্তিকর -- capture_array() আসলে
    # ইতিমধ্যেই BGR অর্ডারে দেয় (OpenCV compatibility এর জন্য), তাই
    # আবার cv2.cvtColor দিয়ে convert করলে Red/Blue উল্টে যায়
    bgr = frame
    cv2.imwrite(filename, bgr)

    picam2.stop()
    print(f"ছবি সেভ হয়েছে: {filename} (আকার: {bgr.shape[1]}x{bgr.shape[0]})")
    print("এখন এই ফাইলটা scp দিয়ে Windows-এ এনে (MS Paint দিয়ে খুলে) pillar-এর")
    print("চারপাশের একটা ছোট অঞ্চলের pixel coordinate নোট করুন।")


if __name__ == "__main__":
    main()
