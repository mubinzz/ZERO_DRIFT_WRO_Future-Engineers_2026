"""
============================================================
TEST 9: camera_processor.py লাইভ যাচাই
============================================================
উদ্দেশ্য: calibration.json এর HSV ভ্যালু দিয়ে camera_processor.py
আসলে ঠিকমতো pillar (red/green ব্লক) ধরছে কিনা লাইভ দেখা। প্রতিটা
শনাক্ত হওয়া ব্লকের চারপাশে রঙিন বক্স আর area/center লেখা দেখাবে।

⚠️ VNC/GUI দরকার (cv2.imshow ব্যবহার করছে)।

ব্যবহার:
    python3 test_camera_processor.py

বন্ধ করতে 'q' চাপুন।

যা যাচাই করবেন:
- লাল/সবুজ কার্ড/pillar ক্যামেরার সামনে ধরলে ঠিক তার চারপাশে বক্স
  আসছে কিনা
- background/অন্য বস্তুতে ভুলভাবে বক্স আসছে কিনা (false positive)
- কার্ড দূরে সরালে/কাছে আনলে area সংখ্যা বাড়ছে/কমছে কিনা (এটা পরে
  distance estimation এর কাজে লাগতে পারে)
- min_area থ্রেশহোল্ড (নিচে কোডে 200) খুব ছোট/বড় মনে হলে বদলে
  দেখুন -- ছোট noise ব্লকও ধরা পড়লে বাড়ান, দূরের ছোট pillar miss
  হলে কমান
============================================================
"""

import sys
import os
import time
import cv2
from picamera2 import Picamera2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import load_calibration
from perception.camera_processor import detect_blocks

MIN_AREA = 200

COLOR_DRAW = {
    "red": (0, 0, 255),    # BGR -- OpenCV আঁকার জন্য
    "green": (0, 255, 0),
}


def main():
    cal = load_calibration()

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()
    time.sleep(2)

    print("Camera detection চালু। 'q' চাপুন বন্ধ করতে।")

    try:
        while True:
            frame = picam2.capture_array()
            # Picamera2-এর "RGB888" নাম বিভ্রান্তিকর -- capture_array() আসলে
            # ইতিমধ্যেই BGR অর্ডারে দেয়, তাই আবার convert করা হচ্ছে না
            bgr = frame

            blocks = detect_blocks(bgr, cal, min_area=MIN_AREA)

            for color_name, draw_color in COLOR_DRAW.items():
                for blk in blocks[color_name]:
                    cv2.rectangle(bgr, (blk.x, blk.y), (blk.x + blk.w, blk.y + blk.h), draw_color, 2)
                    cv2.circle(bgr, (blk.center_x, blk.center_y), 4, draw_color, -1)
                    label = f"{blk.color} A={blk.area:.0f}"
                    cv2.putText(
                        bgr, label, (blk.x, max(0, blk.y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 2
                    )

            cv2.imshow("Detection (q = quit)", bgr)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
