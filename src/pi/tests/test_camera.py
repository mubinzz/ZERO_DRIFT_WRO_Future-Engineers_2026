"""
============================================================
TEST 7: Pi Camera Module 3 টেস্ট
============================================================
উদ্দেশ্য: ক্যামেরা ছবি তুলতে পারছে কিনা, আর OpenCV দিয়ে সেটা
প্রসেস করা যাচ্ছে কিনা যাচাই করা। এখানে লাল/সবুজ pillar detection
এখনো করছি না (সেটা Phase 4 তে), শুধু raw ছবি তোলা আর একটা preview
ফাইলে সেভ করা হচ্ছে।

⚠️ Raspberry Pi 5 তে Camera Module 3 চালাতে picamera2 লাইব্রেরি
লাগবে (পুরনো picamera লাইব্রেরি Pi 5 এর libcamera stack এ কাজ
করে না)। এটা সাধারণত Raspberry Pi OS এ আগে থেকেই ইনস্টল থাকে,
না থাকলে:
    sudo apt install python3-picamera2 python3-opencv

চালানোর নিয়ম:
    python3 test_camera.py

এটা চালালে "camera_test.jpg" নামে একটা ছবি সেভ হবে এই ফোল্ডারেই,
সেটা খুলে দেখুন ছবি ঠিকমতো এসেছে কিনা (ফোকাস, রঙ, ওরিয়েন্টেশন)।

সমস্যা হলে:
- "No cameras available" এলে: raspi-config দিয়ে camera interface
  enable করা আছে কিনা দেখুন, আর ক্যাবল সঠিকভাবে (নীল সাইড ঠিক দিকে)
  লাগানো আছে কিনা চেক করুন।
- ছবি উল্টো/আয়না হয়ে আসলে: ক্যামেরা মাউন্ট করার সময় ফিজিক্যালি
  উল্টো লাগানো হয়ে থাকতে পারে, config এ vflip/hflip সেট করে অথবা
  মাউন্টিং ঠিক করে সমাধান করুন।
============================================================
"""

from picamera2 import Picamera2
import cv2
import time

def main():
    picam2 = Picamera2()

    # video-friendly ছোট রেজোলিউশন বেছে নেওয়া হয়েছে, কারণ পরে
    # real-time প্রসেসিং (Phase 4) এ বড় রেজোলিউশন Pi কে স্লো করে দেবে
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()

    print("ক্যামেরা চালু হচ্ছে, ২ সেকেন্ড warm-up দেওয়া হচ্ছে...")
    time.sleep(2)  # অটো-এক্সপোজার/হোয়াইট-ব্যালান্স স্থির হতে সময় লাগে

    frame = picam2.capture_array()
    print(f"ছবি তোলা হয়েছে। Shape: {frame.shape}")  # (480, 640, 3) হওয়া উচিত

    # ⚠️ Picamera2-এর "RGB888" ফরম্যাট নাম বিভ্রান্তিকর -- capture_array()
    # আসলে ইতিমধ্যেই BGR অর্ডারে array রিটার্ন করে (ইচ্ছাকৃতভাবে, যাতে
    # সরাসরি OpenCV এর সাথে কাজ করে)। তাই এখানে আর cv2.cvtColor দিয়ে
    # RGB->BGR রূপান্তর করা হচ্ছে না -- করলে Red/Blue চ্যানেল উল্টে
    # যাবে (এই বাগের কারণেই আগে লাল রঙ বেগুনি/নীলচে দেখাচ্ছিল)।
    bgr = frame
    cv2.imwrite("camera_test.jpg", bgr)
    print("ছবি সেভ হয়েছে: camera_test.jpg (এই ফাইলটা খুলে চেক করুন)")

    # একটা ছোট লাইভ লুপ — ৫ সেকেন্ড ধরে FPS মেপে দেখাবে
    print("\n৫ সেকেন্ড ধরে FPS মাপা হচ্ছে...")
    frame_count = 0
    start = time.time()
    while time.time() - start < 5:
        frame = picam2.capture_array()
        frame_count += 1

    elapsed = time.time() - start
    fps = frame_count / elapsed
    print(f"গড় FPS: {fps:.1f} (real-time lane/pillar detection এর জন্য "
          f"কমপক্ষে ১৫-২০ FPS থাকা ভালো)")

    picam2.stop()
    print("ক্যামেরা বন্ধ করা হয়েছে।")


if __name__ == "__main__":
    main()
