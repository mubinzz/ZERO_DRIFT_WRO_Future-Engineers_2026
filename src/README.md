# WRO 2026 Future Engineers — My Robot (KMIDS-GFM architecture থেকে অনুপ্রাণিত)

এই ফোল্ডারটা আমার WRO 2026 Future Engineers সেলফ-ড্রাইভিং কার প্রজেক্টের পুরো কোডবেস। চ্যাসিস/মেকানিক্যাল ডিজাইন Team KMIDS-GFM এর 3D structure অনুসরণ করে বানানো হয়েছে, কিন্তু ইলেকট্রনিক্স আলাদা — তাই কোড থেকে শুরু করতে হচ্ছে।

## হার্ডওয়্যার লিস্ট

| Component | কাজ | সংযোগ |
|---|---|---|
| Raspberry Pi 5 | মেইন ব্রেইন — LIDAR + camera প্রসেসিং, সিদ্ধান্ত নেওয়া | — |
| RPLIDAR A3M1 | 360° দূরত্ব মাপা (wall detection, pillar detection) | USB (Pi-তে সরাসরি) |
| Pi Camera Module 3 | রঙ শনাক্ত করা (লাল/সবুজ pillar) | CSI (Pi-তে সরাসরি) |
| ESP32 (DevKit) | লো-লেভেল কন্ট্রোল — মোটর, সার্ভো, এনকোডার, IMU | USB serial (Pi-এর সাথে) |
| N20 300RPM encoder motor | ড্রাইভ মোটর (চাকা ঘোরানো) | ESP32 → TB6612FNG |
| TB6612FNG | মোটর ড্রাইভার | ESP32 |
| MG90S servo | স্টিয়ারিং | ESP32 |
| MPU6050 | IMU (heading/turn tracking) | ESP32 (I2C) |
| Push Button | Start button | ESP32 GPIO 5 (বোর্ডের "D5" লেবেল) |
| 3S 1100mAh 45C LiPo | পাওয়ার সোর্স (~11.1V nominal, ~12.6V full charge) | — |
| XL4016 buck converter | LiPo → 5V (Raspberry Pi 5 পাওয়ার) | — |
| LM2596 buck converter | LiPo → 5V/6V (Servo পাওয়ার, আলাদা rail) | — |

**কেন দুইটা আলাদা বোর্ড (Pi + ESP32)?** ঠিক KMIDS যেভাবে Raspberry Pi 5 + Pi Pico 2 ব্যবহার করেছে — ভারী প্রসেসিং (camera vision, LIDAR ম্যাপিং, decision making) Pi-তে হয়, আর real-time, latency-sensitive কাজ (মোটর PWM, এনকোডার পালস গোনা, সার্ভো পজিশন) ESP32-তে হয়। এতে Pi-এর OS-এর কারণে যে সামান্য timing jitter হয়, সেটা মোটর কন্ট্রোলে প্রভাব ফেলে না।

## Pin Map (ESP32 DevKit — আপনার actual wiring অনুযায়ী কনফিগ ফাইলে বদলে নিন)

```
TB6612FNG (মোটর ড্রাইভার):
  PWMA  -> GPIO 25
  AIN1  -> GPIO 26
  AIN2  -> GPIO 27
  STBY  -> GPIO 14

N20 Encoder (quadrature, 2 চ্যানেল):
  ENC_A -> GPIO 34   (input-only pin, ইন্টারাপ্টের জন্য ভালো)
  ENC_B -> GPIO 35

MG90S Servo:
  SIGNAL -> GPIO 13

MPU6050 (I2C):
  SDA -> GPIO 21
  SCL -> GPIO 22

Push Button (ESP32, Pi না):
  GPIO 5 (বোর্ডের "D5" লেবেল) -> বাটনের এক পা, অন্য পা GND-এ,
  internal pull-up ব্যবহার হবে (কোড INPUT_PULLUP সেট করবে)
```

> ⚠️ এই পিন নাম্বারগুলো standard suggestion। আপনি যেভাবে সোল্ডার করেছেন সেটা অন্যরকম হলে, প্রতিটা ফাইলের উপরে থাকা `// ---- PIN CONFIG ----` সেকশনে গিয়ে বদলে নিন। পুরো কোডে একই জায়গায় pin define করা আছে, তাই এক জায়গায় বদলালেই চলবে।

## Communication Protocol (Pi ↔ ESP32)

সহজ, মানুষের চোখে পড়া যায় এমন text-based protocol, USB serial দিয়ে, baud rate `115200`।

**Pi থেকে ESP32-তে (command):**
```
M:<speed>,S:<angle>\n
```
- `speed` = -255 থেকে 255 (নেগেটিভ = পেছনে, পজিটিভ = সামনে, 0 = ব্রেক)
- `angle` = 0-180 (90 = সোজা, <90 = বামে, >90 = ডানে — calibration-এর পর ঠিক হবে)

উদাহরণ: `M:150,S:110\n` → মোটর ফরওয়ার্ড স্পিড 150, স্টিয়ারিং ডানদিকে।

**ESP32 থেকে Pi-তে (telemetry, প্রতি ~20ms):**
```
ENC:<encoder_count>,YAW:<yaw_degrees>,BTN:<0_or_1>\n
```
`BTN` ফিল্ড যোগ হয়েছে কারণ Start button এখন ESP32-তে (GPIO 5) — তাই
Pi-এর main challenge script গুলো WRO rule 9.10-9.11 অনুযায়ী "start
button চাপা হলে দৌড় শুরু করো" এই সিদ্ধান্তটা এই ফিল্ড দেখে নেবে, নিজের
GPIO থেকে না।

এই protocol টেক্সট-বেসড রাখার কারণ — আপনি সরাসরি Arduino Serial Monitor বা `screen`/PuTTY দিয়ে raw ডেটা চোখে দেখতে পারবেন, ডিবাগ করা সহজ হবে।

## Phase Roadmap (ধাপে ধাপে যেভাবে এগোবো)

1. **Phase 1 — হার্ডওয়্যার টেস্ট** ✅ সম্পূর্ণ: motor, servo, encoder, MPU6050, button, LIDAR, camera — সবগুলো আলাদাভাবে যাচাই হয়েছে।
2. **Phase 2 — ইউনিফায়েড ফার্মওয়্যার + serial link** ✅ সম্পূর্ণ: `esp32_firmware/main_controller/main_controller.ino` + `pi/serial_link.py` + `pi/tests/test_serial_link.py`।
3. **Phase 3 — ক্যালিব্রেশন** 🔧 (এই ধাপে আছি): servo center/limit ও motor deadzone সম্পূর্ণ (`calibration.json`-এ সেভ)। ক্যামেরা HSV-এর জন্য দুটো পদ্ধতি আছে —
   - **`tune_hsv_live.py <red|green>`** (recommended, VNC/মনিটর থাকলে): live trackbar দিয়ে সরাসরি টিউনিং।
   - **`capture_sample.py` + `sample_hsv_region.py`** (headless/SSH-only fallback): ছবি তুলে pixel-region বিশ্লেষণ করে।
   বাকি আছে: LIDAR মাউন্ট অফসেট, gyro drift zero।
4. **Phase 4** — Perception: LIDAR দিয়ে wall detection + turn direction (KMIDS-এর অ্যালগরিদম থেকে অনুপ্রাণিত), camera দিয়ে লাল/সবুজ pillar detection, দুটো ফিউজ করা।
5. **Phase 5** — Wall-following PID control + Open Challenge-এর জন্য সম্পূর্ণ state machine।
6. **Phase 6** — Obstacle Challenge (pillar avoidance) state machine।
7. **Phase 7** — Parallel parking maneuver।
8. **Phase 8** — সম্পূর্ণ কোডবেস বাংলায় ব্যাখ্যা করা ডকুমেন্ট, যাতে নিজে নিজে bug ধরতে পারেন।

## এখন কী করবেন (Phase 1 শুরু করতে)

1. `esp32_firmware/test_motor/` → Arduino IDE-তে খুলে ESP32-তে আপলোড করুন, motor ঘোরে কিনা দেখুন।
2. `esp32_firmware/test_servo/` → servo center এবং range ঠিক আছে কিনা দেখুন।
3. `esp32_firmware/test_encoder/` → চাকা হাতে ঘুরিয়ে pulse count আসছে কিনা দেখুন।
4. `esp32_firmware/test_mpu6050/` → robot ঘুরিয়ে yaw angle পরিবর্তন হচ্ছে কিনা দেখুন।
5. `esp32_firmware/test_button/` → পুশ বাটন (GPIO 5 / D5) চাপলে detect হচ্ছে কিনা।
6. `pi/tests/test_lidar.py` → RPLIDAR ঘুরছে এবং distance data আসছে কিনা।
7. `pi/tests/test_camera.py` → ক্যামেরা ছবি তুলতে পারছে কিনা।

প্রতিটা টেস্ট ফাইলের ভিতরে বাংলায় বিস্তারিত instruction কমেন্ট আকারে লেখা আছে — কীভাবে চালাবেন, কী দেখতে পাবেন, কোনো সমস্যা হলে কী করবেন।

## পরিচিত সমস্যা ও সমাধান (Known Issues)

hardware bring-up এর সময় যেসব বড় সমস্যা পাওয়া গেছে ও যেভাবে সমাধান হয়েছে, ভবিষ্যতে মনে রাখার জন্য এবং engineering journal-এ ব্যবহার করার জন্য এখানে টুকে রাখা হলো:

1. **USB over-current / LIDAR চালু হতেই disconnect হয়ে যাওয়া**: Raspberry Pi 5, PD-negotiated (USB-C) সাপ্লাই ছাড়া USB পোর্টের কারেন্ট বাজেট কমিয়ে রাখে। আমাদের XL4016 buck converter PD negotiate করে না, তাই LIDAR motor স্পিন-আপের কারেন্ট স্পাইকে পুরো USB রেইল over-current trip করছিল। সাময়িক সমাধান: `/boot/firmware/config.txt` তে `usb_max_current_enable=1`। স্থায়ী/নির্ভরযোগ্য সমাধান (পরে করতে হবে): LIDAR+ESP32-এর জন্য আলাদা পাওয়ারের একটা powered USB hub।
2. **`/dev/ttyUSB0` / `/dev/ttyUSB1` উল্টে যাওয়া**: ডিভাইস প্লাগ করার ক্রম বা রিবুটের উপর ভিত্তি করে এই নাম্বার বদলে যেতে পারে। সমাধান: সবসময় `/dev/serial/by-id/...` এর স্থায়ী নাম ব্যবহার করা (vendor+serial number থেকে আসে, কখনো বদলায় না)। আমাদের বোর্ডে: **ESP32 = CH340 (`1a86`)**, **LIDAR = CP2102 (Silicon Labs)** — `ls -l /dev/serial/by-id/` দিয়ে চেক করা যায়।
3. **ESP32Servo লাইব্রেরি + motor-এর `ledcAttach()` একসাথে থাকলে servo সাড়া দেয় না**: দুটোই ESP32-এর LEDC PWM hardware timer claim করতে চায়, conflict হয়। সমাধান: `main_controller.ino`-তে servo-র জন্যও ESP32Servo লাইব্রেরি বাদ দিয়ে সরাসরি `ledcAttach`/`ledcWrite` ব্যবহার করা হয়েছে (motor-এর মতোই), কোনো লাইব্রেরি ছাড়াই।
4. **MPU6050 clone চিপে `Adafruit_MPU6050` লাইব্রেরির `begin()` fail করা**: লাইব্রেরিটা `WHO_AM_I` রেজিস্টার কড়াভাবে যাচাই করে, সস্তা GY-521 মডিউলে সেটা মিলে না যদিও চিপ আসলে কাজ করে। সমাধান: লাইব্রেরি বাদ দিয়ে সরাসরি I2C রেজিস্টার read/write করা।
