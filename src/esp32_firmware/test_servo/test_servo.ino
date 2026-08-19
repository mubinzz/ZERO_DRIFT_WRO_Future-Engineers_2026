/*
  ============================================================
  TEST 2: MG90S সার্ভো (স্টিয়ারিং) টেস্ট
  ============================================================
  উদ্দেশ্য: সার্ভো ঠিকমতো ঘুরছে কিনা, আর কত ডিগ্রিতে "সোজা" (center)
  থাকে সেটার একটা প্রাথমিক ধারণা নেওয়া (পরে calibration ধাপে সঠিক
  ভ্যালু বের করব)।

  ⚠️ লাইব্রেরি ইনস্টল করতে হবে:
  Arduino IDE -> Tools -> Manage Libraries -> "ESP32Servo" by Kevin
  Harrington সার্চ করে ইনস্টল করুন। (সাধারণ Servo.h ESP32-তে ঠিকমতো
  কাজ করে না, তাই এই আলাদা লাইব্রেরি লাগবে)

  ব্যবহার:
  1. আপলোডের পর Serial Monitor খুলুন (baud 115200)।
  2. একটা সংখ্যা (0-180) টাইপ করে Enter দিন -> সার্ভো ঐ কোণে যাবে।
  3. "c" টাইপ করলে center (90°) এ যাবে।
  4. "sweep" টাইপ করলে 0->180->0 স্লো swipe করবে (mechanical range দেখার জন্য)।

  সমস্যা হলে যা চেক করবেন:
  - সার্ভো কাঁপে/buzz করে কিন্তু নড়ে না: পাওয়ার সাপ্লাই দুর্বল, LM2596
    আউটপুট ভোল্টেজ (৫-৬V) আর কারেন্ট capacity চেক করুন। সার্ভোর জন্য
    আলাদা পাওয়ার লাইন থেকে GND অবশ্যই ESP32-এর GND এর সাথে common
    করতে হবে (একসাথে যুক্ত না থাকলে সার্ভো এলোমেলো আচরণ করবে)।
  - পুরো রেঞ্জ (0-180) না গিয়ে মাঝপথে আটকে যায়: এটা mechanical
    limit, MG90S এর horn/linkage কোথাও আটকাচ্ছে কিনা দেখুন।
  ============================================================
*/

#include <ESP32Servo.h>

// ---- PIN CONFIG ----
const int PIN_SERVO = 23;

Servo steeringServo;

void setup() {
  Serial.begin(115200);
  delay(300);

  // ESP32Servo লাইব্রেরির জন্য টাইমার allocate করা প্রয়োজন
  ESP32PWM::allocateTimer(0);
  steeringServo.setPeriodHertz(50);       // স্ট্যান্ডার্ড সার্ভো 50Hz
  steeringServo.attach(PIN_SERVO, 500, 2400); // min/max pulse width (µs), MG90S datasheet অনুযায়ী মোটামুটি ঠিক

  steeringServo.write(90); // শুরুতে center এ রাখা নিরাপদ

  Serial.println("=== Servo test ready ===");
  Serial.println("কমান্ড: সংখ্যা (0-180) / c (center) / sweep");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "c") {
      Serial.println("-> Center (90)");
      steeringServo.write(90);
    } else if (cmd == "sweep") {
      Serial.println("-> Sweeping 0 -> 180 -> 0 ...");
      for (int a = 0; a <= 180; a += 2) {
        steeringServo.write(a);
        delay(20);
      }
      for (int a = 180; a >= 0; a -= 2) {
        steeringServo.write(a);
        delay(20);
      }
      steeringServo.write(90);
      Serial.println("-> Done, back to center");
    } else if (cmd.length() > 0) {
      int angle = cmd.toInt();
      angle = constrain(angle, 0, 180);
      Serial.print("-> Angle set to: ");
      Serial.println(angle);
      steeringServo.write(angle);
    }
  }
}
