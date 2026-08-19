/*
  ============================================================
  TEST 1: N20 মোটর + TB6612FNG ড্রাইভার টেস্ট
  ============================================================
  উদ্দেশ্য: শুধু মোটর ঘোরানো যাচ্ছে কিনা, দুই দিকে (সামনে/পেছনে)
  ঘুরছে কিনা, আর PWM দিয়ে স্পিড কন্ট্রোল করা যাচ্ছে কিনা — এটা যাচাই করা।
  এখানে এখনো Raspberry Pi বা encoder কিছুই লাগছে না, শুধু ESP32 একা।

  ব্যবহার:
  1. Arduino IDE-তে এই ফাইল খুলুন, Board = "ESP32 Dev Module" সিলেক্ট করুন।
  2. ESP32-তে আপলোড করুন।
  3. Serial Monitor খুলুন (baud rate 115200)।
  4. নিচের কমান্ডগুলো টাইপ করে Enter দিন:
       f     -> সামনে ঘুরবে (মাঝারি স্পিড)
       r     -> পেছনে ঘুরবে
       s     -> থামবে (stop)
       120   -> শুধু সংখ্যা লিখলে ঐ স্পিডে (0-255) সামনে ঘুরবে

  সমস্যা হলে যা চেক করবেন:
  - মোটর একদমই না ঘুরলে: STBY পিন HIGH আছে কিনা দেখুন (STBY LOW থাকলে
    ড্রাইভার পুরোপুরি ঘুমিয়ে থাকে, কোনো মোটরই চলবে না)।
  - মোটর ঘুরলেও দিক উল্টা মনে হলে: AIN1/AIN2 এর তার উল্টে দিন অথবা
    কোডের setDirection() ফাংশনে HIGH/LOW উল্টে দিন।
  - মোটর কাঁপে কিন্তু ঘোরে না: PWM ডিউটি (স্পিড ভ্যালু) খুব কম, বাড়িয়ে
    দেখুন। N20 মোটরের জন্য সাধারণত কমপক্ষে ৮০-১০০ (0-255 স্কেলে) লাগে
    ঘোরা শুরু করতে (এটাকে বলে "stall/deadzone", পরে calibration এ বের করব)।
  ============================================================
*/

// ---- PIN CONFIG (আপনার actual wiring অনুযায়ী বদলান) ----
const int PIN_PWMA = 17;   // TB6612 PWMA -> মোটরের স্পিড (PWM)
const int PIN_AIN1 = 4;   // TB6612 AIN1 -> দিক নিয়ন্ত্রণ
const int PIN_AIN2 = 16;   // TB6612 AIN2 -> দিক নিয়ন্ত্রণ
const int PIN_STBY = 14;   // TB6612 STBY -> HIGH করলে ড্রাইভার সচল হয়

// ESP32-এর নতুন LEDC (PWM) API ব্যবহার করছি (Arduino-ESP32 core 3.x)
const int PWM_FREQ = 5000;      // 5kHz PWM frequency, মোটরের জন্য যথেষ্ট
const int PWM_RESOLUTION = 8;   // 8-bit মানে স্পিড value 0-255

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  // PWMA পিনে LEDC PWM অ্যাটাচ করা হচ্ছে
  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RESOLUTION);

  digitalWrite(PIN_STBY, HIGH);  // ড্রাইভার জাগিয়ে রাখা (সবসময় HIGH রাখব)

  Serial.println("=== Motor test ready ===");
  Serial.println("কমান্ড: f (forward) / r (reverse) / s (stop) / সংখ্যা (0-255 স্পিড)");
}

// speed: -255 (পূর্ণ পেছনে) থেকে +255 (পূর্ণ সামনে), 0 = ব্রেক
void setMotor(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
  } else if (speed < 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
  } else {
    // দুটোই LOW করলে মোটর ফ্রি-হুইল (আস্তে থামে)
    // দুটোই HIGH করলে শর্ট-ব্রেক (দ্রুত থামে) -- WRO তে দ্রুত থামা কাজে লাগবে
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
  }

  ledcWrite(PIN_PWMA, abs(speed));
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "f") {
      Serial.println("-> Forward @ 150");
      setMotor(150);
    } else if (cmd == "r") {
      Serial.println("-> Reverse @ 150");
      setMotor(-150);
    } else if (cmd == "s") {
      Serial.println("-> Stop");
      setMotor(0);
    } else if (cmd.length() > 0) {
      int val = cmd.toInt();
      Serial.print("-> Speed set to: ");
      Serial.println(val);
      setMotor(val);
    }
  }
}
