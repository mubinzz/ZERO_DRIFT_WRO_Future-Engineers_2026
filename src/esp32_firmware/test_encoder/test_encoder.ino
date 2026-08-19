/*
  ============================================================
  TEST 3: N20 এনকোডার টেস্ট
  ============================================================
  উদ্দেশ্য: চাকা হাত দিয়ে ঘুরিয়ে দেখা যে এনকোডার pulse count ঠিকমতো
  বাড়ছে/কমছে কিনা, এবং দিক (forward/reverse) সঠিকভাবে ধরা পড়ছে কিনা।
  এই ডেটা পরে PID স্পিড কন্ট্রোল আর distance measurement এর জন্য লাগবে।

  ব্যবহার:
  1. আপলোড করুন, Serial Monitor খুলুন (baud 115200)।
  2. চাকা হাত দিয়ে সামনের দিকে ঘোরান -> count বাড়া উচিত।
  3. উল্টা দিকে ঘোরান -> count কমা উচিত।
  4. এক পূর্ণ চাকা-ঘূর্ণনে (1 wheel revolution) কত pulse আসে সেটা গুনে
     রাখুন — এটা distance calibration এ লাগবে (Phase 3)। সহজ পদ্ধতি:
     চাকায় একটা মার্কার দিন, count রিসেট (এই কোডে 'r' কমান্ড) করে
     ঠিক ১ বার ঘুরিয়ে count নোট করুন।

  সমস্যা হলে যা চেক করবেন:
  - count একদম না বাড়লে: ENC_A পিনে সিগন্যাল আসছে কিনা মাল্টিমিটার/
    অসিলোস্কোপ দিয়ে চেক করুন, অথবা এনকোডারের পাওয়ার (সাধারণত এনকোডার
    মডিউল আলাদা 3.3V/5V লাগে) ঠিকমতো দেওয়া আছে কিনা দেখুন।
  - count শুধু বাড়েই, কমে না (দিক ধরা পড়ছে না): ENC_A আর ENC_B এর
    তার অদলবদল করে দেখুন।
  - random/glitchy বড় জাম্প: pull-up resistor লাগবে কিনা দেখুন
    (নিচের কোডে INPUT_PULLUP ব্যবহার করা হয়েছে, তারপরও সমস্যা হলে
    বাইরে থেকে 10kΩ pull-up যোগ করুন)।
  ============================================================
*/

// ---- PIN CONFIG ----
// GPIO 34, 35 ইচ্ছাকৃতভাবে বেছে নেওয়া কারণ এগুলো ESP32-তে input-only
// পিন এবং interrupt-capable — এনকোডারের মতো দ্রুত সিগন্যালের জন্য ভালো।
const int PIN_ENC_A = 34;
const int PIN_ENC_B = 35;

volatile long encoderCount = 0;

// ইন্টারাপ্ট রুটিন: ENC_A এর প্রতিটা পরিবর্তনে (RISING/FALLING) কল হয়
void IRAM_ATTR onEncoderA() {
  // quadrature decoding: A পিন যখন পরিবর্তন হয়, তখন B পিনের অবস্থা
  // দেখে দিক বোঝা যায়
  if (digitalRead(PIN_ENC_B) == digitalRead(PIN_ENC_A)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), onEncoderA, CHANGE);

  Serial.println("=== Encoder test ready ===");
  Serial.println("চাকা হাত দিয়ে ঘোরান, count দেখুন। 'r' লিখে reset করতে পারবেন।");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "r") {
      encoderCount = 0;
      Serial.println("-> Count reset to 0");
    }
  }

  // প্রতি 200ms এ একবার count প্রিন্ট করি, যাতে Serial Monitor স্প্যাম না হয়
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    Serial.print("Encoder count: ");
    Serial.println(encoderCount);
  }
}
