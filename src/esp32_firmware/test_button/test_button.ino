/*
  ============================================================
  TEST 5: Push Button টেস্ট (ESP32 GPIO 5 / "D5")
  ============================================================
  উদ্দেশ্য: WRO নিয়ম অনুযায়ী (rule 9.10-9.11) robot switch on করার
  পর একটা "Start button" চাপলেই প্রোগ্রাম শুরু হওয়া উচিত। বাটনটা
  এখানে ESP32-তে লাগানো (Pi-তে না), তাই এই টেস্ট ESP32-তেই।

  ওয়্যারিং: বাটনের এক পা GPIO 5 (বোর্ডের "D5" লেবেল) এ, অন্য পা
  GND এ। কোডে internal pull-up ব্যবহার হচ্ছে, তাই বাইরে থেকে
  resistor লাগানো লাগবে না — বাটন না চাপলে pin HIGH থাকবে, চাপলে
  GND এর সাথে শর্ট হয়ে LOW হবে।

  ব্যবহার:
  1. আপলোড করুন, Serial Monitor খুলুন (baud 115200)।
  2. বাটন চাপলে "বাটন চাপা হয়েছে!" প্রিন্ট হবে।

  পরবর্তী ধাপে (Phase 2, unified firmware) এই একই লজিক ESP32 এর
  মূল ফার্মওয়্যারে থাকবে, আর বাটনের অবস্থা Pi কে সিরিয়ালে
  "BTN:1" / "BTN:0" আকারে জানানো হবে, যাতে Pi এর মূল প্রোগ্রাম
  বাটন চাপা পড়লে দৌড় শুরু করতে পারে (WRO rule অনুযায়ী)।

  সমস্যা হলে যা চেক করবেন:
  - "D5" লেবেল আপনার বোর্ডে GPIO 5 না হয়ে অন্য নাম্বার হতে পারে
    (এটা ESP8266 এর NodeMCU এর মতো standard না, বোর্ড ভেদে ভিন্ন
    হয়)। বোর্ডের silkscreen এ GPIO নাম্বারও লেখা থাকলে সেটা মিলিয়ে
    নিন, না থাকলে নিচের PIN_BUTTON ভ্যালু বদলে বদলে টেস্ট করুন।
  - বাটন চাপলেও কিছু না দেখালে: তার আলগা কিনা, GND এ ঠিকমতো
    কানেক্ট আছে কিনা দেখুন।
  - একবার চাপলে বহুবার print হলে ("bounce"): নিচের কোডে debounce
    delay (৫০ms) দিয়ে এটা হ্যান্ডেল করা আছে।
  ============================================================
*/

// ---- PIN CONFIG (আপনার actual wiring অনুযায়ী বদলান) ----
const int PIN_BUTTON = 5; // GPIO 5, বোর্ডের "D5" লেবেল

bool lastButtonState = HIGH; // internal pull-up এর কারণে না চাপা অবস্থায় HIGH থাকে
unsigned long lastChangeTime = 0;
const unsigned long DEBOUNCE_MS = 50;

int pressCount = 0;

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Serial.println("=== Button test ready ===");
  Serial.println("বাটন চেপে দেখুন।");
}

void loop() {
  bool currentState = digitalRead(PIN_BUTTON);

  if (currentState != lastButtonState && (millis() - lastChangeTime) > DEBOUNCE_MS) {
    lastChangeTime = millis();
    lastButtonState = currentState;

    if (currentState == LOW) { // pull-up এর কারণে চাপলে LOW হয়
      pressCount++;
      Serial.print("বাটন চাপা হয়েছে! (মোট ");
      Serial.print(pressCount);
      Serial.println(" বার)");
    } else {
      Serial.println("বাটন ছাড়া হয়েছে।");
    }
  }
}
