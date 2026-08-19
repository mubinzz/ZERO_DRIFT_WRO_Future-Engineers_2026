/*
  ============================================================
  MAIN CONTROLLER — ইউনিফায়েড ESP32 ফার্মওয়্যার (Phase 2)
  ============================================================
  এই ফাইলটা Phase 1-এর ৫টা আলাদা ESP32 টেস্ট (motor, servo,
  encoder, mpu6050, button) একসাথে করে দেয়। এখন থেকে ESP32-তে
  শুধু এই একটা ফার্মওয়্যারই আপলোড থাকবে, আলাদা test_*.ino গুলো
  আর দরকার নেই (কিন্তু রেখে দিন, ভবিষ্যতে কোনো একটা component
  নিয়ে সন্দেহ হলে আবার আলাদা করে টেস্ট করতে কাজে লাগবে)।

  কাজ:
  1. Raspberry Pi থেকে সিরিয়ালে কমান্ড নেওয়া: "M:<speed>,S:<angle>\n"
  2. সেই অনুযায়ী মোটর + সার্ভো চালানো
  3. প্রতি ~20ms পরপর Pi কে টেলিমেট্রি পাঠানো:
     "ENC:<count>,YAW:<deg>,BTN:<0/1>\n"
  4. নিরাপত্তা: Pi থেকে কিছুক্ষণ (৫০০ms) কোনো কমান্ড না এলে ধরে
     নেওয়া হবে Pi crash করেছে বা সিরিয়াল কানেকশন বিচ্ছিন্ন হয়ে
     গেছে, তখন মোটর自動ভাবে থেমে যাবে -- নইলে robot অন্ধের মতো
     আগের speed নিয়ে দেয়ালে গিয়ে ধাক্কা খেতে পারে।

  ⚠️ কেন এখানে ESP32Servo লাইব্রেরি নেই (test_servo.ino তে ছিল):
  ESP32Servo লাইব্রেরি নিজে থেকে ESP32-এর LEDC PWM hardware timer
  manage করে, আর motor-এর জন্য ব্যবহৃত ledcAttach() (নতুন
  Arduino-ESP32 core এর built-in ফাংশন) ও একই timer hardware
  ব্যবহার করে। দুটো একসাথে থাকলে timer/channel বরাদ্দ নিয়ে
  সংঘর্ষ (conflict) হয় -- ফলে servo সাড়া দেয় না। আলাদা টেস্টে এটা
  ধরা পড়েনি কারণ তখন একবারে একটাই PWM ডিভাইস সক্রিয় ছিল। তাই এখানে
  servo-র জন্যও motor-এর মতোই সরাসরি ledc ব্যবহার করা হচ্ছে, কোনো
  এক্সটার্নাল লাইব্রেরি ছাড়াই।

  Pin config, protocol -- সবকিছুর বিস্তারিত README.md-তে আছে।
  ============================================================
*/

#include <Wire.h>

// ---- PIN CONFIG (Phase 1 টেস্ট থেকে যাচাই করা আসল ভ্যালু) ----
const int PIN_PWMA   = 17;
const int PIN_AIN1   = 4;
const int PIN_AIN2   = 16;
const int PIN_STBY   = 14;

const int PIN_SERVO  = 23;

const int PIN_ENC_A  = 34;
const int PIN_ENC_B  = 35;

const int PIN_SDA    = 21;
const int PIN_SCL    = 22;

const int PIN_BUTTON = 5;

// ---- মোটর PWM কনফিগ ----
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;

// ---- সার্ভো PWM কনফিগ (raw ledc, লাইব্রেরি ছাড়া) ----
const int SERVO_FREQ = 50;        // স্ট্যান্ডার্ড হবি-সার্ভো ফ্রিকোয়েন্সি
const int SERVO_RESOLUTION = 16;  // 16-bit -> মসৃণ/নিখুঁত কোণ নিয়ন্ত্রণ
const int SERVO_MIN_US = 500;     // 0° তে পালস প্রস্থ (µs) -- test_servo.ino এর মতোই
const int SERVO_MAX_US = 2400;    // 180° তে পালস প্রস্থ (µs)

// ---- MPU6050 (raw I2C, test_mpu6050.ino এর মতোই) ----
uint8_t MPU_ADDR = 0x68;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_WHO_AM_I     = 0x75;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const float ACCEL_SCALE = 4096.0;
const float GYRO_SCALE  = 65.5;

// ---- নিরাপত্তা: Pi থেকে কমান্ড না এলে কতক্ষণ পর motor থেমে যাবে ----
const unsigned long COMMAND_TIMEOUT_MS = 500;

// ---- টেলিমেট্রি পাঠানোর ইন্টারভাল ----
const unsigned long TELEMETRY_INTERVAL_MS = 20;

volatile long encoderCount = 0;
float yaw = 0.0;
float gyroZBias = 0.0;
unsigned long lastImuUpdate = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastCommandTime = 0;

bool buttonState = HIGH;         // debounce এর পর স্থির অবস্থা
bool rawButtonLast = HIGH;
unsigned long lastButtonChangeTime = 0;
const unsigned long DEBOUNCE_MS = 50;

// ============================================================
// মোটর কন্ট্রোল (test_motor.ino থেকে)
// ============================================================
void setMotor(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
  } else if (speed < 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
  }

  ledcWrite(PIN_PWMA, abs(speed));
}

// ============================================================
// সার্ভো কন্ট্রোল (raw ledc, ESP32Servo লাইব্রেরি ছাড়া)
// ============================================================
void setServoAngle(int angle) {
  angle = constrain(angle, 0, 180);

  // angle (0-180) কে পালস প্রস্থে (µs) রূপান্তর, তারপর সেটাকে
  // 50Hz সিগন্যালের ২০,০০০ µs পিরিয়ডের সাপেক্ষে duty ভ্যালুতে রূপান্তর
  int pulseUs = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  const long periodUs = 1000000L / SERVO_FREQ; // 50Hz -> 20000 µs
  const int maxDuty = (1 << SERVO_RESOLUTION) - 1; // 16-bit -> 65535

  int duty = (int)(((long)pulseUs * maxDuty) / periodUs);
  ledcWrite(PIN_SERVO, duty);
}

// ============================================================
// এনকোডার (test_encoder.ino থেকে)
// ============================================================
void IRAM_ATTR onEncoderA() {
  if (digitalRead(PIN_ENC_B) == digitalRead(PIN_ENC_A)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// ============================================================
// MPU6050 raw I2C (test_mpu6050.ino থেকে)
// ============================================================
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 1);
  if (Wire.available()) return Wire.read();
  return 0;
}

void readRegisters(uint8_t startReg, uint8_t count, uint8_t *buffer) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, (int)count);
  for (uint8_t i = 0; i < count && Wire.available(); i++) {
    buffer[i] = Wire.read();
  }
}

void readImu(float &accelX, float &accelY, float &accelZ,
             float &gyroX, float &gyroY, float &gyroZ) {
  uint8_t buf[14];
  readRegisters(REG_ACCEL_XOUT_H, 14, buf);

  int16_t ax = (buf[0] << 8) | buf[1];
  int16_t ay = (buf[2] << 8) | buf[3];
  int16_t az = (buf[4] << 8) | buf[5];
  int16_t gx = (buf[8]  << 8) | buf[9];
  int16_t gy = (buf[10] << 8) | buf[11];
  int16_t gz = (buf[12] << 8) | buf[13];

  accelX = ax / ACCEL_SCALE;
  accelY = ay / ACCEL_SCALE;
  accelZ = az / ACCEL_SCALE;
  gyroX  = gx / GYRO_SCALE;
  gyroY  = gy / GYRO_SCALE;
  gyroZ  = gz / GYRO_SCALE;
}

void calibrateGyroBias() {
  Serial.println("# Gyro bias calibrate হচ্ছে... robot স্থির রাখুন");
  const int samples = 200;
  double sum = 0;
  float ax, ay, az, gx, gy, gz;
  for (int i = 0; i < samples; i++) {
    readImu(ax, ay, az, gx, gy, gz);
    sum += gz;
    delay(10);
  }
  gyroZBias = sum / samples;
  Serial.print("# Gyro Z bias = ");
  Serial.println(gyroZBias, 4);
}

// ============================================================
// বাটন (test_button.ino থেকে, debounce সহ)
// ============================================================
void updateButton() {
  bool raw = digitalRead(PIN_BUTTON);
  if (raw != rawButtonLast && (millis() - lastButtonChangeTime) > DEBOUNCE_MS) {
    lastButtonChangeTime = millis();
    rawButtonLast = raw;
    buttonState = raw;
  }
}

// ============================================================
// Pi থেকে আসা কমান্ড পার্স করা: "M:<speed>,S:<angle>"
// ============================================================
void processCommand(String line) {
  line.trim();
  if (!line.startsWith("M:")) return; // অচেনা লাইন উপেক্ষা করা হচ্ছে

  int sepIdx = line.indexOf(",S:");
  if (sepIdx == -1) return; // ফরম্যাট ভুল, উপেক্ষা করা হচ্ছে

  int speed = line.substring(2, sepIdx).toInt();
  int angle = line.substring(sepIdx + 3).toInt();

  speed = constrain(speed, -255, 255);
  angle = constrain(angle, 0, 180);

  setMotor(speed);
  setServoAngle(angle);

  lastCommandTime = millis(); // watchdog রিসেট
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // ---- মোটর ----
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RESOLUTION);
  digitalWrite(PIN_STBY, HIGH);
  setMotor(0);

  // ---- সার্ভো ----
  ledcAttach(PIN_SERVO, SERVO_FREQ, SERVO_RESOLUTION);
  setServoAngle(78); // সোজা/সেন্টার পজিশনে শুরু, নিরাপদ ডিফল্ট

  // ---- এনকোডার ----
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), onEncoderA, CHANGE);

  // ---- বাটন ----
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  // ---- MPU6050 ----
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setTimeOut(1000);
  writeRegister(REG_PWR_MGMT_1, 0x01);
  delay(100);
  uint8_t whoami = readRegister(REG_WHO_AM_I);
  Serial.print("# WHO_AM_I: 0x");
  Serial.println(whoami, HEX);
  writeRegister(REG_GYRO_CONFIG, 0x08);  // ±500 dps
  writeRegister(REG_ACCEL_CONFIG, 0x10); // ±8g
  calibrateGyroBias();

  Serial.println("# === Main controller ready ===");

  lastImuUpdate = millis();
  lastCommandTime = millis(); // startup মুহূর্তে সাথে সাথে timeout না হওয়ার জন্য
}

void loop() {
  // ---- ১. Pi থেকে নতুন কমান্ড পড়া (blocking না, non-blocking check) ----
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    processCommand(line);
  }

  // ---- ২. নিরাপত্তা watchdog: অনেকক্ষণ কমান্ড না এলে motor থামানো ----
  if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
    setMotor(0);
  }

  // ---- ৩. IMU আপডেট (প্রতি loop এ, যত ঘনঘন সম্ভব -- ভালো integration accuracy এর জন্য) ----
  float ax, ay, az, gx, gy, gz;
  readImu(ax, ay, az, gx, gy, gz);
  unsigned long now = millis();
  float dt = (now - lastImuUpdate) / 1000.0;
  lastImuUpdate = now;
  yaw += (gz - gyroZBias) * dt;

  // ---- ৪. বাটন আপডেট ----
  updateButton();

  // ---- ৫. প্রতি ~20ms পরপর Pi কে টেলিমেট্রি পাঠানো ----
  if (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = now;
    Serial.print("ENC:");
    Serial.print(encoderCount);
    Serial.print(",YAW:");
    Serial.print(yaw, 2);
    Serial.print(",BTN:");
    Serial.println(buttonState == LOW ? 1 : 0);
  }
}
