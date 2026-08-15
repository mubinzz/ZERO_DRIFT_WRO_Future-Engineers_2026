/*
  ============================================================
  TEST 4: MPU6050 IMU টেস্ট (raw I2C register access)
  ============================================================
  উদ্দেশ্য: robot হাতে ঘুরিয়ে yaw (heading) angle পরিবর্তন হচ্ছে
  কিনা যাচাই করা।

  কেন Adafruit লাইব্রেরি ব্যবহার করছি না?
  আগের ভার্সনে Adafruit_MPU6050 লাইব্রেরি ব্যবহার করা হয়েছিল, কিন্তু
  I2C scanner ডিভাইস (0x68) খুঁজে পেলেও লাইব্রেরির begin() fail
  করছিল। এর কারণ: লাইব্রেরিটা ভেতরে ভেতরে WHO_AM_I রেজিস্টার পড়ে
  চিপ "আসল" MPU6050 কিনা কড়াভাবে যাচাই করে, আর বাজারের অনেক সস্তা
  GY-521 মডিউলে যে চিপ থাকে সেটা এই চেক পাস করে না, যদিও চিপ আসলে
  পুরোপুরি কাজ করে (register map একই)। তাই এই কোডে আমরা সরাসরি I2C
  রেজিস্টারে read/write করছি, কোনো strict চিপ-ভ্যালিডেশন ছাড়াই।
  এতে ২টা এক্সটার্নাল লাইব্রেরির (Adafruit_MPU6050,
  Adafruit_Sensor) উপর নির্ভরতাও দূর হলো।

  ব্যবহার:
  1. আপলোড করুন, Serial Monitor খুলুন (baud 115200)।
  2. robot/সেন্সর একদম স্থির টেবিলে রেখে ২ সেকেন্ড অপেক্ষা করুন
     (শুরুতে গাইরো bias ক্যালিব্রেট হবে)।
  3. robot টা হাতে ধরে ধীরে ধীরে বামে/ডানে ঘোরান -> yaw ভ্যালু
     বাড়া/কমা উচিত।

  গুরুত্বপূর্ণ ধারণা: yaw শুধু gyroscope Z-axis কে সময়ের সাথে
  integrate (যোগ) করে বের করা হচ্ছে। এই পদ্ধতিতে সময়ের সাথে সাথে
  সামান্য "drift" হয় — robot স্থির থাকলেও yaw আস্তে আস্তে বদলাতে
  পারে, এটা স্বাভাবিক। Phase 3 (calibration) এ শুরুতে bias মেপে
  বিয়োগ করে drift কমানো হবে। প্রতিযোগিতার একটা রাউন্ড মাত্র ৩ মিনিট,
  তাই এই সাধারণ পদ্ধতিই যথেষ্ট।

  সমস্যা হলে যা চেক করবেন:
  1. WHO_AM_I লাইনে যা প্রিন্ট হয় সেটা লক্ষ্য করুন — আসল MPU6050
     সাধারণত 0x68 দেখায়, কিন্তু clone চিপে ভিন্ন মান (0x70, 0x72,
     0x98 ইত্যাদি) আসতে পারে। এটা শুধু তথ্যের জন্য প্রিন্ট হচ্ছে,
     কোড এটার উপর ভিত্তি করে বন্ধ হয়ে যাবে না।
  2. AccelZ প্রায় 9.8 এর কাছাকাছি না দেখালে (সেন্সর সমতল টেবিলে
     রাখা অবস্থায়): সেন্সর উল্টো লাগানো বা register কনফিগারেশনে
     ভুল থাকতে পারে।
  ============================================================
*/

#include <Wire.h>

// ---- PIN CONFIG ----
const int PIN_SDA = 21;
const int PIN_SCL = 22;

uint8_t MPU_ADDR = 0x68; // আগের scan এ এই address পাওয়া গিয়েছিল

// ---- MPU6050 register addresses ----
const uint8_t REG_PWR_MGMT_1  = 0x6B;
const uint8_t REG_WHO_AM_I    = 0x75;
const uint8_t REG_GYRO_CONFIG = 0x1B;
const uint8_t REG_ACCEL_CONFIG= 0x1C;
const uint8_t REG_ACCEL_XOUT_H= 0x3B; // এখান থেকে টানা ১৪ বাইট পড়লে accel+temp+gyro সব পাওয়া যায়

// ---- scale factors (কনফিগারেশনের সাথে মিলিয়ে) ----
const float ACCEL_SCALE = 4096.0;  // ±8g রেঞ্জ -> 4096 LSB/g
const float GYRO_SCALE  = 65.5;    // ±500 dps রেঞ্জ -> 65.5 LSB/(deg/s)

float yaw = 0.0;
float gyroZBias = 0.0;
unsigned long lastUpdate = 0;

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false); // repeated start, কানেকশন খোলা রাখে
  Wire.requestFrom((int)MPU_ADDR, 1);
  if (Wire.available()) return Wire.read();
  return 0;
}

// startReg থেকে শুরু করে count সংখ্যক বাইট buffer এ পড়ে
void readRegisters(uint8_t startReg, uint8_t count, uint8_t *buffer) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, (int)count);
  for (uint8_t i = 0; i < count && Wire.available(); i++) {
    buffer[i] = Wire.read();
  }
}

// custom struct return করার বদলে reference parameter (& চিহ্ন) দিয়ে
// মানগুলো "বের করে আনা" হচ্ছে -- Arduino IDE এর auto-prototype
// generator custom struct return-type নিয়ে সমস্যা করে, তাই এই
// পদ্ধতি ব্যবহার করা হলো (embedded কোডে এটাই বেশি প্রচলিত)
void readImu(float &accelX, float &accelY, float &accelZ,
             float &gyroX, float &gyroY, float &gyroZ) {
  uint8_t buf[14];
  readRegisters(REG_ACCEL_XOUT_H, 14, buf);

  int16_t ax = (buf[0] << 8) | buf[1];
  int16_t ay = (buf[2] << 8) | buf[3];
  int16_t az = (buf[4] << 8) | buf[5];
  // buf[6], buf[7] = temperature, এখানে ব্যবহার করছি না
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
  Serial.println("Gyro bias calibrate হচ্ছে... robot স্থির রাখুন (২ সেকেন্ড)");
  const int samples = 200;
  double sum = 0;
  float ax, ay, az, gx, gy, gz;
  for (int i = 0; i < samples; i++) {
    readImu(ax, ay, az, gx, gy, gz);
    sum += gz;
    delay(10);
  }
  gyroZBias = sum / samples;
  Serial.print("Gyro Z bias = ");
  Serial.println(gyroZBias, 4);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== MPU6050 test বুট হচ্ছে (raw I2C mode) ===");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setTimeOut(1000);

  // ঘুম থেকে জাগানো: PWR_MGMT_1 রেজিস্টারে 0x01 লিখলে SLEEP বিট
  // পরিষ্কার হয় এবং clock source হিসেবে gyro X ব্যবহার হয় (বেশি
  // স্ট্যাবল)। ডিফল্টে চিপ SLEEP মোডে বুট হয়, তাই এই স্টেপ বাদ
  // দিলে কোনো ডেটাই আসবে না।
  writeRegister(REG_PWR_MGMT_1, 0x01);
  delay(100);

  uint8_t whoami = readRegister(REG_WHO_AM_I);
  Serial.print("WHO_AM_I রেজিস্টার মান: 0x");
  Serial.println(whoami, HEX);
  Serial.println("(আসল MPU6050 এ সাধারণত 0x68 হয়, clone চিপে ভিন্ন হতে পারে — সমস্যা না)");

  writeRegister(REG_GYRO_CONFIG, 0x08);  // ±500 dps
  writeRegister(REG_ACCEL_CONFIG, 0x10); // ±8g

  Serial.println("=== MPU6050 কনফিগার করা হয়েছে, রিডিং শুরু হচ্ছে ===");
  calibrateGyroBias();

  lastUpdate = millis();
}

void loop() {
  float ax, ay, az, gx, gy, gz;
  readImu(ax, ay, az, gx, gy, gz);

  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0;
  lastUpdate = now;

  float gyroZ_corrected = gz - gyroZBias;
  yaw += gyroZ_corrected * dt;

  static unsigned long lastPrint = 0;
  if (now - lastPrint > 200) {
    lastPrint = now;
    Serial.print("Yaw: ");
    Serial.print(yaw, 2);
    Serial.print(" deg | AccelX: ");
    Serial.print(ax, 2);
    Serial.print("g AccelY: ");
    Serial.print(ay, 2);
    Serial.print("g AccelZ: ");
    Serial.print(az, 2);
    Serial.println("g");
  }
}
