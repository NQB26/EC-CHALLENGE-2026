#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "drive.h"
#include "LineBot.h"
#include "Gripper.h"
#include "BLEGamepad.h"

using namespace LineBot;

Gripper GR;
BLEGamepad gp;
#define SERVO_PIN1 16
#define SERVO_PIN2 5

uint8_t base_speed = 160;
bool start = false;
uint8_t nv = 0;
uint8_t angle = 50;

void onGamepad(const GamepadState& s) {
  // ── 1. XỬ LÝ DI CHUYỂN (Dùng D-Pad với tốc độ cố định) ──────
  if (gp.isStart()) {
    start = true;
  } else {
    start = false;
    return;
  } 
  if (gp.isSelect()) nv++;
  base_speed = gp.getSpeed();
  if (gp.isUp()) {
    motorL.motor_run(base_speed);
    motorR.motor_run(base_speed);
  }
  else if (gp.isDown()) {
    motorL.motor_run(-base_speed);
    motorR.motor_run(-base_speed);
  }
  else if (gp.isLeft()) {
    motorL.motor_run(-120);
    motorR.motor_run(120);
  }
  else if (gp.isRight()) {
    motorL.motor_run(120);
    motorR.motor_run(-120);
  }
  else {
    motorL.motor_run(0);
    motorR.motor_run(0);
  }

  // ── 2. XỬ LÝ TAY GẮP (Phím ABXY) ─────────────────────────────
  if (gp.isA()){  
    GR.open();
    angle = 50;
    Serial.println("A pressed -> Mở tay gắp");
  }

  if (gp.isB()) {
    angle++;
    if (angle > 90) angle = 90;
    else GR.close(angle);
    // GR.close(85);
    Serial.println("B pressed -> Đóng tay gắp");
  }

  if (gp.isY()) {
    GR.lift_up();
    Serial.println("Y pressed -> Nâng tay gắp");
  }

  if (gp.isX()) {
    GR.lift_down();
    Serial.println("X pressed -> Hạ tay gắp");
  }
}

// =====================================================
// HÀM CHẠY CỨNG THEO KHOẢNG CÁCH (mm)
// Thực nghiệm: 100cm (1000mm) đếm được 925 xung -> 1mm = 0.925 xung
// =====================================================
void runDistance(float mm, int speed = 100) {
  long targetCounts = (long)(fabsf(mm) * 0.9388f);
  if (targetCounts <= 0) return;

  motorL.encoder_reset();
  motorR.encoder_reset();

  int pwm = (mm > 0.0f) ? abs(speed) : -abs(speed);

  motorL.motor_run(pwm);
  motorR.motor_run(pwm);

  bool l_done = false;
  bool r_done = false;

  // Vòng lặp chờ cho đến khi cả 2 bánh đạt đủ số xung
  while (!l_done || !r_done) {
    if (!l_done && labs(motorL.encoder_get_count()) >= targetCounts) {
      motorL.motor_stop();
      l_done = true;
    }
    if (!r_done && labs(motorR.encoder_get_count()) >= targetCounts) {
      motorR.motor_stop();
      r_done = true;
    }
    delay(1); // Tránh watchdog reset
  }
}

// =====================================================
// HÀM QUAY THEO GÓC (độ)
// Thực nghiệm: 90 độ đếm được khoảng 120 xung -> 1 độ = 120/90 = 1.3333 xung
// =====================================================
void turnAngle(float angle, int speed = 120) {
  // Trừ hao số xung (chống lố do quán tính ở tốc độ cao). 
  // Bạn có thể tinh chỉnh con số 5 này (tăng lên nếu vẫn lố, giảm nếu chưa tới).
  int overshoot_offset = 5; 
  
  long targetCounts = (long)(fabsf(angle) * 0.9f) - overshoot_offset;
  if (targetCounts <= 0) return;

  motorL.encoder_reset();
  motorR.encoder_reset();

  // Quy ước: angle > 0 là quay phải (bánh trái tiến, bánh phải lùi)
  //          angle < 0 là quay trái (bánh trái lùi, bánh phải tiến)
  int pwmL = (angle > 0.0f) ? abs(speed) : -abs(speed);
  int pwmR = (angle > 0.0f) ? -abs(speed) : abs(speed);

  motorL.motor_run(pwmL);
  motorR.motor_run(pwmR);

  bool l_done = false;
  bool r_done = false;

  // Vòng lặp chờ cho đến khi cả 2 bánh đạt đủ số xung
  while (!l_done || !r_done) {
    if (!l_done && labs(motorL.encoder_get_count()) >= targetCounts) {
      motorL.getMotor().brake(); // Phanh cứng ngay lập tức thay vì thả trôi
      l_done = true;
    }
    if (!r_done && labs(motorR.encoder_get_count()) >= targetCounts) {
      motorR.getMotor().brake(); // Phanh cứng ngay lập tức thay vì thả trôi
      r_done = true;
    }
    delay(1); // Tránh watchdog reset
  }
}

// =====================================================
// NÚT BẤM IO4 VÀ TRẠNG THÁI
// =====================================================
#define BTN_PIN 4
#define BTN_RUN_PIN 2

int current_mode = 0;
bool last_btn_state = HIGH;

bool is_running = false;
bool last_btn_run_state = LOW; // Nút IO2 thường là ACTIVE-HIGH trên mạch này

unsigned long lastPrintTime = 0;

void setup() {
  // Khởi tạo toàn bộ LineBot (Màn hình OLED, Motor, cảm biến MUX, NVS, BLE...)
  LineBot::begin();
  
  GR.init(SERVO_PIN1, 2, 50, SERVO_PIN2, 3, 50);
  gp.begin("Lightning-Seekers");
  gp.onData(onGamepad);

  // Ghi đè lại trạng thái nút bấm cho riêng menu của hàm main
  pinMode(BTN_PIN, INPUT_PULLUP); // IO4 (chọn mode - Active LOW)
  pinMode(BTN_RUN_PIN, INPUT);    // IO2 (chạy/dừng - Active HIGH)
  
  // Reset encoder lúc mới bật
  motorL.encoder_reset();
  motorR.encoder_reset();

  // Gọi hàm chọn calib trước khi vào loop
  runCalibration(); 
}

void nv2() {
  // turnAngle(-90);
  // delay(100);
  // runDistance(100, 80);

  // 1. Dò line đến ngã tư lần 1
  while (1){
    update2();
    // renderOLED(); // IN RA ĐỒ THỊ ĐỂ DEBUG MẮT CẢM BIẾN
    
    // Đếm số lượng mắt cảm biến thấy vạch đen
    int blackCount = 0;
    for(int i=0; i<8; i++) if(sensorBW[i]) blackCount++;

    if (blackCount >= 8) { // Nếu có từ 5 mắt trở lên thấy đen -> Ngã tư
      delay(50); // Trôi lên cho vừa tâm bánh
      motorL.motor_stop(); // Phanh cứng
      motorR.motor_stop();
      delay(200); // Chờ xe dừng hẳn
      break;
    }
  }
  
  // Chờ cho đến khi nút bấm 4 (BTN_PIN) được ấn tiếp thì thoát
  while (1) {
    if (digitalRead(BTN_PIN) == LOW) { // Nút bấm ở trạng thái Active LOW
      delay(200); // Chống dội phím
      break;
    }
    delay(10);
  }

  // turnAngle(-90);
  // delay(50);

  // // 2. Dò line đến ngã tư lần 2 (Đoạn này nhớ chèn lại code VL53L0X nếu cần dừng trước vật cản)
  // while (1){
  //   update2();
  //   int blackCount = 0;
  //   for(int i=0; i<8; i++) if(sensorBW[i]) blackCount++;

  //   if (blackCount >= 6) {
  //     delay(50);
  //     motorL.motor_stop();
  //     motorR.motor_stop();
  //     delay(200);
  //     break;
  //   }
  // }

  // turnAngle(-90);
  // delay(50);
  // runDistance(150, 100);
  // delay(50);
  // turnAngle(90);
  // delay(50);
  // runDistance(200, 100);
  // gr.lift_and_open();
  // delay(500);
  // gr.close_and_lift(85);
  // delay(50);
  // turnAngle(180);
  // delay(50);
  // while (1){
  //   update2();
  //   int blackCount = 0;
  //   for(int i=0; i<8; i++) if(sensorBW[i]) blackCount++;

  //   if (blackCount >= 6) {
  //     delay(50);
  //     motorL.motor_stop();
  //     motorR.motor_stop();
  //     delay(200);
  //     break;
  //   }
  // }
  // turnAngle(-90);
  // delay(50);
  // // runDistance(50);
  // while (1){
  //   update2();
  //   int blackCount = 0;
  //   for(int i=0; i<8; i++) if(sensorBW[i]) blackCount++;

  //   if (blackCount >= 6) {
  //     delay(100);
  //     break;
  //   }
  // }
  // while (1){
  //   update2();
  //   int blackCount = 0;
  //   for(int i=0; i<8; i++) if(sensorBW[i]) blackCount++;

  //   if (blackCount >= 6) {
  //     delay(50);
  //     motorL.motor_stop();
  //     motorR.motor_stop();
  //     delay(200);
  //     break;
  //   }
  // }
  
  // // Đứng im mãi mãi sau khi hoàn thành
  // while(1) {
  //   motorL.getMotor().brake();
  //   motorR.getMotor().brake();
  //   delay(100);
  // }
}

void nv3() {
  LineBot::update();
}
void loop() {
  // R.motor_run(255);
  gp.update();
  if (!start){
    // Serial.println("Tu dong di chuyen.");
    if (nv == 2){
      // drawBW();
      nv2();
      // update2();
      // update2();
      // test();
    }
    if (nv == 3){
      nv3();
    }
  }
}
