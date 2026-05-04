#include <Arduino.h>
#include "BLEGamepad.h"
#include "Gripper.h"
#include "drive.h"
// --- CẤU HÌNH CHÂN KẾT NỐI ---
#define LIN1 25
#define LIN2 33
#define RIN1 26
#define RIN2 27
#define PWML 32
#define PWMR 14

#define SERVO_PIN1 16
#define SERVO_PIN2 5

// #define ENAL VP
// #define ENBL VN
#define ENAR 34
#define ENBR 35

// --- KHAI BÁO ĐỐI TƯỢNG VÀ BIẾN TOÀN CỤC ---
Gripper GR;
Motor L, R;
BLEGamepad gp;

uint8_t base_speed = 160;
void onGamepad(const GamepadState& s) {
  // ── 1. XỬ LÝ DI CHUYỂN (Dùng D-Pad với tốc độ cố định) ──────
  if (gp.isUp()) {
    L.motor_run(base_speed);
    R.motor_run(base_speed);
  }
  else if (gp.isDown()) {
    L.motor_run(-base_speed);
    R.motor_run(-base_speed);
  }
  else if (gp.isLeft()) {
    L.motor_run(base_speed);
    R.motor_run(-base_speed);
  }
  else if (gp.isRight()) {
    L.motor_run(-base_speed);
    R.motor_run(base_speed);
  }
  else {
    L.motor_run(0);
    R.motor_run(0);
  }

  // ── 2. XỬ LÝ TAY GẮP (Phím ABXY) ─────────────────────────────
  if (gp.isA()) {  
    GR.open();
    Serial.println("A pressed -> Mở tay gắp");
  }     

  if (gp.isB()) {  
    GR.close(85);
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

void setup() {
  Serial.begin(115200);
  gp.begin("ESP32-Gripper");
  GR.init(SERVO_PIN1, 0, 50 ,SERVO_PIN2, 1, 10);
  gp.onData(onGamepad);
}

void loop() {
  gp.update();  // Cập nhật dữ liệu từ Web Gamepad (thay cho Dabble.processInput)
}