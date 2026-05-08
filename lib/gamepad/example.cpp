#include <Arduino.h>
#include "BLEGamepad.h"
#include "Gripper.h"
#include "drive.h"
#include "LineBot.h"

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
Gripper  GR;
Motor L, R;
BLEGamepad gp;

uint8_t base_speed = 160;
bool start = false;
uint8_t nv = 0;
uint8_t angle = 50;
void onGamepad(const GamepadState& s) {
  // ── 1. XỬ LÝ DI CHUYỂN (Dùng D-Pad với tốc độ cố định) ──────
  if (gp.isStart()) start = true;
  else {
    return;
  } 
  if (gp.isSelect()) nv++;
  base_speed = gp.getSpeed();
  if (gp.isUp()) {
    L.motor_run(base_speed);
    R.motor_run(base_speed);
  }
  else if (gp.isDown()) {
    L.motor_run(-base_speed);
    R.motor_run(-base_speed);
  }
  else if (gp.isLeft()) {
    L.motor_run(-120);
    R.motor_run(120);
  }
  else if (gp.isRight()) {
    L.motor_run(120);
    R.motor_run(-120);
  }
  else {
    L.motor_run(0);
    R.motor_run(0);
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

void setup() {
  Serial.begin(115200);

  // Gamepad BLE (tên khác BLE_DEVICE_NAME của LineBot)
  gp.begin("Lightning-Seekers");
  gp.onData(onGamepad);

  // Tay gắp: servo lift trên kênh 0, servo grip trên kênh 1
  GR.init(SERVO_PIN1, 2, 50, SERVO_PIN2, 3, 15);
  L.init(LIN1, LIN2, PWML, 0);
  R.init(RIN2, RIN1, PWMR, 1);
}

void loop() {
  // R.motor_run(255);
  gp.update();
  if (!start){
    Serial.println("Tu dong di chuyen.");

  }
}