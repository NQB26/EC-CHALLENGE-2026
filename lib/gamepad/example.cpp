// // /**
// //  * @file main.cpp
// //  * @brief Test Encoder – Chạy motor open-loop (PWM cố định), in RPM + xung.
// //  *
// //  * PID tắt hoàn toàn. Dùng để kiểm tra encoder đọc đúng không.
// //  *
// //  * Lệnh Serial (baud 115200):
// //  *   <số>   → Đặt PWM -255…+255  (vd: 180 = tiến, -180 = lùi)
// //  *   0      → Dừng motor
// //  *   r      → Reset bộ đếm encoder về 0
// //  *
// //  * Sơ đồ nối (chỉnh lại theo phần cứng):
// //  *   TB6612FNG   ESP32
// //  *   AIN1    →   GPIO 25
// //  *   AIN2    →   GPIO 26
// //  *   PWMA    →   GPIO 27
// //  *   Encoder A → GPIO 34
// //  *   Encoder B → GPIO 35
// //  *   STBY    →   3.3 V
// //  */

// // #include "drive.h"
// // #include <Arduino.h>

// // // ── Cấu hình chân (chỉnh theo phần cứng) ─────────────────────────────────────
// // #define PIN_IN1 25
// // #define PIN_IN2 26
// // #define PIN_PWM 27
// // #define PIN_ENC_A 34
// // #define PIN_ENC_B 35
// // #define LEDC_CH 0

// // // ── Chu kỳ in (ms) ───────────────────────────────────────────────────────────
// // #define PRINT_INTERVAL_MS 200

// // // ─────────────────────────────────────────────────────────────────────────────

// // Drive motor;

// // uint32_t lastPrint = 0;
// // int currentPWM = 0;

// // void setup() {
// //   Serial.begin(115200);
// //   delay(200);

// //   Serial.println(F("========================================"));
// //   Serial.println(F("  GA25-370  Encoder Test  (PID = OFF)  "));
// //   Serial.println(F("  Gõ PWM -255..255 rồi nhấn Enter      "));
// //   Serial.println(F("  0 = dừng   |   r = reset encoder     "));
// //   Serial.println(F("========================================"));

// //   motor.motor_init(PIN_IN1, PIN_IN2, PIN_PWM, PIN_ENC_A, PIN_ENC_B, LEDC_CH);

// //   motor.enablePID(false); // PID tắt hoàn toàn
// //   motor.motor_stop();

// //   lastPrint = millis();
// // }

// // void loop() {
// //   // ── 1. Cập nhật encoder (cần để tính RPM, PID bị tắt nên không ảnh hưởng)
// //   motor.motor_update();

// //   // ── 2. Đọc lệnh Serial ────────────────────────────────────────────────────
// //   if (Serial.available()) {
// //     String input = Serial.readStringUntil('\n');
// //     input.trim();

// //     if (input.equalsIgnoreCase("r")) {
// //       motor.encoder_reset();
// //       Serial.println(F(">> Encoder reset về 0"));
// //     } else {
// //       int pwm = input.toInt();
// //       pwm = constrain(pwm, -255, 255);
// //       currentPWM = pwm;
// //       motor.motor_run(pwm);

// //       Serial.print(F(">> PWM = "));
// //       Serial.println(pwm);
// //     }
// //   }

// //   // ── 3. In dữ liệu encoder định kỳ ────────────────────────────────────────
// //   uint32_t now = millis();
// //   if (now - lastPrint >= PRINT_INTERVAL_MS) {
// //     lastPrint = now;

// //     float rpm = motor.motor_get_speed();
// //     long enc = motor.encoder_get_count();

// //     Serial.print(F("PWM: "));
// //     Serial.print(currentPWM);
// //     Serial.print(F("  |  RPM: "));
// //     Serial.print(rpm, 1);
// //     Serial.print(F("  |  Enc: "));
// //     Serial.println(enc);
// //   }
// // }
// #define CUSTOM_SETTINGS
// #define INCLUDE_GAMEPAD_MODULE
// // #include <DabbleESP32.h>
// #include <ESP32Servo.h>
// #include "drive.h"
// #include "Gripper.h"
// #include "LineBot.h"

// // --- CẤU HÌNH CHÂN KẾT NỐI ---
// #define LIN1 25
// #define LIN2 33
// #define RIN1 26
// #define RIN2 27
// #define PWML 32
// #define PWMR 14

// #define SERVO_PIN1 16
// #define SERVO_PIN2 5

// // #define ENAL VP
// // #define ENBL VN
// #define ENAR 34
// #define ENBR 35

// // --- KHAI BÁO ĐỐI TƯỢNG VÀ BIẾN TOÀN CỤC ---
// Gripper GR;
// Motor L, R;

// unsigned long thoiGianCho = 0;
// int trangThaiToHop = 0; 
// // 0: Rảnh rỗi
// // 1: Đang chờ nâng (sau khi đã gắp)
// // 2: Đang chờ nhả (sau khi đã hạ)

// bool choPhepHoatDong = false;       
// bool trangThaiStartTruocDo = false; 

// void setup() {
//   Serial.begin(115200);
  
//   // Khởi tạo Dabble Bluetooth
//   Dabble.begin("ESP32_Robot");
//   GR.init(SERVO_PIN1, 0, 50 ,SERVO_PIN2, 1, 10);
//   // Cấu hình chân động cơ
//   // L.init(LIN1, LIN2, PWML);
//   // R.init(RIN1, RIN2, PWMR);
//   Serial.println("Hệ thống sẵn sàng!");
// }

// // ==========================================
// // CÁC HÀM XỬ LÝ CHỨC NĂNG
// // ==========================================

// void xuLyDiChuyen() {
//   float y = GamePad.getYaxisData(); 
//   float x = GamePad.getXaxisData(); 

//   // Deadzone
//   if (abs(x) < 1.0) x = 0;
//   if (abs(y) < 1.0) y = 0;

//   // Mapping giá trị Joystick sang PWM
//   int tocDoY = (int)(y * 255.0 / 7.0); 
//   int tocDoX = (int)(x * 255.0 / 7.0);

//   // Arcade Drive
//   int tocDoTrai = tocDoY + tocDoX;
//   int tocDoPhai = tocDoY - tocDoX;

//   tocDoTrai = constrain(tocDoTrai, -255, 255);
//   tocDoPhai = constrain(tocDoPhai, -255, 255);

//   L.motor_run(tocDoTrai);
//   R.motor_run(tocDoPhai);
// }

// void xuLyTayGap() {
//   if (GamePad.isCirclePressed()) {
//     GR.close(85);
//     Serial.println("O (Circle) pressed");
//     trangThaiToHop = 1;
    
//   }
  
//   if (GamePad.isCrossPressed()) {
//     GR.open();
//     Serial.println("X (Cross) pressed");
//     trangThaiToHop = 0;
//   }

//   if (GamePad.isTrianglePressed()) {
//     // GR.close_and_lift(85);
//     GR.lift_up();
//     Serial.println("Triangle pressed");
//     trangThaiToHop = 2;
//   }

//   if (GamePad.isSquarePressed()) {
//     // GR.lift_and_open();
//     GR.lift_down();
//     Serial.println("Square pressed");
//     trangThaiToHop = 0;
//   }
// }
// void loop() {
//   // GR.close(85);
//   // delay(1000);
//   // GR.open();
//   // delay(1000);
//   // GR.close_and_lift(80);
//   // delay(1000);
//   // GR.lift_and_open();
//   // delay(1000);
//   Dabble.processInput();
  
//   // --- XỬ LÝ NÚT START (BẬT/TẮT) ---
//   bool trangThaiStartHienTai = GamePad.isStartPressed();
  
//   if (trangThaiStartHienTai == true && trangThaiStartTruocDo == false) {
//      choPhepHoatDong = !choPhepHoatDong; 
     
//      if (choPhepHoatDong) {
//        Serial.println("ĐÃ MỞ KHÓA - Có thể điều khiển!");
//      } else {
//        Serial.println("ĐÃ KHÓA - Dừng mọi hoạt động.");
//      }
//   }
//   trangThaiStartTruocDo = trangThaiStartHienTai;

//   // --- PHÂN LUỒNG ĐIỀU KHIỂN ---
//   if (choPhepHoatDong == true) {
//     //  xuLyDiChuyen();
//      xuLyTayGap();
//   } else {
//     //  dieuKhienDongCo(0, 0); // Ép dừng động cơ khi bị khóa
//     // L.motor_run(0);
//     // R.motor_run(0);
//   }
// }
