#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "drive.h"

// =====================================================
// KHAI BÁO MÀN HÌNH OLED
// =====================================================
#define SCREEN_W  128
#define SCREEN_H   64
#define OLED_ADDR 0x3C
#define OLED_SDA   21
#define OLED_SCL   22

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// =====================================================
// KHAI BÁO CHÂN KẾT NỐI MOTOR & ENCODER
// =====================================================
// Chân Motor Trái
#define AIN1 25
#define AIN2 33
#define PWMA 32
#define ENAL 36
#define ENBL 39
#define CH_A 0

// Chân Motor Phải
#define BIN1 27
#define BIN2 26
#define PWMB 14
#define ENAR 34
#define ENBR 35
#define CH_B 1

Drive motorL;
Drive motorR;

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
  
  long targetCounts = (long)(fabsf(angle) * 1.3333f) - overshoot_offset;
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
  Serial.begin(115200);
  
  // Thiết lập nút bấm
  pinMode(BTN_PIN, INPUT_PULLUP); // IO4 (chọn mode)
  pinMode(BTN_RUN_PIN, INPUT);    // IO2 (chạy/dừng)

  // Khởi tạo OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[ERR] Lỗi khởi tạo OLED!");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Khoi dong Motor...");
    display.display();
  }
  
  // Khởi tạo 2 motor
  motorL.motor_init(AIN1, AIN2, PWMA, ENAL, ENBL, CH_A);
  motorR.motor_init(BIN1, BIN2, PWMB, ENBR, ENAR, CH_B);
  
  // Reset encoder lúc mới bật
  motorL.encoder_reset();
  motorR.encoder_reset();
}

void loop() {
  // 1. Cập nhật motor liên tục để đếm xung
  motorL.motor_update();
  motorR.motor_update();

  // 2. Đọc nút bấm IO4 (chống dội) để chuyển mode
  bool btn_state = digitalRead(BTN_PIN);
  if (btn_state == LOW && last_btn_state == HIGH) {
    delay(20); // debounce
    if (digitalRead(BTN_PIN) == LOW) {
      current_mode++;
      if (current_mode > 3) current_mode = 0; // Quay vòng 4 mode (0->3)
      
      // Reset xung mỗi khi chuyển mode để dễ đếm lại từ đầu
      motorL.encoder_reset();
      motorR.encoder_reset();
    }
  }
  last_btn_state = btn_state;

  // 2.5 Đọc nút bấm IO2 (chống dội) để bật/tắt chạy động cơ
  bool btn_run_state = digitalRead(BTN_RUN_PIN);
  if (btn_run_state == HIGH && last_btn_run_state == LOW) { // ACTIVE-HIGH
    delay(20);
    if (digitalRead(BTN_RUN_PIN) == HIGH) {
      is_running = !is_running;
    }
  }
  last_btn_run_state = btn_run_state;

  // 3. Điều khiển motor theo mode và trạng thái chạy
  if (is_running) {
    switch(current_mode) {
      case 0: // Chạy cứng 10cm
        runDistance(100, 80);
        is_running = false; // Tự động về trạng thái STOP sau khi chạy xong
        break;
      case 1: // Quay 90 độ bằng hàm turnAngle
        turnAngle(90, 120);
        is_running = false; // Tự động dừng sau khi quay xong
        break;
      case 2: // Chạy lùi từ từ
        motorL.motor_run(-100);
        motorR.motor_run(-100);
        break;
      case 3: // Xoay tại chỗ (tốc độ 130 theo yêu cầu)
        motorL.motor_run(130);
        motorR.motor_run(-130);
        break;
    }
  } else {
    // Nếu chưa bấm IO2 để chạy thì luôn dừng
    motorL.motor_run(0);
    motorR.motor_run(0);
  }

  // 4. Hiển thị OLED & Serial mỗi 100ms
  if (millis() - lastPrintTime > 100) {
    long pulseL = motorL.encoder_get_count();
    long pulseR = motorR.encoder_get_count();

    // Hiển thị lên OLED
    display.clearDisplay();
    
    display.setCursor(0, 0);
    display.printf("=== MODE: %d ===", current_mode);
    
    display.setCursor(0, 15);
    display.print("Trang thai: ");
    if (is_running) display.println("RUN");
    else display.println("STOP");
    
    if (current_mode == 0) display.println("  CHAY 10cm");
    else if (current_mode == 1) display.println("  QUAY 90 DO");
    else if (current_mode == 2) display.println("  LUI (-100)");
    else if (current_mode == 3) display.println("  XOAY (130)");

    display.setCursor(0, 40);
    display.printf("Trai : %ld", pulseL);
    
    display.setCursor(0, 50);
    display.printf("Phai : %ld", pulseR);
    
    display.display();

    // Log Serial theo dạng Plotter cho dễ nhìn
    Serial.printf("Run: %d | Mode: %d | L: %ld | R: %ld\n", is_running, current_mode, pulseL, pulseR);

    lastPrintTime = millis();
  }
}