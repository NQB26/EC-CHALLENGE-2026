#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "drive.h"

namespace LineBot {
  extern Adafruit_SSD1306 display;
  extern Drive motorL;
  extern Drive motorR;

  #define NUM_SENSORS 8
  extern bool sensorBW[NUM_SENSORS];

  void begin();

  void update();
  
  // Hàm dò line PID cơ bản (tốc độ thấp, không có state machine phức tạp)
  void runBasicPID(int customSpeed = 100);

  // Cập nhật thông số PID từ BLE và gửi dữ liệu lên app
  void update2();
  void test();
  // In ra đồ thị ASCII hình cột lên Serial Monitor để debug
  void renderOLED();
  void drawDebugBWOLED();

  // Chạy quy trình calib chặn (phù hợp cho hàm main tự quản lý menu)
  void runCalibration();
  void updateCalibration();
}