#include <Arduino.h>
#include "BLEGamepad.h"

BLEGamepad gp;

void onGamepad(const GamepadState& s) {
  // ── Check từng nút ──────────────────────────────────────
  if (gp.isUp())    Serial.println("UP pressed");
  if (gp.isDown())  Serial.println("DOWN pressed");
  if (gp.isLeft())  Serial.println("LEFT pressed");
  if (gp.isRight()) Serial.println("RIGHT pressed");
  if (gp.isA())     Serial.println("A pressed → open gripper");
  if (gp.isB())     Serial.println("B pressed → close gripper");
  if (gp.isX())     Serial.println("X pressed");
  if (gp.isY())     Serial.println("Y pressed");

  // ── Hoặc check bitmask trực tiếp ─────────────────────────
  // if (s.buttons & BTN_A) { ... }

  // ── Joystick & gripper ────────────────────────────────────
  Serial.printf("LX=%d LY=%d  RX=%d RY=%d  Gripper=%d\n",
                s.leftX, s.leftY, s.rightX, s.rightY, s.gripper);
}

void setup() {
  Serial.begin(115200);
  gp.begin("ESP32-Gripper");
  gp.onData(onGamepad);
}

void loop() {
  gp.update();  // gọi callback nếu có data mới
}