/*
 * LineBot library – ESP32 + TB6612FNG + HC4067 + OLED + BLE
 * Tach tu src/main.cpp de sau nay main.cpp chi can include thu vien.
 */

#include "LineBot.h"
#include "drive.h"
#include <Preferences.h>

namespace LineBot {

// NVS – lưu params PID qua reboot
Preferences prefs;

// =====================================================
// OLED
// =====================================================
#define SCREEN_W  128
#define SCREEN_H   64
#define OLED_ADDR 0x3C
#define OLED_SDA   21
#define OLED_SCL   22

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// =====================================================
// Nut bam – 2 nut, ACTIVE HIGH, pull-down ngoai 10k
//   IO2 (BTN_ONE_PIN): calib WHITE/BLACK + RUN/STOP
//   IO4 (BTN_TWO_PIN): calib WHITE/BLACK + RUN/STOP (giong IO2)
// Luu y IO2/IO4 la chan boot-strap, KHONG giu nut khi reset/nap code!
// BLE dashboard cung dieu khien duoc song song.
// =====================================================
#define BTN_ONE_PIN 2
#define BTN_TWO_PIN 4
#define DEBOUNCE_MS 40UL

// ─── Nut 1 (GPIO2) ────────────────────────────────
bool btnStable    = LOW;
bool btnLastRaw   = LOW;
unsigned long btnLastChange = 0;

bool buttonPressedEvent() {
  bool raw = digitalRead(BTN_ONE_PIN);
  if (raw != btnLastRaw) { btnLastRaw = raw; btnLastChange = millis(); }
  if (millis() - btnLastChange > DEBOUNCE_MS) {
    if (raw != btnStable) {
      btnStable = raw;
      if (btnStable == HIGH) return true;
    }
  }
  return false;
}

// ─── Nut 2 (GPIO4) ────────────────────────────────
bool btn2Stable   = LOW;
bool btn2LastRaw  = LOW;
unsigned long btn2LastChange = 0;

bool button2PressedEvent() {
  bool raw = digitalRead(BTN_TWO_PIN);
  if (raw != btn2LastRaw) { btn2LastRaw = raw; btn2LastChange = millis(); }
  if (millis() - btn2LastChange > DEBOUNCE_MS) {
    if (raw != btn2Stable) {
      btn2Stable = raw;
      if (btn2Stable == HIGH) return true;
    }
  }
  return false;
}

// =====================================================
// HC4067 – 16-kênh MUX
// =====================================================
#define MUX_S0   23
#define MUX_S1   19
#define MUX_S2   18
#define MUX_SIG  13    // chân ADC

#define NUM_SENSORS 8

uint16_t sensorRaw [NUM_SENSORS];
uint16_t sensorFilt[NUM_SENSORS];   // raw da loc nhe IIR de giam nhieu ADC
bool     filtReady = false;
uint16_t sensorMin [NUM_SENSORS];
uint16_t sensorMax [NUM_SENSORS];
uint16_t sensorTh  [NUM_SENSORS];
uint16_t sensorNorm[NUM_SENSORS];   // độ đậm đen 0–1000
bool     sensorBW  [NUM_SENSORS];   // mảng nhị phân nhận diện vạch đen
bool     lastSensorBW[NUM_SENSORS] = {false}; // mảng lưu trạng thái lần cuối có line

const int BW_HYST = 30; // Threshold hysteresis


const bool SENSOR_ORDER_REVERSED = false;
bool blackIsHigh = false;
bool calibrated  = false;

// Mask cảm biến: false = bỏ qua (hỏng/không dùng)
// Cảm biến 0 bỏ qua vì bỏ hỏng
const bool SENSOR_ENABLED[NUM_SENSORS] = { true, true, true, true, true, true, true, true };

// =====================================================
// Motor – GA25Motor Drive (TB6612FNG + Encoder + PID)
// =====================================================
// Chân kết nối TB6612FNG
#define AIN1 25
#define AIN2 33
#define PWMA 32
#define BIN1 27
#define BIN2 26
#define PWMB 14

// Chân encoder (input-only OK với ESP32, dùng PCNT hardware)
#define ENAL 36
#define ENBL 39
#define ENAR 34
#define ENBR 35

// Kênh LEDC – mỗi Drive dùng 1 kênh
#define CH_A 0
#define CH_B 1

// Đối tượng Drive cho motor trái (A) và phải (B)
Drive motorL;
Drive motorR;

// =====================================================
// Tham số điều khiển – tuỳ chỉnh qua BLE web dashboard
// =====================================================
float Kp          = 0.050f;
float Ki          = 0.000f;   // tích phân – mặc định 0 (tắt)
float Kd          = 0.220f;
int   baseSpeed   = 150;
int   maxSpeed    = 255;
int   searchSpeed = 110;
int   lineDetTh   = 900;    // tổng black-strength tối thiểu để xem là thấy line

#define DEFAULT_CENTER_POS 3500     // vị trí trung tâm mặc định cho 8 cảm biến
int centerPos = DEFAULT_CENTER_POS;   // tâm thực tế, sẽ tự tính lại nếu bỏ cảm biến

// =====================================================
// Trạng thái robot
// =====================================================
enum RobotState { ST_STOP, ST_CAL, ST_RUN };
RobotState robotState = ST_STOP;

int oledPage = 1;  // 0–3

// =====================================================
// Biến debug runtime
// =====================================================
int   lastPos      = DEFAULT_CENTER_POS;
int   lastErr      = 0;
float integralErr  = 0.0f;   // tích lũy integral PID
int   lastLPWM     = 0;
int   lastRPWM     = 0;
bool  lastFound    = false;
// =====================================================
// Calibration – 2 bước, không chặn loop
// Bước 1: đặt toàn bộ 8 mắt trên NỀN TRẮNG, lấy 500 mẫu
// Bước 2: đặt toàn bộ 8 mắt trên LINE ĐEN/mảng đen, bấm CAL, lấy 500 mẫu
// =====================================================
enum CalPhase { CAL_IDLE, CAL_WHITE_SAMPLING, CAL_WAIT_BLACK, CAL_BLACK_SAMPLING, CAL_DONE };
CalPhase      calPhase   = CAL_IDLE;
unsigned long calTimer   = 0;
int           calSamples = 0;

#define CAL_SAMPLE_N 500

uint32_t calWhiteSum[NUM_SENSORS];
uint32_t calBlackSum[NUM_SENSORS];
uint16_t sensorWhite [NUM_SENSORS];
uint16_t sensorBlack [NUM_SENSORS];

// =====================================================
// BLE – UUIDs & objects
// =====================================================
#define BLE_DEVICE_NAME "LineBot"

// Tạo UUIDs ngẫu nhiên cho service và các characteristic
#define SVC_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define TEL_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // Notify  – telemetry
#define CFG_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"  // R+W     – params
#define CMD_UUID "beb5483e-36e1-4688-b7f5-ea07361b26aa"  // Write   – lệnh

BLECharacteristic *pTelChar = nullptr;
BLECharacteristic *pCfgChar = nullptr;
bool bleConnected = false;

// Cờ giao tiếp giữa BLE callback (task riêng) và loop() chính
volatile bool bleCmdRun    = false;
volatile bool bleCmdStop   = false;
volatile bool bleCmdCal    = false;
volatile bool bleNewParams = false;

// Giá trị params tạm – được set trong callback, áp dụng trong loop()
volatile float vKp, vKi, vKd;
volatile int   vBase, vMax, vSrch, vLineTh;

// ─── BLE Server callbacks ──────────────────────────
class BleServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    bleConnected = true;
  }
  void onDisconnect(BLEServer *pSrv) override {
    bleConnected = false;
    // tự restart advertising để thiết bị khác có thể kết nối lại
    delay(500);
    pSrv->startAdvertising();
  }
};

// ─── Phân tích chuỗi config ───────────────────────
// Định dạng: "KP:0.050|KI:0.000|KD:0.220|BASE:150|MAX:255|SRCH:110|LINETH:900"
// Từng trường là tuỳ chọn (partial update OK)
void parseConfigString(const String &s,
                       float &kp, float &ki, float &kd,
                       int &base, int &mx, int &srch, int &lineTh) {
  kp=Kp; ki=Ki; kd=Kd; base=baseSpeed; mx=maxSpeed;
  srch=searchSpeed; lineTh=lineDetTh;  // giá trị mặc định

  int p = 0;
  while (p < (int)s.length()) {
    int pipe  = s.indexOf('|', p);
    if (pipe < 0) pipe = s.length();
    String tok = s.substring(p, pipe);
    int    col  = tok.indexOf(':');
    if (col > 0) {
      String key = tok.substring(0, col);
      String val = tok.substring(col + 1);
      if      (key == "KP")     kp     = val.toFloat();
      else if (key == "KI")     ki     = val.toFloat();
      else if (key == "KD")     kd     = val.toFloat();
      else if (key == "BASE")   base   = val.toInt();
      else if (key == "MAX")    mx     = val.toInt();
      else if (key == "SRCH")   srch   = val.toInt();
      else if (key == "LINETH") lineTh = val.toInt();
    }
    p = pipe + 1;
  }
}

// ─── Config characteristic callback ───────────────
class CfgCB : public BLECharacteristicCallbacks {
  // Laptop GHI params mới xuống ESP32
  void onWrite(BLECharacteristic *c) override {
    String v = c->getValue().c_str();

    // Parse vao bien thuong truoc, vi vKp/vKi/vKd/... la volatile
    float kpTmp, kiTmp, kdTmp;
    int baseTmp, maxTmp, srchTmp, lineThTmp;

    parseConfigString(v, kpTmp, kiTmp, kdTmp, baseTmp, maxTmp, srchTmp, lineThTmp);

    vKp     = kpTmp;
    vKi     = kiTmp;
    vKd     = kdTmp;
    vBase   = baseTmp;
    vMax    = maxTmp;
    vSrch   = srchTmp;
    vLineTh = lineThTmp;

    bleNewParams = true;
  }
  // Laptop ĐỌC params hiện tại từ ESP32
  void onRead(BLECharacteristic *c) override {
    char buf[90];
    snprintf(buf, sizeof(buf),
      "KP:%.3f|KI:%.4f|KD:%.3f|BASE:%d|MAX:%d|SRCH:%d|LINETH:%d",
      Kp, Ki, Kd, baseSpeed, maxSpeed, searchSpeed, lineDetTh);
    c->setValue(buf);
  }
};

// ─── Command characteristic callback ──────────────
class CmdCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String cmd = c->getValue().c_str();
    cmd.trim();
    if      (cmd == "RUN")  bleCmdRun  = true;
    else if (cmd == "STOP") bleCmdStop = true;
    else if (cmd == "CAL")  bleCmdCal  = true;
  }
};

// ─── Khởi tạo BLE ─────────────────────────────────
void setupBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(185);  // đủ cho cả telemetry lẫn config string

  BLEServer  *pSrv = BLEDevice::createServer();
  pSrv->setCallbacks(new BleServerCB());

  BLEService *pSvc = pSrv->createService(SVC_UUID);

  // Telemetry – chỉ NOTIFY (ESP32 → laptop)
  pTelChar = pSvc->createCharacteristic(TEL_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pTelChar->addDescriptor(new BLE2902());

  // Config – READ + WRITE (hai chiều)
  pCfgChar = pSvc->createCharacteristic(CFG_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pCfgChar->setCallbacks(new CfgCB());

  // Command – chỉ WRITE (laptop → ESP32)
  BLECharacteristic *pCmdChar = pSvc->createCharacteristic(CMD_UUID,
    BLECharacteristic::PROPERTY_WRITE);
  pCmdChar->setCallbacks(new CmdCB());

  pSvc->start();

  BLEAdvertising *pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SVC_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);  // giúp iOS/Android nhận diện tốt hơn
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising as \"" BLE_DEVICE_NAME "\"");
}

// ─── Gửi telemetry ─────────────────────────────────
void sendTelemetry() {
  static unsigned long lastT = 0;
  if (!bleConnected || millis() - lastT < 120) return;
  lastT = millis();

  char bw[9];
  for (int i = 0; i < 8; i++) bw[i] = sensorBW[i] ? '1' : '0';
  bw[8] = '\0';

  const char *st = (robotState == ST_RUN) ? "RUN"
                 : (robotState == ST_CAL) ? "CAL" : "STOP";

  char buf[80];
  snprintf(buf, sizeof(buf),
    "S:%s|P:%d|E:%d|L:%d|R:%d|F:%d|BW:%s",
    st, lastPos, lastErr, lastLPWM, lastRPWM, (int)lastFound, bw);

  pTelChar->setValue(buf);
  pTelChar->notify();
}

// =====================================================
// MUX – đọc cảm biến
// =====================================================
#define ADC_SETTLE_US   80    // HC4067 + ADC ESP32 can thoi gian on dinh sau khi doi kenh
#define ADC_SAMPLES      7    // lay 7 mau, loc median/trimmed mean
#define RAW_IIR_NUM      2    // loc IIR: raw = (old*2 + new)/3
#define RAW_IIR_DEN      3
#define BW_HYST         45    // hysteresis de BW khong nhay loan quanh nguong

void selectMux(uint8_t ch) {
  digitalWrite(MUX_S0,  ch       & 1);
  digitalWrite(MUX_S1, (ch >> 1) & 1);
  digitalWrite(MUX_S2, (ch >> 2) & 1);
}

static void sortSmall(uint16_t *a, int n) {
  for (int i = 1; i < n; i++) {
    uint16_t key = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > key) {
      a[j + 1] = a[j];
      j--;
    }
    a[j + 1] = key;
  }
}

uint16_t readMuxChRawStable(uint8_t ch) {
  selectMux(ch);

  // Sau khi doi kenh MUX, ADC ESP32 de bi dinh dien ap cua kenh truoc.
  // Vi vay cho lau hon, doc bo 2 mau dau roi moi lay mau that.
  delayMicroseconds(ADC_SETTLE_US);
  analogRead(MUX_SIG);
  delayMicroseconds(30);
  analogRead(MUX_SIG);
  delayMicroseconds(30);

  uint16_t a[ADC_SAMPLES];
  for (int i = 0; i < ADC_SAMPLES; i++) {
    a[i] = analogRead(MUX_SIG);
    delayMicroseconds(20);
  }

  sortSmall(a, ADC_SAMPLES);

  // Bo 2 mau thap nhat va 2 mau cao nhat, lay trung binh 3 mau giua.
  // Cach nay on hon average thuan khi ADC co spike.
  uint32_t sum = 0;
  for (int i = 2; i <= 4; i++) sum += a[i];
  return (uint16_t)(sum / 3);
}

void readAllRaw() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!SENSOR_ENABLED[i]) {
      // Cảm biến bị vô hiệu: đặt về 0, bỏ qua
      sensorRaw[i]  = 0;
      sensorFilt[i] = 0;
      continue;
    }
    uint16_t v = readMuxChRawStable(i);

    if (!filtReady) {
      sensorFilt[i] = v;
    } else {
      sensorFilt[i] = (uint16_t)(((uint32_t)sensorFilt[i] * RAW_IIR_NUM + v) / RAW_IIR_DEN);
    }

    sensorRaw[i] = sensorFilt[i];
  }
  filtReady = true;
}

// =====================================================
// Motor
// =====================================================
// driveMotor() không còn cần thiết – Drive::motor_run() xử lý nội bộ

void setMotors(int l, int r) {
  l = constrain(l, -255, 255);
  r = constrain(r, -255, 255);
  motorL.motor_run(l);
  motorR.motor_run(r);
  lastLPWM = l;  lastRPWM = r;
}

void stopMotors() {
  motorL.motor_stop();
  motorR.motor_stop();
  lastLPWM = 0;  lastRPWM = 0;
}

// =====================================================
// Sensor processing
// =====================================================
void clearCalibration() {
  filtReady = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
    sensorTh [i] = 2048;
  }
}

// Tự tính tâm line theo các mắt còn dùng.
// Nếu bỏ 1 mắt bên phải/trái thì PID không bị lệch tâm ảo nữa.
void computeCenterPos() {
  long sum = 0;
  int count = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!SENSOR_ENABLED[i]) continue;

    int idx = SENSOR_ORDER_REVERSED ? (NUM_SENSORS - 1 - i) : i;
    sum += idx * 1000;
    count++;
  }

  centerPos = (count > 0) ? (int)(sum / count) : DEFAULT_CENTER_POS;
}

void computeThresholds() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorMax[i] < sensorMin[i] + 10) sensorMax[i] = sensorMin[i] + 10;
    sensorTh[i] = (sensorMin[i] + sensorMax[i]) / 2;
  }
}

// =====================================================
// Biến mới – Quản lý Giai đoạn (State Machine)
// =====================================================
enum MapPhase {
  PHASE_NORMAL,     // Dò line bình thường đến góc cua 145 độ
  PHASE_TURN_145,   // Thực hiện code cứng cua 145 độ phải
  PHASE_INVERTED,   // Line trắng, nền đen
  PHASE_CIRCLE,     // Line vòng tròn
  PHASE_DOTTED,     // Line đứt đoạn
  PHASE_FINISH      // Về đích
};

MapPhase currentPhase = PHASE_NORMAL;
bool invertLineMode   = false;      // Cờ đảo màu line động
unsigned long phaseStartTime = 0;   // Lưu thời điểm bắt đầu 1 giai đoạn

int   lostLineDir   = 0;        // +1=mất về phải, -1=mất về trái
bool  wasReversing  = false;
float filtDErr      = 0.0f;
unsigned long lostLineTime = 0;

void processSensors() {
  readAllRaw();
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!SENSOR_ENABLED[i]) {
      // Cảm biến bị vô hiệu: luôn đặt về trắng
      sensorNorm[i] = 0;
      sensorBW[i]   = false;
      continue;
    }
    int lo = sensorMin[i], hi = sensorMax[i];
    if (hi < lo + 10) hi = lo + 10;

    long m = map(sensorRaw[i], lo, hi, 0, 1000);
    m = constrain(m, 0, 1000);

    bool wasBlack = sensorBW[i];
    
    // Đảo logic nếu đang ở giai đoạn Inverted (Line trắng nền đen)
    bool activeBlackIsHigh = invertLineMode ? !blackIsHigh : blackIsHigh;

    if (activeBlackIsHigh) {
      sensorNorm[i] = (uint16_t)m;
      if (wasBlack) sensorBW[i] = (sensorRaw[i] > sensorTh[i] - BW_HYST);
      else          sensorBW[i] = (sensorRaw[i] > sensorTh[i] + BW_HYST);
    } else {
      sensorNorm[i] = (uint16_t)(1000 - m);
      if (wasBlack) sensorBW[i] = (sensorRaw[i] < sensorTh[i] + BW_HYST);
      else          sensorBW[i] = (sensorRaw[i] < sensorTh[i] - BW_HYST);
    }
  }
}

bool getLinePos(int &posOut) {
  processSensors();
  long ws = 0, total = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!SENSOR_ENABLED[i]) continue;

    int idx = SENSOR_ORDER_REVERSED ? (NUM_SENSORS - 1 - i) : i;
    ws    += (long)sensorNorm[i] * idx * 1000;
    total += sensorNorm[i];
  }

  if (total < lineDetTh) return false;
  posOut = (int)(ws / total);
  return true;
}

// =====================================================
// Calibration 2 bước WHITE/BLACK – KHÔNG CHẶN loop()
// =====================================================
void resetCalSums() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    calWhiteSum[i] = 0;
    calBlackSum[i] = 0;
  }
}

void startCalibration() {
  robotState = ST_CAL;
  calibrated = false;
  stopMotors();
  clearCalibration();
  resetCalSums();
  calSamples = 0;
  calTimer   = millis();
  calPhase   = CAL_WHITE_SAMPLING;

  Serial.println("[CAL] Step 1/2: dat CA 8 MAT tren NEN TRANG, giu yen. Dang lay 500 mau WHITE...");
}

void startBlackSampling() {
  if (calPhase != CAL_WAIT_BLACK) return;

  for (int i = 0; i < NUM_SENSORS; i++) calBlackSum[i] = 0;
  calSamples = 0;
  calTimer   = millis();
  calPhase   = CAL_BLACK_SAMPLING;

  Serial.println("[CAL] Step 2/2: dat CA 8 MAT tren LINE DEN / MANG DEN, giu yen. Dang lay 500 mau BLACK...");
}

void finishTwoPointCalibration() {
  uint32_t sumWhiteAll = 0;
  uint32_t sumBlackAll = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorWhite[i] = (uint16_t)(calWhiteSum[i] / CAL_SAMPLE_N);
    sensorBlack[i] = (uint16_t)(calBlackSum[i] / CAL_SAMPLE_N);

    sensorMin[i] = min(sensorWhite[i], sensorBlack[i]);
    sensorMax[i] = max(sensorWhite[i], sensorBlack[i]);

    if (sensorMax[i] < sensorMin[i] + 20) {
      // Tránh chia cho 0 nếu mắt nào đó không khác biệt nhiều giữa trắng/đen
      sensorMax[i] = sensorMin[i] + 20;
    }

    sensorTh[i] = (sensorWhite[i] + sensorBlack[i]) / 2;

    sumWhiteAll += sensorWhite[i];
    sumBlackAll += sensorBlack[i];
  }

  // Với đa số module: đen có thể cho ADC thấp hơn hoặc cao hơn trắng.
  // Tự nhận diện polarity dựa trên trung bình cả 8 mắt.
  blackIsHigh = (sumBlackAll > sumWhiteAll);

  calibrated = true;
  calPhase   = CAL_DONE;

  Serial.println("[CAL] Done 2-point calibration.");
  Serial.printf("[CAL] blackIsHigh=%d | WHITE avg=%lu | BLACK avg=%lu\n",
                (int)blackIsHigh,
                (unsigned long)(sumWhiteAll / NUM_SENSORS),
                (unsigned long)(sumBlackAll / NUM_SENSORS));

  Serial.print("[CAL] WHITE=");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorWhite[i]);
    if (i != NUM_SENSORS - 1) Serial.print(',');
  }
  Serial.println();

  Serial.print("[CAL] BLACK=");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorBlack[i]);
    if (i != NUM_SENSORS - 1) Serial.print(',');
  }
  Serial.println();

  Serial.print("[CAL] TH=");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorTh[i]);
    if (i != NUM_SENSORS - 1) Serial.print(',');
  }
  Serial.println();
}

void updateCalibration() {
  if (calPhase == CAL_IDLE) return;

  switch (calPhase) {

    case CAL_WHITE_SAMPLING:
      readAllRaw();
      for (int i = 0; i < NUM_SENSORS; i++) {
        calWhiteSum[i] += sensorRaw[i];
      }
      calSamples++;

      if (calSamples >= CAL_SAMPLE_N) {
        for (int i = 0; i < NUM_SENSORS; i++) {
          sensorWhite[i] = (uint16_t)(calWhiteSum[i] / CAL_SAMPLE_N);
        }

        calSamples = 0;
        calPhase   = CAL_WAIT_BLACK;
        calTimer   = millis();

        Serial.println("[CAL] WHITE sampled x500 OK.");
        Serial.println("[CAL] Dat CA 8 MAT len LINE DEN / MANG DEN roi BAM CAL lan nua de lay BLACK x500.");
      }
      break;

    case CAL_WAIT_BLACK:
      // Đang chờ người đặt cảm biến lên đen và bấm CAL lần nữa.
      // Vẫn đọc raw để OLED/Serial dễ quan sát.
      readAllRaw();
      break;

    case CAL_BLACK_SAMPLING:
      readAllRaw();
      for (int i = 0; i < NUM_SENSORS; i++) {
        calBlackSum[i] += sensorRaw[i];
      }
      calSamples++;

      if (calSamples >= CAL_SAMPLE_N) {
        finishTwoPointCalibration();
      }
      break;

    case CAL_DONE:
      // Calib xong thì đứng yên ở STOP, không tự lao xe.
      robotState = ST_STOP;
      stopMotors();
      calPhase = CAL_IDLE;
      lastErr = 0;
      Serial.println("[CAL] Ready. Bam RUN de chay.");
      break;

    default:
      break;
  }
}

// =====================================================
// OLED – 4 trang
// =====================================================
const char *stateStr() {
  if (robotState == ST_RUN) return "RUN";
  if (robotState == ST_CAL) return "CAL";
  return "STOP";
}

void drawCalPage() {
  display.setCursor(0, 0);
  display.print(">>> CALIB 2-POINT <<<");

  switch (calPhase) {
    case CAL_WHITE_SAMPLING: {
      int pct = calSamples * 100 / CAL_SAMPLE_N;
      display.setCursor(0, 12); display.print("1/2 NEN TRANG");
      display.setCursor(0, 24); display.print("Giu yen ca 8 mat");
      display.setCursor(0, 36); display.print("Sample: ");
      display.print(calSamples);
      display.print("/");
      display.print(CAL_SAMPLE_N);
      display.drawRect(0, 50, 128, 9, SSD1306_WHITE);
      display.fillRect(1, 51, pct * 126 / 100, 7, SSD1306_WHITE);
      break;
    }

    case CAL_WAIT_BLACK:
      display.setCursor(0, 12); display.print("WHITE OK x500");
      display.setCursor(0, 24); display.print("2/2 Dat len DEN");
      display.setCursor(0, 36); display.print("roi BAM CAL nua");
      display.setCursor(0, 50); display.print("Can DEN rong ca 8 mat");
      break;

    case CAL_BLACK_SAMPLING: {
      int pct2 = calSamples * 100 / CAL_SAMPLE_N;
      display.setCursor(0, 12); display.print("2/2 LINE DEN");
      display.setCursor(0, 24); display.print("Giu yen ca 8 mat");
      display.setCursor(0, 36); display.print("Sample: ");
      display.print(calSamples);
      display.print("/");
      display.print(CAL_SAMPLE_N);
      display.drawRect(0, 50, 128, 9, SSD1306_WHITE);
      display.fillRect(1, 51, pct2 * 126 / 100, 7, SSD1306_WHITE);
      break;
    }

    default:
      break;
  }
}

void drawPageRaw() {
  bool hi = ((millis() / 1200) % 2) == 1;
  int  s  = hi ? 4 : 0;
  display.setCursor(0, 0); display.print("RAW "); display.print(hi ? "4-7" : "0-3");
  display.print("  "); display.print(stateStr());
  for (int r = 0; r < 4; r++) {
    int ch = s + r;
    display.setCursor(0, 14 + r * 12);
    display.print("C"); display.print(ch); display.print(":"); display.print(sensorRaw[ch]);
  }
}

void drawPageBW() {
  // ── Bar chart: mỗi mắt enabled là 1 cột ──────────────
  // Layout 128×64: 54px cho cột, 10px dưới cùng cho label số mắt
  const int BAR_AREA_H = 54;
  const int BAR_H_MAX  = BAR_AREA_H - 2;  // chiều cao khi CÓ line
  const int BAR_H_MIN  = 4;               // chiều cao khi KHÔNG có line

  // Đếm số mắt enabled
  int enabledCount = 0;
  for (int i = 0; i < NUM_SENSORS; i++)
    if (SENSOR_ENABLED[i]) enabledCount++;

  // Tính độ rộng cột, khoảng cách 2px giữa các cột
  const int GAP = 2;
  int colW = (SCREEN_W - GAP * (enabledCount - 1)) / enabledCount;
  if (colW < 1) colW = 1;

  int x = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!SENSOR_ENABLED[i]) continue;

    int fillH = sensorBW[i] ? BAR_H_MAX : BAR_H_MIN;
    int y     = BAR_AREA_H - fillH;  // cột mọc từ dưới lên

    if (sensorBW[i])
      display.fillRect(x, y, colW, fillH, SSD1306_WHITE);  // đặc = có line
    else
      display.drawRect(x, y, colW, fillH, SSD1306_WHITE);  // viền = không có line

    x += colW + GAP;
  }

  // ── Label số mắt ở dưới cùng ─────────────────────────
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  x = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!SENSOR_ENABLED[i]) continue;
    int labelX = x + colW / 2 - 3;
    display.setCursor(labelX < 0 ? 0 : labelX, 56);
    display.print(i);
    x += colW + GAP;
  }
}

void drawPageThreshold() {
  bool hi = ((millis() / 1200) % 2) == 1;
  int  s  = hi ? 4 : 0;
  display.setCursor(0, 0); display.print("TH "); display.print(hi ? "4-7" : "0-3");
  display.print(calibrated ? " OK" : " NO");
  for (int r = 0; r < 4; r++) {
    int ch = s + r;
    display.setCursor(0, 14 + r * 12);
    display.print("C"); display.print(ch); display.print(":"); display.print(sensorTh[ch]);
  }
}

void drawPageMotor() {
  display.setCursor(0,  0); display.print("PID  "); display.print(stateStr());
  display.setCursor(0, 10); display.print("Kp:"); display.print(Kp, 3);
  display.setCursor(64, 10); display.print("Ki:"); display.print(Ki, 4);
  display.setCursor(0, 20); display.print("Kd:"); display.print(Kd, 3);
  display.setCursor(64, 20); display.print("I:"); display.print((int)integralErr);
  display.setCursor(0, 30); display.print("base:"); display.print(baseSpeed);
  display.print(" max:"); display.print(maxSpeed);
  display.setCursor(0, 40); display.print("srch:"); display.print(searchSpeed);
  display.print(" th:"); display.print(lineDetTh);
  display.setCursor(0, 50); display.print("L:"); display.print(lastLPWM);
  display.print("  R:"); display.print(lastRPWM);
  display.setCursor(0, 58); display.print("BLE:");
  display.print(bleConnected ? "ON " : "-- ");
  display.print(BLE_DEVICE_NAME);
}

extern uint8_t nv; // Biến nv từ main.cpp

void renderOLED() {
  static unsigned long lastT = 0;
  if (millis() - lastT < 80) return;
  lastT = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (calPhase != CAL_IDLE) {
    drawCalPage();
  } else {
    switch (oledPage) {
      case 0: drawPageRaw();       break;
      case 1: drawPageBW();        break;
      case 2: drawPageThreshold(); break;
      case 3: drawPageMotor();     break;
      default: drawPageRaw();      break;
    }
  }

  // Luôn hiển thị chế độ NV góc trên bên phải
  display.setTextSize(1);
  // Vẽ nền đen chèn lên chữ cũ nếu bị đè
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); 
  display.setCursor(100, 0);
  display.print("NV:");
  display.print(nv);

  display.display();
}

// =====================================================
// Serial debug
// =====================================================
void renderSerial() {
  static unsigned long lastT = 0;
  if (millis() - lastT < 200) return;
  lastT = millis();

  Serial.printf("STATE=%s POS=%d ERR=%d L=%d R=%d BW=",
    stateStr(), lastPos, lastErr, lastLPWM, lastRPWM);
  for (int i = 0; i < NUM_SENSORS; i++) Serial.print(sensorBW[i] ? '1' : '0');
  Serial.print(" RAW=");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorRaw[i]);
    if (i != NUM_SENSORS - 1) Serial.print(',');
  }
  Serial.println();
}

// =====================================================
// Debug – Hiển thị trạng thái mắt đọc line dạng cột dọc
// =====================================================
//
//  Ví dụ output (Serial Monitor):
//
//   S1  S2  S3  S4  S5  S6  S7
//  ┌──┐     ┌──┐          ┌──┐
//  │██│     │██│          │██│
//  │██│     │██│          │██│
//  │██│     │██│          │██│
//  └──┘     └──┘          └──┘
//  [1] [ ] [1] [ ] [ ] [ ] [1]   BW
//
//  Mắt disabled (S0) bị ẩn hoàn toàn.
//  Chiều cao cột tỉ lệ theo sensorNorm (0–1000).
// ─────────────────────────────────────────────────────

void printSensorBar() {
  static unsigned long lastT = 0;
  if (millis() - lastT < 150) return;   // ~6 fps, không làm tắc Serial
  lastT = millis();

  const int BAR_HEIGHT = 6;             // Số dòng hiển thị chiều cao cột
  const int COL_W     = 4;             // Độ rộng 1 cột (ký tự)

  // ── Header tên mắt ──────────────────────────────────
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!SENSOR_ENABLED[i]) continue;
    char buf[COL_W + 1];
    snprintf(buf, sizeof(buf), " S%-*d", COL_W - 1, i);
    Serial.print(buf);
  }
  Serial.println();

  // ── Cột dọc tỉ lệ theo sensorNorm ───────────────────
  // sensorNorm: 0 = trắng, 1000 = đen đậm nhất
  for (int row = BAR_HEIGHT; row >= 1; row--) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (!SENSOR_ENABLED[i]) continue;
      // Ngưỡng để hiển thị block ở dòng này
      int threshold = (int)((float)(row - 1) / BAR_HEIGHT * 1000.0f);
      bool fill = (sensorNorm[i] >= (uint16_t)threshold);
      // Thêm dấu sao nếu vượt ngưỡng BW (sensorBW = true)
      if (fill && sensorBW[i])  Serial.print(" [*]");
      else if (fill)            Serial.print(" [|]");
      else                      Serial.print("    ");
    }
    Serial.println();
  }

  // ── Dòng đáy: nhãn BW nhị phân ──────────────────────
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!SENSOR_ENABLED[i]) continue;
    Serial.print(sensorBW[i] ? " [1]" : " [ ]");
  }
  Serial.println("  BW");
  Serial.println();   // Dòng trống phân cách frame
}

// =====================================================
// Xử lý nút và lệnh BLE
// =====================================================
void handleInputs() {
  // Nhận tín hiệu từ cả GPIO2, GPIO4 và BLE
  bool pressed = buttonPressedEvent() || button2PressedEvent();

  // ── 1 nút duy nhất, pull-down ngoài, active HIGH, chỉ debounce ─────────
  // Bấm lần 1 khi chưa calib        : lấy WHITE x500
  // Bấm lần 2 khi OLED báo WHITE OK : lấy BLACK x500
  // Sau khi calib xong              : RUN / STOP
  if (pressed) {
    if (robotState == ST_CAL && calPhase == CAL_WAIT_BLACK) {
      startBlackSampling();
    } else if (!calibrated && calPhase == CAL_IDLE) {
      startCalibration();
    } else if (calibrated && calPhase == CAL_IDLE) {
      if (robotState == ST_RUN) {
        robotState = ST_STOP;
        stopMotors();
        Serial.println("[BTN] STOP");
      } else {
        robotState  = ST_RUN;
        lastErr     = 0;
        integralErr = 0.0f;
        Serial.println("[BTN] RUN");
      }
    }
  }

  // ── Lệnh BLE (được set trong BLE callback task) ─
  // BLE vẫn giữ lại để debug/chỉnh PID. CAL trên dashboard vẫn dùng được dự phòng,
  // nhưng quy trình chính bây giờ là bấm nút vật lý.
  if (bleNewParams) {
    bleNewParams = false;
    // Áp dụng và kẹp giới hạn an toàn
    Kp          = constrain(vKp,    0.000f, 50.000f);
    Ki          = constrain(vKi,    0.000f, 20.000f);
    Kd          = constrain(vKd,    0.000f,100.000f);
    baseSpeed   = constrain(vBase,  0, 255);
    maxSpeed    = constrain(vMax,   0, 255);
    searchSpeed = constrain(vSrch,  0, 255);
    lineDetTh   = constrain(vLineTh, 50, 8000);
    integralErr = 0.0f;  // reset tích phân khi đổi params
    Serial.printf("[BLE] Params: Kp=%.3f Ki=%.4f Kd=%.3f base=%d max=%d srch=%d lineTh=%d\n",
      Kp, Ki, Kd, baseSpeed, maxSpeed, searchSpeed, lineDetTh);

    // ── Lưu vào NVS để giữ sau khi reboot ───────
    prefs.begin("linebot", false);
    prefs.putFloat("kp",     Kp);
    prefs.putFloat("ki",     Ki);
    prefs.putFloat("kd",     Kd);
    prefs.putInt  ("base",   baseSpeed);
    prefs.putInt  ("max",    maxSpeed);
    prefs.putInt  ("srch",   searchSpeed);
    prefs.putInt  ("lineth", lineDetTh);
    prefs.end();
    Serial.println("[NVS] Params saved.");
  }

  if (bleCmdRun) {
    bleCmdRun = false;
    if (calibrated && calPhase == CAL_IDLE) {
      robotState  = ST_RUN;
      lastErr     = 0;
      integralErr = 0.0f;
      Serial.println("[BLE] RUN");
    } else {
      Serial.println("[BLE] RUN ignored: chua calib xong.");
    }
  }

  if (bleCmdStop) {
    bleCmdStop = false;
    robotState = ST_STOP;
    stopMotors();
    Serial.println("[BLE] STOP");
  }

  if (bleCmdCal) {
    bleCmdCal = false;
    if (robotState == ST_CAL && calPhase == CAL_WAIT_BLACK) {
      startBlackSampling();
    } else {
      startCalibration();
    }
  }
}

// =====================================================
// Line following
// =====================================================
/*
 * ============================================================
 * LineBot – runLineFollower() đã sửa
 * Thay thế toàn bộ phần "Line following" trong LineBot.cpp
 * Giữ nguyên tất cả phần còn lại (BLE, OLED, Calibration,...)
 * ============================================================
 */

// =====================================================


// =====================================================
// Line following – SỬA TOÀN BỘ
// =====================================================

void runBasicPID(int customSpeed) {
  int pos = centerPos;
  bool found = getLinePos(pos);
  lastFound = found;  // [FIX] Cập nhật trạng thái cho telemetry / OLED

  // Mảng tĩnh lưu lại trạng thái của cảm biến ở lần đọc có line gần nhất đã được chuyển thành biến toàn cục

  if (!found) {
    // [FIX] Reset tích phân khi mất line để tránh drift khi bắt lại
    integralErr = 0.0f;

    // Khi mất line, phân tích mảng lastSensorBW để xem vạch đen nằm ở đâu
    int leftWeight  = 0;
    int rightWeight = 0;

    // Đánh trọng số cao hơn cho các mắt ở xa tâm (bên trái: 1, 2, 3)
    // if (lastSensorBW[0]) leftWeight += 4;
    if (lastSensorBW[1]) leftWeight += 3*3;
    if (lastSensorBW[2]) leftWeight += 2*2;
    if (lastSensorBW[3]) leftWeight += 1;

    // Đánh trọng số cao hơn cho các mắt ở xa tâm (bên phải: 6, 5, 4)
    // if (lastSensorBW[7]) rightWeight += 4;
    if (lastSensorBW[6]) rightWeight += 3*3;
    if (lastSensorBW[5]) rightWeight += 2*2;
    if (lastSensorBW[4]) rightWeight += 1;

    int spinSpeed = searchSpeed;  // Tốc độ xoay tìm line (mặc định 110)

    if (rightWeight > leftWeight) {
      // Vạch đen nằm ở nửa phải trước khi mất -> Quay phải
      setMotors(spinSpeed, -spinSpeed);
    } else if (leftWeight > rightWeight) {
      // Vạch đen nằm ở nửa trái trước khi mất -> Quay trái
      setMotors(-spinSpeed, spinSpeed);
    } else {
      // Nếu không phân biệt được, dự phòng bằng lastErr
      if      (lastErr > 0) setMotors( spinSpeed, -spinSpeed);
      else if (lastErr < 0) setMotors(-spinSpeed,  spinSpeed);
      else                  stopMotors();
    }
    return;
  }

  // Nếu vẫn thấy line, cập nhật mảng giá trị cũ
  for (int i = 0; i < NUM_SENSORS; i++) {
    lastSensorBW[i] = sensorBW[i];
  }

  // ── Hết phát hiện ngã rẽ ───────────────────────────────────────────────

  int err  = pos - centerPos;
  int dErr = err - lastErr;
  lastErr  = err;

  // Hệ số lọc D: 0.7 old / 0.3 new – giảm nhiễu, đủ nhạy
  filtDErr = filtDErr * 0.70f + (float)dErr * 0.30f;

  // Tích phân (I) – chỉ tích lũy khi err nhỏ (anti-windup đơn giản)
  if (abs(err) < 2000) {
    integralErr += err;
    integralErr  = constrain(integralErr, -5000.0f, 5000.0f);
  }

  // Tính giá trị bù PID
  float corr = Kp * (float)err + Ki * integralErr + Kd * filtDErr;

  // Cho phép PWM âm (-maxSpeed..maxSpeed) để bánh có thể phanh/lùi khi cua gấp
  int lPWM = constrain(customSpeed + (int)corr, -maxSpeed, maxSpeed);
  int rPWM = constrain(customSpeed - (int)corr, -maxSpeed, maxSpeed);

  setMotors(lPWM, rPWM);
}

void executePID(bool found, int pos, int customBaseSpeed) {
  if (!found) return; // Nếu mất line thì State Machine bên trên sẽ lo

  int   err  = pos - centerPos;
  int   dErr = err - lastErr;
  lastErr    = err;

  filtDErr = filtDErr * 0.55f + (float)dErr * 0.45f;
  integralErr += err;
  integralErr  = constrain(integralErr, -5000.0f, 5000.0f);

  float corr = Kp * (float)err + Ki * integralErr + Kd * filtDErr;

  // Ngưỡng động theo tốc độ truyền vào
  int TH_SOFT = map(customBaseSpeed, 60, 255, 600,  1200);
  int TH_HARD = map(customBaseSpeed, 60, 255, 1100, 2200);
  TH_SOFT = constrain(TH_SOFT, 500,  1400);
  TH_HARD = constrain(TH_HARD, 900,  2400);

  int absErr = abs(err);
  int lPWM, rPWM;

  if (absErr < TH_SOFT) {
    if (wasReversing) { integralErr *= 0.25f; wasReversing = false; }
    corr = constrain(corr, -(float)maxSpeed, (float)maxSpeed);
    lPWM = constrain(customBaseSpeed + (int)corr, 0, maxSpeed);
    rPWM = constrain(customBaseSpeed - (int)corr, 0, maxSpeed);
  } else if (absErr < TH_HARD) {
    int reducedBase = map(absErr, TH_SOFT, TH_HARD, customBaseSpeed, customBaseSpeed * 65 / 100);
    reducedBase = constrain(reducedBase, 60, maxSpeed);
    corr = constrain(corr, -(float)maxSpeed, (float)maxSpeed);
    lPWM = constrain(reducedBase + (int)corr, 0, maxSpeed);
    rPWM = constrain(reducedBase - (int)corr, 0, maxSpeed);
    wasReversing = false;
  } else {
    wasReversing = true;
    int outerPWM = map(absErr, TH_HARD, 3500, customBaseSpeed * 80 / 100, maxSpeed * 85 / 100);
    outerPWM = constrain(outerPWM, 70, maxSpeed);
    int innerPWM = map(absErr, TH_HARD, 3500, 60, maxSpeed * 70 / 100);
    innerPWM = constrain(innerPWM, 55, maxSpeed);

    if (err > 0) { lPWM =  outerPWM; rPWM = -innerPWM; } 
    else         { lPWM = -innerPWM; rPWM =  outerPWM; }
  }
  setMotors(lPWM, rPWM);
}
void runLineFollower() {
  int pos = centerPos;
  bool found = getLinePos(pos);

  // Tạo mask 8-bit từ 8 mắt cảm biến (mắt 0 là bit 0, mắt 7 là bit 7)
  uint8_t mask = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorBW[i]) {
      mask |= (1 << i);
    }
  }

  // Ghi nhớ hướng khi vừa mất line
  if (lastFound && !found) {
    lostLineDir = (lastErr >= 0) ? 1 : -1;
    if ((sensorBW[3] || sensorBW[4]) && !sensorBW[0] && !sensorBW[7]) lostLineDir = 0;
  }
  lastFound = found;
  if (found) {
    lastPos = pos;
  }

  // ==============================================================
  // XỬ LÝ THEO MASK (Thay thế State Machine cũ)
  // ==============================================================
  
  switch (mask) {
    case 0b00000000: {
      // Trường hợp 1: mask = 00000000 (Mất line hoàn toàn)
      int spinPWM = searchSpeed;
      if (lostLineDir > 0) {
        setMotors(spinPWM, -spinPWM); // Quay phải
      } else if (lostLineDir < 0){
        setMotors(-spinPWM, spinPWM); // Quay trái
      } else {
        setMotors(baseSpeed, baseSpeed);
      }
      break; // Khi found = false, executePID bên dưới sẽ tự return sớm, không sao cả.
    }
    case 0b11111111: {
      // Trường hợp 2: mask = 11111111 (Vạch ngang, ngã tư, đi qua đường tròn)
      setMotors(baseSpeed, baseSpeed);
      delay(1000);
      return; // <--- QUAN TRỌNG: return để KHÔNG bị executePID ghi đè setMotors bên trên!
    }
    case 0b11100111:
    case 0b11110111:
    case 0b11101111:
    case 0b11001111:
    case 0b11110011: {
      // XỬ LÝ ĐẢO LINE (Line trắng nền đen)
      static unsigned long lastInvertTime = 0;
      // Chống dội (debounce) 500ms để không bị đảo liên tục khi xe chần chừ ở ranh giới
      if (millis() - lastInvertTime > 500) {
        invertLineMode = !invertLineMode;
        lastInvertTime = millis();
        // Serial.println(invertLineMode ? "[INFO] ENTERED INVERTED LINE" : "[INFO] EXITED INVERTED LINE");
      }
      return; // Return bỏ qua loop này, chờ vòng lặp sau cập nhật lại mask mới
    }
    case 0b10011000:
    case 0b10001000:
    case 0b10010000:
    case 0b01011000:
    case 0b01001000:
    case 0b01010000: {
      // Trường hợp rẽ gắt
      setMotors(searchSpeed, -searchSpeed);
      delay(100);
      return; // <--- QUAN TRỌNG: return để KHÔNG bị executePID ghi đè
    }
    default: {
      // Các trường hợp còn lại (Thấy line một phần)
      // -> Dò line bằng PID bình thường
      executePID(found, pos, baseSpeed);
      break;
    }
  }
}
// =====================================================
// Setup
// =====================================================
void begin() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] LineBot v2.4 active-high button");

  // Nút bấm – GPIO2 & GPIO4 (pull-down ngoài, active HIGH)
  pinMode(BTN_ONE_PIN, INPUT);
  pinMode(BTN_TWO_PIN, INPUT);

  // MUX
  pinMode(MUX_S0, OUTPUT); pinMode(MUX_S1, OUTPUT); pinMode(MUX_S2, OUTPUT);
  digitalWrite(MUX_S0, LOW); digitalWrite(MUX_S1, LOW); digitalWrite(MUX_S2, LOW);

  // Motor – khởi tạo qua Drive (GA25Motor)
  // motor_init(in1, in2, pwmPin, encA, encB, ledcCh)
  motorL.motor_init(AIN1, AIN2, PWMA, ENAL, ENBL, CH_A);
  motorR.motor_init(BIN1, BIN2, PWMB, ENAR, ENBR, CH_B);

  // ADC
  analogReadResolution(12);
  analogSetPinAttenuation(MUX_SIG, ADC_11db);  // doc du dai 0-3.3V hon, hop voi cam bien line dung VCC 3V3

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[ERR] OLED init failed!");
    while (1) delay(1000);
  }

  clearCalibration();
  computeCenterPos();
  Serial.printf("[SENSOR] centerPos=%d\n", centerPos);
  stopMotors();
  readAllRaw();

  // Load params từ NVS (giữ lại giá trị đã chỉnh qua BLE)
  prefs.begin("linebot", true);   // read-only
  Kp          = prefs.getFloat("kp",     Kp);
  Ki          = prefs.getFloat("ki",     Ki);
  Kd          = prefs.getFloat("kd",     Kd);
  baseSpeed   = prefs.getInt  ("base",   baseSpeed);
  maxSpeed    = prefs.getInt  ("max",    maxSpeed);
  searchSpeed = prefs.getInt  ("srch",   searchSpeed);
  lineDetTh   = prefs.getInt  ("lineth", lineDetTh);
  prefs.end();
  Serial.printf("[NVS] Loaded: Kp=%.3f Ki=%.4f Kd=%.3f base=%d max=%d srch=%d lineTh=%d\n",
    Kp, Ki, Kd, baseSpeed, maxSpeed, searchSpeed, lineDetTh);

  // BLE
  setupBLE();

  // Splash screen
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10,  0); display.print("== LINE BOT v2.5 ==");
  display.setCursor(0,  10); display.print("BLE: "); display.print(BLE_DEVICE_NAME);
  display.setCursor(0,  20); display.print("Nut IO2 & IO4 ACT-HI");
  display.setCursor(0,  30); display.print("[1] WHITE x500");
  display.setCursor(0,  39); display.print("[2] BLACK x500");
  display.setCursor(0,  48); display.print("[3+] RUN / STOP");
  display.setCursor(0,  57); display.print("BLE: CAL/RUN/STOP ok");
  display.display();
  delay(2500);
}

// =====================================================
// Loop
// =====================================================
void update() {
  handleInputs();
  updateCalibration();

  if (robotState == ST_RUN && calibrated && calPhase == CAL_IDLE) {
    runLineFollower();
  } else {
    // Không chạy → dừng motor, vẫn đọc cảm biến để debug
    if (robotState != ST_CAL) stopMotors();
    if (calibrated) processSensors();
    else            readAllRaw();
  }

  renderOLED();
  renderSerial();
  sendTelemetry();
}

void update2() {
  Kp          = 0.04f;
  Ki          = 0.0f;
  Kd          = 0.4f;
  baseSpeed   = 70;
  maxSpeed    = 150;
  searchSpeed = 130;

  // [FIX] Khởi tạo calib mặc định một lần duy nhất
  // Thay WHITE_VAL / BLACK_VAL bằng giá trị đo thực tế của bạn
  static bool calInitDone = false;
  if (!calInitDone) {
    const uint16_t WHITE_VAL = 3200;  // ← đo thực tế ADC trên nền trắng
    const uint16_t BLACK_VAL =  800;  // ← đo thực tế ADC trên line đen
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorMin[i]   = BLACK_VAL;
      sensorMax[i]   = WHITE_VAL;
      sensorTh[i]    = (WHITE_VAL + BLACK_VAL) / 2;  // = 2000
      sensorWhite[i] = WHITE_VAL;
      sensorBlack[i] = BLACK_VAL;
    }
    blackIsHigh = false;  // ADC thấp = đen (điều chỉnh nếu module của bạn ngược)
    calibrated  = true;
    calInitDone = true;
    filtReady   = false;  // cho IIR khởi động lại sạch
  }

  // processSensors();
  runBasicPID(baseSpeed);
  renderOLED();
} 

void runCalibration() {
  stopMotors();

  // Nhả nút nếu đang giữ
  while(digitalRead(2) == HIGH) { delay(10); }
  delay(50);

  // ── Chờ bấm WHITE ─────────────────────────────────
  while(digitalRead(2) == LOW) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,  0); display.println("=== CALIBRATION ===");
    display.setCursor(0, 12); display.println("1. Dat tren NEN TRANG");
    display.setCursor(0, 24); display.println("   -> Bam IO2");
    display.display();
    delay(10);
  }
  while(digitalRead(2) == HIGH) { delay(10); }
  delay(50);

  // ── Lấy mẫu WHITE – update OLED theo tiến trình ───
  startCalibration();  // calPhase = CAL_WHITE_SAMPLING
  while (calPhase == CAL_WHITE_SAMPLING) {
    updateCalibration();
    // [FIX] Vẽ progress bar trực tiếp
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,  0); display.println(">>> CALIB 2-POINT <<<");
    display.setCursor(0, 12); display.println("1/2 NEN TRANG");
    display.setCursor(0, 24); display.println("Giu yen ca 8 mat");
    display.setCursor(0, 36); display.print("Sample: ");
    display.print(calSamples); display.print("/"); display.print(CAL_SAMPLE_N);
    int pct = calSamples * 100 / CAL_SAMPLE_N;
    display.drawRect(0, 50, 128, 9, SSD1306_WHITE);
    display.fillRect(1, 51, pct * 126 / 100, 7, SSD1306_WHITE);
    display.display();
    delay(2);
  }

  // ── Chờ bấm BLACK ─────────────────────────────────
  while(digitalRead(2) == LOW) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,  0); display.println(">>> CALIB 2-POINT <<<");
    display.setCursor(0, 12); display.println("WHITE OK x500");
    display.setCursor(0, 24); display.println("2/2 Dat len LINE DEN");
    display.setCursor(0, 36); display.println("   -> Bam IO2");
    display.display();
    delay(10);
  }
  while(digitalRead(2) == HIGH) { delay(10); }
  delay(50);

  // ── Lấy mẫu BLACK ─────────────────────────────────
  startBlackSampling();  // calPhase = CAL_BLACK_SAMPLING
  while (calPhase == CAL_BLACK_SAMPLING) {
    updateCalibration();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,  0); display.println(">>> CALIB 2-POINT <<<");
    display.setCursor(0, 12); display.println("2/2 LINE DEN");
    display.setCursor(0, 24); display.println("Giu yen ca 8 mat");
    display.setCursor(0, 36); display.print("Sample: ");
    display.print(calSamples); display.print("/"); display.print(CAL_SAMPLE_N);
    int pct2 = calSamples * 100 / CAL_SAMPLE_N;
    display.drawRect(0, 50, 128, 9, SSD1306_WHITE);
    display.fillRect(1, 51, pct2 * 126 / 100, 7, SSD1306_WHITE);
    display.display();
    delay(2);
  }

  // Lúc này calPhase = CAL_DONE → gọi updateCalibration() một lần để chuyển về CAL_IDLE
  updateCalibration();  // xử lý case CAL_DONE: set calibrated=true, calPhase=CAL_IDLE

  // ── Xong, chờ bấm IO4 để chạy ─────────────────────
  while(digitalRead(4) == LOW) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,  0); display.println("CALIB THANH CONG!");
    display.setCursor(0, 16); display.println("-> Bam IO4 de CHAY");
    display.display();
    delay(10);
  }
  while(digitalRead(4) == HIGH) { delay(10); }
  delay(50);
}

void test() {
  handleInputs();
  updateCalibration();

  if (robotState == ST_RUN && calibrated && calPhase == CAL_IDLE) {
    runBasicPID(baseSpeed);
  } else {  
    // Không chạy → dừng motor, vẫn đọc cảm biến để debug
    if (robotState != ST_CAL) stopMotors();
    if (calibrated) { processSensors(); }
    else              readAllRaw();
  }

  renderOLED();
  // renderSerial();
  sendTelemetry();
}

// =====================================================
// DEBUG OLED: Xem 2 mảng BW và LAST_BW cùng lúc
// =====================================================
void drawDebugBWOLED() {
  processSensors();
  display.clearDisplay();
  
  int colW = 12;
  int gap = 2;
  int startX = 8;
  
  // Hàng 1: sensorBW (NOW)
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("NOW:");
  
  int x = startX;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorBW[i]) display.fillRect(x, 10, colW, 20, SSD1306_WHITE);
    else             display.drawRect(x, 10, colW, 20, SSD1306_WHITE);
    x += colW + gap;
  }
  
  // Hàng 2: lastSensorBW (LAST)
  display.setCursor(0, 34);
  display.print("LAST:");
  
  x = startX;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (lastSensorBW[i]) display.fillRect(x, 44, colW, 20, SSD1306_WHITE);
    else                 display.drawRect(x, 44, colW, 20, SSD1306_WHITE);
    x += colW + gap;
  }
  
  display.display();
}

} // namespace LineBot