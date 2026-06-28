#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <SimpleFOC.h>
#include "HX710AB.h"
#include "DisplayManager.h"
#include "ImageManager.h"
#include "MotorManager.h"

static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 240;

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight);

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 50];

static void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)&color_p->full, w * h, true);
  tft.endWrite();
  lv_disp_flush_ready(disp_drv);
}

static void hardResetTFT() {
  pinMode(TFT_RST, OUTPUT);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, LOW);
  digitalWrite(TFT_RST, HIGH);
  delay(10);
  digitalWrite(TFT_RST, LOW);
  delay(20);
  digitalWrite(TFT_RST, HIGH);
  delay(120);
  digitalWrite(TFT_BL, HIGH);
}

// ---------- Motor ----------
#define kp_stiffness 5
#define gearnum 30

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MotorManager motor_manager(sensor, gearnum, kp_stiffness);
DisplayManager display;

// ---------- Button & HX710 press ----------
#define BUTTON_PIN 4
#define DOUT 35
#define PD_SCK 36
#define debounceDelay 500

HX710B HX(DOUT, PD_SCK);
int32_t baselineValue = 0;
int32_t NOISE_THRESHOLD = 500000;
int32_t TRIGGER_THRESHOLD = 1000000;
unsigned long lastPressTime = 0;
unsigned long currentTime = 0;
bool PressedFlag = false;
volatile bool PushbuttonPressed = false;
volatile unsigned long lastInterruptTime = 0;
volatile uint32_t pushCount = 0;

void calculateBaseline() {
  const int samplingInterval = 50;
  const int samplingDuration = 1000;
  int32_t sum = 0;
  int count = 0;
  uint32_t startTime = millis();
  Serial.println("Calculating baseline...");
  while (millis() - startTime < samplingDuration) {
    int32_t rawValue = HX.read();
    if (rawValue > NOISE_THRESHOLD) {
      sum += rawValue;
      count++;
    }
    delay(samplingInterval);
  }
  if (count > 0) {
    baselineValue = sum / count;
    TRIGGER_THRESHOLD = baselineValue * 1.022f;
    NOISE_THRESHOLD = baselineValue * 0.5f;
    Serial.print("Baseline:  ");
    Serial.println(baselineValue);
  } else {
    Serial.println("Baseline FAILED (no valid samples)");
  }
}

void handleButtonPress() {
  unsigned long now = millis();
  if (now - lastInterruptTime > debounceDelay) {
    PushbuttonPressed = true;
    pushCount++;
    lastInterruptTime = now;
  }
}

void hx710ProcessingTask(void *parameter) {
  const int REQUIRED_COUNT = 3;
  int pressCount = 0;
  for (;;) {
    int32_t v = HX.read();
    if (v < NOISE_THRESHOLD) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    currentTime = millis();
    if (v > TRIGGER_THRESHOLD) {
      pressCount++;
      if (pressCount >= REQUIRED_COUNT &&
          (currentTime - lastPressTime) > debounceDelay) {
        PressedFlag = true;
        lastPressTime = currentTime;
      }
    } else {
      pressCount = 0;
    }
    if (PressedFlag && (currentTime - lastPressTime) > debounceDelay) {
      PressedFlag = false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void motorControlTask(void *parameter) {
  for (;;) {
    motor_manager.run();
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n=== Step 4: HX710 press + button ===");

  hardResetTFT();
  Serial.println("TFT hard reset done");

  tft.begin();
  tft.setRotation(0);
  Serial.println("tft.begin OK");

  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 50);
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  display.init();
  Serial.println("display.init OK");

  Wire.begin(19, 8);
  Wire.setClock(400000);
  Serial.println("Wire begin OK");

  sensor.init(&Wire);
  Serial.println("sensor.init OK");
  Serial.printf("angle=%.3f\n", sensor.getAngle());

  motor_manager.init();
  Serial.println("motor_manager.init OK");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("button configured (INPUT_PULLUP)");

  HX.begin();
  Serial.println("HX710 begin OK");
  calculateBaseline();

  display.updateModeDisplay(DisplayManager::StartPage);
  display.showMessage({
      {"Hello, Deskknob!", LV_ALIGN_CENTER, 0, 0},
      {"Press to Start", LV_ALIGN_BOTTOM_MID, 0, -80}
  });
  Serial.println("StartPage shown");

  xTaskCreatePinnedToCore(motorControlTask, "Motor", 8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(hx710ProcessingTask, "HX710", 2048, NULL, 1, NULL, 0);
  Serial.println("tasks started");

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);
  Serial.println("button interrupt attached");
}

void loop() {
  lv_timer_handler();
  static uint32_t t0 = 0;
  if (millis() - t0 > 2000) {
    t0 = millis();
    int32_t v = HX.read();
    Serial.printf("alive angle=%.3f hx=%d Press=%d pushes=%u base=%d trig=%d\n",
                  motor_manager.GetAngle(), v, PressedFlag, pushCount,
                  baselineValue, TRIGGER_THRESHOLD);
    PressedFlag = false;
  }
  delay(5);
}