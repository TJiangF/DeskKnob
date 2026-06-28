#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <SimpleFOC.h>
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

// Force a long, clean reset of the GC9A01 before tft.begin(),
// otherwise the panel may fail to init after MCU reset.
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

void motorControlTask(void *parameter) {
  for (;;) {
    motor_manager.run();
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n=== Step 3: Motor + Display ===");

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

  // ---- init I2C + motor ----
  Wire.setClock(400000);
  Wire.begin(19, 8);
  Serial.println("Wire begin OK (SDA=19 SCL=8)");

  sensor.init(&Wire);
  Serial.println("sensor.init OK");
  Serial.printf("angle=%.3f\n", sensor.getAngle());

  motor_manager.init();
  Serial.println("motor_manager.init OK");

  display.updateModeDisplay(DisplayManager::StartPage);
  display.showMessage({
      {"Hello, Deskknob!", LV_ALIGN_CENTER, 0, 0},
      {"Press to Start", LV_ALIGN_BOTTOM_MID, 0, -80}
  });
  Serial.println("StartPage shown");

  // ---- launch FOC on core 1 ----
  xTaskCreatePinnedToCore(
      motorControlTask, "Motor", 4096, NULL, 2, NULL, 1);
  Serial.println("motor task started on core 1");
}

void loop() {
  lv_timer_handler();
  static uint32_t t0 = 0;
  if (millis() - t0 > 2000) {
    t0 = millis();
    Serial.printf("alive angle=%.3f\n", sensor.getAngle());
  }
  delay(5);
}