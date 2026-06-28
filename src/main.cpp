#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>

static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 240;

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight);

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 50];

/* Display flushing */
static void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)&color_p->full, w * h, true);
  tft.endWrite();
  lv_disp_flush_ready(disp_drv);
}

static uint32_t lastMillis = 0;

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n=== Step 1: LVGL on TFT_eSPI ===");

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  Serial.println("BL ON");

  Serial.println("tft.begin...");
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

  lv_obj_t *label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, "Hello, Deskknob!\nStep 1: LVGL");
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

  Serial.println("setup done");
}

void loop() {
  lv_timer_handler();
  uint32_t now = millis();
  if (now - lastMillis > 2000) {
    lastMillis = now;
    Serial.print("alive ");
    Serial.println(millis());
  }
  delay(5);
}