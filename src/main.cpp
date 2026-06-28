#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <SimpleFOC.h>
#include "HX710AB.h"
#include "DisplayManager.h"
#include "ImageManager.h"
#include "MotorManager.h"
#include "MenuManager.h"
#include <vector>
#include <string>

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
MenuManager menuManager(display, motor_manager);

// ---------- Menu ----------
std::vector<MenuItem> rootMenu = {
    MenuItem("Music", &Image_music, {
        MenuItem("Volume", &Image_volume),
        MenuItem("Play", &Image_play),
        MenuItem("Back", &Image_back)
    }),
    MenuItem("Video", &Image_video, {
        MenuItem("Track", &Image_track),
        MenuItem("Back", &Image_back)
    }),
    MenuItem("Tools", &Image_tools, {
        MenuItem("Calculator", &Image_calc),
        MenuItem("Explorer", &Image_explorer),
        MenuItem("Rotation", &Image_rotation),
        MenuItem("Back", &Image_back)
    }),
    MenuItem("Settings", &Image_settings, {
        MenuItem("Option1", &Image_settings),
        MenuItem("Option2", &Image_settings),
        MenuItem("Back", &Image_back)
    })
};
MenuItem* currentMenu = nullptr;

enum UIMode { StartPage, MainMenu, SecondMenu, Volume };
UIMode CurrentUIMode = StartPage;
bool UIUpdated = false;
int32_t UIswitch_time = 0;
int VolumeChange = 0;

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

void onGearChange(int new_gear) {
  UIUpdated = false;
  VolumeChange = new_gear;
}

// LVGL/UI state machine on core 0
void osTask(void *pvParameters) {
  int volumetmp = 0;
  int volumetimetmp = 0;
  bool fragileVolumeFlag = false;
  for (;;) {
    switch (CurrentUIMode) {
      case StartPage:
        if (!UIUpdated) {
          menuManager.updateMainPage();
          UIUpdated = true;
        }
        if (PressedFlag) {
          UIUpdated = false;
          menuManager.enterMainMenu(24, rootMenu.size());
          display.initMainMenuDisplay(rootMenu);
          display.updateMenuDisplay(rootMenu, 0);
          CurrentUIMode = MainMenu;
          PressedFlag = false;
          UIswitch_time = millis();
        }
        break;

      case MainMenu:
        if (!UIUpdated) {
          display.updateMenuDisplay(rootMenu, motor_manager.GetCurrentGear());
          UIUpdated = true;
        }
        if (PressedFlag && (millis() - UIswitch_time >= debounceDelay)) {
          UIswitch_time = millis();
          PressedFlag = false;
          currentMenu = &rootMenu[motor_manager.GetCurrentGear()];
          CurrentUIMode = SecondMenu;
          UIUpdated = false;
          menuManager.enterMainMenu(24, currentMenu->subMenu.size());
          display.initMainMenuDisplay(currentMenu->subMenu);
        }
        break;

      case SecondMenu:
        if (!UIUpdated) {
          display.updateMenuDisplay(currentMenu->subMenu, motor_manager.GetCurrentGear());
          UIUpdated = true;
        } else if (PushbuttonPressed) {
          PushbuttonPressed = false;
          CurrentUIMode = MainMenu;
          UIUpdated = false;
          menuManager.enterMainMenu(24, rootMenu.size());
          display.initMainMenuDisplay(rootMenu);
          display.updateMenuDisplay(rootMenu, 0);
        } else if (PressedFlag) {
          if (motor_manager.GetCurrentGear() == (int)currentMenu->subMenu.size() - 1) {
            UIswitch_time = millis();
            PressedFlag = false;
            CurrentUIMode = MainMenu;
            UIUpdated = false;
            menuManager.enterMainMenu(24, rootMenu.size());
            display.initMainMenuDisplay(rootMenu);
            display.updateMenuDisplay(rootMenu, 0);
            continue;
          } else {
            if (currentMenu->subMenu[motor_manager.GetCurrentGear()].label == "Volume") {
              PressedFlag = false;
              CurrentUIMode = Volume;
              UIUpdated = false;
              menuManager.enterVolumeControl();
              display.initVolumeDisplay();
            }
          }
        }
        break;

      case Volume:
        if (!UIUpdated) {
          switch (VolumeChange) {
            case 1:  volumetmp = 1; volumetimetmp = millis(); fragileVolumeFlag = true; break;
            case -1: volumetmp = 2; volumetimetmp = millis(); fragileVolumeFlag = true; break;
            default: volumetmp = 0; break;
          }
          display.updateVolumeDisplay(volumetmp);
          UIUpdated = true;
        }
        if (fragileVolumeFlag && millis() - volumetimetmp > 1000) {
          display.updateVolumeDisplay(0);
          fragileVolumeFlag = false;
        }
        if (PressedFlag) {
          if (volumetmp == 3) {
            volumetmp = 0; display.updateVolumeDisplay(0);
          } else {
            volumetmp = 3; display.updateVolumeDisplay(3);
          }
          PressedFlag = false;
        }
        if (PushbuttonPressed) {
          CurrentUIMode = SecondMenu;
          UIUpdated = false;
          currentMenu = &rootMenu[0];  // back to Music second menu
          menuManager.enterMainMenu(24, currentMenu->subMenu.size());
          display.initMainMenuDisplay(currentMenu->subMenu);
          PushbuttonPressed = false;
        }
        break;

      default: break;
    }

    uint32_t t = lv_timer_handler();
    lv_refr_now(NULL);   // force synchronous refresh every cycle (bypass paused timer)
    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n=== Step 5: Menu state machine ===");

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

  motor_manager.init();
  Serial.println("motor_manager.init OK");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  HX.begin();
  calculateBaseline();

  Serial.println("StartPage");
  CurrentUIMode = StartPage;
  UIUpdated = false;

  motor_manager.setGearChangeCallback(onGearChange);

  xTaskCreatePinnedToCore(motorControlTask, "Motor", 8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(hx710ProcessingTask, "HX710", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(osTask, "OS", 8192, NULL, 2, NULL, 0);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);
  Serial.println("setup done");
}

void loop() {
  // idle - all work in tasks
  vTaskDelay(pdMS_TO_TICKS(100));
}