#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <SimpleFOC.h>
#include <LittleFS.h>
#include "HX710AB.h"
#include "DisplayManager.h"
#include "ImageManager.h"
#include "MotorManager.h"
#include "MenuManager.h"
#include <vector>
#include <string>
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

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
    MenuItem("Music", Image_music, {
        MenuItem("Volume", Image_volume),
        MenuItem("Play", Image_play),
        MenuItem("Back", Image_back)
    }),
    MenuItem("Video", Image_video, {
        MenuItem("Track", Image_track),
        MenuItem("Back", Image_back)
    }),
    MenuItem("Tools", Image_tools, {
        MenuItem("Calculator", Image_calc),
        MenuItem("Explorer", Image_explorer),
        MenuItem("Rotation", Image_rotation),
        MenuItem("Back", Image_back)
    }),
    MenuItem("Settings", Image_settings, {
        MenuItem("Option1", Image_settings),
        MenuItem("Torque Set", Image_torque),
        MenuItem("Back", Image_back)
    })
};
MenuItem* currentMenu = nullptr;

enum UIMode { StartPage, MainMenu, SecondMenu, Volume, TorqueSet };
UIMode CurrentUIMode = StartPage;
bool UIUpdated = false;
int32_t UIswitch_time = 0;
int VolumeChange = 0;

// ---------- TorqueSet state ----------
int torqueSetValue = 24;       // 当前 effective value (默认 24)
int torqueSetEditing = 24;     // 编辑期临时值
float torqueSetInitAngle = 0;  // 进入时 sensor 角度作为基准
uint32_t torqueSetAnimStartMs = 0;  // 动画开始时间，超时强制退出
bool torqueSetConfirm = false; // pressure 触发确认

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
          menuManager.enterMainMenu(torqueSetValue, rootMenu.size());
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
          menuManager.enterMainMenu(torqueSetValue, currentMenu->subMenu.size());
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
          menuManager.enterMainMenu(torqueSetValue, rootMenu.size());
          display.initMainMenuDisplay(rootMenu);
          display.updateMenuDisplay(rootMenu, 0);
        } else if (PressedFlag) {
          if (motor_manager.GetCurrentGear() == (int)currentMenu->subMenu.size() - 1) {
            UIswitch_time = millis();
            PressedFlag = false;
            CurrentUIMode = MainMenu;
            UIUpdated = false;
            menuManager.enterMainMenu(torqueSetValue, rootMenu.size());
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
            } else if (currentMenu->subMenu[motor_manager.GetCurrentGear()].label == "Torque Set") {
              PressedFlag = false;
              CurrentUIMode = TorqueSet;
              UIUpdated = false;
              // 进入扭矩模式但用一个固定大档位 (60) 作为编辑器的转轴阻力
              motor_manager.updateControlMode(MotorManager::Infinite_TorqueControl);
              motor_manager.updateGearnum(60);
              torqueSetInitAngle = motor_manager.GetAngle();
              torqueSetEditing = torqueSetValue;
              display.initTorqueSetDisplay();
              display.updateTorqueSetDisplay(torqueSetEditing, true);
            }
          }
        }
        break;

      case TorqueSet:
        if (!UIUpdated) {
          // 直接基于 sensor 角位移计算 editing 值
          // attractor_distance = 2*PI/60, 一档角位移 = 6°
          float delta = motor_manager.GetAngle() - torqueSetInitAngle;
          int deltaGears = (int)lroundf(delta / (2 * PI / 60));
          int newVal = torqueSetValue + deltaGears;
          if (newVal < 2) newVal = 2;
          if (newVal > 60) newVal = 60;
          if (newVal != torqueSetEditing) {
            torqueSetEditing = newVal;
            display.updateTorqueSetDisplay(torqueSetEditing, true);
          }
          UIUpdated = true;
        }
        // pressure 一次确认
        if (PressedFlag) {
          PressedFlag = false;
          torqueSetValue = torqueSetEditing;
          motor_manager.updateGearnum(torqueSetValue);    // 全局生效
          display.showSuccessAnimation();
          torqueSetAnimStartMs = millis();
          torqueSetConfirm = true;
          UIUpdated = false;
        }
        // 动画超时 (~1.2s) 或显式结束 → 退出
        if (torqueSetConfirm && (millis() - torqueSetAnimStartMs > 1200)) {
          torqueSetConfirm = false;
          display.resetAnimState();
          CurrentUIMode = SecondMenu;
          UIUpdated = false;
          currentMenu = &rootMenu[3];   // back to Settings second menu
          menuManager.enterMainMenu(torqueSetValue, currentMenu->subMenu.size());
          display.initMainMenuDisplay(currentMenu->subMenu);
        }
        // button = cancel without saving
        if (PushbuttonPressed) {
          PushbuttonPressed = false;
          display.resetAnimState();
          CurrentUIMode = SecondMenu;
          UIUpdated = false;
          currentMenu = &rootMenu[3];
          menuManager.enterMainMenu(torqueSetValue, currentMenu->subMenu.size());
          display.initMainMenuDisplay(currentMenu->subMenu);
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
          menuManager.enterMainMenu(torqueSetValue, currentMenu->subMenu.size());
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

  // Mount LittleFS partition containing the PNG icons (flashed via `pio run -t uploadfs`)
  if (!LittleFS.begin(true)) {
    Serial.println("ERR: LittleFS mount failed - run 'pio run -t uploadfs' first");
  } else {
    Serial.printf("LittleFS OK: %u bytes used / %u total\n",
                  (unsigned)LittleFS.usedBytes(), (unsigned)LittleFS.totalBytes());
    // List '/' so we can verify the icons are present in the serial log
    File root = LittleFS.open("/");
    File f = root.openNextFile();
    while (f) {
      Serial.printf("  fs: %s (%u bytes)\n", f.name(), (unsigned)f.size());
      f = root.openNextFile();
    }
  }

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