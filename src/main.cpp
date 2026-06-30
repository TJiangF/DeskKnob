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
#include "WifiManager.h"
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
        MenuItem("WiFi", Image_wifi),
        MenuItem("Back", Image_back)
    })
};
MenuItem* currentMenu = nullptr;

enum UIMode { StartPage, MainMenu, SecondMenu, Volume, TorqueSet,
              WifiList, WifiPassword, WifiConnecting, WifiResult };
UIMode CurrentUIMode = StartPage;

// ---------- WiFi state ----------
WifiManager wifiMgr;
int wifiSelectedSSIDIndex = -1;   // WifiList 选中索引 (found[])
String wifiSelectedSSID;
String wifiPasswordBuf;          // 密码输入缓冲
int wifiCharIdx = 0;             // 当前字符表选中
String wifiResultMsg;
String wifiConnectedSSID;       // 已连接的 SSID（StartPage 用）
std::vector<std::pair<String,int>> wifiNets;   // WifiList 显示列表
uint32_t wifiConnectStartMs = 0;

// 异步扫描任务通知
volatile bool wifiScanDone = false;
std::vector<String> wifiScanFound;
std::vector<int> wifiScanRSSI;

void wifiScanTask(void *p) {
  wifiScanFound.clear();
  wifiScanRSSI.clear();
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++) {
    String s = WiFi.SSID(i);
    if (s.length() == 0) continue;
    bool dup = false;
    for (size_t k = 0; k < wifiScanFound.size(); k++) {
      if (wifiScanFound[k] == s) { dup = true; break; }
    }
    if (dup) continue;
    wifiScanFound.push_back(s);
    wifiScanRSSI.push_back(WiFi.RSSI(i));
  }
  WiFi.scanDelete();
  wifiScanDone = true;
  vTaskDelete(NULL);
}
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
          if (wifiMgr.isConnected()) {
            wifiConnectedSSID = WiFi.SSID();
            display.showWifiOnStartPage(true, wifiConnectedSSID);
          }
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
            } else if (currentMenu->subMenu[motor_manager.GetCurrentGear()].label == "WiFi") {
              PressedFlag = false;
              CurrentUIMode = WifiList;
              UIUpdated = false;
              motor_manager.updateControlMode(MotorManager::Infinite_TorqueControl);
              motor_manager.updateGearnum(20);
              wifiSelectedSSIDIndex = 0;
              wifiMgr.loadStored();
              wifiNets.clear();
              wifiScanDone = false;
              for (int i = 0; i < wifiMgr.storedCount(); i++) {
                wifiNets.push_back(std::make_pair(wifiMgr.storedAt(i).ssid, 0));
              }
              // 异步扫描 (避免 osTask 阻塞触发 watchdog)
              xTaskCreate(wifiScanTask, "WiFiScan", 4096, NULL, 1, NULL);
              wifiPasswordBuf = "";
              wifiCharIdx = 0;
              display.initWifiListDisplay(wifiNets);
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

    // ---- WiFi sub-state machines ----
    switch (CurrentUIMode) {
      case WifiList: {
        // 异步扫描完成后合并结果到列表
        if (wifiScanDone) {
          wifiScanDone = false;
          Serial.printf("[WiFi] scan done: %u found\n", (unsigned)wifiScanFound.size());
          for (size_t i = 0; i < wifiScanFound.size(); i++) {
            Serial.printf("  %s @ %d dBm\n", wifiScanFound[i].c_str(), wifiScanRSSI[i]);
          }
          for (size_t i = 0; i < wifiScanFound.size(); i++) {
            bool dup = false;
            for (size_t k = 0; k < wifiNets.size(); k++) {
              if (wifiNets[k].first == wifiScanFound[i]) { dup = true; break; }
            }
            if (!dup) wifiNets.push_back(std::make_pair(wifiScanFound[i], wifiScanRSSI[i]));
          }
          display.updateWifiListDisplay(wifiNets, wifiSelectedSSIDIndex);
        }
        // 用 gear 切换选中项
        int g = motor_manager.GetCurrentGear();
        if (g >= (int)wifiNets.size()) g = (int)wifiNets.size() - 1;
        if (g < 0) g = 0;
        if (g != wifiSelectedSSIDIndex) {
          wifiSelectedSSIDIndex = g;
          display.updateWifiListDisplay(wifiNets, g);
        }
        if (PressedFlag) {
          PressedFlag = false;
          if (!wifiNets.empty()) {
            wifiSelectedSSID = wifiNets[wifiSelectedSSIDIndex].first;
            // 检查 stored 里有没有该 SSID 密码
            String pass = "";
            wifiMgr.loadStored();
            for (int i = 0; i < wifiMgr.storedCount(); i++) {
              if (wifiMgr.storedAt(i).ssid == wifiSelectedSSID) {
                pass = wifiMgr.storedAt(i).pass;
                break;
              }
            }
            if (pass.length() > 0) {
              // 已存密码直接连
              CurrentUIMode = WifiConnecting;
              UIUpdated = false;
              wifiConnectStartMs = millis();
              display.showWifiConnecting(wifiSelectedSSID);
              WiFi.mode(WIFI_STA);
              WiFi.begin(wifiSelectedSSID.c_str(), pass.c_str());
            } else {
              // 跳到密码输入
              CurrentUIMode = WifiPassword;
              UIUpdated = false;
              wifiPasswordBuf = "";
              wifiCharIdx = 0;
              display.initWifiPasswordDisplay(wifiSelectedSSID, "", 0);
            }
          }
        }
        if (PushbuttonPressed) {
          PushbuttonPressed = false;
          CurrentUIMode = SecondMenu;
          UIUpdated = false;
          currentMenu = &rootMenu[3];
          menuManager.enterMainMenu(torqueSetValue, currentMenu->subMenu.size());
          display.initMainMenuDisplay(currentMenu->subMenu);
        }
        break;
      }

      case WifiPassword: {
        // 字符表: index 0 = ✓(确认), 1..76 = 字符, 末尾再额外一个 ⌫(删除)
        // 共 78 项, gear 直接 mod 78
        int g = motor_manager.GetCurrentGear();
        wifiCharIdx = g % 78;
        static int lastCharUpdate = -1;
        if (wifiCharIdx != lastCharUpdate) {
          display.updateWifiPasswordDisplay(wifiPasswordBuf, wifiCharIdx);
          lastCharUpdate = wifiCharIdx;
        }
        if (PressedFlag) {
          PressedFlag = false;
          if (wifiCharIdx == 0) {
            // ✓ 确认连接
            CurrentUIMode = WifiConnecting;
            UIUpdated = false;
            wifiConnectStartMs = millis();
            display.showWifiConnecting(wifiSelectedSSID);
            WiFi.mode(WIFI_STA);
            WiFi.begin(wifiSelectedSSID.c_str(), wifiPasswordBuf.c_str());
          } else if (wifiCharIdx == 77) {
            // ⌫ 删除最后一个字符
            if (wifiPasswordBuf.length() > 0) {
              wifiPasswordBuf.remove(wifiPasswordBuf.length() - 1);
              display.updateWifiPasswordDisplay(wifiPasswordBuf, wifiCharIdx);
            }
          } else {
            // 1..76 -> 字符 0..75
            const char* WIFI_CHARSET = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!@#$%^&*-_+=?";
            char ch = WIFI_CHARSET[wifiCharIdx - 1];
            wifiPasswordBuf += ch;
            display.updateWifiPasswordDisplay(wifiPasswordBuf, wifiCharIdx);
          }
        }
        if (PushbuttonPressed) {
          PushbuttonPressed = false;
          if (wifiPasswordBuf.length() > 0) {
            wifiPasswordBuf.remove(wifiPasswordBuf.length() - 1);
            display.updateWifiPasswordDisplay(wifiPasswordBuf, wifiCharIdx);
          } else {
            CurrentUIMode = WifiList;
            UIUpdated = false;
            display.initWifiListDisplay(wifiNets);
          }
        }
        break;
      }

      case WifiConnecting: {
        // 用 .. 显示动态点 + 每 200ms 检查连接状态
        static uint32_t lastDotMs = 0;
        if (millis() - lastDotMs > 300) {
          lastDotMs = millis();
          int dots = (millis() - wifiConnectStartMs) / 300;
          display.updateWifiConnecting(dots);
        }
        if (WiFi.status() == WL_CONNECTED) {
          // 成功
          wifiMgr.addEntry(wifiSelectedSSID, wifiPasswordBuf);
          wifiConnectedSSID = wifiSelectedSSID;
          CurrentUIMode = WifiResult;
          UIUpdated = false;
          char buf[80];
          snprintf(buf, sizeof(buf), "Connected!\n%s\n%s", wifiSelectedSSID.c_str(),
                   WiFi.localIP().toString().c_str());
          display.showWifiResult(buf, true);
          wifiConnectStartMs = millis();   // 用于显示时长倒计时
        } else if (millis() - wifiConnectStartMs > 9000) {
          // 超时失败
          CurrentUIMode = WifiResult;
          UIUpdated = false;
          display.showWifiResult("Failed to connect", false);
          wifiConnectStartMs = millis();
        }
        break;
      }

      case WifiResult: {
        // 显示 2 秒后自动返回; 成功回 Settings, 失败回 WifiList 让用户重新选
        if (millis() - wifiConnectStartMs > 2000 || PushbuttonPressed) {
          PushbuttonPressed = false;
          bool wasOk = wifiMgr.isConnected();
          if (wasOk) {
            CurrentUIMode = SecondMenu;
            UIUpdated = false;
            currentMenu = &rootMenu[3];
            menuManager.enterMainMenu(torqueSetValue, currentMenu->subMenu.size());
            display.initMainMenuDisplay(currentMenu->subMenu);
            display.showWifiOnStartPage(true, wifiConnectedSSID);
          } else {
            // 失败 -> 退回 WifiList (重新扫描)
            CurrentUIMode = WifiList;
            UIUpdated = false;
            wifiScanDone = false;
            wifiNets.clear();
            for (int i = 0; i < wifiMgr.storedCount(); i++) {
              wifiNets.push_back(std::make_pair(wifiMgr.storedAt(i).ssid, 0));
            }
            xTaskCreate(wifiScanTask, "WiFiScan", 4096, NULL, 1, NULL);
            display.initWifiListDisplay(wifiNets);
          }
        }
        break;
      }

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

  // 自动重连已存 WiFi (后台异步连接, 不阻塞启动)
  WiFi.mode(WIFI_STA);
  wifiMgr.loadStored();
  if (wifiMgr.storedCount() > 0) {
    Serial.printf("[WiFi] auto-reconnect to stored #%d: %s\n",
                  0, wifiMgr.storedAt(0).ssid.c_str());
    WiFi.begin(wifiMgr.storedAt(0).ssid.c_str(),
               wifiMgr.storedAt(0).pass.c_str());
  }

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