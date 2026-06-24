#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <SimpleFOC.h>
#include "HX710AB.h"
#include "BleKeyboard.h"
#include "FastLED.h"

#include "DisplayManager.h" 
#include "ImageManager.h"
#include "MotorManager.h"
#include "MenuManager.h"

#include <vector>
#include <string>




#define TFT_BL 9
#define BUTTON_PIN 4 // Button Pin
#define PWM_UH 40    // 高电平引脚 U
#define PWM_UL 38    // 低电平引脚 U
#define PWM_VH 41    // 高电平引脚 V
#define PWM_VL 37    // 低电平引脚 V
#define PWM_WH 42    // 高电平引脚 W
#define PWM_WL 39    // 低电平引脚 W
#define TMCEN  45    // 低电平引脚 W
#define kp_stiffness 5
#define gearnum 30

#define WidgetsNum 2   // 


#define DOUT 35
#define PD_SCK 36
#define HX710Threshold 100000
#define debounceDelay 500
#define FILTER_SIZE 5  // 滑动窗口大小




int32_t filterBuffer[FILTER_SIZE] = {0};  // 滤波缓冲区
int filterIndex = 0;
int32_t baselineValue = 0; // 存储基准值
int32_t NOISE_THRESHOLD = 500000;    // 噪声阈值（低于此值认为是噪声）
int32_t PRESS_THRESHOLD = 22000;   // 按下阈值（超过此值认为是按下）
int32_t TRIGGER_THRESHOLD = 1000000;   // 按下阈值（超过此值认为是按下）
bool isPressed = false;               // 记录是否已经检测到按下


unsigned long lastPressTime = 0;  // 上次触发的时间戳
unsigned long currentTime = 0; // 当前时间戳
// HX710 ps;
HX710B HX(DOUT, PD_SCK);
int32_t hx710value = 300000;
int32_t hx710Lastvalue = 300000;
bool PressedFlag = false;



////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////   Motor and Sensor Settings //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// MagneticSensorI2C sensor = MagneticSensorI2C(0x06, 14, 0x03, 8);
MotorManager motor_manager(sensor, gearnum, kp_stiffness);
DisplayManager display;
MenuManager menuManager(display, motor_manager);




////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////   GC9A01 Display Settings ////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino_DataBus *bus = new Arduino_SWSPI(10 /* DC */, 11 /* CS */, 12 /* SCK */, 13 /* MOSI */, -1 /* MISO */);
// Arduino_GFX *gfx = new Arduino_GC9A01(bus, 14 /* RST */, 0 /* rotation */, true /* IPS */);
static lv_color_t colors[] = {
    lv_color_hex(0xFFFFFF), // 白色
    lv_color_hex(0xFF0000), // 红色
    lv_color_hex(0x00FF00), // 绿色
    lv_color_hex(0x0000FF), // 蓝色
    lv_color_hex(0xFFFF00), // 黄色
    lv_color_hex(0x00FFFF)  // 青色
};

int current_mode_index = 0;


static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 240;

lv_obj_t* angle_label;
TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

static uint8_t colorIndex = 0; // 记录当前背景颜色索引
static uint32_t lastPressedTime = 0; // 上一次按下的时间
// static uint32_t lastPressedTime = 0; // 上一次按下的时间

////////////////////////////////////////////////////////////////////////////////////////////////////////////////





volatile bool PushbuttonPressed = false;
volatile unsigned long lastInterruptTime = 0;


int motor_mode = 1;
// 0 - Angle Mode
// 1 - Torque Mode

int rotation = 0;
bool rotationFlag = false;
float target_angle = 0.0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////   Bluetooth Settings /////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// BleKeyboard bleKeyboard("Deskknob", "jtf", 100);
bool VolumeFlag = false;
int VolumeChange = 0;

// struct MenuItem {
//     std::string label;                // 菜单显示文字
//     std::string logo;                 // 菜单对应的 PNG logo 路径
//     std::vector<MenuItem> subMenu;    // 子菜单列表

//     // 构造函数
//     MenuItem(std::string l, std::string lg, std::vector<MenuItem> sm = {})
//         : label(std::move(l)), logo(std::move(lg)), subMenu(std::move(sm)) {}
// };
std::vector<MenuItem> rootMenu = {
    MenuItem("Music", &Image_music, { // 使用指向 Image_music 的指针
        MenuItem("Volume", &Image_volume), // 没有对应图像时传 nullptr
        MenuItem("Play", &Image_play),
        MenuItem("Back", &Image_back)
    }),
    MenuItem("Video", &Image_video,{
      MenuItem("Track", &Image_track),
        MenuItem("Back", &Image_back)
    }),
    MenuItem("Tools", &Image_tools,{
      MenuItem("Calculator", &Image_calc),
      MenuItem("Explorer", &Image_explorer),
      MenuItem("Rotation", &Image_rotation),
        MenuItem("Back", &Image_back)
    }),
    MenuItem("Settings", &Image_settings, {
        MenuItem("Option1", &Image_settings), // 没有图像时可以传 nullptr
        MenuItem("Option2", &Image_settings),
        MenuItem("Back", &Image_back)
    })
};

MenuItem* currentMenu;

enum UIMode {
    StartPage,
    MainMenu,
    SecondMenu,
    Volume
};

UIMode CurrentUIMode = StartPage;
bool UIUpdated = false;
int32_t UIswitch_time = millis();


void handleButtonPress() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > debounceDelay) { // 防抖间隔200ms
    PushbuttonPressed = true;
    lastInterruptTime = currentTime;
  }
}

void calculateBaseline() {
    const int samplingInterval = 50; // 采样间隔（毫秒）
    const int samplingDuration = 1000; // 采样总时长（1秒）
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
        PRESS_THRESHOLD = baselineValue * 0.075f;
        NOISE_THRESHOLD = baselineValue * 0.5f;
        TRIGGER_THRESHOLD = baselineValue * 1.022f;
        Serial.print("Baseline:  ");
        Serial.println(baselineValue);
    }

}

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp_drv );
}



#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif




void osTask(void *pvParameters) {
  int volumetmp = 0;
  int volumetimetmp = 0;
  bool fragileVolumeFlag = false;
    for (;;) { 
      switch (CurrentUIMode){
        case StartPage:
          if (!UIUpdated)
          {
            menuManager.updateMainPage();
            UIUpdated = true;
          }
          if (PressedFlag)
          {
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
          if (!UIUpdated)
          {
            display.updateMenuDisplay(rootMenu, motor_manager.GetCurrentGear());
            UIUpdated = true;
          }
          if (PressedFlag && (millis() - UIswitch_time >= debounceDelay))
          {
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
          if (!UIUpdated)
          {
            display.updateMenuDisplay(currentMenu->subMenu, motor_manager.GetCurrentGear());
            UIUpdated = true;
          }else if(PushbuttonPressed){
            PushbuttonPressed = false;
            CurrentUIMode = MainMenu;
            UIUpdated = false;
            menuManager.enterMainMenu(24, rootMenu.size());
            display.initMainMenuDisplay(rootMenu);
            display.updateMenuDisplay(rootMenu, 0);
          }else if(PressedFlag){
            if(motor_manager.GetCurrentGear() == currentMenu->subMenu.size()-1){
                UIswitch_time = millis();
                PressedFlag = false;
                CurrentUIMode = MainMenu;
                UIUpdated = false;
                menuManager.enterMainMenu(24, rootMenu.size());
                display.initMainMenuDisplay(rootMenu);
                display.updateMenuDisplay(rootMenu, 0);
                continue;
            } else{
              if(currentMenu->subMenu[motor_manager.GetCurrentGear()].label == "Volume"){
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
          if(!UIUpdated){
            switch(VolumeChange){
              case 1:
                volumetmp = 1;
                volumetimetmp = millis();
                fragileVolumeFlag = true;
                break;
              case -1:
                volumetmp = 2;
                volumetimetmp = millis();
                fragileVolumeFlag = true;
                break;
              case 0:
                volumetmp = 0;
                break;
            }
            display.updateVolumeDisplay(volumetmp);
            UIUpdated = true;
          }
          if(fragileVolumeFlag && millis() - volumetimetmp > 1000){
            display.updateVolumeDisplay(0);
            fragileVolumeFlag = false;
          }
          if(PressedFlag){
            if(volumetmp == 3){
              volumetmp = 0; display.updateVolumeDisplay(0);
            }else{
              volumetmp = 3; display.updateVolumeDisplay(3);
              }
            PressedFlag = false;
          }
          if(PushbuttonPressed){
            CurrentUIMode = SecondMenu;
            UIUpdated = false;
            currentMenu =  &rootMenu[0]; //back to Music second menu
            menuManager.enterMainMenu(24, currentMenu->subMenu.size());
            display.initMainMenuDisplay(currentMenu->subMenu);
            PushbuttonPressed = false;
          }
          break;
        default:
          break;
      }


        // // lvgl handler refresh 
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}


void motorControlTask(void *parameter) {
    for (;;) {
        motor_manager.run();
    }
}

// HX710 processing task
void hx710ProcessingTask(void *parameter) {
    const int REQUIRED_COUNT = 3;        // 连续满足条件的次数
    int pressCount = 0;                 // 连续满足条件的计数器
    for (;;) {
        
        int32_t hx710value = HX.read();
        if (hx710value  < NOISE_THRESHOLD) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        currentTime = millis();

        // if ((hx710value - hx710Lastvalue) > PRESS_THRESHOLD) {
        if (hx710value > TRIGGER_THRESHOLD) {
            pressCount++; // 条件满足，增加计数器
            if (pressCount >= REQUIRED_COUNT &&
                (currentTime - lastPressTime) > debounceDelay) {
                  PressedFlag = true;
                  lastPressTime = currentTime;
                  // Serial.printf("val: %d", hx710value);
                  // Serial.printf("   last: %d\r\n", hx710Lastvalue);
                  // motor_manager.Haptic(true);
            }
        }else{
          pressCount = 0;
        }
        if(PressedFlag && (currentTime - lastPressTime) > debounceDelay){
          PressedFlag = false;
        }
        // hx710Lastvalue = hx710value;
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}





void onGearChange(int new_gear) {
    UIUpdated = false;
    // if(new_gear == 1){
    //   // VolumeFlag = true;
    //   VolumeChange++;
    // }else if(new_gear == -1){
    //   // VolumeFlag = true;
    //   VolumeChange--;
    // }
    VolumeChange = new_gear;
    
    
}


void setup()
{
  Serial.begin(115200);
  while (!Serial);
  Wire.setClock(400000);
  Wire.begin(19, 8);
  // bleKeyboard.begin();

   // 按钮引脚初始化
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PWM_UH, OUTPUT);
  pinMode(PWM_UL, OUTPUT);
  pinMode(PWM_VH, OUTPUT);
  pinMode(PWM_VL, OUTPUT);
  pinMode(PWM_WH, OUTPUT);
  pinMode(PWM_WL, OUTPUT);
  pinMode(TMCEN, OUTPUT);

  
  lv_init();

#if LV_USE_LOG != 0
  lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif
  tft.begin();          /* TFT init */

  tft.setRotation( 0 ); /* Landscape orientation, flipped */
  static lv_disp_draw_buf_t draw_buf;
  static lv_color_t buf[screenWidth * 50];
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 50);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init( &disp_drv );
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register( &disp_drv );
  
  
  display.init();
 
  sensor.init(&Wire);
  motor_manager.init();
  // menuManager.init();

  // Callback function
  motor_manager.setGearChangeCallback(onGearChange);
  
  // PUSH sensor
  HX.begin();
  calculateBaseline(); // 计算基准值

  // xTaskCreatePinnedToCore(
  //     displayTask,       // 任务函数
  //     "DisplayTask",     // 任务名称
  //     8192,              // 堆栈大小
  //     NULL,              // 参数
  //     2,                 // 优先级（较低）
  //     NULL,              // 任务句柄
  //     1                  // 绑定到核心 0
  // );

    xTaskCreatePinnedToCore(
      motorControlTask,       // 任务函数
      "Motor Control Task",   // 任务名称
      4096,                   // 堆栈大小
      NULL,                   // 参数
      2,                      // 优先级（较高）
      NULL,                   // 任务句柄
      0                       // 绑定到核心 1
  );

  // 创建 HX710 数据处理任务
  xTaskCreatePinnedToCore(
      hx710ProcessingTask,    // 任务函数
      "HX710 Task",           // 任务名称
      2048,                   // 堆栈大小
      NULL,                   // 参数
      1,                      // 优先级（较低）
      NULL,                   // 任务句柄
      0                       // 绑定到核心 0
  );

  xTaskCreatePinnedToCore(
      osTask,    // 任务函数
      "OS Task",           // 任务名称
      8192,                   // 堆栈大小
      NULL,                   // 参数
      2,                      // 优先级（较低）
      NULL,                   // 任务句柄
      1                       // 绑定到核心 0
  );

  // xTaskCreatePinnedToCore(
  //     BLETask,    // 任务函数
  //     "BLE task",           // 任务名称
  //     4096,                   // 堆栈大小
  //     NULL,                   // 参数
  //     1,                      // 优先级（较低）
  //     NULL,       // 任务句柄
  //     1                       // 绑定到核心 1
  // );

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);
  _delay(1000);
}



void loop()
{
  // if(CurrentUIMode == SecondMenu){
  //   Serial.println(currentMenu->label.c_str());
  //   delay(400);
  // }
  // if(bleKeyboard.isConnected())
  // {
  //   if(VolumeFlag){
  //     for(int i = 0; i < abs(VolumeChange); i++){
  //       if(VolumeChange > 0){
  //         bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
  //         Serial.println("+1");
  //         VolumeChange--;
  //       }else if(VolumeChange < 0){
  //         bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN);
  //         Serial.println("-1");
  //         VolumeChange++;
  //       }
        
  //       delay(100);
  //     }
  //     VolumeFlag = false;

  //   }
  // }

}






