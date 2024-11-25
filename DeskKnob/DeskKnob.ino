#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include <SimpleFOC.h>
#include "DisplayManager.h" // 包含显示管理器
#include "HX710.h"
#include "MotorManager.h"
#include <BleKeyboard.h>




#define GFX_BL 9
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


#define DOUT 35
#define PD_SCK 36
#define HX710Threshold 100000
#define debounceDelay 400
#define FILTER_SIZE 5  // 滑动窗口大小

int32_t filterBuffer[FILTER_SIZE] = {0};  // 滤波缓冲区
int filterIndex = 0;
int32_t baselineValue = 0; // 存储基准值
int32_t NOISE_THRESHOLD = 100000;    // 噪声阈值（低于此值认为是噪声）
int32_t PRESS_THRESHOLD = 0;   // 按下阈值（超过此值认为是按下）
bool isPressed = false;               // 记录是否已经检测到按下


unsigned long lastPressTime = 0;  // 上次触发的时间戳
unsigned long currentTime = 0; // 当前时间戳
HX710 ps;
int32_t hx710value = 300000;
int32_t hx710Lastvalue = 300000;
bool PressedFlag = false;



////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////   Motor and Sensor Settings //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MotorManager motor_manager(sensor, gearnum, kp_stiffness);




////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////   GC9A01 Display Settings ////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Arduino_DataBus *bus = new Arduino_SWSPI(10 /* DC */, 11 /* CS */, 12 /* SCK */, 13 /* MOSI */, -1 /* MISO */);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, 14 /* RST */, 0 /* rotation */, true /* IPS */);
static lv_color_t colors[] = {
    lv_color_hex(0xFFFFFF), // 白色
    lv_color_hex(0xFF0000), // 红色
    lv_color_hex(0x00FF00), // 绿色
    lv_color_hex(0x0000FF), // 蓝色
    lv_color_hex(0xFFFF00), // 黄色
    lv_color_hex(0x00FFFF)  // 青色
};
int current_mode_index = 0;


static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;
static uint8_t colorIndex = 0; // 记录当前背景颜色索引
static uint32_t lastPressedTime = 0; // 上一次按下的时间
// static uint32_t lastPressedTime = 0; // 上一次按下的时间
DisplayManager display; // 创建显示管理器对象
////////////////////////////////////////////////////////////////////////////////////////////////////////////////





volatile bool PushbuttonPressed = false;
volatile unsigned long lastInterruptTime = 0;


int motor_mode = 0;
// 0 - Angle Mode
// 1 - Torque Mode

int rotation = 0;
bool rotationFlag = false;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////   Bluetooth Settings /////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

BleKeyboard bleKeyboard("Deskknob", "jtf", 100);
bool VolumeFlag = false;
int VolumeChange = 0;




// void onRotation(int diff) {
//     // Serial.printf("Knob rotated by: %d steps\n", diff);
//     if(diff != 0){
//       rotationFlag = true;
//     }
//     rotation = diff;
// }

void handleButtonPress() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > 200) { // 防抖间隔200ms
    PushbuttonPressed = true;
    lastInterruptTime = currentTime;
  }
}


/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
   uint32_t w = (area->x2 - area->x1 + 1);
   uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
   gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
   gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

   lv_disp_flush_ready(disp);
}


void displayTask(void *pvParameters) {
    for (;;) {
        if (PressedFlag) {
            // 更新背景颜色
            motor_mode =  (motor_manager.GetControlMode() + 1) % 2;
            motor_manager.updateControlMode(motor_mode);
            // display.setBackgroundColor(colors[colorIndex]);

            current_mode_index = (current_mode_index + 1) % 2; // 3 是 Mode 枚举值的数量
            DisplayManager::Mode new_mode = static_cast<DisplayManager::Mode>(current_mode_index);
            display.updateModeDisplay(new_mode);

            display.showPressedMessage();
            // colorIndex = (colorIndex + 1) % (sizeof(colors) / sizeof(colors[0]));
            lastPressedTime = millis();
            PressedFlag = false;
        }

        // 检查是否需要清除 Pressed 信息（显示 1 秒后）
        if (millis() - lastPressedTime >= 1000) {
            display.clearPressedMessage();
        }
        switch(motor_mode){
          case 0:
            display.updateAngle(motor_manager.current_angle);
            display.updateTarAngle(motor_manager.angletarget);
            display.updateVelocity(sensor.getVelocity());
            // display.updateAngle(sensor.getAngle());
            // display.updateTarAngle(motor_manager.GetTargetangle());
            // display.updateVelocity(sensor.getVelocity());
            break;
          case 1:
            display.updateTorqueMode(motor_manager.GetCurrentGear(), motor_manager.GetGearnum());
            break;
          default:
            break;
        }
        
        // display.updatepsvalue(hx710value);

        // 刷新 LVGL
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void motorControlTask(void *parameter) {
    for (;;) {
        // motor.loopFOC();
        if(PushbuttonPressed)
        {
          switch(motor_mode){
            case 0:
              motor_manager.updateAngletarget(motor_manager.GetTargetangle() + 3.0);
              break;
            case 1:
              motor_manager.updateGearnum(motor_manager.GetGearnum() + 1);
              break;
            default:
              break;

          }
          PushbuttonPressed = false;
        }
         motor_manager.run();
         

        // vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

// HX710 processing task
void hx710ProcessingTask(void *parameter) {
    const int REQUIRED_COUNT = 3;        // 连续满足条件的次数
    int pressCount = 0;                 // 连续满足条件的计数器
    for (;;) {
        while(!ps.isReady()); 
        ps.readAndSelectNextData( HX710_DIFFERENTIAL_INPUT_10HZ );
        int32_t rawValue = -ps.getLastDifferentialInput();
        if (rawValue  < NOISE_THRESHOLD) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        hx710value = smoothValue(rawValue);
        currentTime = millis();

        if ((hx710value - hx710Lastvalue) > PRESS_THRESHOLD) {
            pressCount++; // 条件满足，增加计数器
            if (pressCount >= REQUIRED_COUNT &&
                (currentTime - lastPressTime) > debounceDelay) {
                  PressedFlag = true;
                  lastPressTime = currentTime;
                  isPressed = true;  // 标记按下已处理
                  motor_manager.Haptic(true);
            }
        }else{
          pressCount = 0;
        }
        // 监测松开动作
        if (isPressed && !PressedFlag) {
            isPressed = false;  //
            motor_manager.Haptic(false);
        }
        hx710Lastvalue = hx710value;
        vTaskDelay(pdMS_TO_TICKS(40)); 
    }
}


// void BLETask(void *parameter) {
//     for(;;){
//       if(VolumeFlag){
//         for(int i = 0; i < abs(VolumeChange); i++){
//           if(VolumeChange > 0){
//             bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
//             Serial.println("+1");
//             VolumeChange--;
//           }else if(VolumeChange < 0){
//             bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN);
//             Serial.println("-1");
//             VolumeChange++;
//           }
          
//           delay(100);
//         }
//         VolumeFlag = false;

//       }
//       vTaskDelay(pdMS_TO_TICKS(100)); 
//     }
// }


void onGearChange(int new_gear) {
    
    // if(new_gear == 1){
    //   bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
    // }else if(new_gear == -1){
    //   bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN);
    // }
    // delay(100);
    if(new_gear == 1){
      VolumeFlag = true;
      VolumeChange++;
    }else if(new_gear == -1){
      VolumeFlag = true;
      VolumeChange--;
    }

}


void setup()
{
  Serial.begin(115200);
  while (!Serial);
  Wire.setClock(400000);
  Wire.begin(20, 19);
  bleKeyboard.begin();

   // 按钮引脚初始化
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PWM_UH, OUTPUT);
  pinMode(PWM_UL, OUTPUT);
  pinMode(PWM_VH, OUTPUT);
  pinMode(PWM_VL, OUTPUT);
  pinMode(PWM_WH, OUTPUT);
  pinMode(PWM_WL, OUTPUT);
  pinMode(TMCEN, OUTPUT);

// PUSH sensor
   ps.initialize(PD_SCK, DOUT);
   calculateBaseline(); // 计算基准值

   // Init Display
   gfx->begin();
   gfx->fillScreen(BLACK);

#ifdef GFX_BL
   pinMode(GFX_BL, OUTPUT);
   digitalWrite(GFX_BL, HIGH);
#endif

   lv_init();

   screenWidth = gfx->width();
   screenHeight = gfx->height();
#ifdef ESP32
   disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * 10, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#else
   disp_draw_buf = (lv_color_t *)malloc(sizeof(lv_color_t) * screenWidth * 10);
#endif
   if (!disp_draw_buf)
   {
      Serial.println("LVGL disp_draw_buf allocate failed!");
   }
   else
   {
      lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * 10);

      lv_disp_drv_init(&disp_drv);
      disp_drv.hor_res = screenWidth;
      disp_drv.ver_res = screenHeight;
      disp_drv.flush_cb = my_disp_flush;
      disp_drv.draw_buf = &draw_buf;
      lv_disp_drv_register(&disp_drv);

      // 初始化显示管理器
      display.init();

      Serial.println("Setup done");
   }
  sensor.init(&Wire);
  motor_manager.init();


  // Callback function
  // motor_manager.setRotationCallback(onRotation);
  motor_manager.setGearChangeCallback(onGearChange);


  xTaskCreatePinnedToCore(
      displayTask,       // 任务函数
      "DisplayTask",     // 任务名称
      4096,              // 堆栈大小
      NULL,              // 参数
      2,                 // 优先级（较低）
      NULL,              // 任务句柄
      1                  // 绑定到核心 0
  );

    xTaskCreatePinnedToCore(
      motorControlTask,       // 任务函数
      "Motor Control Task",   // 任务名称
      2048,                   // 堆栈大小
      NULL,                   // 参数
      2,                      // 优先级（较高）
      NULL,       // 任务句柄
      0                       // 绑定到核心 1
  );

  // 创建 HX710 数据处理任务
  xTaskCreatePinnedToCore(
      hx710ProcessingTask,    // 任务函数
      "HX710 Task",           // 任务名称
      2048,                   // 堆栈大小
      NULL,                   // 参数
      2,                      // 优先级（较低）
      NULL,       // 任务句柄
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
  if(bleKeyboard.isConnected())
  {
    if(VolumeFlag){
      for(int i = 0; i < abs(VolumeChange); i++){
        if(VolumeChange > 0){
          bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
          Serial.println("+1");
          VolumeChange--;
        }else if(VolumeChange < 0){
          bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN);
          Serial.println("-1");
          VolumeChange++;
        }
        
        delay(100);
      }
      VolumeFlag = false;

    }
  }

}

void calculateBaseline() {
    const int samplingInterval = 50; // 采样间隔（毫秒）
    const int samplingDuration = 2000; // 采样总时长（1秒）
    int32_t sum = 0;
    int count = 0;
    uint32_t startTime = millis();

    Serial.println("Calculating baseline...");

    // 采集 1 秒钟的数据
    while (millis() - startTime < samplingDuration) {
        if (ps.isReady()) {
            ps.readAndSelectNextData(HX710_DIFFERENTIAL_INPUT_10HZ);
            int32_t rawValue = -ps.getLastDifferentialInput();

            if (rawValue > NOISE_THRESHOLD) { // 忽略可能的噪声值
                sum += rawValue;
                count++;
            }
        }
        delay(samplingInterval); // 控制采样频率
    }
    if (count > 0) {
        baselineValue = sum / count;
        PRESS_THRESHOLD = baselineValue * 0.012;
        NOISE_THRESHOLD = baselineValue * 0.50;
        Serial.print("Baseline:  ");
        Serial.println(baselineValue);
    }

}

int32_t smoothValue(int32_t newValue) {
    filterBuffer[filterIndex] = newValue;        // 插入新数据
    filterIndex = (filterIndex + 1) % FILTER_SIZE; // 更新索引
    int32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += filterBuffer[i];
    }
    return sum / FILTER_SIZE;  // 返回滑动平均值
}



