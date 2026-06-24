// MenuManager.h
#ifndef MENU_MANAGER_H
#define MENU_MANAGER_H

#include "DisplayManager.h"
#include "MotorManager.h"
#include "FastLED.h"
#include <vector>
#include <functional>
#include <string>

#define LED_PIN     47
#define NUM_LEDS    8
#define BRIGHTNESS  255
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

#define UPDATES_PER_SECOND 100

CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;


class MenuManager {
public:
    MenuManager(DisplayManager& display, MotorManager& motor);

    void syncDisplayWithMenu();
    void handlePress();
    void run();
    // void handleButtonPress(); // 处理按钮按下
    // void handleBack();        // 返回上一级菜单
    // void handleNext();        // 切换到下一个菜单项
    // void handlePrev();        // 切换到上一个菜单项


    void updateMainPage();    // 更新主页面显示
    void enterMainMenu(int menu_num, int gear_limit);     // 进入主菜单
    void exitMainMenu();      // 退出主菜单
    // void updateMenuDisplay(); // 更新菜单显示

    void enterVolumeControl(); // 进入音量控制
    void enterPlayControl();   // 进入播放控制

    //led
    void SetupTotallyRandomPalette();
    void SetupBlackAndWhiteStripedPalette();
    void SetupPurpleAndGreenPalette();

private:
    DisplayManager& display;
    MotorManager& motor;
    // std::vector<MenuItem> rootMenu;
    // std::vector<MenuItem>* currentMenu;
    // std::vector<std::vector<MenuItem>*> menuStack;
    int currentIndex;
    bool isMainPage;
};

#endif // MENU_MANAGER_H