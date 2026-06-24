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
extern CRGB leds[NUM_LEDS];

#define UPDATES_PER_SECOND 100

extern CRGBPalette16 currentPalette;
extern TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;


class MenuManager {
public:
    MenuManager(DisplayManager& display, MotorManager& motor);

    void syncDisplayWithMenu();
    void handlePress();
    void run();

    void updateMainPage();
    void enterMainMenu(int menu_num, int gear_limit);
    void exitMainMenu();

    void enterVolumeControl();
    void enterPlayControl();

    //led
    void SetupTotallyRandomPalette();
    void SetupBlackAndWhiteStripedPalette();
    void SetupPurpleAndGreenPalette();

private:
    DisplayManager& display;
    MotorManager& motor;
    int currentIndex;
    bool isMainPage;
};

#endif // MENU_MANAGER_H