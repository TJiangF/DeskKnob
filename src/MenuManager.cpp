// MenuManager.cpp
#include "MenuManager.h"
#include <string>

// Global variable definitions
CRGB leds[NUM_LEDS];
CRGBPalette16 currentPalette;
TBlendType currentBlending;

MenuManager::MenuManager(DisplayManager& _display, MotorManager& _motor)
    : display(_display), motor(_motor), currentIndex(0), isMainPage(true) 
    {
        FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
        FastLED.setBrightness(  BRIGHTNESS );
        currentPalette = RainbowColors_p;
        currentBlending = LINEARBLEND;
    }





void MenuManager::run() {
  if(isMainPage){
    
  }else{
    // updateMenuDisplay();

  }

}


void MenuManager::handlePress() {
    // if (isMainPage) {
    //     enterMainMenu();
    // }
    // } else if (currentMenu) {
    //     auto& selected = (*currentMenu)[currentIndex];
    //     if (!selected.subMenu.empty()) {
    //         menuStack.push_back(currentMenu);
    //         currentMenu = &selected.subMenu;
    //         currentIndex = 0;
    //         updateMenuDisplay();
    //     } else if (selected.action) {
    //         selected.action();
    //     }
    // }
}

// void MenuManager::handleButtonPress() {
//     if (isMainPage) {
//         enterMainMenu();
//     } else if (currentMenu) {
//         auto& selected = (*currentMenu)[currentIndex];
//         if (!selected.subMenu.empty()) {
//             menuStack.push_back(currentMenu);
//             currentMenu = &selected.subMenu;
//             currentIndex = 0;
//             updateMenuDisplay();
//         } else if (selected.action) {
//             selected.action();
//         }
//     }
// }

// void MenuManager::handleBack() {
//     if (!menuStack.empty()) {
//         currentMenu = menuStack.back();
//         menuStack.pop_back();
//         currentIndex = 0;
//         updateMenuDisplay();
//     } else {
//         exitMainMenu();
//     }
// }

// void MenuManager::handleNext() {
//     if (currentMenu && !currentMenu->empty()) {
//         currentIndex = (currentIndex + 1) % currentMenu->size();
//         updateMenuDisplay();
//     }
// }

// void MenuManager::handlePrev() {
//     if (currentMenu && !currentMenu->empty()) {
//         currentIndex = (currentIndex - 1 + currentMenu->size()) % currentMenu->size();
//         updateMenuDisplay();
//     }
// }
void MenuManager::syncDisplayWithMenu() {
    // std::vector<std::string> labels;
    // for (const auto& item : rootMenu) {
    //     labels.push_back(item.label);
    // }
    // display.syncMenuLabels(labels);
}


void MenuManager::updateMainPage() {
    display.updateModeDisplay(DisplayManager::Mode::StartPage); // 主页面模式
    display.showMessage({
        {"Hello, Deskknob!", LV_ALIGN_CENTER, 0, 0},   // 左上角，偏移 (10, 20)
        {"Press to Start", LV_ALIGN_BOTTOM_MID, 0, -80} // 右下角，偏移 (-15, -30)
    });
    motor.updateControlMode(MotorManager::AngleControl); // angle mode and align zero
    motor.updateAngletarget(0); // angle mode and align zero
    
    isMainPage = true;
}

void MenuManager::enterMainMenu(int gear_num, int gear_limit) {
    display.clearMessage();
    motor.updateControlMode(MotorManager::ControlMode::Uninfinite_TorqueControl, gear_limit);
    motor.updateGearnum(gear_num);    // 8 档
    isMainPage = false;
}

void MenuManager::exitMainMenu() {
    updateMainPage();
}



void MenuManager::enterVolumeControl() {
    // display.clearMessage();
    motor.updateControlMode(MotorManager::ControlMode::Infinite_TorqueControl);
    motor.updateGearnum(30);  // 20 档
    // display.showMessage({
    //     {"Volume", LV_ALIGN_CENTER, 0, 0}
    // });
}

void MenuManager::enterPlayControl() {
    display.clearMessage();
    // motor.updateControlMode(0); // 角度模式
    motor.updateControlMode(MotorManager::ControlMode::AngleControl);
    motor.updateAngletarget(0);
    display.showMessage({
        {"play/pause, next/prev", LV_ALIGN_CENTER, 0, 0}
    });
}


// This function fills the palette with totally random colors.
void MenuManager::SetupTotallyRandomPalette()
{
    for( int i = 0; i < 16; ++i) {
        currentPalette[i] = CHSV( random8(), 255, random8());
    }
}

// This function sets up a palette of black and white stripes,
// using code.  Since the palette is effectively an array of
// sixteen CRGB colors, the various fill_* functions can be used
// to set them up.
void MenuManager::SetupBlackAndWhiteStripedPalette()
{
    // 'black out' all 16 palette entries...
    fill_solid( currentPalette, 16, CRGB::Black);
    // and set every fourth one to white.
    currentPalette[0] = CRGB::White;
    currentPalette[4] = CRGB::White;
    currentPalette[8] = CRGB::White;
    currentPalette[12] = CRGB::White;
    
}

// This function sets up a palette of purple and green stripes.
void MenuManager::SetupPurpleAndGreenPalette()
{
    CRGB purple = CHSV( HUE_PURPLE, 255, 255);
    CRGB green  = CHSV( HUE_GREEN, 255, 255);
    CRGB black  = CRGB::Black;
    
    currentPalette = CRGBPalette16(
                                   green,  green,  black,  black,
                                   purple, purple, black,  black,
                                   green,  green,  black,  black,
                                   purple, purple, black,  black );
}


// This example shows how to set up a static color palette
// which is stored in PROGMEM (flash), which is almost always more
// plentiful than RAM.  A static PROGMEM palette like this
// takes up 64 bytes of flash.
const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM =
{
    CRGB::Red,
    CRGB::Gray, // 'white' is too bright compared to red and blue
    CRGB::Blue,
    CRGB::Black,
    
    CRGB::Red,
    CRGB::Gray,
    CRGB::Blue,
    CRGB::Black,
    
    CRGB::Red,
    CRGB::Red,
    CRGB::Gray,
    CRGB::Gray,
    CRGB::Blue,
    CRGB::Blue,
    CRGB::Black,
    CRGB::Black
};
