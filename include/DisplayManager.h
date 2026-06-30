#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
#include <lvgl.h>
#include <vector>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include <tuple>
#include "ImageManager.h"


struct MenuItem {
    std::string label;                 // 菜单显示文字
    const char* logo;                  // 菜单对应 PNG 文件的 LVGL 路径 (如 "L:/music.png")
    std::vector<MenuItem> subMenu;     // 子菜单列表

    // 构造函数
    MenuItem(std::string l, const char* lg, std::vector<MenuItem> sm = {})
        : label(std::move(l)), logo(lg), subMenu(std::move(sm)) {}
};

class DisplayManager {
public:

    enum Mode {
        StartPage,
        AngleMode,
        TorqueMode,
        // VelocityMode,
        // 可以扩展更多模式
    };

    DisplayManager();                      // 构造函数
    void init();                           // 初始化显示

    // show needed messages
    void showMessage(const std::vector<std::tuple<std::string, lv_align_t, int, int>>& messages);
    void clearMessage();                  // clear messages from showMessage()

    void initMainMenuDisplay(const std::vector<MenuItem>& menu);
    void updateMenuDisplay(const std::vector<MenuItem>& menu, int selectedIndex);
    
    void initVolumeDisplay();
    void updateVolumeDisplay(int volumestatus);

    void initTorqueSetDisplay();
    void updateTorqueSetDisplay(int value, bool editing);
    void showSuccessAnimation();

    bool isAnimRunning() const { return lv_anim_count_running() > 0; }
    void resetAnimState() { success_anim_running = false; }

    // ---------- WiFi displays ----------
    void initWifiListDisplay(const std::vector<std::pair<String,int>>& nets);
    void updateWifiListDisplay(const std::vector<std::pair<String,int>>& nets, int sel);

    void initWifiPasswordDisplay(const String& ssid, const String& pwd, int charSel);
    void updateWifiPasswordDisplay(const String& pwd, int charSel);

    void showWifiConnecting(const String& ssid);
    void updateWifiConnecting(int dots);

    void showWifiResult(const String& msg, bool ok);

    void showWifiOnStartPage(bool connected, const String& ssid);
    void initWifiActionDisplay(const String& ssid, bool isCurrent, int sel);
    void updateWifiActionDisplay(int sel);

private:
    lv_obj_t* dial_wifi_list = nullptr;
    lv_obj_t* wifi_list_labels[5] = {nullptr};
    lv_obj_t* dial_wifi_pwd = nullptr;
    lv_obj_t* wifi_ssid_label = nullptr;
    lv_obj_t* wifi_pwd_label = nullptr;
    lv_obj_t* wifi_char_label = nullptr;
    lv_obj_t* dial_wifi_status = nullptr;
    lv_obj_t* wifi_status_label = nullptr;
    lv_obj_t* wifi_start_icon = nullptr;


public:
    void updateAngle(float angle);         // 更新角度显示
    void updateTarAngle(float Target_angle); // 更新目标角度显示
    void updateVelocity(float velocity);   // 更新速度显示

    void updateTorqueMode(int current, int total); // 更新档位信息显示

    void updateModeDisplay(Mode mode);     // 更新模式显示

    void showPressedMessage(); // 显示按下的信息
    void clearPressedMessage(); // 清除按下的信息

    void setBackgroundColor(lv_color_t color); // 设置背景颜色
    // void clearAllMessage();                // 清空所有显示内容



private:

    lv_obj_t *dial = nullptr;               // 主圆形表盘背景
    lv_obj_t* logos[8] = {nullptr};         // 圆周分布的图标对象
    lv_obj_t* volumelogos= nullptr;         // 圆周分布的图标对象
    lv_obj_t* backlogo= nullptr;         // 圆周分布的图标对象
    lv_obj_t* dial_torque = nullptr;        // TorqueSet 屏幕
    lv_obj_t* torque_value_label = nullptr;   // TorqueSet 数字
    lv_obj_t* torque_hint_label = nullptr;    // TorqueSet 提示
    lv_obj_t* success_label = nullptr;        // 设置成功提示
    volatile bool success_anim_running = false;
    // std::vector<lv_obj_t*> logos ;         // 圆周分布的图标对象
    // lv_obj_t *center_circle = nullptr;      // 中心圆
    lv_obj_t *center_label = nullptr;       // 中心圆内的标签
    lv_style_t logo_default_style;          // 默认样式
    lv_style_t logo_highlight_style;        // 高亮样式
    lv_style_t center_label_style;           // 中央标签样式
    bool styles_inited = false;              // 防止 lv_style_init 重复调用导致内存破坏

    // static lv_obj_t *logos[8];         // Logo 图标对象
    // lv_obj_t* center_circle;
    // lv_obj_t* center_label;

    std::map<Mode, std::vector<lv_obj_t*>> mode_ui_map; // 各模式的 UI 对象组
    std::vector<lv_obj_t*> common_ui_objects;          // 公共 UI 对象
    Mode current_mode;                                 // 当前模式
    lv_obj_t* pressed_label;
    std::vector<lv_obj_t*> message_labels;            // 用于显示自定义消息的标签集合

    std::vector<std::string> menuLabels; 
};

#endif // DISPLAY_MANAGER_H
