#include "DisplayManager.h"
#include "MenuManager.h"
#include <cstdio>
#include <vector>
#include <map>
#include <string>
#include <tuple>
#include <math.h>
#include "ImageManager.h"


#define DEG_TO_RAD (M_PI / 180.0f) // 将角度转换为弧度的常量

// lv_obj_t* DisplayManager::logos[8] = {nullptr}; // 初始化数组为nullptr



DisplayManager::DisplayManager(): pressed_label(nullptr), current_mode(AngleMode) {}

void DisplayManager::init() {

    auto angle_label = lv_label_create(lv_scr_act());
    lv_label_set_text(angle_label, "Angle: 0.00");
    lv_obj_align(angle_label, LV_ALIGN_CENTER, 0, -40);
    mode_ui_map[AngleMode].push_back(angle_label);


    auto target_angle_label = lv_label_create(lv_scr_act());
    lv_label_set_text(target_angle_label, "Tar Ang: 0.00");
    lv_obj_align(target_angle_label, LV_ALIGN_CENTER, 0, -20);
    mode_ui_map[AngleMode].push_back(target_angle_label);

    // AngleMode UI
    auto velocity_label = lv_label_create(lv_scr_act());
    lv_label_set_text(velocity_label, "Velocity: 0.00");
    lv_obj_align(velocity_label, LV_ALIGN_CENTER, 0, 0);
    mode_ui_map[AngleMode].push_back(velocity_label);

    // TorqueMode UI
    auto gear_label = lv_label_create(lv_scr_act());
    lv_label_set_text(gear_label, "Gear: 0/0");
    lv_obj_align(gear_label, LV_ALIGN_CENTER, 0, 0);
    mode_ui_map[TorqueMode].push_back(gear_label);

    auto progress_arc = lv_arc_create(lv_scr_act());
    lv_obj_set_size(progress_arc, 225, 225);
    lv_arc_set_range(progress_arc, 0, 100);
    lv_obj_align(progress_arc, LV_ALIGN_CENTER, 0, 0);
    mode_ui_map[TorqueMode].push_back(progress_arc);

    // pressed_label = lv_label_create(lv_scr_act());
    // lv_label_set_text(pressed_label, "Pressed!");
    // lv_obj_align(pressed_label, LV_ALIGN_CENTER, 0, -60);



    // 初始化为当前模式
    updateModeDisplay(current_mode);
}



void DisplayManager::initMainMenuDisplay(const std::vector<MenuItem>& menu) {
    // if (dial != nullptr) return;  // 如果已经初始化，直接返回
    // 创建圆形表盘背景，只在初始化时创建一次
    dial = lv_obj_create(NULL);
    lv_obj_set_size(dial, 240, 240);
    // lv_obj_align(dial, LV_ALIGN_CENTER, 0, 0);
    // lv_obj_set_style_radius(dial, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dial, lv_color_white(), 0);
    int total_items = std::min((int)menu.size(), 8); // 最多显示8个菜单项
    for (int i = 0; i < 8; i++) {
        logos[i] = lv_img_create(dial);
        lv_obj_set_size(logos[i], 50, 50); // 限制图片大小
    }

    if (!styles_inited) {
        lv_style_init(&logo_default_style);
        lv_style_set_radius(&logo_default_style, 10);
        lv_style_set_bg_opa(&logo_default_style, LV_OPA_COVER);
        lv_style_set_bg_color(&logo_default_style, lv_palette_lighten(LV_PALETTE_GREY, 2));

        // 创建高亮样式
        lv_style_init(&logo_highlight_style);
        lv_style_set_radius(&logo_highlight_style, 10);
        lv_style_set_bg_opa(&logo_highlight_style, LV_OPA_COVER);
        lv_style_set_bg_color(&logo_highlight_style, lv_palette_main(LV_PALETTE_BLUE));
        lv_style_set_shadow_width(&logo_highlight_style, 20);
        lv_style_set_shadow_color(&logo_highlight_style, lv_palette_main(LV_PALETTE_BLUE));
        lv_style_set_shadow_ofs_x(&logo_highlight_style, 10);
        lv_style_set_shadow_ofs_y(&logo_highlight_style, 10);

        // 创建标签样式，并设置更大的字体
        lv_style_init(&center_label_style);
        lv_style_set_text_font(&center_label_style, &lv_font_montserrat_20);
        lv_style_set_text_color(&center_label_style, lv_color_black());
        styles_inited = true;
    }

    center_label = lv_label_create(dial);
    lv_obj_align(center_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_style(center_label, &center_label_style, 0);
}

void DisplayManager::updateMenuDisplay(const std::vector<MenuItem>& menu, int selectedIndex) {
    if (menu.empty() || selectedIndex >= (int)menu.size()) return;
    if (lv_scr_act() != dial) {
        lv_obj_set_pos(dial, 0, 0);
        lv_scr_load(dial);
    }
    // 更新圆形表盘的图标和标签
    int total_items = std::min((int)menu.size(), 8); // 最多显示8个菜单项
    for(int i = total_items; i < 8; i++){
        lv_obj_add_flag(logos[i], LV_OBJ_FLAG_HIDDEN);
    }
    for (int i = 0; i < total_items; i++) {
        // 计算每个图标的位置，按照8等分的圆周分布
        int angle = (360 / 8 * (i + 4)) % 360; // 每个图标的角度固定为8等分
        int x = 85 * cos(angle * DEG_TO_RAD); // 半径 90 的圆周
        int y = 85 * sin(angle * DEG_TO_RAD);
        lv_obj_clear_flag(logos[i], LV_OBJ_FLAG_HIDDEN);
        lv_img_set_src(logos[i], menu[i].logo);

        // 切换主样式：先移除高亮和默认样式，再按选择重新加
        lv_obj_remove_style(logos[i], &logo_default_style, 0);
        lv_obj_remove_style(logos[i], &logo_highlight_style, 0);
        lv_obj_align(logos[i], LV_ALIGN_CENTER, x, y);
        lv_obj_add_style(logos[i], &logo_default_style, 0);

        // 如果是选中的 logo，应用高亮样式
        if (i == selectedIndex) {
            lv_obj_add_style(logos[i], &logo_highlight_style, 0);
        }
        lv_obj_invalidate(logos[i]);
    }

    // 更新中心标签
    lv_label_set_text(center_label, menu[selectedIndex].label.c_str());
    lv_obj_invalidate(center_label);
    lv_obj_invalidate(dial);
}


void DisplayManager::initVolumeDisplay(){
    // if (dial != nullptr) return;  // 如果已经初始化，直接返回
    dial = lv_obj_create(NULL);
    lv_obj_set_size(dial, 240, 240);
    lv_obj_set_style_bg_color(dial, lv_color_white(), 0);

    volumelogos = lv_img_create(dial);
    backlogo = lv_img_create(dial);
    
    lv_obj_set_size(volumelogos, 120, 120); // 限制图片大小
    lv_obj_align(volumelogos, LV_ALIGN_CENTER, 0, 0);
    // lv_obj_set_size(backlogo, 30, 30); // 限制图片大小
    lv_obj_align(backlogo, LV_ALIGN_CENTER, -65, 75);

    center_label = lv_label_create(dial);
    lv_obj_align(center_label, LV_ALIGN_CENTER, 0, -60);

    if (!styles_inited) {
        lv_style_init(&center_label_style);
        lv_style_set_text_font(&center_label_style, &lv_font_montserrat_16);
        lv_style_set_text_color(&center_label_style, lv_color_black());
        // 也把 logo styles 初始化（volume 模式不直接用了，但保持初始化以免未定义行为）
        lv_style_init(&logo_default_style);
        lv_style_set_bg_color(&logo_default_style, lv_palette_lighten(LV_PALETTE_GREY, 2));
        lv_style_init(&logo_highlight_style);
        lv_style_set_bg_color(&logo_highlight_style, lv_palette_main(LV_PALETTE_BLUE));
        styles_inited = true;
    }
    lv_obj_add_style(center_label, &center_label_style, 0);
}

void DisplayManager::updateVolumeDisplay(int volumestatus){
    if (lv_scr_act() != dial) {
        lv_obj_set_pos(dial, 0, 0);
        lv_scr_load(dial);
    }

    for(int i = 0; i < 4; i++){
        // if(i != volumestatus)
        //     lv_obj_add_flag(volumelogos, LV_OBJ_FLAG_HIDDEN);
        // else{
            lv_obj_clear_flag(volumelogos, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(backlogo, LV_OBJ_FLAG_HIDDEN);
            lv_img_set_src(backlogo, Image_back);
            switch(volumestatus){
                case 0:
                    lv_img_set_src(volumelogos, Image_volume120);
                    lv_label_set_text(center_label, "Volume Tunning");
                    break;
                case 1:
                    lv_img_set_src(volumelogos, Image_volumeup);
                    lv_label_set_text(center_label, "Volume Up ++");
                    break;
                case 2:
                    lv_img_set_src(volumelogos, Image_volumedown);
                    lv_label_set_text(center_label, "Volume Down --");
                    break;
                case 3:
                    lv_img_set_src(volumelogos, Image_volumemute);
                    lv_label_set_text(center_label, "Volume Muted");
                    break;
            }
        // }
    }
    lv_img_set_zoom(backlogo, 95);
    lv_obj_invalidate(dial);
}




void DisplayManager::updateAngle(float angle) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "Angle: %.2f", angle);
    lv_label_set_text(mode_ui_map[AngleMode][0], buffer);
    // lv_label_set_text(angle_label, buffer);
}

void DisplayManager::updateTarAngle(float Target_angle) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "Tar Ang: %.2f", Target_angle);
    lv_label_set_text(mode_ui_map[AngleMode][1], buffer);
}

void DisplayManager::updateVelocity(float velocity) {
    if (current_mode == AngleMode && !mode_ui_map[AngleMode].empty()) {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "Velocity: %.2f", velocity);
        lv_label_set_text(mode_ui_map[AngleMode][2], buffer);
    }
}

void DisplayManager::updateTorqueMode(int current, int total) {
    if (current_mode == TorqueMode) {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "Gear: %d/%d", current, total);
        lv_label_set_text(mode_ui_map[TorqueMode][0], buffer);

        float percentage = (total > 0) ? (current * 100 / total) : 0;
        lv_arc_set_value(mode_ui_map[TorqueMode][1], percentage);
    }
}

void DisplayManager::updateModeDisplay(Mode mode) {
    // 隐藏所有 UI 对象
    for (auto &[_, objects] : mode_ui_map) {
        for (auto obj : objects) {
            lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
        }
    }
    // for (auto obj : common_ui_objects) {
    //     lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
    // }

    // 显示当前模式的 UI 对象
    for (auto obj : mode_ui_map[mode]) {
        lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
    }
    // for (auto obj : common_ui_objects) {
    //     lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
    // }

    // 更新当前模式
    current_mode = mode;
}

void DisplayManager::showMessage(const std::vector<std::tuple<std::string, lv_align_t, int, int>>& messages) {
    // 清除现有的消息标签
    clearMessage();

    // 创建新的消息标签
    for (const auto& [text, align, x_offset, y_offset] : messages) {
        lv_obj_t* label = lv_label_create(lv_scr_act());
        lv_label_set_text(label, text.c_str()); // 设置文本
        lv_obj_align(label, align, x_offset, y_offset); // 设置对齐和偏移
        message_labels.push_back(label);       // 保存标签
    }
}

void DisplayManager::clearMessage() {
    // 隐藏并删除所有消息标签
    if(!message_labels.empty()){
      for (auto* label : message_labels) {
            lv_obj_del(label);
      }
    }
    message_labels.clear(); // 清空标签列表
}




void DisplayManager::clearPressedMessage() {
    lv_obj_add_flag(pressed_label, LV_OBJ_FLAG_HIDDEN); // 隐藏标签
}


void DisplayManager::setBackgroundColor(lv_color_t color) {
    lv_obj_set_style_bg_color(lv_scr_act(), color, LV_PART_MAIN | LV_STATE_DEFAULT);
}

// ===================== TorqueSet & success animation =====================

void DisplayManager::initTorqueSetDisplay() {
    if (!dial_torque) {
        dial_torque = lv_obj_create(NULL);
        lv_obj_set_size(dial_torque, 240, 240);
        lv_obj_set_style_bg_color(dial_torque, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(dial_torque, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(dial_torque, 0, 0);
        lv_obj_set_style_pad_all(dial_torque, 0, 0);

        torque_value_label = lv_label_create(dial_torque);
        lv_obj_align(torque_value_label, LV_ALIGN_CENTER, 0, -20);
        lv_obj_set_style_text_font(torque_value_label, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(torque_value_label, lv_color_white(), 0);

        torque_hint_label = lv_label_create(dial_torque);
        lv_label_set_text(torque_hint_label, "Turn to set\nPress to confirm");
        lv_obj_align(torque_hint_label, LV_ALIGN_CENTER, 0, 50);
        lv_obj_set_style_text_color(torque_hint_label, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);

        success_label = lv_label_create(dial_torque);
        lv_label_set_text(success_label, "Saved!");
        lv_obj_align(success_label, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_text_color(success_label, lv_palette_main(LV_PALETTE_GREEN), 0);
        lv_obj_set_style_text_font(success_label, &lv_font_montserrat_20, 0);
        lv_obj_add_flag(success_label, LV_OBJ_FLAG_HIDDEN);
    }
    lv_scr_load(dial_torque);
    lv_obj_invalidate(dial_torque);
}

void DisplayManager::updateTorqueSetDisplay(int value, bool editing) {
    if (!dial_torque) return;
    if (lv_scr_act() != dial_torque) lv_scr_load(dial_torque);
    if (success_anim_running) return;     // 动画期间不刷数字

    lv_label_set_text_fmt(torque_value_label, "%d", value);
    lv_obj_clear_flag(torque_value_label, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(torque_hint_label, editing ? "Turn to set\nPress to confirm"
                                                 : "Saved!");
    lv_obj_clear_flag(torque_hint_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(success_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_invalidate(dial_torque);
}

// 动画执行：交替显示/隐藏 "Saved!" 制造闪烁效果
static void blink_anim_cb(void * obj, int32_t v) {
    if (v) lv_obj_clear_flag(reinterpret_cast<lv_obj_t*>(obj), LV_OBJ_FLAG_HIDDEN);
    else   lv_obj_add_flag(reinterpret_cast<lv_obj_t*>(obj), LV_OBJ_FLAG_HIDDEN);
}

void DisplayManager::showSuccessAnimation() {
    if (!dial_torque || !success_label) return;
    success_anim_running = true;

    // 隐藏数字与提示
    lv_obj_add_flag(torque_value_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(torque_hint_label, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(success_label, "Saved!");
    lv_obj_clear_flag(success_label, LV_OBJ_FLAG_HIDDEN);

    // 用 5 步 blink 让 Saved! 闪三次（5*200 = 1s）
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, success_label);
    lv_anim_set_values(&a, 1, 0);    // 1=visible, 0=hidden (用 path_step 离散)
    lv_anim_set_time(&a, 200);
    lv_anim_set_repeat_count(&a, 4);  // 总动画时间 ~1s
    lv_anim_set_exec_cb(&a, blink_anim_cb);
    lv_anim_start(&a);

    lv_obj_invalidate(dial_torque);
}


// void DisplayManager::clearAllMessage() {
//     // 隐藏所有对象（包括公共和模式特定对象）
//     for (auto obj : common_ui_objects) {
//         lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
//     }
//     for (auto &[_, objects] : mode_ui_map) {
//         for (auto obj : objects) {
//             lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
//         }
//     }
// }

// ===================== WiFi displays =====================

// 字符表：0-9 a-z A-Z 符号 (順时针旋转选择)
static const char* WIFI_CHARSET = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!@#$%^&*-_+=?";
static const int WIFI_CHARSET_LEN = 26 + 26 + 10 + 14;  // = 76

void DisplayManager::initWifiListDisplay(const std::vector<std::pair<String,int>>& nets) {
    if (!dial_wifi_list) {
        dial_wifi_list = lv_obj_create(NULL);
        lv_obj_set_size(dial_wifi_list, 240, 240);
        lv_obj_set_style_bg_color(dial_wifi_list, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(dial_wifi_list, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(dial_wifi_list, 0, 0);
        lv_obj_set_style_pad_all(dial_wifi_list, 0, 0);
        lv_obj_set_scrollbar_mode(dial_wifi_list, LV_SCROLLBAR_MODE_OFF);

        // 5 行列表，画布高度 5*28 = 140，居中 y=50..190（避开圆形顶部与底部角落）
        for (int i = 0; i < 5; i++) {
            wifi_list_labels[i] = lv_label_create(dial_wifi_list);
            lv_obj_set_style_text_color(wifi_list_labels[i], lv_color_white(), 0);
            lv_label_set_text(wifi_list_labels[i], "");
            lv_obj_align(wifi_list_labels[i], LV_ALIGN_TOP_MID, 0, 50 + i * 28);
        }
    }
    updateWifiListDisplay(nets, 0);
}

void DisplayManager::updateWifiListDisplay(const std::vector<std::pair<String,int>>& nets, int sel) {
    if (!dial_wifi_list) return;
    if (lv_scr_act() != dial_wifi_list) lv_scr_load(dial_wifi_list);
    int n = (int)nets.size();
    int show = 5;                       // 5 行 (上下 2 行 + 中间高亮)
    int start = sel - 2;                // sel 居中
    if (start < 0) start = 0;
    if (start + show > n) {
        start = n - show;
        if (start < 0) start = 0;
    }
    for (int row = 0; row < 5; row++) {
        int i = start + row;
        if (i < n) {
            char buf[64];
            const char* prefix = (i == sel) ? "> " : "  ";
            // 截断长 SSID 以适配 240px 圆屏
            String ssid = nets[i].first;
            if (ssid.length() > 16) ssid = ssid.substring(0, 15) + ".";
            if (nets[i].second != 0) {
                snprintf(buf, sizeof(buf), "%s%s %ddBm", prefix, ssid.c_str(), nets[i].second);
            } else {
                snprintf(buf, sizeof(buf), "%s%s", prefix, ssid.c_str());
            }
            lv_label_set_text(wifi_list_labels[row], buf);
            lv_obj_set_style_text_color(wifi_list_labels[row],
                (i == sel) ? lv_palette_main(LV_PALETTE_BLUE) : lv_color_white(), 0);
            lv_obj_clear_flag(wifi_list_labels[row], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_label_set_text(wifi_list_labels[row], "");
            lv_obj_add_flag(wifi_list_labels[row], LV_OBJ_FLAG_HIDDEN);
        }
    }
    lv_obj_invalidate(dial_wifi_list);
}

void DisplayManager::initWifiPasswordDisplay(const String& ssid, const String& pwd, int charSel) {
    if (!dial_wifi_pwd) {
        dial_wifi_pwd = lv_obj_create(NULL);
        lv_obj_set_size(dial_wifi_pwd, 240, 240);
        lv_obj_set_style_bg_color(dial_wifi_pwd, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(dial_wifi_pwd, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(dial_wifi_pwd, 0, 0);

        wifi_ssid_label = lv_label_create(dial_wifi_pwd);
        lv_obj_align(wifi_ssid_label, LV_ALIGN_TOP_MID, 0, 12);
        lv_obj_set_style_text_color(wifi_ssid_label, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);

        wifi_pwd_label = lv_label_create(dial_wifi_pwd);
        lv_obj_align(wifi_pwd_label, LV_ALIGN_TOP_MID, 0, 55);
        lv_obj_set_style_text_color(wifi_pwd_label, lv_color_white(), 0);
        lv_obj_set_style_text_font(wifi_pwd_label, &lv_font_montserrat_20, 0);

        wifi_char_label = lv_label_create(dial_wifi_pwd);
        lv_obj_align(wifi_char_label, LV_ALIGN_TOP_MID, 0, 100);
        lv_obj_set_style_text_font(wifi_char_label, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(wifi_char_label, lv_palette_main(LV_PALETTE_GREEN), 0);

        lv_obj_t* hint = lv_label_create(dial_wifi_pwd);
        lv_label_set_text(hint, "Turn=pick  Press=add\nBtn=back");
        lv_obj_align(hint, LV_ALIGN_TOP_MID, 0, 145);
        lv_obj_set_style_text_color(hint, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
        lv_obj_set_user_data(dial_wifi_pwd, hint);
    }
    char ssidBuf[80];
    snprintf(ssidBuf, sizeof(ssidBuf), "SSID: %s", ssid.c_str());
    lv_label_set_text(wifi_ssid_label, ssidBuf);

    // 显示密码 + 当前选中的字符
    updateWifiPasswordDisplay(pwd, charSel);

    lv_scr_load(dial_wifi_pwd);
    lv_obj_invalidate(dial_wifi_pwd);
}

void DisplayManager::updateWifiPasswordDisplay(const String& pwd, int charSel) {
    if (!dial_wifi_pwd) return;
    if (lv_scr_act() != dial_wifi_pwd) lv_scr_load(dial_wifi_pwd);
    // 密码显示 (mask * 但留最后 2 字符可见)
    String display = "";
    int len = (int)pwd.length();
    for (int i = 0; i < len - 2; i++) display += "*";
    if (len >= 2) display += pwd.substring(len - 2);
    else display = pwd;
    lv_label_set_text(wifi_pwd_label, display.c_str());

    // charSel 0 = EXIT, 1 = CONNECT, 2..77 = 字符 0..75
    const char* show;
    if (charSel == 0) show = "EXIT";              // 退出
    else if (charSel == 1) show = "CONNECT";      // 确认连接
    else {
        char ch = WIFI_CHARSET[(charSel - 2) % WIFI_CHARSET_LEN];
        char buf[2] = {ch, 0};
        lv_label_set_text(wifi_char_label, buf);
        lv_obj_invalidate(dial_wifi_pwd);
        return;
    }
    lv_label_set_text(wifi_char_label, show);

    lv_obj_invalidate(dial_wifi_pwd);
}

void DisplayManager::showWifiConnecting(const String& ssid) {
    if (!dial_wifi_status) {
        dial_wifi_status = lv_obj_create(NULL);
        lv_obj_set_size(dial_wifi_status, 240, 240);
        lv_obj_set_style_bg_color(dial_wifi_status, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(dial_wifi_status, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(dial_wifi_status, 0, 0);

        wifi_status_label = lv_label_create(dial_wifi_status);
        lv_obj_align(wifi_status_label, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_text_color(wifi_status_label, lv_color_white(), 0);
        lv_obj_set_style_text_font(wifi_status_label, &lv_font_montserrat_20, 0);
    }
    char buf[80];
    snprintf(buf, sizeof(buf), "Connecting to\n%s\n...", ssid.c_str());
    lv_label_set_text(wifi_status_label, buf);
    lv_scr_load(dial_wifi_status);
    lv_obj_invalidate(dial_wifi_status);
}

void DisplayManager::updateWifiConnecting(int dots) {
    if (!wifi_status_label) return;
    String text = "Connecting";
    for (int i = 0; i < dots % 4; i++) text += ".";
    lv_label_set_text(wifi_status_label, text.c_str());
    lv_obj_invalidate(dial_wifi_status);
}

void DisplayManager::showWifiResult(const String& msg, bool ok) {
    if (!wifi_status_label) return;
    lv_label_set_text(wifi_status_label, msg.c_str());
    lv_obj_set_style_text_color(wifi_status_label,
        ok ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_invalidate(dial_wifi_status);
}

void DisplayManager::showWifiOnStartPage(bool connected, const String& ssid) {
    if (!connected) {
        if (wifi_start_icon) {
            lv_obj_del(wifi_start_icon);
            wifi_start_icon = nullptr;
        }
        return;
    }
    if (!wifi_start_icon) {
        wifi_start_icon = lv_img_create(lv_scr_act());
        lv_img_set_src(wifi_start_icon, Image_wifi_on);
        lv_obj_align(wifi_start_icon, LV_ALIGN_TOP_RIGHT, -10, 10);
    }
lv_obj_clear_flag(wifi_start_icon, LV_OBJ_FLAG_HIDDEN);
    lv_obj_invalidate(wifi_start_icon);
}
