#include "DisplayManager.h"
#include <cstdio>

DisplayManager::DisplayManager(): pressed_label(nullptr), current_mode(AngleMode) {}

void DisplayManager::init() {
    // 公共 UI 初始化
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

    pressed_label = lv_label_create(lv_scr_act());
    lv_label_set_text(pressed_label, "Pressed!");
    lv_obj_align(pressed_label, LV_ALIGN_CENTER, 0, -60);
    common_ui_objects.push_back(pressed_label);


    // 初始化为当前模式
    updateModeDisplay(current_mode);
}

void DisplayManager::updateAngle(float angle) {
    // 示例更新方法
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "Angle: %.2f", angle);
    lv_label_set_text(mode_ui_map[AngleMode][0], buffer);
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

        int percentage = (total > 0) ? (current * 100 / total) : 0;
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

void DisplayManager::showPressedMessage() {
    if (pressed_label == nullptr) {
        pressed_label = lv_label_create(lv_scr_act());
    }

    lv_label_set_text(pressed_label, "Pressed!");
    lv_obj_align(pressed_label, LV_ALIGN_CENTER, 0, -60);
    lv_obj_clear_flag(pressed_label, LV_OBJ_FLAG_HIDDEN); // 显示标签
}

void DisplayManager::clearPressedMessage() {
    if (pressed_label != nullptr) {
        lv_obj_add_flag(pressed_label, LV_OBJ_FLAG_HIDDEN); // 隐藏标签
    }
}


void DisplayManager::setBackgroundColor(lv_color_t color) {
    lv_obj_set_style_bg_color(lv_scr_act(), color, LV_PART_MAIN | LV_STATE_DEFAULT);
}


void DisplayManager::clearAllMessage() {
    // 隐藏所有对象（包括公共和模式特定对象）
    for (auto obj : common_ui_objects) {
        lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
    }
    for (auto &[_, objects] : mode_ui_map) {
        for (auto obj : objects) {
            lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
        }
    }
}
