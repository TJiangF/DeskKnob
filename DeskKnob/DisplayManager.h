// #ifndef DISPLAY_MANAGER_H
// #define DISPLAY_MANAGER_H

// #include <lvgl.h>
// #include <vector>
// class DisplayManager {
// public:
//     DisplayManager();  // 构造函数
//     void init();       // 初始化显示


//     void updateAngle(float angle);    // 更新角度显示
//     void updateTarAngle(float Target_angle);    // 更新角度显示
//     void updateVelocity(float velocity); // 更新速度显示
//     void updatepsvalue(int32_t);

    
//     void showPressedMessage(); // 显示按下的信息  
//     void clearPressedMessage(); // 清除按下的信息

//     void updateTorqueMode(int current, int total); // 更新档位信息显示


//     void setBackgroundColor(lv_color_t color); // 设置背景颜色
//     void clearAllMessage();


//     // void showAngle();  // 显示角度
//     // void showVelocity(); // 显示速度

// private:

//     std::vector<lv_obj_t*> ui_objects;     // 保存所有 UI 对象的容器
//     lv_obj_t *angle_label;    // 用于显示角度的标签
//     lv_obj_t *target_angle_label;    // 用于显示角度的标签
//     lv_obj_t *velocity_label; // 用于显示速度的标签
//     lv_obj_t *pressed_label; // 用于显示速度的标签
//     lv_obj_t *psvalue_label; // 用于显示速度的标签
//     lv_obj_t *gear_label;                   // 显示档位信息的标签
//     lv_obj_t *progress_arc;                 // 显示当前档位百分比的圆弧

// };

// #endif // DISPLAY_MANAGER_H


#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <lvgl.h>
#include <vector>
#include <map>

class DisplayManager {
public:
    enum Mode {
        AngleMode,
        TorqueMode,
        // VelocityMode,
        // 可以扩展更多模式
    };

    DisplayManager();                      // 构造函数
    void init();                           // 初始化显示

    void updateAngle(float angle);         // 更新角度显示
    void updateTarAngle(float Target_angle); // 更新目标角度显示
    void updateVelocity(float velocity);   // 更新速度显示

    void updateTorqueMode(int current, int total); // 更新档位信息显示

    void updateModeDisplay(Mode mode);     // 更新模式显示

    void showPressedMessage(); // 显示按下的信息  
    void clearPressedMessage(); // 清除按下的信息

    void setBackgroundColor(lv_color_t color); // 设置背景颜色
    void clearAllMessage();                // 清空所有显示内容

private:
    std::map<Mode, std::vector<lv_obj_t*>> mode_ui_map; // 各模式的 UI 对象组
    std::vector<lv_obj_t*> common_ui_objects;          // 公共 UI 对象
    Mode current_mode;                                 // 当前模式
    lv_obj_t* pressed_label;
};

#endif // DISPLAY_MANAGER_H
