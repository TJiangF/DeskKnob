#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <SimpleFOC.h>

class MotorManager {
public:
    MotorManager(MagneticSensorI2C& sensor, int positions = 8, float stiffness = 5.0f);
    void init();                                // 初始化电机
    void updateControlMode(int motor_mode);     // 切换控制模式
    int GetControlMode();                       // 切换控制模式
    void updateAngletarget(float new_target);   // 更新目标角度/扭矩模式的档位数
    float GetTargetangle();                     // 更新目标角度/扭矩模式的档位数
    void updateGearnum(int new_num);            // 更新目标角度/扭矩模式的档位数
    int GetGearnum();
    int GetCurrentGear();

    void Haptic(bool press);    // 模拟震动反馈


    void setGearChangeCallback(std::function<void(int)> callback);

    void run();                                 // 在循环中运行控制逻辑
    
    

    int ControlMode;
    // 0 - angle 
    // 1 - torque

    MagneticSensorI2C& sensor;                     // 引用传感器对象

    BLDCMotor motor;                               // BLDC 电机
    BLDCDriver6PWM driver;                         // 电机驱动器
    float current_angle;
    int num_positions;                             // 档位数量（仅在扭矩模式中有用）
    int current_gear;                              // the position the knob is at
    int last_gear;                                 // the position the knob is at
    float attractor_distance;                      // 档位之间的距离
    float torque_k;                                // 扭矩刚度系数
    float angletarget;                             // 当前目标（角度或档位）
    std::function<void(int)> gear_change_callback;
    // std::function<void(int)> rotation_callback;    // 旋钮变化回调函数
};

#endif // MOTOR_MANAGER_H
