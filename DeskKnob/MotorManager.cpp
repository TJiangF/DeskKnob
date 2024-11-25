#include "MotorManager.h"
#include <algorithm>


MotorManager::MotorManager(MagneticSensorI2C& sensor_ref, int positions, float stiffness)
    : sensor(sensor_ref),   // 使用引用初始化传感器
      motor(7),         // 设置电机极对数
      driver(40, 38, 41, 37, 42, 39, 45),  // PWM 引脚固定
      ControlMode(0),
      num_positions(positions),  // 使用传参设置默认档位数
      torque_k(stiffness),       // 使用传参设置扭矩刚度
      attractor_distance(0),
      angletarget(0.0f),
      current_gear(0),
      last_gear(0),
      current_angle(0) {}

void MotorManager::init() {
    sensor.init(&Wire);
    motor.linkSensor(&sensor);

    driver.dead_zone = 0.02f;
    driver.voltage_power_supply = 3.3;
    driver.voltage_limit = 5;
    driver.pwm_frequency = 32000;
    driver.init();

    motor.linkDriver(&driver);
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.PID_velocity.P = 0.2;
    motor.PID_velocity.I = 5;
    motor.PID_velocity.D = 0.001;
    motor.PID_velocity.output_ramp = 250;
    motor.LPF_velocity.Tf = 0.05;
    motor.voltage_limit = 5;
    motor.current_limit = 0.6;
    motor.P_angle.P = 9.0f;
    motor.velocity_limit = 60;

    motor.controller = MotionControlType::angle;
    // motor.useMonitoring(Serial);
    motor.init();
    motor.enable();
    motor.initFOC();
    current_angle = sensor.getAngle();
    attractor_distance = 2 * PI / num_positions;
}


int MotorManager::GetControlMode() {
    return ControlMode;
}


void MotorManager::updateControlMode(int motor_mode) {
    switch(motor_mode)
    {
      case 0:
        ControlMode = 0;
        motor.controller = MotionControlType::angle;
        break;
      case 1:
        ControlMode = 1;
        motor.controller = MotionControlType::torque;
        break;
    }
}


void MotorManager::updateAngletarget(float new_target) {
      angletarget = new_target;
}

float MotorManager::GetTargetangle(){
    return angletarget;

}

int MotorManager::GetGearnum(){
    return num_positions;

}

int MotorManager::GetCurrentGear(){
    return current_gear;
}


void MotorManager::updateGearnum(int new_num){
      num_positions = new_num;
      attractor_distance = 2 * PI / num_positions;
}

// void MotorManager::setRotationCallback(std::function<void(int)> callback) {
//     rotation_callback = callback;
// }


void MotorManager::Haptic(bool press) {
    motor.controller = MotionControlType::torque;
    float strength =  press ? 5.0f : 1.5f;
    motor.move(strength * torque_k);
    for (uint8_t i = 0; i < 4; i++) {
        motor.loopFOC();
        delay(1); // 短延迟
    }

    motor.move(-strength * torque_k);
    for (uint8_t i = 0; i < 4; i++) {
        motor.loopFOC();
        delay(1); // 短延迟
    }


    motor.move(0);
    motor.loopFOC();
    if(ControlMode != 1)  {
      motor.controller = MotionControlType::angle;
    }
}

void MotorManager::setGearChangeCallback(std::function<void(int)> callback) {
    gear_change_callback = callback;
}

void MotorManager::run() {
    motor.loopFOC();
    current_angle = sensor.getAngle();
    switch(ControlMode){
      case 0:
        motor.move(angletarget);
        break;
      case 1:
        current_gear = std::max(0, std::min(static_cast<int>(round(current_angle / attractor_distance)), num_positions));
        if (current_gear != last_gear) {
            if (gear_change_callback) {
                gear_change_callback(current_gear - last_gear); // 调用回调函数
            }
            last_gear = current_gear;
        }
        float torque_target = current_gear * attractor_distance;
        float torque = torque_k * std::min(0.8f, static_cast<float>(torque_target - current_angle));// limit the max torque when exceeds the zero or num
        motor.move(torque);  // 扭矩控制
        break;

    }
    
}
