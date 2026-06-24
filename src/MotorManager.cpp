#include "MotorManager.h"
#include <algorithm>
#include <math.h>

MotorManager::MotorManager(MagneticSensorI2C& sensor_ref, int positions, float stiffness)
    : sensor(sensor_ref),   // 使用引用初始化传感器
      motor(7),         // 设置电机极对数
      driver(40, 38, 41, 37, 42, 39, 45),  // PWM 引脚固定
      CurrentControlMode(AngleControl),
      num_positions(positions),  // 使用传参设置默认档位数
      torque_k(stiffness),       // 使用传参设置扭矩刚度
      attractor_distance(0),
      angletarget(0.0f),
      current_gear(0),
      max_gear(0),
      last_gear(0),
      current_angle(0),
      zero_angle(0) {}

void MotorManager::init() {
    SimpleFOCDebug::enable(&Serial);
    sensor.init(&Wire);
    motor.linkSensor(&sensor);

    driver.dead_zone = 0.02f;
    driver.voltage_power_supply = 3.3;
    driver.voltage_limit = 5;
    driver.pwm_frequency = 32000;
    driver.init();

    motor.linkDriver(&driver);
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.PID_velocity.P = 0.15;
    motor.PID_velocity.I = 5;
    motor.PID_velocity.D = 0.001;
    motor.PID_velocity.output_ramp = 200;
    motor.LPF_velocity.Tf = 0.05;
    motor.voltage_limit = 3.7;
    motor.current_limit = 0.5;
    motor.P_angle.P = 7.0f;
    motor.velocity_limit = 60;

    motor.controller = MotionControlType::angle;
    // motor.useMonitoring(Serial);
    motor.init();
    motor.initFOC();
    motor.enable();
    
    current_angle = sensor.getAngle();
    zero_angle = current_angle;
    attractor_distance = 2 * PI / num_positions;
}


MotorManager::ControlMode MotorManager::getControlMode() const {
    return CurrentControlMode;
}


void MotorManager::updateControlMode(ControlMode mode, int Gear_limit) {
    CurrentControlMode = mode;
    switch (mode) {
        case AngleControl:
            zero_angle = sensor.getAngle();
            motor.controller = MotionControlType::angle;
            break;
        case Infinite_TorqueControl:
            zero_angle = sensor.getAngle();
            current_gear = 0;
            last_gear = 0;
            motor.controller = MotionControlType::torque;
            break;
        case Uninfinite_TorqueControl:
            zero_angle = sensor.getAngle();
            current_gear = 0;
            last_gear = 0;
            max_gear = Gear_limit;
            motor.controller = MotionControlType::torque;
            break;        
    }
}


void MotorManager::updateAngletarget(float new_target) {
      angletarget = new_target;
}

float MotorManager::GetTargetangle(){return angletarget;}
float MotorManager::GetAngle(){return sensor.getAngle();}
float MotorManager::GetVelocity(){return sensor.getVelocity();}
int MotorManager::GetGearnum(){return num_positions;}
int MotorManager::GetCurrentGear(){return current_gear;}


void MotorManager::updateGearnum(int new_num){
      num_positions = new_num;
      attractor_distance = 2 * PI / num_positions;
}

// void MotorManager::setRotationCallback(std::function<void(int)> callback) {
//     rotation_callback = callback;
// }


// void MotorManager::Haptic(bool press) {
//     motor.controller = MotionControlType::torque;
//     float strength =  press ? 5.0f : 1.5f;
//     // motor.move(strength * torque_k);
//     for (uint8_t i = 0; i < 4; i++) {
//         motor.loopFOC();
//         motor.move(strength * torque_k);
//         // delay(1); // 短延迟
//     }

//     // motor.move(-strength * torque_k);
//     for (uint8_t i = 0; i < 4; i++) {
//         motor.loopFOC();
//         motor.move(-strength * torque_k);
//         // delay(1); // 短延迟
//     }
//     motor.move(0);
//     motor.loopFOC();
//     if(ControlMode != 1)  {
//       motor.controller = MotionControlType::angle;
//     }
// }



void MotorManager::setGearChangeCallback(std::function<void(int)> callback) {
    gear_change_callback = callback;
}

void MotorManager::run() {
    motor.loopFOC();
    current_angle = sensor.getAngle() - zero_angle;
    float torque_target = 0;
    float torque = 0;
    switch (CurrentControlMode) {
      case AngleControl:
        // current_angle = sensor.getAngle();
        motor.move(angletarget + zero_angle);
        break;
      case Infinite_TorqueControl:
        current_gear = round(current_angle / attractor_distance);
        if (current_gear != last_gear && gear_change_callback) {
            gear_change_callback(current_gear - last_gear); // 调用回调函数
            last_gear = current_gear;
        }
        torque_target = current_gear * attractor_distance;
        torque = torque_k * std::min(0.8f, static_cast<float>(torque_target - current_angle));// limit the max torque when exceeds the zero or num
        motor.move(torque);  // 扭矩控制
        break;      
      case Uninfinite_TorqueControl:
        // current_angle = sensor.getAngle() - zero_angle;
        current_gear = int(std::min(float(max_gear - 1), std::max(0.0f, round(current_angle / attractor_distance))));
        // current_gear = int(round(current_angle / attractor_distance)) % num_positions; // not exceeding num_positions
        if (current_gear != last_gear && gear_change_callback) {
            gear_change_callback(current_gear - last_gear); // 调用回调函数
            last_gear = current_gear;
        }
        torque_target = current_gear * attractor_distance;
        torque = torque_k * std::min(0.7f, static_cast<float>(torque_target - current_angle));// limit the max torque when exceeds the zero or num
        motor.move(torque);  // 扭矩控制
        break;

    }
    
}
