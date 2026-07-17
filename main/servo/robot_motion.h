/*
 * 功能：ZP 总线舵机底层驱动——初始化、回中、单关节姿态设置、串口调试控制台。
 * 配置：UART1, TX=GPIO17→ZLink RX, RX=GPIO16←ZLink TX, 115200 8N1,
 *       广播回中 PWM/CONFIG_TABLE_ROBOT_SERVO_CENTER_PWM, BOOT 引脚长按回中。
 */

#pragma once

#include "esp_err.h"

/* 舵机模块生命周期：main.c 只调用这四个入口。 */
void robot_motion_init(void);
void robot_motion_start_debug_console(void);
void robot_motion_center_all(void);
void robot_motion_poll(void);

/*
 * 上半身基础姿态接口。
 * 角度均以回中位（PWM 1500）为基准，传入 0 即回中。
 * ZP 协议有效范围 ±270°，各关节有额外方向约束（见 .c 实现）。
 */

esp_err_t robot_motion_set_head_yaw(int degrees);
esp_err_t robot_motion_set_head_pitch(int degrees);
esp_err_t robot_motion_set_torso_pitch(int degrees);
esp_err_t robot_motion_set_base_yaw(int degrees);

esp_err_t robot_motion_set_right_shoulder(int degrees);
esp_err_t robot_motion_set_left_shoulder(int degrees);
esp_err_t robot_motion_set_shoulders(int degrees);

esp_err_t robot_motion_set_right_forearm(int degrees);
esp_err_t robot_motion_set_right_upper_arm(int degrees);
esp_err_t robot_motion_set_left_upper_arm(int degrees);
esp_err_t robot_motion_set_left_forearm(int degrees);

/*
 * 动作组回位：六个手臂关节以 time_ms 时长分两批（4+2）回中，
 * 保证同时运动的舵机不超过 4 个。
 */
esp_err_t robot_motion_center_arms(int time_ms);
