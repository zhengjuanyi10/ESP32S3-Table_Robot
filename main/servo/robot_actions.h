/*
 * 功能：上半身动作组——招手、抬手、鼓掌、摇头、点头、转身等同步动作。
 * 配置：使用 1-8 号关节，9 号弯腰和 10 号底座旋转不参与动作组。
 *       所有函数同步返回，内部等待动作完成。
 */

#pragma once

#include "esp_err.h"

/* 手臂关节分两批（4+2）回中，不带动 9 号弯腰舵机。 */
esp_err_t robot_actions_home_arms(void);

/* 单臂招手，完成后手臂回中。 */
esp_err_t robot_actions_wave_right(void);
esp_err_t robot_actions_wave_left(void);

/* 双手抬起后放下。 */
esp_err_t robot_actions_raise_both_hands(void);

/* 鼓掌：双肩微抬，大臂略内收，小臂开合。 */
esp_err_t robot_actions_clap(void);

/* 抬手：只动大臂抬起再收回，重复两次，肩和小臂保持中位。 */
esp_err_t robot_actions_lift_upper_arms(void);

/* 摇头：左右偏航摆动，完成后 8 号舵机回中。 */
esp_err_t robot_actions_shake_head(void);

/* 打招呼组合：右招手后摇头。 */
esp_err_t robot_actions_greet(void);

/* 点头：小幅度俯仰两次，使用 7 号舵机。 */
esp_err_t robot_actions_nod(void);

/* 底座转身：10 号舵机，后续与小车转向联动，暂不调试。 */
esp_err_t robot_actions_turn(int degrees);
