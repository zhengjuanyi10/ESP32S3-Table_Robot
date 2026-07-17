/*
 * 功能：上半身同步动作组——招手、抬手、鼓掌、摇头、点头、转身。
 * 配置：所有函数同步返回，内部调用 robot_motion 关节接口并等待舵机完成。
 *       关节方向约束由 robot_motion.c 运行时检查，此处仅定义幅度与节奏。
 */

#include "servo/robot_actions.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "servo/robot_motion.h"

static const char *TAG = "robot_actions";

/*
 * 动作幅度：若实际方向与名称相反，只改 robot_motion.c 中对应 SIGN。
 *
 * 关节方向约束（robot_motion.c 运行时检查）：
 *   大臂 deg<0（向内过中位）需同侧肩 ≥70° 且最多 -10°；
 *   小臂 deg<0（向外过中位）需同侧肩 ≥45°；
 *   回中 deg=0 始终允许。
 */
#define ACTION_SHOULDER_WAVE_DEG 100        /* 招手肩部前抬 */
#define ACTION_SHOULDER_CLAP_DEG 70         /* 鼓掌肩部，比招手低 */
#define ACTION_FOREARM_WAVE_IN_DEG 35       /* 招手小臂向内起点 */
#define ACTION_FOREARM_WAVE_OUT_DEG 85      /* 招手小臂向外终点 */
#define ACTION_FOREARM_CLAP_OPEN_DEG 25     /* 鼓掌小臂开 */
#define ACTION_FOREARM_CLAP_CLOSE_DEG 55    /* 鼓掌小臂合 */
#define ACTION_CLAP_UPPER_ARM_INWARD_DEG 15 /* 鼓掌大臂内收幅度 */
#define ACTION_BOTH_ARMS_RAISE_DEG 50       /* 抬双手大臂幅度 */
#define ACTION_UPPER_ARM_LIFT_DEG 40        /* 抬手大臂幅度 */
#define ACTION_HEAD_SHAKE_DEG 25            /* 摇头偏航幅度 */
#define ACTION_NOD_DEG 8                    /* 点头俯仰幅度 */

/* 等待时间需覆盖对应角度的舵机运动时间（move_time_ms = 400 + deg×13）。 */
#define ACTION_FOREARM_DELAY_MS 800         /* 肩抬到约 45° 后允许小臂动 */
#define ACTION_SHOULDER_SETTLE_MS 1800      /* 肩 100° 到位（约 1700 ms） */
#define ACTION_CLAP_SETTLE_MS 1400          /* 肩 70° 到位（约 1310 ms） */
#define ACTION_POSE_WAIT_MS 900             /* 姿态稳定等待 */
#define ACTION_WAVE_WAIT_MS 750             /* 招手小臂摆动，短于运动时间产生弹跳感 */
#define ACTION_CLAP_WAIT_MS 700             /* 鼓掌小臂摆动 */
#define ACTION_LIFT_WAIT_MS 1000            /* 大臂抬起／收回等待 */
#define ACTION_HEAD_WAIT_MS 800             /* 摇头偏航等待 */
#define ACTION_NOD_WAIT_MS 600              /* 点头俯仰等待 */
#define ACTION_HOME_TIME_MS 1700            /* 回中时长，收尾柔和 */
#define ACTION_TURN_WAIT_MS 2000            /* 转身等待，暂不调试 */

static void wait_ms(uint32_t milliseconds)
{
    vTaskDelay(pdMS_TO_TICKS(milliseconds));
}

/* 六个手臂关节分两批（4+2）回中；不用广播，避免带动 9 号弯腰舵机。 */
esp_err_t robot_actions_home_arms(void)
{
    return robot_motion_center_arms(ACTION_HOME_TIME_MS);
}

/* ────────────────────────────────────────────
 * 单臂招手：统一左右实现，消除重复代码
 * ──────────────────────────────────────────── */

typedef enum { ARM_RIGHT, ARM_LEFT } arm_side_t;

static esp_err_t wave_side(arm_side_t side)
{
    esp_err_t (*set_shoulder)(int) = (side == ARM_RIGHT)
        ? robot_motion_set_right_shoulder : robot_motion_set_left_shoulder;
    esp_err_t (*set_forearm)(int) = (side == ARM_RIGHT)
        ? robot_motion_set_right_forearm : robot_motion_set_left_forearm;
    const char *name = (side == ARM_RIGHT) ? "right" : "left";

    ESP_LOGI(TAG, "Action: wave %s", name);

    /* 抬肩 → 等肩到 45° → 小臂向内 */
    if (set_shoulder(ACTION_SHOULDER_WAVE_DEG) != ESP_OK) return ESP_FAIL;
    wait_ms(ACTION_FOREARM_DELAY_MS);
    if (set_forearm(ACTION_FOREARM_WAVE_IN_DEG) != ESP_OK) return ESP_FAIL;
    wait_ms(ACTION_POSE_WAIT_MS);

    /* 招手 2 次 */
    for (int i = 0; i < 2; ++i) {
        if (set_forearm(ACTION_FOREARM_WAVE_OUT_DEG) != ESP_OK) return ESP_FAIL;
        wait_ms(ACTION_WAVE_WAIT_MS);
        if (set_forearm(ACTION_FOREARM_WAVE_IN_DEG) != ESP_OK) return ESP_FAIL;
        wait_ms(ACTION_WAVE_WAIT_MS);
    }

    return robot_actions_home_arms();
}

esp_err_t robot_actions_wave_right(void) { return wave_side(ARM_RIGHT); }
esp_err_t robot_actions_wave_left(void)  { return wave_side(ARM_LEFT); }

/* ────────────────────────────────────────────
 * 抬双手
 * ──────────────────────────────────────────── */

esp_err_t robot_actions_raise_both_hands(void)
{
    ESP_LOGI(TAG, "Action: raise both hands");
    if (robot_motion_set_shoulders(ACTION_SHOULDER_WAVE_DEG) != ESP_OK ||
        robot_motion_set_right_upper_arm(ACTION_BOTH_ARMS_RAISE_DEG) != ESP_OK ||
        robot_motion_set_left_upper_arm(ACTION_BOTH_ARMS_RAISE_DEG) != ESP_OK) {
        return ESP_FAIL;
    }
    wait_ms(ACTION_SHOULDER_SETTLE_MS);
    return robot_actions_home_arms();
}

/* ────────────────────────────────────────────
 * 鼓掌：双肩微抬，大臂略内收，小臂开合
 * ──────────────────────────────────────────── */

esp_err_t robot_actions_clap(void)
{
    ESP_LOGI(TAG, "Action: clap");

    /* 双肩到 70°，等肩到 45° 后大臂内收、小臂到位 */
    if (robot_motion_set_shoulders(ACTION_SHOULDER_CLAP_DEG) != ESP_OK) return ESP_FAIL;
    wait_ms(ACTION_FOREARM_DELAY_MS);
    if (robot_motion_set_right_upper_arm(-ACTION_CLAP_UPPER_ARM_INWARD_DEG) != ESP_OK ||
        robot_motion_set_left_upper_arm(-ACTION_CLAP_UPPER_ARM_INWARD_DEG) != ESP_OK ||
        robot_motion_set_right_forearm(ACTION_FOREARM_CLAP_OPEN_DEG) != ESP_OK ||
        robot_motion_set_left_forearm(ACTION_FOREARM_CLAP_OPEN_DEG) != ESP_OK)
        return ESP_FAIL;
    wait_ms(ACTION_CLAP_SETTLE_MS);

    /* 鼓掌 3 次，只动小臂 */
    for (int i = 0; i < 3; ++i) {
        if (robot_motion_set_right_forearm(ACTION_FOREARM_CLAP_CLOSE_DEG) != ESP_OK ||
            robot_motion_set_left_forearm(ACTION_FOREARM_CLAP_CLOSE_DEG) != ESP_OK)
            return ESP_FAIL;
        wait_ms(ACTION_CLAP_WAIT_MS);
        if (robot_motion_set_right_forearm(ACTION_FOREARM_CLAP_OPEN_DEG) != ESP_OK ||
            robot_motion_set_left_forearm(ACTION_FOREARM_CLAP_OPEN_DEG) != ESP_OK)
            return ESP_FAIL;
        wait_ms(ACTION_CLAP_WAIT_MS);
    }

    return robot_actions_home_arms();
}

/* ────────────────────────────────────────────
 * 抬手：只动大臂，肩和小臂不动，抬起→收回×2
 * ──────────────────────────────────────────── */

esp_err_t robot_actions_lift_upper_arms(void)
{
    ESP_LOGI(TAG, "Action: lift upper arms x2");

    for (int i = 0; i < 2; ++i) {
        if (robot_motion_set_right_upper_arm(ACTION_UPPER_ARM_LIFT_DEG) != ESP_OK ||
            robot_motion_set_left_upper_arm(ACTION_UPPER_ARM_LIFT_DEG) != ESP_OK)
            return ESP_FAIL;
        wait_ms(ACTION_LIFT_WAIT_MS);
        if (robot_motion_set_right_upper_arm(0) != ESP_OK ||
            robot_motion_set_left_upper_arm(0) != ESP_OK)
            return ESP_FAIL;
        wait_ms(ACTION_LIFT_WAIT_MS);
    }

    return ESP_OK;
}

/* ────────────────────────────────────────────
 * 摇头：左右偏航摆动，最后回中
 * ──────────────────────────────────────────── */

esp_err_t robot_actions_shake_head(void)
{
    ESP_LOGI(TAG, "Action: shake head");
    if (robot_motion_set_head_yaw(ACTION_HEAD_SHAKE_DEG) != ESP_OK) return ESP_FAIL;
    wait_ms(ACTION_HEAD_WAIT_MS);
    if (robot_motion_set_head_yaw(-ACTION_HEAD_SHAKE_DEG) != ESP_OK) return ESP_FAIL;
    wait_ms(ACTION_HEAD_WAIT_MS);
    if (robot_motion_set_head_yaw(ACTION_HEAD_SHAKE_DEG) != ESP_OK) return ESP_FAIL;
    wait_ms(ACTION_HEAD_WAIT_MS);
    return robot_motion_set_head_yaw(0);
}

/* ────────────────────────────────────────────
 * 招呼 = 右招手 + 摇头
 * ──────────────────────────────────────────── */

esp_err_t robot_actions_greet(void)
{
    if (robot_actions_wave_right() != ESP_OK) return ESP_FAIL;
    return robot_actions_shake_head();
}

/* ────────────────────────────────────────────
 * 点头：小幅度俯仰两次
 * ──────────────────────────────────────────── */

esp_err_t robot_actions_nod(void)
{
    ESP_LOGI(TAG, "Action: nod");

    for (int i = 0; i < 2; ++i) {
        if (robot_motion_set_head_pitch(ACTION_NOD_DEG) != ESP_OK) return ESP_FAIL;
        wait_ms(ACTION_NOD_WAIT_MS);
        if (robot_motion_set_head_pitch(0) != ESP_OK) return ESP_FAIL;
        wait_ms(ACTION_NOD_WAIT_MS);
    }

    return ESP_OK;
}

/* ────────────────────────────────────────────
 * 底座转身：10 号舵机，后续与小车转向联动
 * ──────────────────────────────────────────── */

esp_err_t robot_actions_turn(int degrees)
{
    ESP_LOGI(TAG, "Action: turn %d deg", degrees);
    if (robot_motion_set_base_yaw(degrees) != ESP_OK) return ESP_FAIL;
    wait_ms(ACTION_TURN_WAIT_MS);
    return ESP_OK;
}
