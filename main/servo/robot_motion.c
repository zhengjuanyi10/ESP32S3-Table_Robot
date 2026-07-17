#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "servo/robot_motion.h"
#include "servo/robot_actions.h"

/*
 * 功能：ZP 总线舵机驱动——UART 收发、回中校验、单关节姿态、串口调试控制台。
 * 配置：UART1, TX=GPIO17→ZLink RX, RX=GPIO16←ZLink TX, 115200 8N1,
 *       广播回中 PWM/CONFIG_TABLE_ROBOT_SERVO_CENTER_PWM,
 *       BOOT 引脚长按 CONFIG_TABLE_ROBOT_SERVO_CENTER_HOLD_MS 触发回中。
 */

static const char *TAG = "robot_motion";

#if CONFIG_TABLE_ROBOT_SERVO_ENABLE

static const uart_port_t SERVO_UART = UART_NUM_1;

#define SERVO_MIN_PWM 500
#define SERVO_MAX_PWM 2500
#define SERVO_RANGE_DEGREES 270
#define SERVO_COMMAND_GAP_MS 20
#define SERVO_BUS_LOCK_MS 1000
#define SERVO_LINE_ENDING "\r\n"
#define SERVO_LINE_ENDING_LENGTH 2

/* 关节 ID 与方向。若实际方向相反，只改对应的 SIGN。 */
#define RIGHT_FOREARM_ID 1
#define RIGHT_UPPER_ARM_ID 2
#define RIGHT_SHOULDER_ID 3
#define LEFT_SHOULDER_ID 4
#define LEFT_UPPER_ARM_ID 5
#define LEFT_FOREARM_ID 6
#define HEAD_PITCH_ID 7
#define HEAD_YAW_ID 8
#define TORSO_PITCH_ID 9
#define BASE_YAW_ID 10

#define RIGHT_FOREARM_SIGN -1
#define RIGHT_UPPER_ARM_SIGN -1
#define RIGHT_SHOULDER_SIGN 1
#define LEFT_SHOULDER_SIGN -1  /* 左右肩镜像安装 */
#define LEFT_UPPER_ARM_SIGN 1
#define LEFT_FOREARM_SIGN 1
#define HEAD_PITCH_SIGN 1
#define HEAD_YAW_LEFT_SIGN 1
#define TORSO_BOW_SIGN 1
#define BASE_YAW_SIGN 1

static const uint8_t configured_ids[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
static SemaphoreHandle_t servo_bus_mutex;
static int servo_baud_rate = CONFIG_TABLE_ROBOT_SERVO_BAUD_RATE;
static bool test_running;
static bool action_running;
static bool verify_running;

/* 跟踪双肩角度，用于关节运动方向约束。仅记录通过 shoulder API 设置的值。 */
static int g_shoulder_right_deg = 0;
static int g_shoulder_left_deg = 0;
#define SHOULDER_DEG_FOR_UPPER_ARM_INWARD 70  /* 大臂向内过中位需肩 ≥70° */
#define SHOULDER_DEG_FOR_FOREARM_OUTWARD 45   /* 小臂向外过中位需肩 ≥45° */
#define UPPER_ARM_INWARD_MAX_DEG 10           /* 大臂向内过中位绝对上限 */

#if CONFIG_TABLE_ROBOT_SERVO_VERIFY_CENTER_AFTER_MOVE
static void verify_center_task(void *arg);
#endif

#if CONFIG_TABLE_ROBOT_SERVO_AUTO_CENTER_ON_BOOT
static void auto_center_task(void *arg);
#endif

static bool lock_servo_bus(void)
{
    if (servo_bus_mutex == NULL ||
        xSemaphoreTake(servo_bus_mutex, pdMS_TO_TICKS(SERVO_BUS_LOCK_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Servo UART is busy");
        return false;
    }
    return true;
}

static void unlock_servo_bus(void)
{
    xSemaphoreGive(servo_bus_mutex);
}

static bool set_servo_baud_rate(int baud_rate)
{
    if (!lock_servo_bus()) {
        return false;
    }

    esp_err_t err = uart_wait_tx_done(SERVO_UART, pdMS_TO_TICKS(100));
    if (err == ESP_OK) {
        err = uart_set_baudrate(SERVO_UART, baud_rate);
    }
    if (err == ESP_OK) {
        uart_flush_input(SERVO_UART);
    }

    uint32_t actual_baud = 0;
    if (err == ESP_OK) {
        err = uart_get_baudrate(SERVO_UART, &actual_baud);
    }
    unlock_servo_bus();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not set servo UART baud rate to %d: %s", baud_rate,
                 esp_err_to_name(err));
        return false;
    }

    servo_baud_rate = (int)actual_baud;
    ESP_LOGW(TAG, "Servo UART baud rate: %d", servo_baud_rate);
    return true;
}

static bool write_command(const char *command, int length)
{
    if (!lock_servo_bus()) {
        return false;
    }

    /* 商家 ESP32 例程使用 Serial.println()，所以每条命令后追加 CRLF。 */
    const int written = uart_write_bytes(SERVO_UART, command, length);
    const int ending_written = written == length
                                   ? uart_write_bytes(SERVO_UART, SERVO_LINE_ENDING,
                                                      SERVO_LINE_ENDING_LENGTH)
                                   : 0;
    esp_err_t tx_err = ESP_FAIL;
    if (written == length && ending_written == SERVO_LINE_ENDING_LENGTH) {
        tx_err = uart_wait_tx_done(SERVO_UART, pdMS_TO_TICKS(100));
    }
    unlock_servo_bus();

    if (written != length || ending_written != SERVO_LINE_ENDING_LENGTH || tx_err != ESP_OK) {
        ESP_LOGE(TAG, "UART TX failed (frame=%d/%d, CRLF=%d/2, wait=%s)", written, length,
                 ending_written, esp_err_to_name(tx_err));
        return false;
    }

    /* 给 ZLink/舵机留出解析一帧命令的时间，避免动作组连续发送时丢帧。 */
    vTaskDelay(pdMS_TO_TICKS(SERVO_COMMAND_GAP_MS));
    return true;
}

static bool send_position(uint8_t id, int pwm, int time_ms)
{
    /* ZP 写位置协议：#IDP位置T时间! */
    char command[20];
    const int length = snprintf(command, sizeof(command), "#%03uP%04dT%04d!", id, pwm, time_ms);
    if (length < 0 || length >= (int)sizeof(command) ||
        !write_command(command, length)) {
        return false;
    }
    ESP_LOGD(TAG, "TX: %s", command);
    return true;
}

static bool read_position(uint8_t id, int *pwm)
{
    /* 读取反馈前清空旧数据，只接受完整的 #IDPxxxx! 帧。 */
    char command[16];
    char response[32] = {0};
    const int length = snprintf(command, sizeof(command), "#%03uPRAD!", id);
    if (length < 0 || length >= (int)sizeof(command)) {
        return false;
    }

    if (!lock_servo_bus()) {
        return false;
    }

    uart_flush_input(SERVO_UART);
    if (uart_write_bytes(SERVO_UART, command, length) != length ||
        uart_write_bytes(SERVO_UART, SERVO_LINE_ENDING, SERVO_LINE_ENDING_LENGTH) !=
            SERVO_LINE_ENDING_LENGTH ||
        uart_wait_tx_done(SERVO_UART, pdMS_TO_TICKS(100)) != ESP_OK) {
        unlock_servo_bus();
        return false;
    }

    size_t used = 0;
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(300);
    while (used < sizeof(response) - 1 && xTaskGetTickCount() < deadline) {
        const int received = uart_read_bytes(SERVO_UART, (uint8_t *)response + used,
                                             sizeof(response) - 1 - used, pdMS_TO_TICKS(20));
        if (received > 0) {
            used += received;
            response[used] = '\0';
            if (strchr(response, '!') != NULL) {
                break;
            }
        }
    }
    unlock_servo_bus();

    int response_id = -1;
    int response_pwm = -1;
    const char *frame = strchr(response, '#');
    if (frame == NULL || sscanf(frame, "#%3dP%4d!", &response_id, &response_pwm) != 2 ||
        response_id != id || response_pwm < SERVO_MIN_PWM || response_pwm > SERVO_MAX_PWM) {
        if (used > 0) {
            ESP_LOGW(TAG, "Unexpected reply to servo %u: %s", id, response);
        }
        return false;
    }

    *pwm = response_pwm;
    return true;
}

static int target_from_angle(int center_pwm, int degrees)
{
    /* ZP 的 500~2500 PWM 对应约 270 度，末端自动限位。 */
    const int pwm_delta = (abs(degrees) * 2000 + SERVO_RANGE_DEGREES / 2) / SERVO_RANGE_DEGREES;
    int target = center_pwm + (degrees >= 0 ? pwm_delta : -pwm_delta);
    if (target < SERVO_MIN_PWM) {
        return SERVO_MIN_PWM;
    }
    return target > SERVO_MAX_PWM ? SERVO_MAX_PWM : target;
}

static int move_time_ms(int degrees)
{
    /* 基础 400ms + 每度 13ms，整体约为初版（500+每度20）的 1.5 倍速。 */
    const int time_ms = 400 + abs(degrees) * 13;
    return time_ms > 5000 ? 5000 : time_ms;
}

static bool is_valid_angle(int degrees)
{
    return degrees >= -270 && degrees <= 270;
}

static esp_err_t move_pose(uint8_t id, int degrees, int sign, const char *name)
{
    const int target = target_from_angle(CONFIG_TABLE_ROBOT_SERVO_CENTER_PWM, degrees * sign);
    if (!send_position(id, target, move_time_ms(degrees))) {
        ESP_LOGE(TAG, "Failed to send %s pose", name);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "%s: %d deg from center (servo %u PWM %d)", name, degrees, id, target);
    return ESP_OK;
}

/* 所有已启用的单关节 API 共用这一条路径，保证角度语义一致。 */
static esp_err_t set_joint(uint8_t id, int degrees, int sign, const char *name)
{
    return is_valid_angle(degrees) ? move_pose(id, degrees, sign, name) : ESP_ERR_INVALID_ARG;
}

void robot_motion_init(void)
{
    /* UART1 连接 ZLink：GPIO17 发、GPIO16 收；BOOT 长按用于紧急回中。 */
    const uart_config_t uart_config = {
        .baud_rate = CONFIG_TABLE_ROBOT_SERVO_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    const gpio_config_t button_config = {
        .pin_bit_mask = 1ULL << CONFIG_TABLE_ROBOT_SERVO_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    if (SERVO_UART == CONFIG_ESP_CONSOLE_UART_NUM) {
        ESP_LOGE(TAG, "Servo UART%d conflicts with the serial console", SERVO_UART);
        abort();
    }
    servo_bus_mutex = xSemaphoreCreateMutex();
    if (servo_bus_mutex == NULL) {
        ESP_LOGE(TAG, "Could not create servo UART mutex");
        abort();
    }

    ESP_ERROR_CHECK(uart_param_config(SERVO_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(SERVO_UART, CONFIG_TABLE_ROBOT_SERVO_TX_GPIO,
                                 CONFIG_TABLE_ROBOT_SERVO_RX_GPIO, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(gpio_set_drive_capability(CONFIG_TABLE_ROBOT_SERVO_TX_GPIO,
                                               GPIO_DRIVE_CAP_3));
    ESP_ERROR_CHECK(gpio_set_pull_mode(CONFIG_TABLE_ROBOT_SERVO_RX_GPIO, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(uart_driver_install(SERVO_UART, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(gpio_config(&button_config));

    uint32_t actual_baud = 0;
    ESP_ERROR_CHECK(uart_get_baudrate(SERVO_UART, &actual_baud));
    servo_baud_rate = (int)actual_baud;
    ESP_LOGI(TAG, "ZP bus ready: UART%d, TX=GPIO%d, RX=GPIO%d, %d 8N1", SERVO_UART,
             CONFIG_TABLE_ROBOT_SERVO_TX_GPIO, CONFIG_TABLE_ROBOT_SERVO_RX_GPIO,
             servo_baud_rate);

#if CONFIG_TABLE_ROBOT_SERVO_AUTO_CENTER_ON_BOOT
    if (xTaskCreate(auto_center_task, "servo_auto_center", 3072, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Could not start automatic servo centering");
    }
#endif
}

void robot_motion_center_all(void)
{
    /* 255 是广播 ID：总线上所有舵机以设定时间一起回中。 */
    char command[20];
    const int length = snprintf(command, sizeof(command), "#255P%04dT%04d!",
                                CONFIG_TABLE_ROBOT_SERVO_CENTER_PWM,
                                CONFIG_TABLE_ROBOT_SERVO_CENTER_TIME_MS);
    if (length < 0 || length >= (int)sizeof(command) ||
        !write_command(command, length)) {
        ESP_LOGE(TAG, "Failed to send center command");
        return;
    }
    ESP_LOGI(TAG, "Center command transmitted: %s", command);

#if CONFIG_TABLE_ROBOT_SERVO_VERIFY_CENTER_AFTER_MOVE
    if (verify_running) {
        ESP_LOGW(TAG, "Center verification is already running");
    } else if (xTaskCreate(verify_center_task, "servo_verify", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Could not start center verification");
    } else {
        verify_running = true;
    }
#endif
}

#if CONFIG_TABLE_ROBOT_SERVO_AUTO_CENTER_ON_BOOT
static void auto_center_task(void *arg)
{
    ESP_LOGW(TAG, "Automatic servo centering in %d ms",
             CONFIG_TABLE_ROBOT_SERVO_AUTO_CENTER_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(CONFIG_TABLE_ROBOT_SERVO_AUTO_CENTER_DELAY_MS));
    robot_motion_center_all();
    vTaskDelete(NULL);
}
#endif

esp_err_t robot_motion_set_head_yaw(int degrees)
{
    return set_joint(HEAD_YAW_ID, degrees, HEAD_YAW_LEFT_SIGN, "Head yaw");
}

esp_err_t robot_motion_set_head_pitch(int degrees)
{
    return set_joint(HEAD_PITCH_ID, degrees, HEAD_PITCH_SIGN, "Head pitch");
}

esp_err_t robot_motion_set_torso_pitch(int degrees)
{
    return set_joint(TORSO_PITCH_ID, degrees, TORSO_BOW_SIGN, "Torso pitch");
}

esp_err_t robot_motion_set_base_yaw(int degrees)
{
    /* 10 号底座旋转：后续与小车转向联动，暂不接入动作组调试。 */
    return set_joint(BASE_YAW_ID, degrees, BASE_YAW_SIGN, "Base yaw");
}

esp_err_t robot_motion_set_right_shoulder(int degrees)
{
    esp_err_t err = set_joint(RIGHT_SHOULDER_ID, degrees, RIGHT_SHOULDER_SIGN, "Right shoulder");
    if (err == ESP_OK) {
        g_shoulder_right_deg = degrees;
    }
    return err;
}

esp_err_t robot_motion_set_left_shoulder(int degrees)
{
    esp_err_t err = set_joint(LEFT_SHOULDER_ID, degrees, LEFT_SHOULDER_SIGN, "Left shoulder");
    if (err == ESP_OK) {
        g_shoulder_left_deg = degrees;
    }
    return err;
}

esp_err_t robot_motion_set_shoulders(int degrees)
{
    if (!is_valid_angle(degrees)) {
        return ESP_ERR_INVALID_ARG;
    }

    const int right_target = target_from_angle(CONFIG_TABLE_ROBOT_SERVO_CENTER_PWM,
                                                degrees * RIGHT_SHOULDER_SIGN);
    const int left_target = target_from_angle(CONFIG_TABLE_ROBOT_SERVO_CENTER_PWM,
                                               degrees * LEFT_SHOULDER_SIGN);
    const int time_ms = move_time_ms(degrees);
    if (!send_position(RIGHT_SHOULDER_ID, right_target, time_ms) ||
        !send_position(LEFT_SHOULDER_ID, left_target, time_ms)) {
        ESP_LOGE(TAG, "Failed to send shoulder pair pose");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Shoulder pair: %d deg from center", degrees);
    g_shoulder_right_deg = degrees;
    g_shoulder_left_deg = degrees;
    return ESP_OK;
}

esp_err_t robot_motion_set_right_forearm(int degrees)
{
    /* 小臂不向外过中位，除非右肩已抬起 ≥45°。 */
    if (degrees < 0 && g_shoulder_right_deg < SHOULDER_DEG_FOR_FOREARM_OUTWARD) {
        ESP_LOGW(TAG, "Right forearm outward denied: shoulder %d° (< %d°)",
                 g_shoulder_right_deg, SHOULDER_DEG_FOR_FOREARM_OUTWARD);
        return ESP_ERR_INVALID_ARG;
    }
    return set_joint(RIGHT_FOREARM_ID, degrees, RIGHT_FOREARM_SIGN, "Right forearm");
}

esp_err_t robot_motion_set_right_upper_arm(int degrees)
{
    /* 大臂不向内过中位，除非右肩已抬起足够角度；且最多向内 10°。 */
    if (degrees < -UPPER_ARM_INWARD_MAX_DEG) {
        ESP_LOGW(TAG, "Right upper arm inward %d° exceeds max %d°",
                 degrees, -UPPER_ARM_INWARD_MAX_DEG);
        return ESP_ERR_INVALID_ARG;
    }
    if (degrees < 0 && g_shoulder_right_deg < SHOULDER_DEG_FOR_UPPER_ARM_INWARD) {
        ESP_LOGW(TAG, "Right upper arm inward denied: shoulder %d° (< %d°)",
                 g_shoulder_right_deg, SHOULDER_DEG_FOR_UPPER_ARM_INWARD);
        return ESP_ERR_INVALID_ARG;
    }
    return set_joint(RIGHT_UPPER_ARM_ID, degrees, RIGHT_UPPER_ARM_SIGN, "Right upper arm");
}

esp_err_t robot_motion_set_left_upper_arm(int degrees)
{
    /* 大臂不向内过中位，除非左肩已抬起足够角度；且最多向内 10°。 */
    if (degrees < -UPPER_ARM_INWARD_MAX_DEG) {
        ESP_LOGW(TAG, "Left upper arm inward %d° exceeds max %d°",
                 degrees, -UPPER_ARM_INWARD_MAX_DEG);
        return ESP_ERR_INVALID_ARG;
    }
    if (degrees < 0 && g_shoulder_left_deg < SHOULDER_DEG_FOR_UPPER_ARM_INWARD) {
        ESP_LOGW(TAG, "Left upper arm inward denied: shoulder %d° (< %d°)",
                 g_shoulder_left_deg, SHOULDER_DEG_FOR_UPPER_ARM_INWARD);
        return ESP_ERR_INVALID_ARG;
    }
    return set_joint(LEFT_UPPER_ARM_ID, degrees, LEFT_UPPER_ARM_SIGN, "Left upper arm");
}

esp_err_t robot_motion_set_left_forearm(int degrees)
{
    /* 小臂不向外过中位，除非左肩已抬起 ≥45°。 */
    if (degrees < 0 && g_shoulder_left_deg < SHOULDER_DEG_FOR_FOREARM_OUTWARD) {
        ESP_LOGW(TAG, "Left forearm outward denied: shoulder %d° (< %d°)",
                 g_shoulder_left_deg, SHOULDER_DEG_FOR_FOREARM_OUTWARD);
        return ESP_ERR_INVALID_ARG;
    }
    return set_joint(LEFT_FOREARM_ID, degrees, LEFT_FOREARM_SIGN, "Left forearm");
}

/*
 * 动作组回位：六个手臂关节分两批回中，保证同时运动的舵机不超过 4 个。
 * 第一批 4 个一起动，到位后再动第二批 2 个。
 */
esp_err_t robot_motion_center_arms(int time_ms)
{
    static const uint8_t batch_a[] = {
        RIGHT_FOREARM_ID, RIGHT_UPPER_ARM_ID, RIGHT_SHOULDER_ID, LEFT_SHOULDER_ID,
    };
    static const uint8_t batch_b[] = {
        LEFT_UPPER_ARM_ID, LEFT_FOREARM_ID,
    };

    for (size_t i = 0; i < sizeof(batch_a) / sizeof(batch_a[0]); ++i) {
        if (!send_position(batch_a[i], CONFIG_TABLE_ROBOT_SERVO_CENTER_PWM, time_ms)) {
            return ESP_FAIL;
        }
    }
    /* 等第一批到位后再发第二批，避免 6 个舵机同时运动。 */
    vTaskDelay(pdMS_TO_TICKS(time_ms + SERVO_COMMAND_GAP_MS));
    for (size_t i = 0; i < sizeof(batch_b) / sizeof(batch_b[0]); ++i) {
        if (!send_position(batch_b[i], CONFIG_TABLE_ROBOT_SERVO_CENTER_PWM, time_ms)) {
            return ESP_FAIL;
        }
    }
    g_shoulder_right_deg = 0;
    g_shoulder_left_deg = 0;
    return ESP_OK;
}

#if CONFIG_TABLE_ROBOT_SERVO_VERIFY_CENTER_AFTER_MOVE
static void verify_center_task(void *arg)
{
    /* 校验 configured_ids 中所有舵机。 */
    int passed = 0;
    int failed = 0;
    vTaskDelay(pdMS_TO_TICKS(CONFIG_TABLE_ROBOT_SERVO_CENTER_TIME_MS + 500));

    ESP_LOGI(TAG, "Verifying %u configured servos", (unsigned)(sizeof(configured_ids) / sizeof(configured_ids[0])));
    for (size_t i = 0; i < sizeof(configured_ids) / sizeof(configured_ids[0]); ++i) {
        int actual = 0;
        const uint8_t id = configured_ids[i];
        if (!read_position(id, &actual)) {
            ++failed;
            ESP_LOGE(TAG, "CENTER FAIL: servo %u did not reply", id);
        } else {
            const int error = abs(actual - CONFIG_TABLE_ROBOT_SERVO_CENTER_PWM);
            if (error <= CONFIG_TABLE_ROBOT_SERVO_VERIFY_TOLERANCE_PWM) {
                ++passed;
                ESP_LOGI(TAG, "CENTER OK: servo %u = %d (error %d PWM)", id, actual, error);
            } else {
                ++failed;
                ESP_LOGE(TAG, "CENTER FAIL: servo %u = %d (error %d PWM)", id, actual, error);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    ESP_LOGI(TAG, "Center verification: %d passed, %d failed", passed, failed);
    if (failed == 0) {
        ESP_LOGI(TAG, "Servo bus verified: ZLink and configured servos replied");
    } else if (passed == 0) {
        ESP_LOGE(TAG, "No servo replied; check ZLink power, GPIO17->RX, GPIO16<-TX, and common GND");
    }
    verify_running = false;
    vTaskDelete(NULL);
}
#endif

static void diagnose_bus(void)
{
    int replied = 0;
    ESP_LOGI(TAG, "Diagnosing %u configured servos...",
             (unsigned)(sizeof(configured_ids) / sizeof(configured_ids[0])));
    for (size_t i = 0; i < sizeof(configured_ids) / sizeof(configured_ids[0]); ++i) {
        const uint8_t id = configured_ids[i];
        int pwm = 0;
        if (read_position(id, &pwm)) {
            ++replied;
            ESP_LOGI(TAG, "DIAG OK: servo %u replied at PWM %d", id, pwm);
        } else {
            ESP_LOGE(TAG, "DIAG FAIL: servo %u did not reply", id);
        }
        vTaskDelay(pdMS_TO_TICKS(SERVO_COMMAND_GAP_MS));
    }

    if (replied == 0) {
        ESP_LOGE(TAG, "Servo bus offline: verify servo power, ZLink power, crossed TX/RX, and common GND");
    } else {
        ESP_LOGI(TAG, "Servo diagnosis complete: %d/%u replied", replied,
                 (unsigned)(sizeof(configured_ids) / sizeof(configured_ids[0])));
    }
}

/*
 * 仅在需要时切换常见波特率并读取已有 ID，不发送位置命令。
 * 找到回复后保留该波特率；全部失败则恢复原设置，便于定位接线问题。
 */
static void probe_bus_baud_rate(void)
{
    const int original_baud = servo_baud_rate;
    const int candidates[] = {115200, 9600};

    ESP_LOGI(TAG, "PROBE: checking configured servo IDs at 115200 and 9600 baud");
    for (size_t rate_index = 0; rate_index < sizeof(candidates) / sizeof(candidates[0]); ++rate_index) {
        const int baud_rate = candidates[rate_index];
        if (!set_servo_baud_rate(baud_rate)) {
            continue;
        }

        for (size_t i = 0; i < sizeof(configured_ids) / sizeof(configured_ids[0]); ++i) {
            int pwm = 0;
            const uint8_t id = configured_ids[i];
            if (read_position(id, &pwm)) {
                ESP_LOGI(TAG, "PROBE OK: servo %u replied at %d baud, PWM %d", id, baud_rate, pwm);
                return;
            }
        }
        ESP_LOGW(TAG, "PROBE: no reply at %d baud", baud_rate);
    }

    (void)set_servo_baud_rate(original_baud);
    ESP_LOGE(TAG, "PROBE FAIL: no reply at either rate; this is a ZLink/servo power/wiring issue");
}

/* GPIO17 与 GPIO16 短接时，直接验证 UART1 的发送和接收，不经过 ZLink。 */
static void run_uart_loopback_test(void)
{
    static const char test_frame[] = "UART1_LOOPBACK_17_TO_16\r\n";
    char received[sizeof(test_frame)] = {0};

    if (!lock_servo_bus()) {
        return;
    }

    uart_flush_input(SERVO_UART);
    const int written = uart_write_bytes(SERVO_UART, test_frame, sizeof(test_frame) - 1);
    const esp_err_t tx_err = written == (int)sizeof(test_frame) - 1
                                 ? uart_wait_tx_done(SERVO_UART, pdMS_TO_TICKS(100))
                                 : ESP_FAIL;
    const int received_length = tx_err == ESP_OK
                                    ? uart_read_bytes(SERVO_UART, received,
                                                      sizeof(received) - 1,
                                                      pdMS_TO_TICKS(500))
                                    : 0;
    unlock_servo_bus();

    if (received_length == (int)sizeof(test_frame) - 1 &&
        memcmp(received, test_frame, sizeof(test_frame) - 1) == 0) {
        ESP_LOGI(TAG, "LOOPBACK OK: GPIO17 TX -> GPIO16 RX, %d bytes matched",
                 received_length);
        return;
    }

    ESP_LOGE(TAG, "LOOPBACK FAIL: wrote %d bytes (%s), received %d; disconnect ZLink and short GPIO17 to GPIO16",
             written, esp_err_to_name(tx_err), received_length);
    if (received_length > 0) {
        ESP_LOGW(TAG, "LOOPBACK RX: %.*s", received_length, received);
    }
}

/* 广播移动到指定 PWM：不依赖舵机 ID、不回读，用于单舵机可见性测试。 */
static void broadcast_move(int pwm, int time_ms)
{
    char command[20];
    const int length = snprintf(command, sizeof(command), "#255P%04dT%04d!", pwm, time_ms);
    if (length < 0 || length >= (int)sizeof(command) ||
        !write_command(command, length)) {
        ESP_LOGE(TAG, "Failed to send broadcast move");
        return;
    }
    ESP_LOGI(TAG, "Broadcast move: PWM %d, %d ms", pwm, time_ms);
}

static void move_relative(uint8_t id, int degrees)
{
    /* 原始串口调试命令：以当前反馈位置为起点移动。 */
    int current = 0;
    if (!read_position(id, &current)) {
        ESP_LOGE(TAG, "Servo %u did not reply", id);
        return;
    }
    const int target = target_from_angle(current, degrees);
    if (send_position(id, target, move_time_ms(degrees))) {
        ESP_LOGI(TAG, "Servo %u: PWM %d -> %d (%+d deg)", id, current, target, degrees);
    }
}

static void test_task(void *arg)
{
    /* 上电检查：逐个偏移、回中，不同时驱动多个关节。 */
    ESP_LOGW(TAG, "Sequential test using %d baud", servo_baud_rate);
    for (size_t i = 0; i < sizeof(configured_ids) / sizeof(configured_ids[0]); ++i) {
        const uint8_t id = configured_ids[i];
        ESP_LOGI(TAG, "TEST: servo %u", id);
        (void)send_position(id, 1800, 1000);
        vTaskDelay(pdMS_TO_TICKS(1300));
        (void)send_position(id, CONFIG_TABLE_ROBOT_SERVO_CENTER_PWM, 1000);
        vTaskDelay(pdMS_TO_TICKS(1300));
    }
    test_running = false;
    ESP_LOGI(TAG, "Sequential test complete");
    vTaskDelete(NULL);
}

static void run_baud_test(void)
{
    const int original_baud = servo_baud_rate;
    const int candidates[] = {115200, 9600};

    ESP_LOGW(TAG, "Baud test uses servo 1 only: first 115200, then 9600");
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
        const int baud_rate = candidates[i];
        if (!set_servo_baud_rate(baud_rate)) {
            continue;
        }

        ESP_LOGW(TAG, "BAUDTEST %d: servo 1 -> PWM 1650", baud_rate);
        (void)send_position(RIGHT_FOREARM_ID, 1650, 1000);
        vTaskDelay(pdMS_TO_TICKS(1300));
        ESP_LOGW(TAG, "BAUDTEST %d: servo 1 -> center", baud_rate);
        (void)send_position(RIGHT_FOREARM_ID, CONFIG_TABLE_ROBOT_SERVO_CENTER_PWM, 1000);
        vTaskDelay(pdMS_TO_TICKS(1300));
    }

    (void)set_servo_baud_rate(original_baud);
    ESP_LOGW(TAG, "Baud test complete; use 'baud 115200' or 'baud 9600' for the rate that moved");
}

static void set_default_id(uint8_t new_id)
{
    char command[16];
    const int length = snprintf(command, sizeof(command), "#000PID%03u!", new_id);
    if (length < 0 || length >= (int)sizeof(command) ||
        !write_command(command, length)) {
        ESP_LOGE(TAG, "Failed to send ID assignment");
        return;
    }
    ESP_LOGW(TAG, "Transmitted %s; connect exactly one default-ID-0 servo", command);
}

/* 串口动作任务：动作函数会等待舵机完成，放到独立任务中避免阻塞串口读取。 */
static void action_task(void *arg)
{
    const int action_id = (int)(intptr_t)arg;
    esp_err_t err = ESP_ERR_INVALID_ARG;

    switch (action_id) {
    case 1:
        err = robot_actions_wave_right();
        break;
    case 2:
        err = robot_actions_wave_left();
        break;
    case 3:
        err = robot_actions_raise_both_hands();
        break;
    case 4:
        err = robot_actions_shake_head();
        break;
    case 5:
        err = robot_actions_clap();
        break;
    case 6:
        err = robot_actions_lift_upper_arms();
        break;
    case 7:
        err = robot_actions_nod();
        break;
    default:
        break;
    }

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Action %d command sequence transmitted", action_id);
    } else {
        ESP_LOGE(TAG, "Action %d failed: %s", action_id, esp_err_to_name(err));
    }
    action_running = false;
    vTaskDelete(NULL);
}

static void start_action(int action_id)
{
    if (action_running || test_running || verify_running) {
        ESP_LOGW(TAG, "Servo action/test/verification is busy; command ignored");
        return;
    }
    if (xTaskCreate(action_task, "servo_action", 4096, (void *)(intptr_t)action_id, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Could not start action %d", action_id);
        return;
    }
    action_running = true;
}

static void debug_console_task(void *arg)
{
    /* 串口只保留最小调试集，正式动作请直接调用 servo/robot_motion.h。 */
    char line[32];
    ESP_LOGI(TAG, "Actions: 1=wave R, 2=wave L, 3=raise hands, 4=shake, 5=clap, 6=lift, 7=nod");
    ESP_LOGI(TAG, "Debug: loopback probe diagnose baudtest baud test center home move <pwm> <id> <deg>");
    while (true) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* 串口工具发送 CRLF 时可能产生一个额外空行，直接忽略。 */
        if (line[0] == '\0' || line[0] == '\r' || line[0] == '\n') {
            continue;
        }
        if (strncmp(line, "center", 6) == 0) {
            robot_motion_center_all();
            continue;
        }
        if (strncmp(line, "move", 4) == 0) {
            int pwm = 0;
            if (sscanf(line + 4, "%d", &pwm) == 1 &&
                pwm >= SERVO_MIN_PWM && pwm <= SERVO_MAX_PWM) {
                broadcast_move(pwm, 1000);
            } else {
                ESP_LOGW(TAG, "Use: move <500-2500>; broadcasts to all servos");
            }
            continue;
        }
        if (strncmp(line, "home", 4) == 0) {
            if (action_running || test_running || verify_running) {
                ESP_LOGW(TAG, "Servo action/test/verification is busy; home ignored");
            } else {
                robot_actions_home_arms();
            }
            continue;
        }
        if (strncmp(line, "test", 4) == 0) {
            if (action_running || verify_running) {
                ESP_LOGW(TAG, "Servo action or verification is busy; test ignored");
            } else if (!test_running &&
                       xTaskCreate(test_task, "servo_test", 4096, NULL, 5, NULL) == pdPASS) {
                test_running = true;
            }
            continue;
        }
        if (strncmp(line, "baudtest", 8) == 0) {
            if (action_running || test_running || verify_running) {
                ESP_LOGW(TAG, "Servo action/test/verification is busy; baud test ignored");
            } else {
                run_baud_test();
            }
            continue;
        }
        if (strncmp(line, "loopback", 8) == 0) {
            if (action_running || test_running || verify_running) {
                ESP_LOGW(TAG, "Servo action/test/verification is busy; loopback ignored");
            } else {
                run_uart_loopback_test();
            }
            continue;
        }
        if (strncmp(line, "probe", 5) == 0) {
            if (action_running || test_running || verify_running) {
                ESP_LOGW(TAG, "Servo action/test/verification is busy; probe ignored");
            } else {
                probe_bus_baud_rate();
            }
            continue;
        }
        if (strncmp(line, "baud", 4) == 0) {
            int baud_rate = 0;
            if (sscanf(line + 4, "%d", &baud_rate) == 1 &&
                (baud_rate == 115200 || baud_rate == 9600)) {
                (void)set_servo_baud_rate(baud_rate);
            } else {
                ESP_LOGW(TAG, "Use: baud 115200 or baud 9600");
            }
            continue;
        }
        if (strncmp(line, "diagnose", 8) == 0) {
            diagnose_bus();
            continue;
        }
        if (strncmp(line, "setid", 5) == 0) {
            int id = 0;
            if (sscanf(line + 5, "%d", &id) == 1 && id > 0 && id < 255) {
                set_default_id((uint8_t)id);
            } else {
                ESP_LOGW(TAG, "Use: setid <1-254>; only one default-ID-0 servo connected");
            }
            continue;
        }

        /* 单独输入 1-7 时触发动作；带角度的 "<id> <deg>" 仍是单舵机调试。 */
        int action_id = 0;
        char extra = '\0';
        if (sscanf(line, "%d %c", &action_id, &extra) == 1 && action_id >= 1 && action_id <= 7) {
            start_action(action_id);
            continue;
        }

        int id = 0;
        int degrees = 0;
        if (sscanf(line, "%d %d", &id, &degrees) == 2 && id >= 0 && id < 255 &&
            degrees >= -270 && degrees <= 270 && degrees != 0) {
            move_relative((uint8_t)id, degrees);
        } else {
            ESP_LOGW(TAG, "Use: 1-7, loopback probe diagnose baudtest baud test center home move <pwm> <id> <deg>");
        }
    }
}

void robot_motion_start_debug_console(void)
{
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM,
                                                           ESP_LINE_ENDINGS_CR));
    ESP_ERROR_CHECK(uart_vfs_dev_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM,
                                                           ESP_LINE_ENDINGS_CRLF));
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    setvbuf(stdin, NULL, _IONBF, 0);
    xTaskCreate(debug_console_task, "servo_debug", 4096, NULL, 5, NULL);
}

void robot_motion_poll(void)
{
    /* BOOT 长按一次只发一次回中命令，松开后才允许下一次触发。 */
    static TickType_t started_at;
    static bool sent;
    const bool pressed = gpio_get_level(CONFIG_TABLE_ROBOT_SERVO_BUTTON_GPIO) == 0;
    if (!pressed) {
        started_at = 0;
        sent = false;
    } else if (started_at == 0) {
        started_at = xTaskGetTickCount();
    } else if (!sent && xTaskGetTickCount() - started_at >=
               pdMS_TO_TICKS(CONFIG_TABLE_ROBOT_SERVO_CENTER_HOLD_MS)) {
        sent = true;
        robot_motion_center_all();
    }
}

#else

void robot_motion_init(void) {}
void robot_motion_start_debug_console(void) {}
void robot_motion_center_all(void) {}
void robot_motion_poll(void) {}
esp_err_t robot_motion_set_head_yaw(int degrees) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t robot_motion_set_head_pitch(int degrees) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t robot_motion_set_torso_pitch(int degrees) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t robot_motion_set_base_yaw(int degrees) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t robot_motion_set_right_shoulder(int degrees) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t robot_motion_set_left_shoulder(int degrees) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t robot_motion_set_shoulders(int degrees) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t robot_motion_set_right_forearm(int degrees) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t robot_motion_set_right_upper_arm(int degrees) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t robot_motion_set_left_upper_arm(int degrees) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t robot_motion_set_left_forearm(int degrees) { return ESP_ERR_NOT_SUPPORTED; }

#endif
