/**
 * @file ai_chat.h
 * @brief AI 对话模块——通过 HTTPS 调用 OpenAI 兼容 API
 *
 * 配置项（在 Kconfig.projbuild 中设置）:
 *   - AI_API_KEY    API 密钥
 *   - AI_API_URL    API 地址 (默认: api.openai.com)
 *   - AI_MODEL      模型名 (默认: gpt-3.5-turbo)
 */

#pragma once

#include "esp_err.h"

/** 对话消息结构 */
typedef struct {
    char role[16];     /* "user" 或 "assistant" */
    char content[512]; /* 消息内容 */
} chat_message_t;

/** 对话上下文（维护最近 N 条消息） */
#define MAX_HISTORY 8

typedef struct {
    chat_message_t messages[MAX_HISTORY];
    int count;
} chat_context_t;

/**
 * @brief 初始化聊天上下文
 */
void chat_init(chat_context_t *ctx);

/**
 * @brief 发送消息给 AI 并获取回复
 * @param ctx     对话上下文（自动保存历史）
 * @param user_msg 用户输入
 * @param out_reply 输出缓冲区（至少 512 字节）
 * @return ESP_OK 成功
 */
esp_err_t chat_send(chat_context_t *ctx, const char *user_msg, char *out_reply);
