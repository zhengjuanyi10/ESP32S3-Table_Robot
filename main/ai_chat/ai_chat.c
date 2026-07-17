/**
 * @file ai_chat.c
 * @brief AI 对话实现——通过 esp_http_client 调用 OpenAI API
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"

#include "ai_chat.h"

static const char *TAG = "AI_CHAT";

/* 从 menuconfig 读取配置 */
#ifdef CONFIG_AI_API_KEY
    #define API_KEY CONFIG_AI_API_KEY
#else
    #define API_KEY ""
#endif

#ifdef CONFIG_AI_API_URL
    #define API_URL CONFIG_AI_API_URL
#else
    #define API_URL "api.deepseek.com"
#endif

#ifdef CONFIG_AI_MODEL
    #define AI_MODEL CONFIG_AI_MODEL
#else
    #define AI_MODEL "deepseek-chat"
#endif

/* API 路径 */
#define API_PATH "/v1/chat/completions"

/* ========== 初始化上下文 ========== */

void chat_init(chat_context_t *ctx)
{
    if (ctx) {
        memset(ctx, 0, sizeof(*ctx));
    }
}

/* ========== 构建请求 JSON ========== */

static char *build_request_json(chat_context_t *ctx, const char *user_msg)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *messages = cJSON_CreateArray();

    /* 系统提示词：要求简短回复 */
    cJSON *sys = cJSON_CreateObject();
    cJSON_AddStringToObject(sys, "role", "system");
    cJSON_AddStringToObject(sys, "content", "Please keep responses very brief, within 3 sentences or 80 words maximum. Answer directly without extra explanation.");
    cJSON_AddItemToArray(messages, sys);

    /* 添加历史消息 */
    int start = 0;
    if (ctx->count >= MAX_HISTORY) {
        start = ctx->count - MAX_HISTORY + 1;
    }
    for (int i = start; i < ctx->count; i++) {
        cJSON *m = cJSON_CreateObject();
        cJSON_AddStringToObject(m, "role", ctx->messages[i].role);
        cJSON_AddStringToObject(m, "content", ctx->messages[i].content);
        cJSON_AddItemToArray(messages, m);
    }

    /* 添加当前用户消息 */
    cJSON *user = cJSON_CreateObject();
    cJSON_AddStringToObject(user, "role", "user");
    cJSON_AddStringToObject(user, "content", user_msg);
    cJSON_AddItemToArray(messages, user);

    cJSON_AddItemToObject(root, "messages", messages);
    cJSON_AddStringToObject(root, "model", AI_MODEL);
    cJSON_AddNumberToObject(root, "max_tokens", 150);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

/* ========== HTTP 请求回调 ========== */

typedef struct {
    char *buffer;
    size_t len;
    size_t cap;
} resp_buf_t;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    resp_buf_t *rb = (resp_buf_t *)evt->user_data;
    if (!rb) return ESP_OK;

    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        size_t new_len = rb->len + evt->data_len;
        if (new_len >= rb->cap) {
            rb->cap = new_len + 1024;
            rb->buffer = realloc(rb->buffer, rb->cap);
        }
        memcpy(rb->buffer + rb->len, evt->data, evt->data_len);
        rb->len = new_len;
        rb->buffer[rb->len] = '\0';
    }
    return ESP_OK;
}

/* ========== 解析回复 JSON ========== */

static int parse_reply(const char *json, char *out, size_t out_size)
{
    cJSON *root = cJSON_Parse(json);
    if (!root) {
        ESP_LOGW(TAG, "JSON parse failed");
        return -1;
    }

    cJSON *choices = cJSON_GetObjectItem(root, "choices");
    if (!choices || !cJSON_IsArray(choices) || cJSON_GetArraySize(choices) == 0) {
        cJSON_Delete(root);
        return -1;
    }

    cJSON *first = cJSON_GetArrayItem(choices, 0);
    cJSON *msg = cJSON_GetObjectItem(first, "message");
    cJSON *content = cJSON_GetObjectItem(msg, "content");

    if (content && cJSON_IsString(content)) {
        strncpy(out, content->valuestring, out_size - 1);
        out[out_size - 1] = '\0';
        cJSON_Delete(root);
        return 0;
    }

    cJSON_Delete(root);
    return -1;
}

/* ========== 发送消息 ========== */

esp_err_t chat_send(chat_context_t *ctx, const char *user_msg, char *out_reply)
{
    if (!ctx || !user_msg || !out_reply) return ESP_ERR_INVALID_ARG;
    if (strlen(API_KEY) == 0) {
        ESP_LOGE(TAG, "API key not set! Configure in menuconfig");
        snprintf(out_reply, 512, "[ERROR] API key not configured. Run 'idf.py menuconfig' and set AI_API_KEY.");
        return ESP_FAIL;
    }

    /* 构建请求 JSON */
    char *body = build_request_json(ctx, user_msg);
    if (!body) {
        snprintf(out_reply, 512, "[ERROR] Failed to build request");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sending to AI API...");

    /* 准备 HTTP 客户端 */
    resp_buf_t resp = {0};
    resp.cap = 2048;
    resp.buffer = calloc(1, resp.cap);

    esp_http_client_config_t cfg = {
        .host = API_URL,
        .path = API_PATH,
        .method = HTTP_METHOD_POST,
        .cert_pem = NULL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .event_handler = http_event_handler,
        .user_data = &resp,
        .timeout_ms = 60000,
        .buffer_size = 4096,
        .buffer_size_tx = 2048,
        .keep_alive_enable = false,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);

    /* 设置请求头 */
    char auth_header[128];
    snprintf(auth_header, sizeof(auth_header), "Bearer %s", API_KEY);
    esp_http_client_set_header(client, "Authorization", auth_header);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, body, strlen(body));

    /* 执行请求 */
    esp_err_t err = esp_http_client_perform(client);
    free(body);

    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "HTTP status: %d, response: %d bytes", status, resp.len);

        if (status == 200) {
            if (parse_reply(resp.buffer, out_reply, 511) == 0) {
                /* 保存到历史 */
                int idx = ctx->count % MAX_HISTORY;
                snprintf(ctx->messages[idx].role, 16, "%s", "user");
                strncpy(ctx->messages[idx].content, user_msg, 511);
                ctx->count++;

                idx = ctx->count % MAX_HISTORY;
                snprintf(ctx->messages[idx].role, 16, "%s", "assistant");
                strncpy(ctx->messages[idx].content, out_reply, 511);
                ctx->count++;

                err = ESP_OK;
            } else {
                snprintf(out_reply, 512, "[ERROR] Failed to parse response");
                ESP_LOGE(TAG, "Parse failed. Raw: %s", resp.buffer);
                err = ESP_FAIL;
            }
        } else if (status == 401) {
            snprintf(out_reply, 512, "[ERROR] Invalid API key (401)");
            err = ESP_FAIL;
        } else {
            snprintf(out_reply, 512, "[ERROR] API returned %d", status);
            ESP_LOGE(TAG, "API error %d: %s", status, resp.buffer);
            err = ESP_FAIL;
        }
    } else {
        snprintf(out_reply, 512, "[ERROR] HTTP request failed: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "HTTP error: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    free(resp.buffer);
    return err;
}
