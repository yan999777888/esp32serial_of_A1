#include <Arduino.h>
#include "uart_protocol.h"

// ==========================================================
// 硬件引脚配置
// ==========================================================
#define UART_TXD_PIN       42
#define UART_RXD_PIN       41
#define UART_BAUD_RATE     3000000      // 3Mbps，与A1一致

// ==========================================================
// 图像重组缓冲区
// ==========================================================
#define IMG_MAX_JPEG_SIZE  (30 * 1024)  // 最大JPEG帧 30KB

typedef struct {
    uint8_t  buf[IMG_MAX_JPEG_SIZE];
    uint32_t total_len;
    uint32_t received_len;
    uint16_t width;
    uint16_t height;
    uint16_t next_seq;
    bool     active;
} ImgAssembler_t;

static ImgAssembler_t g_img = {0};

// 图像就绪回调 — 在此处对接 MQTT 或其他上层处理
static void on_jpeg_ready(const uint8_t *jpeg_buf, uint32_t jpeg_len,
                           uint16_t width, uint16_t height)
{
    Serial.printf("[INFO] [IMG] JPEG就绪: %u bytes, %ux%u\n", jpeg_len, width, height);
    // TODO: 将 jpeg_buf/jpeg_len 传递给 MQTT 推送模块
}

// ==========================================================
// 工具函数
// ==========================================================
static const char* alert_level_str(uint8_t level) {
    if (level == CMD_ALERT_HIGH) return "HIGH";
    if (level == CMD_ALERT_MID)  return "MID";
    if (level == CMD_ALERT_LOW)  return "LOW";
    return "NONE";
}

static const char* event_type_str(uint8_t event_type) {
    switch (event_type) {
        case 0: return "fall";
        case 1: return "cat";
        case 2: return "dog";
        case 3: return "smoke";
        case 4: return "fire";
        default: return "unknown";
    }
}

// ==========================================================
// 告警状态机
// ==========================================================
#define ALERT_COOLDOWN_MS     3000
#define DET_PRINT_INTERVAL_MS 1000

typedef struct {
    uint8_t      level;
    uint8_t      event_type;
    unsigned long last_report_ms;
} AlertState_t;

static AlertState_t   g_alert            = {0};
static unsigned long  g_det_last_print_ms = 0;

static void handle_alert(uint8_t level, const UartAlertPayload_t *p) {
    unsigned long now_ms    = millis();
    unsigned long elapsed_ms = now_ms - g_alert.last_report_ms;

    bool is_new   = (g_alert.level != level || g_alert.event_type != p->event_type);
    bool cooldown = (elapsed_ms >= ALERT_COOLDOWN_MS);

    if (!is_new && !cooldown) return;

    if (is_new && g_alert.level != 0) {
        Serial.printf("[WARN] 告警变化: [%s/%s] -> [%s/%s]\n",
            alert_level_str(g_alert.level), event_type_str(g_alert.event_type),
            alert_level_str(level),         event_type_str(p->event_type));
    }

    g_alert.level          = level;
    g_alert.event_type     = p->event_type;
    g_alert.last_report_ms = now_ms;

    Serial.printf("[ERROR] [告警-%s] 事件:%-6s 置信度:%3d%% 坐标:(%4d,%4d)\n",
             alert_level_str(level), event_type_str(p->event_type),
             p->confidence, p->target_x, p->target_y);
}

// ==========================================================
// 校验和计算
// ==========================================================
static uint8_t calc_checksum(uint8_t cmd, uint8_t len, const uint8_t *data) {
    uint8_t sum = cmd + len;
    for (int i = 0; i < len; i++) sum += data[i];
    return sum & 0xFF;
}

// ==========================================================
// 发送帧至 A1
// ==========================================================
void uart_send_frame(uint8_t cmd, const uint8_t *data, uint8_t len) {
    if (len > UART_MAX_DATA_LEN) return;

    uint8_t frame[UART_MAX_DATA_LEN + 5];
    int idx = 0;
    frame[idx++] = UART_FRAME_HEAD1;
    frame[idx++] = UART_FRAME_HEAD2;
    frame[idx++] = cmd;
    frame[idx++] = len;
    if (len > 0 && data != NULL) { memcpy(&frame[idx], data, len); idx += len; }
    frame[idx++] = calc_checksum(cmd, len, data);

    Serial1.write(frame, idx);
}

// ==========================================================
// 图像分包重组
// ==========================================================
static void handle_img_start(uint8_t len, const uint8_t *data) {
    if (len < (uint8_t)sizeof(UartImgStart_t)) {
        Serial.printf("[WARN] [IMG] IMG_START 长度异常: %d\n", len);
        return;
    }
    const UartImgStart_t *hdr = (const UartImgStart_t *)data;
    if (hdr->total_len > IMG_MAX_JPEG_SIZE) {
        Serial.printf("[WARN] [IMG] JPEG too large: %u, 丢弃\n", hdr->total_len);
        return;
    }
    g_img.total_len    = hdr->total_len;
    g_img.width        = hdr->width;
    g_img.height       = hdr->height;
    g_img.received_len = 0;
    g_img.next_seq     = 0;
    g_img.active       = true;
}

static void handle_img_data(uint8_t len, const uint8_t *data) {
    if (!g_img.active || len < 3) return;

    uint16_t seq        = ((uint16_t)data[0] << 8) | data[1];
    uint8_t  chunk_size = len - 2;

    if (seq != g_img.next_seq) {
        Serial.printf("[WARN] [IMG] 序号乱序: 期望%d 收到%d，丢弃本帧\n", g_img.next_seq, seq);
        g_img.active = false;
        return;
    }

    if (g_img.received_len + chunk_size > g_img.total_len) {
        Serial.println("[WARN] [IMG] 数据超出预期长度，丢弃本帧");
        g_img.active = false;
        return;
    }

    memcpy(&g_img.buf[g_img.received_len], data + 2, chunk_size);
    g_img.received_len += chunk_size;
    g_img.next_seq++;
}

static void handle_img_end(uint8_t len, const uint8_t *data) {
    if (!g_img.active) return;

    if (len < 2) {
        Serial.println("[WARN] [IMG] IMG_END 长度异常");
        g_img.active = false;
        return;
    }

    if (g_img.received_len != g_img.total_len) {
        Serial.printf("[WARN] [IMG] 长度不匹配: 期望%u 实收%u，丢弃\n",
                      g_img.total_len, g_img.received_len);
        g_img.active = false;
        return;
    }

    uint16_t expected_cs = ((uint16_t)data[0] << 8) | data[1];
    uint16_t actual_cs   = 0;
    for (uint32_t i = 0; i < g_img.received_len; i++) {
        actual_cs += g_img.buf[i];
    }

    if (expected_cs != actual_cs) {
        Serial.printf("[WARN] [IMG] 校验和错误: 期望0x%04X 实际0x%04X，丢弃\n",
                      expected_cs, actual_cs);
        g_img.active = false;
        return;
    }

    on_jpeg_ready(g_img.buf, g_img.received_len, g_img.width, g_img.height);
    g_img.active = false;
}

// ==========================================================
// 处理一帧有效数据
// ==========================================================
#define MAX_CACHE_RES 5
static UartDetResult_t g_cached_res[MAX_CACHE_RES];
static uint8_t         g_cached_res_cnt = 0;

static void process_valid_frame(uint8_t cmd, uint8_t len, const uint8_t *data) {
    switch (cmd) {
        case CMD_HEARTBEAT:
            break;

        case CMD_DET_COUNT:
            if (len == 1) {
                unsigned long now_ms     = millis();
                bool det_throttle = ((now_ms - g_det_last_print_ms) >= DET_PRINT_INTERVAL_MS);

                if (data[0] == 0) {
                    g_cached_res_cnt = 0;
                    if (g_alert.level != 0) {
                        Serial.printf("[INFO] 目标消失，告警解除 (前一告警: %s/%s)\n",
                                 alert_level_str(g_alert.level),
                                 event_type_str(g_alert.event_type));
                        g_alert.level       = 0;
                        g_det_last_print_ms = now_ms;
                    } else if (det_throttle) {
                        Serial.println("[INFO] 画面中无目标");
                        g_det_last_print_ms = now_ms;
                    }
                } else if (det_throttle) {
                    Serial.printf("[INFO] 画面中共 %d 个目标\n", data[0]);
                    for (int i = 0; i < g_cached_res_cnt; i++) {
                        UartDetResult_t *res = &g_cached_res[i];
                        Serial.printf("[INFO]  目标: %-6s 置信度:%3d%% 中心:(%4d,%4d) 尺寸:%dx%d\n",
                                 event_type_str(res->class_id), res->score,
                                 res->center_x, res->center_y, res->width, res->height);
                    }
                    g_det_last_print_ms = now_ms;
                    g_cached_res_cnt    = 0;
                } else {
                    g_cached_res_cnt = 0;
                }
            }
            break;

        case CMD_DET_RESULT:
            if (len == sizeof(UartDetResult_t) && g_cached_res_cnt < MAX_CACHE_RES) {
                memcpy(&g_cached_res[g_cached_res_cnt], data, sizeof(UartDetResult_t));
                g_cached_res_cnt++;
            }
            break;

        case CMD_ALERT_HIGH:
        case CMD_ALERT_MID:
        case CMD_ALERT_LOW:
            if (len == sizeof(UartAlertPayload_t)) {
                handle_alert(cmd, (UartAlertPayload_t *)data);
            }
            break;

        case CMD_IMG_START:
            handle_img_start(len, data);
            break;
        case CMD_IMG_DATA:
            handle_img_data(len, data);
            break;
        case CMD_IMG_END:
            handle_img_end(len, data);
            break;

        default:
            Serial.printf("[DEBUG] 未处理指令 CMD:0x%02X LEN:%d\n", cmd, len);
            break;
    }
}

// ==========================================================
// 全局状态机变量
// ==========================================================
static int     rx_state    = 0;
static uint8_t rx_cmd      = 0, rx_len = 0;
static uint8_t rx_data_buf[UART_MAX_DATA_LEN];
static int     rx_data_idx = 0;

// ==========================================================
// Arduino 标准初始化
// ==========================================================
void setup() {
    Serial.begin(115200);
    Serial1.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RXD_PIN, UART_TXD_PIN);
    Serial.println("[INFO] 智能家居执行终端 (ESP32-S3 Arduino版) 启动，3Mbps");
}

// ==========================================================
// Arduino 主循环
// ==========================================================
void loop() {
    while (Serial1.available() > 0) {
        uint8_t rx_byte = Serial1.read();

        switch (rx_state) {
            case 0: if (rx_byte == UART_FRAME_HEAD1) rx_state = 1; break;
            case 1:
                if      (rx_byte == UART_FRAME_HEAD2) rx_state = 2;
                else if (rx_byte == UART_FRAME_HEAD1) rx_state = 1;
                else                                  rx_state = 0;
                break;
            case 2: rx_cmd = rx_byte; rx_state = 3; break;
            case 3:
                rx_len = rx_byte; rx_data_idx = 0;
                if      (rx_len > UART_MAX_DATA_LEN) rx_state = 0;
                else if (rx_len > 0)                 rx_state = 4;
                else                                 rx_state = 5;
                break;
            case 4:
                rx_data_buf[rx_data_idx++] = rx_byte;
                if (rx_data_idx == rx_len) rx_state = 5;
                break;
            case 5:
                if (rx_byte == calc_checksum(rx_cmd, rx_len, rx_data_buf)) {
                    process_valid_frame(rx_cmd, rx_len, rx_data_buf);
                } else {
                    Serial.printf("[ERROR] 帧校验失败，丢弃 CMD:0x%02X\n", rx_cmd);
                }
                rx_state = 0;
                break;
        }
    }
}
