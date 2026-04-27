#include <Arduino.h>
#include "uart_protocol.h"

// ==========================================================
// 硬件引脚配置
// ==========================================================
#define UART_TXD_PIN       42
#define UART_RXD_PIN       41
#define UART_BAUD_RATE     115200

// ==========================================================
// 工具函数：将协议字段转换为可读字符串
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
// 告警状态机 (微秒换成了更轻量的 Arduino 毫秒 millis())
// ==========================================================
#define ALERT_COOLDOWN_MS   1000  
#define DET_PRINT_INTERVAL_MS 1000  

typedef struct {
    uint8_t level;            // 上次告警级别 (0=无告警)
    uint8_t event_type;       // 上次事件类型
    unsigned long last_report_ms; // 上次打印时间戳 (毫秒)
} AlertState_t;

static AlertState_t g_alert = {0};
static unsigned long g_det_last_print_ms = 0;  // 检测打印节流时间戳

static void handle_alert(uint8_t level, const UartAlertPayload_t *p) {
    unsigned long now_ms = millis();
    unsigned long elapsed_ms = now_ms - g_alert.last_report_ms;

    bool is_new   = (g_alert.level != level || g_alert.event_type != p->event_type);
    bool cooldown = (elapsed_ms >= ALERT_COOLDOWN_MS);

    if (!is_new && !cooldown) return;  // 持续中且未到冷却，静默

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

    // 【替换为 Arduino 串口发送】
    Serial1.write(frame, idx);
}

// ==========================================================
// 处理一帧有效数据
// ==========================================================
#define MAX_CACHE_RES 5
static UartDetResult_t g_cached_res[MAX_CACHE_RES];
static uint8_t g_cached_res_cnt = 0;

static void process_valid_frame(uint8_t cmd, uint8_t len, const uint8_t *data) {
    switch (cmd) {
        case CMD_HEARTBEAT:
            // Serial.println("[DEBUG] A1 在线 (心跳)");
            break;

        case CMD_DET_COUNT:
            if (len == 1) {
                unsigned long now_ms = millis();
                bool det_throttle = ((now_ms - g_det_last_print_ms) >= DET_PRINT_INTERVAL_MS);
                
                if (data[0] == 0) {
                    g_cached_res_cnt = 0; // 画面无目标，清空缓存
                    if (g_alert.level != 0) {
                        Serial.printf("[INFO] 目标消失，告警解除 (前一告警: %s/%s)\n",
                                 alert_level_str(g_alert.level),
                                 event_type_str(g_alert.event_type));
                        g_alert.level = 0;
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
                    g_cached_res_cnt = 0; 
                } else {
                    g_cached_res_cnt = 0; 
                }
            }
            break;

        case CMD_DET_RESULT:
            if (len == sizeof(UartDetResult_t)) {
                if (g_cached_res_cnt < MAX_CACHE_RES) {
                    memcpy(&g_cached_res[g_cached_res_cnt], data, sizeof(UartDetResult_t));
                    g_cached_res_cnt++;
                }
            }
            break;

        case CMD_ALERT_HIGH:
        case CMD_ALERT_MID:
        case CMD_ALERT_LOW:
            if (len == sizeof(UartAlertPayload_t)) {
                handle_alert(cmd, (UartAlertPayload_t *)data);
            }
            break;

        default:
            Serial.printf("[DEBUG] 未处理指令 CMD:0x%02X LEN:%d\n", cmd, len);
            break;
    }
}

// ==========================================================
// 全局状态机变量 (从 FreeRTOS Task 移出)
// ==========================================================
static int     rx_state = 0;
static uint8_t rx_cmd   = 0, rx_len = 0;
static uint8_t rx_data_buf[UART_MAX_DATA_LEN];
static int     rx_data_idx = 0;

// ==========================================================
// Arduino 标准初始化
// ==========================================================
void setup() {
    // 1. 初始化终端日志串口 (默认占用 USB/UART0)
    Serial.begin(115200);
    
    // 2. 初始化与 A1 通信的串口 (占用 UART1)
    // 替代了原本复杂的 uart_driver_install 和 uart_param_config
    Serial1.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RXD_PIN, UART_TXD_PIN);

    Serial.println("[INFO] 智能家居执行终端 (ESP32-S3 Arduino版) 启动");
}

// ==========================================================
// Arduino 主循环 (替代原本的 uart_rx_task)
// ==========================================================
void loop() {
    // 只要底层缓冲区有数据，就不停地抛给状态机处理
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
                    Serial.println("[ERROR] 帧校验失败，丢弃");
                }
                rx_state = 0;
                break;
        }
    }

    // 这里可以放其他代码，比如按键检测、OLED 刷新等
    // 因为 Serial1.available() 是非阻塞的，不会卡死在这里
}