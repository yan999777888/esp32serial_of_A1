#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <math.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/*
  ESP32-S3 作为 A1/上位机 与 浏览器前端之间的“HTTP 中枢”

  功能：
  1. UART1 接收 A1 发来的协议帧：[AA][55][CMD][LEN][DATA][CHECKSUM]
  2. 解析检测结果：fire / smoke / person / fallen_person + 坐标
  3. 解析告警结果：fire_alarm / smoke_alarm / fall_detected
  4. 解析 JPEG 分包：IMG_START / IMG_DATA / IMG_END，并重组成最新图片
  5. 给前端提供 HTTP 接口：
     - GET  /api/alerts?limit=1       前端轮询告警
     - GET  /api/telemetry            前端轮询 DS18B20 实时温度
     - GET  /api/status               状态检查
     - POST /api/ptz                  云台指令，控制两个舵机
     - POST /api/mode                 模式切换：monitor/search/track，并转发给 A1
     - POST /api/search               文本查找前通知 ESP32 准备最新图像
     - POST /api/track/refresh        刷新追踪/重置前端状态
     - POST /api/reset                重置系统状态，清空旧告警
     - GET  /stream                   MJPEG 实时图片流
     - GET  /latest.jpg 或 /capture   最新一张 JPG

  说明：
  - 这里没有使用 ESP32-S3 自带摄像头；图片来自 A1/上位机 UART 分包。
  - 前端文档是“浏览器轮询中枢”，所以 ESP32-S3 不需要主动 POST 给前端；
    前端访问 ESP32-S3 的 /api/alerts 和 /stream 即可。
*/

// ========================== 你需要改的配置 ==========================
const char *WIFI_SSID     = "551412";
const char *WIFI_PASSWORD = "li020616";

#define DEVICE_ID           "esp32s3-a1"

// 接线：A1_TX 接 ESP32S3 IO6；A1_RX 接 ESP32S3 IO7；GND 必须共地。
#define UART_RXD_PIN        6
#define UART_TXD_PIN        7
#define UART_BAUD_RATE      3000000
#define UART_RX_BUF_SIZE    (64 * 1024)

// ========================== DS18B20 温度传感器配置 ==========================
// 接线：DS18B20 模块 VCC->3V3，GND->GND，DQ/OUT->IO4。
// 注意：ESP32S3 GPIO 只能接 3.3V 逻辑，模块建议用 3V3 供电，不要用 5V 上拉到数据脚。
#define DS18B20_DATA_PIN          4
#define DS18B20_READ_INTERVAL_MS  1000
#define DS18B20_RESOLUTION_BITS   12
// DS18B20 偶发读失败时，不立刻让 /api/telemetry 返回 null。
// 保留最近一次有效温度，超过这个时间仍然读失败才认为传感器离线。
#define DS18B20_MAX_STALE_MS      15000

// ========================== 系统模式配置 ==========================
// 前端 /api/mode 使用：monitor / search / track。
// 默认 monitor。monitor：图片+三种危险状态+语音+云台。
// search：提供最新图片给本机 YOLOE，语音、手动云台、告警和自动追踪关闭。
// track：图片+坐标+自动云台，语音、手动云台和告警关闭。
// A1 文档中 CMD_SWITCH_MODEL 的 model_id：0=模型A，1=模型B，2=模型C。
// 现在你的需求是：monitor=模型A；track 使用“原查找模式”的模型，所以默认设为模型B。
// 如果 A1 同学说追踪模式不是模型B，只改 A1_MODEL_TRACK 这一行。
#define A1_MODEL_MONITOR      0
#define A1_MODEL_SEARCH       1
#define A1_MODEL_TRACK        1


// ========================== 语音模块串口配置 ==========================
// A1 已经占用 Serial1：IO6=ESP32S3_RX，IO7=ESP32S3_TX。
// USB 串口监视器占用 Serial。
// 语音模块使用另一个硬件串口 Serial2，避免互相影响。
// 接线：ESP32S3 IO18(TX) -> 语音模块 RX；ESP32S3 IO15(RX) <- 语音模块 TX，可不接；GND 必须共地。
#define VOICE_TXD_PIN          18
#define VOICE_RXD_PIN          15
#define VOICE_UART_BAUD_RATE   115200    // 大多数语音模块默认 9600；如果你的模块说明书是 115200，就只改这里

// 一句话大概 2 秒。这里给 2.2 秒，保证上一句播完后再发下一条指令。
#define VOICE_PLAY_GAP_MS      2200

// A1 持续发送同一种危险状态时，超过这个时间仍收到该状态，就认为还在持续告警。
// 如果上位机发送频率很低，可以把 3000 改成 5000。
#define VOICE_ALARM_HOLD_MS    3000


// ========================== 云台舵机配置 ==========================
// 左右 360°舵机：IO16；上下 180°舵机：IO17。不要占用 IO6/IO7，它们给 A1 串口用。
#define PTZ_PAN_SERVO_PIN       16      // 左右：360°舵机信号线
#define PTZ_TILT_SERVO_PIN      17      // 上下：180°舵机信号线

#define PTZ_PWM_FREQ_HZ         50      // 舵机标准 PWM：50Hz，周期 20ms
#define PTZ_PWM_RES_BITS        14
#define PTZ_PWM_PERIOD_US       20000
#define PTZ_SERVO_MIN_US        500     // 0.5ms
#define PTZ_SERVO_MAX_US        2500    // 2.5ms

// 统一步进角度：后面想每次动 10°，只改这里即可。
#define PTZ_STEP_ANGLE          5

// 360°舵机参数
// 如果你的 360°舵机是“连续旋转舵机”，不能真正按角度定位，只能转一小段时间后停止，近似 5°。
#define PTZ_PAN_MIN_ANGLE       0
#define PTZ_PAN_MAX_ANGLE       360
#define PTZ_PAN_HOME_ANGLE      180
#define PTZ_PAN_STEP_ANGLE      PTZ_STEP_ANGLE
#define PTZ_PAN_CONTINUOUS_MODE 1       // 1=连续旋转360°舵机；0=角度型360°舵机
#define PTZ_PAN_STOP_US         1500    // 停止脉宽；停不住就微调 1480~1520
#define PTZ_PAN_LEFT_US         1700    // 左转速度：已反向，前端按“左”时实际向左
#define PTZ_PAN_RIGHT_US        1300    // 右转速度：已反向，前端按“右”时实际向右
#define PTZ_PAN_MS_PER_5_DEG    110      // 近似 5°所需时间；转多改小，转少改大

// 180°舵机参数
#define PTZ_TILT_CAL_MIN_ANGLE  9       // PWM 标定范围，不能随机械安全限位一起修改
#define PTZ_TILT_CAL_MAX_ANGLE  171
#define PTZ_TILT_MIN_ANGLE      35      // 摄像头与转盘保持安全间隙，禁止继续向下压
#define PTZ_TILT_MAX_ANGLE      165
#define PTZ_TILT_HOME_ANGLE     160     // 镜像朝向下，向下微调 5°
#define PTZ_TILT_STEP_ANGLE     PTZ_STEP_ANGLE

// ========================== 追踪模式自动云台闭环参数 ==========================
// 追踪模式下，A1 发来的检测框中心点会自动驱动云台，让目标靠近画面中心。
// A1 发送 1920x1080 原图像素坐标，ESP32 使用标定内参转换为归一化相机坐标。
#define PTZ_TRACK_ENABLE          1
#define CAMERA_IMAGE_WIDTH        1920
#define CAMERA_IMAGE_HEIGHT       1080
#define CAMERA_FX                 905.3101f
#define CAMERA_FY                 900.8898f
#define CAMERA_CX                 628.4194f
#define CAMERA_CY                 499.6724f
// 追踪目标是 1920x1080 画面的几何中心，不是标定光心 (cx,cy)。
#define PTZ_TRACK_TARGET_PIXEL_X  (CAMERA_IMAGE_WIDTH * 0.5f)
#define PTZ_TRACK_TARGET_PIXEL_Y  (CAMERA_IMAGE_HEIGHT * 0.5f)
#define PTZ_TRACK_TARGET_CAM_X    ((PTZ_TRACK_TARGET_PIXEL_X - CAMERA_CX) / CAMERA_FX)
#define PTZ_TRACK_TARGET_CAM_Y    ((PTZ_TRACK_TARGET_PIXEL_Y - CAMERA_CY) / CAMERA_FY)
#define PTZ_TRACK_DEADBAND_X_PIXELS 80.0f  // 水平轴允许范围 ±80px，提高左右响应
#define PTZ_TRACK_DEADBAND_Y_PIXELS 120.0f // 画面中心垂直方向允许范围 ±120px
#define PTZ_TRACK_ERROR_LIMIT_X   (PTZ_TRACK_DEADBAND_X_PIXELS / CAMERA_FX)
#define PTZ_TRACK_ERROR_LIMIT_Y   (PTZ_TRACK_DEADBAND_Y_PIXELS / CAMERA_FY)
#define PTZ_TRACK_PAN_START_PIXELS 120.0f // 停止后超出 ±120px 才重新启动
#define PTZ_TRACK_TILT_START_PIXELS 160.0f // 俯仰停止后超出 ±160px 才重新启动
#define PTZ_TRACK_PAN_START_LIMIT (PTZ_TRACK_PAN_START_PIXELS / CAMERA_FX)
#define PTZ_TRACK_TILT_START_LIMIT (PTZ_TRACK_TILT_START_PIXELS / CAMERA_FY)
#define PTZ_TRACK_SAMPLE_MS       50      // 视觉伺服控制周期约 20Hz
#define PTZ_TRACK_INPUT_NOISE_LIMIT 0.015f // 约 14 像素以内的测量变化直接忽略
#define PTZ_TRACK_MEDIAN_SIZE     3       // 3 点中值滤波，兼顾抗跳动和低延迟
#define PTZ_TRACK_FILTER_ALPHA    0.45f   // 快速视觉伺服使用较高的新坐标权重
#define PTZ_TRACK_UNSTABLE_ALPHA  0.15f   // 检测框尺寸突变时仍保持有限响应
#define PTZ_TRACK_BOX_CHANGE_LIMIT 0.25f  // 框面积变化超过 25% 视为不稳定测量
#define PTZ_TRACK_MAX_COORD_JUMP  0.12f   // 限制单帧坐标突变，约 108 像素
#define PTZ_TRACK_PAN_MIN_OFFSET_US 120   // 带摄像头负载时克服连续舵机死区
#define PTZ_TRACK_PAN_MAX_OFFSET_US 180   // 连续舵机最大跟踪速度偏移
#define PTZ_TRACK_TARGET_TIMEOUT_MS 600   // 坐标超时后强制停止水平舵机
#define PTZ_TRACK_PAN_GAIN        0.15f   // 降低连续舵机增益，避免刚启动就高速冲过中心
#define PTZ_TRACK_TILT_GAIN       0.70f   // 俯仰轴比例增益
#define PTZ_TRACK_MIN_STEP_ANGLE  1       // 俯仰轴采用连续小步位置更新
#define PTZ_TRACK_MAX_STEP_ANGLE  3       // 小步调整，同时避免连续舵机脉冲过短
#define PTZ_TRACK_TILT_STEP_MAX   1       // 俯仰轴每次最多更新 1°，防止命令累加超调
#define PTZ_TRACK_TILT_COMMAND_MS 120     // 给位置舵机留出机械响应时间
#define PTZ_TRACK_SETTLE_MS       0       // 快速视觉伺服不执行动作后阻塞等待
#define PTZ_TRACK_CLASS_ID        0       // 只追踪 fire；多目标时优先锁定靠近上一位置的火焰
#define PTZ_TRACK_PAN_REVERSE     false   // 左右方向反了就改成 true
#define PTZ_TRACK_TILT_REVERSE    false   // 上下方向反了就改成 true

#define PTZ_LEDC_TIMER          LEDC_TIMER_0
#define PTZ_LEDC_MODE           LEDC_LOW_SPEED_MODE
#define PTZ_PAN_CHANNEL         LEDC_CHANNEL_0
#define PTZ_TILT_CHANNEL        LEDC_CHANNEL_1

// 告警冷却：同一种告警 3 秒内只加入一次缓存，避免前端重复刷屏。
#define ALERT_COOLDOWN_MS   3000
#define ALERT_BUFFER_SIZE   20
// 点击前端“重置系统状态”后，短暂屏蔽串口中可能还在路上的旧告警，避免刚清空又立刻刷回红警。
#define ALERT_RESET_SUPPRESS_MS 1500
// 手动重置后，不再让同一个持续存在的危险状态立刻把前端刷回红警。
// 逻辑：重置时确认当前危险；只要 A1 还持续发这个危险，就暂时不上报；
// 等危险消失超过这个时间或 DET_COUNT=0 后，才重新允许同类危险触发。
#define ALERT_ACK_CLEAR_MS      3500
// 前端告警显示最长保留时间：超过这个时间没有新的 ALERT 包，就自动回到安全状态
#define ALERT_FRONTEND_TTL_MS   5000
// 点击“重置系统状态”后，强制保持安全状态的时间，防止串口残留旧告警立即刷回来
#define ALERT_RESET_SAFE_HOLD_MS 8000
#define ALERT_BIT_FIRE          0x01
#define ALERT_BIT_SMOKE         0x02
#define ALERT_BIT_FALL          0x04
#define ALERT_BIT_ALL           (ALERT_BIT_FIRE | ALERT_BIT_SMOKE | ALERT_BIT_FALL)

// JPEG 最大缓存。协议里通常 5~10KB，这里放大，避免偶发大图导致溢出。
#define MAX_JPEG_SIZE       (260 * 1024)

// ========================== UART 协议常量 ==========================
#define UART_FRAME_HEAD1    0xAA
#define UART_FRAME_HEAD2    0x55
#define UART_MAX_DATA_LEN   24

#define CMD_HEARTBEAT       0x01
#define CMD_DET_RESULT      0x10
#define CMD_DET_COUNT       0x11
#define CMD_ALERT_LOW       0x20
#define CMD_ALERT_MID       0x21
#define CMD_ALERT_HIGH      0x22
#define CMD_IMG_START       0x30
#define CMD_IMG_DATA        0x31
#define CMD_IMG_END         0x32
#define CMD_ACK             0x80
#define CMD_SWITCH_MODEL    0x83
#define CMD_SWITCH_ACK      0x84

#define CLASS_FIRE          0
#define CLASS_SMOKE         1
#define CLASS_PERSON        2
#define CLASS_FALLEN        3

// ========================== 数据结构 ==========================
typedef struct {
  uint8_t *buf;
  uint32_t total_len;
  uint32_t received_len;
  uint16_t width;
  uint16_t height;
  uint16_t next_seq;
  bool active;
} ImgAssembler_t;

typedef struct {
  char event_type[24];         // 前端需要：fire_alarm / smoke_alarm / fall_detected
  char level[12];              // 前端需要：critical / warning / info
  char message[64];            // 前端需要：摘要说明
  char received_at[40];        // 前端需要：字符串，优先 ISO8601 UTC
  uint8_t class_id;
  uint8_t confidence_percent;  // 串口协议为 0~100；HTTP 返回时转为 0~1
  uint16_t target_x;
  uint16_t target_y;
  uint16_t box_w;
  uint16_t box_h;
  uint32_t local_ms;
} AlertEvent_t;

typedef struct {
  uint8_t class_id;
  uint8_t score;
  float   cam_x;        // 归一化相机坐标 X
  float   cam_y;        // 归一化相机坐标 Y
  uint16_t box_width;
  uint16_t box_height;
  uint32_t local_ms;
} Detection_t;

typedef enum {
  SYS_MODE_MONITOR = 0,
  SYS_MODE_SEARCH  = 1,
  SYS_MODE_TRACK   = 2
} SystemMode_t;

static ImgAssembler_t g_img = {0};
static uint8_t *g_latest_jpeg = NULL;
static uint8_t *g_http_copy = NULL;
static uint32_t g_latest_len = 0;
static uint16_t g_latest_width = 0;
static uint16_t g_latest_height = 0;
static uint32_t g_latest_seq = 0;
static SemaphoreHandle_t g_jpeg_mutex = NULL;

// 图像链路调试计数：用于判断到底有没有收到 IMG_START / IMG_DATA / IMG_END。
static volatile uint32_t g_uart_ok_frame_count = 0;
static volatile uint32_t g_uart_checksum_error_count = 0;
static volatile uint32_t g_img_start_count = 0;
static volatile uint32_t g_img_data_count = 0;
static volatile uint32_t g_img_end_count = 0;
static volatile uint32_t g_img_ready_count = 0;
static volatile uint32_t g_img_drop_count = 0;
static char g_img_last_error[96] = "no image yet";

static AlertEvent_t g_alerts[ALERT_BUFFER_SIZE];
static uint8_t g_alert_head = 0;    // 下一次写入位置
static uint8_t g_alert_count = 0;   // 当前缓存数量
static SemaphoreHandle_t g_alert_mutex = NULL;
static uint32_t g_alert_reset_until_ms = 0;
static uint8_t g_ack_alarm_mask = 0;       // 重置后确认过的持续危险类型，未消失前不上报
static uint32_t g_danger_fire_seen_ms = 0;
static uint32_t g_danger_smoke_seen_ms = 0;
static uint32_t g_danger_fall_seen_ms = 0;

static Detection_t g_latest_detection = {0};
static bool g_has_detection = false;
static SemaphoreHandle_t g_detection_mutex = NULL;

static float g_temperature_celsius = NAN;
static bool g_temperature_valid = false;
static uint32_t g_temperature_last_ok_ms = 0;
static SemaphoreHandle_t g_sensor_mutex = NULL;

static OneWire g_onewire(DS18B20_DATA_PIN);
static DallasTemperature g_ds18b20(&g_onewire);
static bool g_ds18b20_found = false;

static uint32_t g_last_fire_ms = 0;
static uint32_t g_last_smoke_ms = 0;
static uint32_t g_last_fall_ms = 0;

static SemaphoreHandle_t g_voice_mutex = NULL;
static uint32_t g_voice_fire_seen_ms = 0;
static uint32_t g_voice_smoke_seen_ms = 0;
static uint32_t g_voice_fall_seen_ms = 0;
static uint8_t g_voice_last_played_class = 255;

static httpd_handle_t g_api_server = NULL;
static httpd_handle_t g_stream_server = NULL;

static int g_pan_angle = PTZ_PAN_HOME_ANGLE;
static int g_tilt_angle = PTZ_TILT_HOME_ANGLE;
static SemaphoreHandle_t g_ptz_mutex = NULL;

// 追踪模式自动云台状态：配合 A1 坐标反馈，按最高置信度目标闭环调整。
static bool g_track_enabled = true;
static uint32_t g_track_last_move_ms = 0;
static uint32_t g_track_last_sample_ms = 0;
static bool g_track_waiting_feedback = false;
static bool g_track_has_det_count = false;
static uint8_t g_track_expected_count = 0;
static uint8_t g_track_received_count = 0;
static bool g_track_best_valid = false;
static Detection_t g_track_best_det = {0};
static bool g_track_filter_valid = false;
static float g_track_filtered_x = 0.0f;
static float g_track_filtered_y = 0.0f;
static uint32_t g_track_last_box_area = 0;
static float g_track_history_x[PTZ_TRACK_MEDIAN_SIZE] = {0};
static float g_track_history_y[PTZ_TRACK_MEDIAN_SIZE] = {0};
static uint8_t g_track_history_count = 0;
static uint8_t g_track_history_index = 0;
static int8_t g_track_pending_axis = 0;
static int8_t g_track_pending_direction = 0;
static uint8_t g_track_pending_frames = 0;
static volatile bool g_track_pan_active = false;
static volatile uint32_t g_track_last_target_ms = 0;
static uint32_t g_track_last_tilt_command_ms = 0;
static bool g_track_tilt_active = false;

static SystemMode_t g_sys_mode = SYS_MODE_MONITOR;
static uint8_t g_a1_model_id = A1_MODEL_MONITOR;
static SemaphoreHandle_t g_mode_mutex = NULL;
static SemaphoreHandle_t g_uart_tx_mutex = NULL;

// ========================== 小工具函数 ==========================
static uint16_t u16le(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t u32le(const uint8_t *p) {
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static uint16_t u16be(const uint8_t *p) {
  return ((uint16_t)p[0] << 8) | (uint16_t)p[1];
}

static uint8_t calcChecksum(uint8_t cmd, uint8_t len, const uint8_t *data) {
  uint8_t sum = cmd + len;
  for (uint8_t i = 0; i < len; i++) {
    sum += data ? data[i] : 0;
  }
  return sum;
}

static const char *className(uint8_t class_id) {
  switch (class_id) {
    case CLASS_FIRE:   return "fire";
    case CLASS_SMOKE:  return "smoke";
    case CLASS_PERSON: return "person";
    case CLASS_FALLEN: return "fallen_person";
    default:           return "unknown";
  }
}

static const char *frontendEventType(uint8_t class_id) {
  switch (class_id) {
    case CLASS_FIRE:   return "fire_alarm";
    case CLASS_SMOKE:  return "smoke_alarm";
    case CLASS_FALLEN: return "fall_detected";
    default:           return NULL;
  }
}

static const char *frontendLevel(uint8_t class_id, uint8_t confidence_percent) {
  // 前端红警主要看 event_type；level 按文档给 critical / warning。
  if (class_id == CLASS_FIRE || class_id == CLASS_SMOKE) return "critical";
  if (class_id == CLASS_FALLEN) {
    return confidence_percent >= 90 ? "critical" : "warning";
  }
  return "info";
}

static const char *frontendMessage(uint8_t class_id, uint8_t confidence_percent) {
  if (class_id == CLASS_FIRE || class_id == CLASS_SMOKE) return "Fire or smoke alarm";
  if (class_id == CLASS_FALLEN) {
    return confidence_percent >= 90 ? "Fall detected with very high confidence" : "Fall detected";
  }
  return "No alarm";
}

static uint8_t alertCmdForClass(uint8_t class_id) {
  switch (class_id) {
    case CLASS_SMOKE:  return CMD_ALERT_MID;
    case CLASS_FIRE:
    case CLASS_FALLEN: return CMD_ALERT_HIGH;
    default:           return 0;
  }
}

static void makeTimestamp(char *out, size_t out_size) {
  if (!out || out_size == 0) return;

  time_t now = 0;
  time(&now);
  if (now > 1700000000) {
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);
    strftime(out, out_size, "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  } else {
    snprintf(out, out_size, "millis:%lu", (unsigned long)millis());
  }
}

static void uartSendFrame(uint8_t cmd, const uint8_t *data, uint8_t len) {
  if (len > UART_MAX_DATA_LEN) return;

  uint8_t frame[UART_MAX_DATA_LEN + 5];
  int idx = 0;
  frame[idx++] = UART_FRAME_HEAD1;
  frame[idx++] = UART_FRAME_HEAD2;
  frame[idx++] = cmd;
  frame[idx++] = len;

  if (len > 0 && data) {
    memcpy(&frame[idx], data, len);
    idx += len;
  }

  frame[idx++] = calcChecksum(cmd, len, data);

  if (g_uart_tx_mutex) xSemaphoreTake(g_uart_tx_mutex, pdMS_TO_TICKS(50));
  Serial1.write(frame, idx);
  Serial1.flush();
  if (g_uart_tx_mutex) xSemaphoreGive(g_uart_tx_mutex);
}

// ========================== 模式切换 ==========================
static const char *modeName(SystemMode_t mode) {
  switch (mode) {
    case SYS_MODE_MONITOR: return "monitor";
    case SYS_MODE_SEARCH:  return "search";
    case SYS_MODE_TRACK:   return "track";
    default:               return "unknown";
  }
}

static bool modeFromString(const String &s, SystemMode_t *out_mode) {
  if (!out_mode) return false;
  String m = s;
  m.trim();
  m.toLowerCase();

  if (m == "monitor" || m == "监控") {
    *out_mode = SYS_MODE_MONITOR;
    return true;
  }

  if (m == "search" || m == "查找" || m == "查找模式") {
    *out_mode = SYS_MODE_SEARCH;
    return true;
  }

  if (m == "track" || m == "tracking" || m == "追踪" || m == "追踪模式") {
    *out_mode = SYS_MODE_TRACK;
    return true;
  }
  return false;
}

static uint8_t modelIdForMode(SystemMode_t mode) {
  if (mode == SYS_MODE_MONITOR) return A1_MODEL_MONITOR;
  if (mode == SYS_MODE_SEARCH) return A1_MODEL_SEARCH;
  if (mode == SYS_MODE_TRACK) return A1_MODEL_TRACK;
  return A1_MODEL_MONITOR;
}

static SystemMode_t getCurrentMode() {
  SystemMode_t mode = SYS_MODE_MONITOR;
  if (g_mode_mutex && xSemaphoreTake(g_mode_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    mode = g_sys_mode;
    xSemaphoreGive(g_mode_mutex);
  } else {
    mode = g_sys_mode;
  }
  return mode;
}

static bool isMonitorMode() {
  return getCurrentMode() == SYS_MODE_MONITOR;
}

static bool modeVoiceEnabled() {
  return isMonitorMode();
}

static bool modePtzEnabled() {
  return isMonitorMode();
}


static uint8_t alarmBitForClass(uint8_t class_id) {
  switch (class_id) {
    case CLASS_FIRE:   return ALERT_BIT_FIRE;
    case CLASS_SMOKE:  return ALERT_BIT_SMOKE;
    case CLASS_FALLEN: return ALERT_BIT_FALL;
    default:           return 0;
  }
}

static const char *alarmBitName(uint8_t bit) {
  if (bit == ALERT_BIT_FIRE) return "fire";
  if (bit == ALERT_BIT_SMOKE) return "smoke";
  if (bit == ALERT_BIT_FALL) return "fall";
  return "unknown";
}

static void markDangerSeen(uint8_t class_id) {
  uint8_t bit = alarmBitForClass(class_id);
  if (bit == 0) return;

  uint32_t now = millis();
  if (g_alert_mutex && xSemaphoreTake(g_alert_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (class_id == CLASS_FIRE) g_danger_fire_seen_ms = now;
    else if (class_id == CLASS_SMOKE) g_danger_smoke_seen_ms = now;
    else if (class_id == CLASS_FALLEN) g_danger_fall_seen_ms = now;
    xSemaphoreGive(g_alert_mutex);
  } else {
    if (class_id == CLASS_FIRE) g_danger_fire_seen_ms = now;
    else if (class_id == CLASS_SMOKE) g_danger_smoke_seen_ms = now;
    else if (class_id == CLASS_FALLEN) g_danger_fall_seen_ms = now;
  }
}

static bool seenRecently(uint32_t last_ms, uint32_t now, uint32_t window_ms) {
  return last_ms != 0 && (now - last_ms) <= window_ms;
}

static uint8_t buildRecentDangerMaskLocked(uint32_t now) {
  uint8_t mask = 0;
  if (seenRecently(g_danger_fire_seen_ms, now, ALERT_ACK_CLEAR_MS)) mask |= ALERT_BIT_FIRE;
  if (seenRecently(g_danger_smoke_seen_ms, now, ALERT_ACK_CLEAR_MS)) mask |= ALERT_BIT_SMOKE;
  if (seenRecently(g_danger_fall_seen_ms, now, ALERT_ACK_CLEAR_MS)) mask |= ALERT_BIT_FALL;

  // 再看缓存里最近一条/几条告警，防止点击重置时 danger_seen 还没来得及更新。
  for (int i = 0; i < g_alert_count && i < ALERT_BUFFER_SIZE; i++) {
    int idx = (int)g_alert_head - 1 - i;
    while (idx < 0) idx += ALERT_BUFFER_SIZE;
    idx %= ALERT_BUFFER_SIZE;
    if (seenRecently(g_alerts[idx].local_ms, now, ALERT_ACK_CLEAR_MS * 2)) {
      mask |= alarmBitForClass(g_alerts[idx].class_id);
    }
  }
  return mask;
}

static void refreshAckMaskLocked(uint32_t now) {
  // 被确认的危险类型，只有在 A1 不再持续上报它超过 ALERT_ACK_CLEAR_MS 后才自动解除。
  if ((g_ack_alarm_mask & ALERT_BIT_FIRE) && !seenRecently(g_danger_fire_seen_ms, now, ALERT_ACK_CLEAR_MS)) {
    g_ack_alarm_mask &= ~ALERT_BIT_FIRE;
    Serial.println("[ACK] fire cleared, re-armed");
  }
  if ((g_ack_alarm_mask & ALERT_BIT_SMOKE) && !seenRecently(g_danger_smoke_seen_ms, now, ALERT_ACK_CLEAR_MS)) {
    g_ack_alarm_mask &= ~ALERT_BIT_SMOKE;
    Serial.println("[ACK] smoke cleared, re-armed");
  }
  if ((g_ack_alarm_mask & ALERT_BIT_FALL) && !seenRecently(g_danger_fall_seen_ms, now, ALERT_ACK_CLEAR_MS)) {
    g_ack_alarm_mask &= ~ALERT_BIT_FALL;
    Serial.println("[ACK] fall cleared, re-armed");
  }
}

static bool isAlarmAcknowledged(uint8_t class_id) {
  uint8_t bit = alarmBitForClass(class_id);
  if (bit == 0) return false;

  bool acked = false;
  uint32_t now = millis();
  if (g_alert_mutex && xSemaphoreTake(g_alert_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    refreshAckMaskLocked(now);
    acked = (g_ack_alarm_mask & bit) != 0;
    xSemaphoreGive(g_alert_mutex);
  } else {
    acked = (g_ack_alarm_mask & bit) != 0;
  }
  return acked;
}

static void acknowledgeCurrentAlarms(const char *reason) {
  uint8_t mask = 0;
  uint32_t now = millis();

  if (g_alert_mutex && xSemaphoreTake(g_alert_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    mask = buildRecentDangerMaskLocked(now);
    // 如果没判断出具体类型，但用户点了重置，就至少短暂确认所有危险，避免刚点完立刻被串口残留帧刷回去。
    if (mask == 0) mask = ALERT_BIT_ALL;
    g_ack_alarm_mask |= mask;
    xSemaphoreGive(g_alert_mutex);
  } else {
    mask = ALERT_BIT_ALL;
    g_ack_alarm_mask |= mask;
  }

  Serial.printf("[ACK] current alarms acknowledged mask=0x%02X reason=%s\n", mask, reason ? reason : "reset");
}

static void clearAckAndDangerState(const char *reason) {
  if (g_alert_mutex && xSemaphoreTake(g_alert_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    g_ack_alarm_mask = 0;
    g_danger_fire_seen_ms = 0;
    g_danger_smoke_seen_ms = 0;
    g_danger_fall_seen_ms = 0;
    xSemaphoreGive(g_alert_mutex);
  } else {
    g_ack_alarm_mask = 0;
    g_danger_fire_seen_ms = 0;
    g_danger_smoke_seen_ms = 0;
    g_danger_fall_seen_ms = 0;
  }
  Serial.printf("[ACK] all alarms re-armed, reason=%s\n", reason ? reason : "clear");
}

static void clearAlertBuffer() {
  if (g_alert_mutex && xSemaphoreTake(g_alert_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    memset(g_alerts, 0, sizeof(g_alerts));
    g_alert_head = 0;
    g_alert_count = 0;
    xSemaphoreGive(g_alert_mutex);
  }
  g_last_fire_ms = 0;
  g_last_smoke_ms = 0;
  g_last_fall_ms = 0;
}

static void clearDetectionState() {
  if (g_detection_mutex && xSemaphoreTake(g_detection_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    memset(&g_latest_detection, 0, sizeof(g_latest_detection));
    g_has_detection = false;
    xSemaphoreGive(g_detection_mutex);
  }
}

static void clearVoiceState();
static void clearA1UartRxBuffer();

static void resetFrontendSystemState(const char *reason) {
  // 先确认当前危险，再清空缓存。
  // 同时强制保持一段时间安全状态，避免 A1 串口缓冲区里的旧 fall/fire/smoke 帧把前端马上刷回危险状态。
  acknowledgeCurrentAlarms(reason);
  clearAlertBuffer();
  clearVoiceState();
  clearDetectionState();

  // 重置时直接确认全部告警类型。后续 DET_COUNT=0 或危险消失后会重新布防。
  if (g_alert_mutex && xSemaphoreTake(g_alert_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    g_ack_alarm_mask = ALERT_BIT_ALL;
    xSemaphoreGive(g_alert_mutex);
  } else {
    g_ack_alarm_mask = ALERT_BIT_ALL;
  }

  clearA1UartRxBuffer();

  uint32_t now = millis();
  g_alert_reset_until_ms = now + ALERT_RESET_SAFE_HOLD_MS;
  Serial.printf("[RESET] system state cleared, safe_hold_ms=%u, alert_ttl_ms=%u, reason=%s\n",
                (unsigned)ALERT_RESET_SAFE_HOLD_MS,
                (unsigned)ALERT_FRONTEND_TTL_MS,
                reason ? reason : "manual");
}

static void clearVoiceState() {
  if (g_voice_mutex && xSemaphoreTake(g_voice_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    g_voice_fire_seen_ms = 0;
    g_voice_smoke_seen_ms = 0;
    g_voice_fall_seen_ms = 0;
    g_voice_last_played_class = 255;
    xSemaphoreGive(g_voice_mutex);
  }
}

static void trackResetDetCache();
static void trackResetControllerState();

static void setWorkMode(SystemMode_t new_mode, bool notify_a1) {
  // A1 当前固件不支持模型切换；保留参数兼容现有调用，但不再发送 0x83 命令。
  (void)notify_a1;
  uint8_t model_id = modelIdForMode(new_mode);
  bool changed = true;

  if (g_mode_mutex && xSemaphoreTake(g_mode_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    changed = (g_sys_mode != new_mode) || (g_a1_model_id != model_id);
    g_sys_mode = new_mode;
    g_a1_model_id = model_id;
    xSemaphoreGive(g_mode_mutex);
  } else {
    changed = (g_sys_mode != new_mode) || (g_a1_model_id != model_id);
    g_sys_mode = new_mode;
    g_a1_model_id = model_id;
  }

  // 只在 track 模式下启用自动追踪；其余模式关闭
  g_track_enabled = (new_mode == SYS_MODE_TRACK);

  // 切换模式时清空追踪帧缓存，避免上一帧目标坐标继续影响云台。
  trackResetControllerState();

  // 离开监控模式时，立即清掉语音待播状态、旧告警和旧坐标，避免页面还显示上一个状态。
  if (new_mode != SYS_MODE_MONITOR) {
    resetFrontendSystemState("mode_change");
  } else {
    // 回到监控模式时重新布防，避免 track 模式下的确认状态影响新的监控告警。
    clearAckAndDangerState("mode_monitor");
  }

  Serial.printf("[MODE] current=%s a1_model=%u voice=%s ptz=%s%s\n",
                modeName(new_mode), model_id,
                modeVoiceEnabled() ? "on" : "off",
                modePtzEnabled() ? "on" : "off",
                changed ? "" : " unchanged");
}

static void handleSwitchAck(uint8_t len, const uint8_t *data) {
  if (len != 1) {
    Serial.printf("[MODE<-A1] SWITCH_ACK len error: %u\n", len);
    return;
  }
  uint8_t result = data[0];
  if (result == 0xFF) {
    Serial.println("[MODE<-A1] SWITCH_ACK failed");
  } else {
    Serial.printf("[MODE<-A1] SWITCH_ACK success, model_id=%u\n", result);
  }
}

static uint8_t *allocJpegBuffer(const char *name) {
  uint8_t *p = (uint8_t *)heap_caps_malloc(MAX_JPEG_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!p) {
    p = (uint8_t *)heap_caps_malloc(MAX_JPEG_SIZE, MALLOC_CAP_8BIT);
  }

  if (p) {
    Serial.printf("[OK] %s malloc %u bytes\n", name, (unsigned)MAX_JPEG_SIZE);
  } else {
    Serial.printf("[FATAL] %s malloc failed\n", name);
  }
  return p;
}

static bool initBuffers() {
  g_img.buf = allocJpegBuffer("assemble_buf");
  g_latest_jpeg = allocJpegBuffer("latest_jpeg_buf");
  g_http_copy = allocJpegBuffer("http_copy_buf");

  g_jpeg_mutex = xSemaphoreCreateMutex();
  g_alert_mutex = xSemaphoreCreateMutex();
  g_detection_mutex = xSemaphoreCreateMutex();
  g_sensor_mutex = xSemaphoreCreateMutex();
  g_voice_mutex = xSemaphoreCreateMutex();
  g_ptz_mutex = xSemaphoreCreateMutex();
  g_mode_mutex = xSemaphoreCreateMutex();
  g_uart_tx_mutex = xSemaphoreCreateMutex();

  return g_img.buf && g_latest_jpeg && g_http_copy &&
         g_jpeg_mutex && g_alert_mutex && g_detection_mutex && g_sensor_mutex &&
         g_voice_mutex && g_ptz_mutex && g_mode_mutex && g_uart_tx_mutex;
}

// ========================== DS18B20 温度采集 ==========================
static void updateTemperatureCelsius(float t) {
  if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    uint32_t now = millis();

    if (!isnan(t)) {
      // 读到有效温度：更新前端显示值。
      g_temperature_celsius = t;
      g_temperature_valid = true;
      g_temperature_last_ok_ms = now;
    } else {
      // 偶发读失败：不要立刻清空温度，否则前端会在真实温度和默认温度之间来回跳。
      // 只有从未读成功，或者长时间读不到，才返回 null。
      if (!g_temperature_valid || (now - g_temperature_last_ok_ms > DS18B20_MAX_STALE_MS)) {
        g_temperature_celsius = NAN;
        g_temperature_valid = false;
      }
    }

    xSemaphoreGive(g_sensor_mutex);
  }
}

static void initDs18b20() {
  g_ds18b20.begin();
  int count = g_ds18b20.getDeviceCount();
  g_ds18b20_found = (count > 0);

  if (g_ds18b20_found) {
    g_ds18b20.setResolution(DS18B20_RESOLUTION_BITS);
    g_ds18b20.setWaitForConversion(true);
    Serial.printf("[OK] DS18B20 started: DATA=IO%d, count=%d, resolution=%d-bit\n",
                  DS18B20_DATA_PIN, count, DS18B20_RESOLUTION_BITS);
  } else {
    updateTemperatureCelsius(NAN);
    Serial.printf("[WARN] DS18B20 not found on IO%d. /api/telemetry will return null.\n", DS18B20_DATA_PIN);
  }
}

static void ds18b20Task(void *param) {
  (void)param;

  while (true) {
    if (!g_ds18b20_found) {
      // 支持重新插线/接线修复后自动恢复。
      g_ds18b20.begin();
      g_ds18b20_found = (g_ds18b20.getDeviceCount() > 0);
      if (g_ds18b20_found) {
        g_ds18b20.setResolution(DS18B20_RESOLUTION_BITS);
        g_ds18b20.setWaitForConversion(true);
        Serial.printf("[OK] DS18B20 detected again on IO%d\n", DS18B20_DATA_PIN);
      } else {
        updateTemperatureCelsius(NAN);
        vTaskDelay(pdMS_TO_TICKS(2000));
        continue;
      }
    }

    g_ds18b20.requestTemperatures();
    float t = g_ds18b20.getTempCByIndex(0);

    if (t == DEVICE_DISCONNECTED_C || t < -55.0f || t > 125.0f) {
      updateTemperatureCelsius(NAN);
      g_ds18b20_found = false;
      Serial.printf("[WARN] DS18B20 read failed on IO%d, value=%.2f\n", DS18B20_DATA_PIN, t);
    } else {
      updateTemperatureCelsius(t);
      Serial.printf("[TEMP] DS18B20 %.2f C\n", t);
    }

    vTaskDelay(pdMS_TO_TICKS(DS18B20_READ_INTERVAL_MS));
  }
}

// ========================== 云台舵机控制 ==========================
static int clampInt(int v, int min_v, int max_v) {
  if (v < min_v) return min_v;
  if (v > max_v) return max_v;
  return v;
}

static uint32_t pulseUsToDuty(uint32_t pulse_us) {
  const uint32_t max_duty = (1UL << PTZ_PWM_RES_BITS) - 1;
  return (uint32_t)(((uint64_t)pulse_us * max_duty) / PTZ_PWM_PERIOD_US);
}

static uint32_t angleToPulseUs(int angle, int min_angle, int max_angle) {
  angle = clampInt(angle, min_angle, max_angle);
  if (max_angle <= min_angle) return 1500;

  return PTZ_SERVO_MIN_US +
         (uint32_t)(((uint64_t)(angle - min_angle) * (PTZ_SERVO_MAX_US - PTZ_SERVO_MIN_US)) /
                    (uint32_t)(max_angle - min_angle));
}

static void writeServoPulseUs(ledc_channel_t channel, uint32_t pulse_us) {
  pulse_us = constrain((int)pulse_us, PTZ_SERVO_MIN_US, PTZ_SERVO_MAX_US);
  uint32_t duty = pulseUsToDuty(pulse_us);
  ledc_set_duty(PTZ_LEDC_MODE, channel, duty);
  ledc_update_duty(PTZ_LEDC_MODE, channel);
}

static uint32_t panStepDelayMs() {
  // 以 PTZ_PAN_MS_PER_5_DEG 作为 5°标定值。
  // PTZ_STEP_ANGLE 改成 10 时，转动时间自动约为 2 倍。
  uint32_t ms = (uint32_t)(((uint64_t)PTZ_PAN_MS_PER_5_DEG * PTZ_PAN_STEP_ANGLE) / 5);
  if (ms < 20) ms = 20;
  return ms;
}

static void stopPanContinuousServo() {
  writeServoPulseUs(PTZ_PAN_CHANNEL, PTZ_PAN_STOP_US);
}

static void writePtzAnglesLocked() {
#if PTZ_PAN_CONTINUOUS_MODE
  // 连续旋转 360°舵机保持停止脉宽，否则会一直转。
  stopPanContinuousServo();
#else
  uint32_t pan_us = angleToPulseUs(g_pan_angle, PTZ_PAN_MIN_ANGLE, PTZ_PAN_MAX_ANGLE);
  writeServoPulseUs(PTZ_PAN_CHANNEL, pan_us);
#endif

  uint32_t tilt_us = angleToPulseUs(g_tilt_angle, PTZ_TILT_CAL_MIN_ANGLE, PTZ_TILT_CAL_MAX_ANGLE);
  writeServoPulseUs(PTZ_TILT_CHANNEL, tilt_us);
}

static bool movePtzByDirection(int direction, char *action, size_t action_size) {
  if (action && action_size > 0) action[0] = '\0';

  if (xSemaphoreTake(g_ptz_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return false;
  }

  switch (direction) {
    case 1: // 上：180°舵机 +5°
      g_tilt_angle = clampInt(g_tilt_angle + PTZ_TILT_STEP_ANGLE, PTZ_TILT_MIN_ANGLE, PTZ_TILT_MAX_ANGLE);
      writePtzAnglesLocked();
      if (action) snprintf(action, action_size, "up");
      break;

    case 2: // 下：180°舵机 -5°
      g_tilt_angle = clampInt(g_tilt_angle - PTZ_TILT_STEP_ANGLE, PTZ_TILT_MIN_ANGLE, PTZ_TILT_MAX_ANGLE);
      writePtzAnglesLocked();
      if (action) snprintf(action, action_size, "down");
      break;

    case 3: // 左：360°连续旋转舵机转一小段后停止，近似 5°
#if PTZ_PAN_CONTINUOUS_MODE
      g_pan_angle = clampInt(g_pan_angle - PTZ_PAN_STEP_ANGLE, PTZ_PAN_MIN_ANGLE, PTZ_PAN_MAX_ANGLE);
      writeServoPulseUs(PTZ_PAN_CHANNEL, PTZ_PAN_LEFT_US);
      vTaskDelay(pdMS_TO_TICKS(panStepDelayMs()));
      stopPanContinuousServo();
      vTaskDelay(pdMS_TO_TICKS(30));
      stopPanContinuousServo();
#else
      g_pan_angle = clampInt(g_pan_angle - PTZ_PAN_STEP_ANGLE, PTZ_PAN_MIN_ANGLE, PTZ_PAN_MAX_ANGLE);
      writePtzAnglesLocked();
#endif
      if (action) snprintf(action, action_size, "left");
      break;

    case 4: // 右：360°连续旋转舵机转一小段后停止，近似 5°
#if PTZ_PAN_CONTINUOUS_MODE
      g_pan_angle = clampInt(g_pan_angle + PTZ_PAN_STEP_ANGLE, PTZ_PAN_MIN_ANGLE, PTZ_PAN_MAX_ANGLE);
      writeServoPulseUs(PTZ_PAN_CHANNEL, PTZ_PAN_RIGHT_US);
      vTaskDelay(pdMS_TO_TICKS(panStepDelayMs()));
      stopPanContinuousServo();
      vTaskDelay(pdMS_TO_TICKS(30));
      stopPanContinuousServo();
#else
      g_pan_angle = clampInt(g_pan_angle + PTZ_PAN_STEP_ANGLE, PTZ_PAN_MIN_ANGLE, PTZ_PAN_MAX_ANGLE);
      writePtzAnglesLocked();
#endif
      if (action) snprintf(action, action_size, "right");
      break;

    default:
      xSemaphoreGive(g_ptz_mutex);
      return false;
  }

  Serial.printf("[PTZ] direction=%d action=%s pan=%d tilt=%d\n",
                direction, action ? action : "", g_pan_angle, g_tilt_angle);

  xSemaphoreGive(g_ptz_mutex);
  return true;
}

static void initPtzServo() {
  ledc_timer_config_t timer_conf = {};
  timer_conf.speed_mode = PTZ_LEDC_MODE;
  timer_conf.duty_resolution = LEDC_TIMER_14_BIT;
  timer_conf.timer_num = PTZ_LEDC_TIMER;
  timer_conf.freq_hz = PTZ_PWM_FREQ_HZ;
  timer_conf.clk_cfg = LEDC_AUTO_CLK;
  esp_err_t ret = ledc_timer_config(&timer_conf);
  if (ret != ESP_OK) {
    Serial.printf("[ERR] PTZ LEDC timer config failed: 0x%x\n", ret);
    return;
  }

  ledc_channel_config_t pan_conf = {};
  pan_conf.gpio_num = PTZ_PAN_SERVO_PIN;
  pan_conf.speed_mode = PTZ_LEDC_MODE;
  pan_conf.channel = PTZ_PAN_CHANNEL;
  pan_conf.intr_type = LEDC_INTR_DISABLE;
  pan_conf.timer_sel = PTZ_LEDC_TIMER;
  pan_conf.duty = 0;
  pan_conf.hpoint = 0;
  ret = ledc_channel_config(&pan_conf);
  if (ret != ESP_OK) {
    Serial.printf("[ERR] PTZ pan channel config failed: 0x%x\n", ret);
    return;
  }

  ledc_channel_config_t tilt_conf = {};
  tilt_conf.gpio_num = PTZ_TILT_SERVO_PIN;
  tilt_conf.speed_mode = PTZ_LEDC_MODE;
  tilt_conf.channel = PTZ_TILT_CHANNEL;
  tilt_conf.intr_type = LEDC_INTR_DISABLE;
  tilt_conf.timer_sel = PTZ_LEDC_TIMER;
  tilt_conf.duty = 0;
  tilt_conf.hpoint = 0;
  ret = ledc_channel_config(&tilt_conf);
  if (ret != ESP_OK) {
    Serial.printf("[ERR] PTZ tilt channel config failed: 0x%x\n", ret);
    return;
  }

  if (xSemaphoreTake(g_ptz_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    g_pan_angle = PTZ_PAN_HOME_ANGLE;
    g_tilt_angle = PTZ_TILT_HOME_ANGLE;
    writePtzAnglesLocked();
    xSemaphoreGive(g_ptz_mutex);
  }

  Serial.printf("[OK] PTZ servo started: pan IO%d, tilt IO%d, PWM=%dHz\n",
                PTZ_PAN_SERVO_PIN, PTZ_TILT_SERVO_PIN, PTZ_PWM_FREQ_HZ);
}

static void getPtzAngles(int *pan, int *tilt) {
  if (!pan || !tilt) return;
  if (xSemaphoreTake(g_ptz_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    *pan = g_pan_angle;
    *tilt = g_tilt_angle;
    xSemaphoreGive(g_ptz_mutex);
  }
}

// ========================== 火焰快速视觉伺服控制 ==========================
static bool isTrackTargetClass(uint8_t class_id) {
#if PTZ_TRACK_CLASS_ID < 0
  (void)class_id;
  return true;
#else
  return class_id == (uint8_t)PTZ_TRACK_CLASS_ID;
#endif
}

static void stopTrackPanServo() {
#if PTZ_PAN_CONTINUOUS_MODE
  if (!g_track_pan_active) return;
  if (xSemaphoreTake(g_ptz_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    stopPanContinuousServo();
    g_track_pan_active = false;
    xSemaphoreGive(g_ptz_mutex);
    Serial.println("[TRACK-PAN] stop");
  }
#endif
}

static bool movePtzByDeltaForTrack(int pan_delta, int tilt_delta, const char *reason) {
  if (pan_delta == 0 && tilt_delta == 0) return true;

  if (xSemaphoreTake(g_ptz_mutex, pdMS_TO_TICKS(80)) != pdTRUE) {
    Serial.println("[TRACK-PTZ] g_ptz_mutex busy, skip this coordinate");
    return false;
  }

  bool moved = false;

  // 俯仰轴是位置舵机，可与水平轴同时更新目标位置。
  if (tilt_delta != 0) {
    int old = g_tilt_angle;
    g_tilt_angle = clampInt(g_tilt_angle + tilt_delta, PTZ_TILT_MIN_ANGLE, PTZ_TILT_MAX_ANGLE);
    int effective_delta = g_tilt_angle - old;
    if (effective_delta != 0) {
      uint32_t tilt_us = angleToPulseUs(g_tilt_angle, PTZ_TILT_CAL_MIN_ANGLE, PTZ_TILT_CAL_MAX_ANGLE);
      writeServoPulseUs(PTZ_TILT_CHANNEL, tilt_us);
      moved = true;
      Serial.printf("[TRACK-MOVE-Y] %s tilt %d -> %d delta=%d\n",
                    reason ? reason : "track", old, g_tilt_angle, effective_delta);
    }
  }

  if (pan_delta != 0) {
    int old = g_pan_angle;

#if PTZ_PAN_CONTINUOUS_MODE
    int effective_delta = pan_delta;
    g_pan_angle = (g_pan_angle + effective_delta) % 360;
    if (g_pan_angle < 0) g_pan_angle += 360;

    int magnitude = abs(effective_delta);
    int speed_offset = PTZ_TRACK_PAN_MIN_OFFSET_US + (magnitude - 1) * 40;
    speed_offset = clampInt(speed_offset,
                            PTZ_TRACK_PAN_MIN_OFFSET_US,
                            PTZ_TRACK_PAN_MAX_OFFSET_US);
    uint32_t pulse = (effective_delta > 0)
                       ? (PTZ_PAN_STOP_US - speed_offset)
                       : (PTZ_PAN_STOP_US + speed_offset);
    writeServoPulseUs(PTZ_PAN_CHANNEL, pulse);
    g_track_pan_active = true;
    moved = true;
#else
    g_pan_angle = clampInt(g_pan_angle + pan_delta, PTZ_PAN_MIN_ANGLE, PTZ_PAN_MAX_ANGLE);
    int effective_delta = g_pan_angle - old;
    if (effective_delta != 0) {
      uint32_t pan_us = angleToPulseUs(g_pan_angle, PTZ_PAN_MIN_ANGLE, PTZ_PAN_MAX_ANGLE);
      writeServoPulseUs(PTZ_PAN_CHANNEL, pan_us);
      moved = true;
    }
#endif

    if (moved) {
      Serial.printf("[TRACK-MOVE-X] %s pan %d -> %d command=%d\n",
                    reason ? reason : "track", old, g_pan_angle, pan_delta);
    }
  }

  xSemaphoreGive(g_ptz_mutex);
  return moved;
}

static void trackResetDetCache() {
  g_track_has_det_count = false;
  g_track_expected_count = 0;
  g_track_received_count = 0;
  g_track_best_valid = false;
  g_track_best_det = {};
}

static void trackResetControllerState() {
  stopTrackPanServo();
  trackResetDetCache();
  g_track_filter_valid = false;
  g_track_filtered_x = 0.0f;
  g_track_filtered_y = 0.0f;
  g_track_last_box_area = 0;
  g_track_history_count = 0;
  g_track_history_index = 0;
  g_track_pending_axis = 0;
  g_track_pending_direction = 0;
  g_track_pending_frames = 0;
  g_track_waiting_feedback = false;
  g_track_last_move_ms = 0;
  g_track_last_sample_ms = 0;
  g_track_last_tilt_command_ms = 0;
  g_track_tilt_active = false;
}

static int trackDeltaForError(float error, float error_limit, float gain, bool reverse) {
  if (fabsf(error) <= error_limit) return 0;

  // cam_x/cam_y 是归一化成像平面坐标，atan 可近似换算成视线夹角。
  float angle_error = atanf(error) * (180.0f / PI) * gain;
  int delta = (int)lroundf(angle_error);
  if (delta == 0) delta = (error > 0.0f) ? PTZ_TRACK_MIN_STEP_ANGLE : -PTZ_TRACK_MIN_STEP_ANGLE;
  delta = clampInt(delta, -PTZ_TRACK_MAX_STEP_ANGLE, PTZ_TRACK_MAX_STEP_ANGLE);
  if (reverse) delta = -delta;
  return delta;
}

static void trackHandleDetCount(uint8_t count) {
#if PTZ_TRACK_ENABLE
  if (!g_track_enabled) {
    trackResetDetCache();
    return;
  }

  g_track_has_det_count = true;
  g_track_expected_count = count;
  g_track_received_count = 0;
  g_track_best_valid = false;

  if (count == 0) {
    stopTrackPanServo();
    g_track_filter_valid = false;
    g_track_last_box_area = 0;
    g_track_history_count = 0;
    g_track_history_index = 0;
    g_track_pending_axis = 0;
    g_track_pending_direction = 0;
    g_track_pending_frames = 0;
    g_track_waiting_feedback = false;
    g_track_last_sample_ms = 0;
    g_track_last_tilt_command_ms = 0;
    g_track_tilt_active = false;
    static uint32_t last_print_ms = 0;
    if (millis() - last_print_ms > 1000) {
      Serial.println("[TRACK] 当前帧没有目标，不移动云台");
      last_print_ms = millis();
    }
  }
#else
  (void)count;
#endif
}

static void trackFeedbackControlOnce(const Detection_t &det) {
#if PTZ_TRACK_ENABLE
  if (!g_track_enabled) return;
  if (!isfinite(det.cam_x) || !isfinite(det.cam_y)) {
    Serial.println("[TRACK-WARN] invalid camera coordinate, ignored");
    return;
  }

  uint32_t now = millis();
  if (now - g_track_last_move_ms < PTZ_TRACK_SETTLE_MS) {
    return;
  }
  if (now - g_track_last_sample_ms < PTZ_TRACK_SAMPLE_MS) {
    return;
  }
  g_track_last_sample_ms = now;

  g_track_history_x[g_track_history_index] = det.cam_x;
  g_track_history_y[g_track_history_index] = det.cam_y;
  g_track_history_index = (g_track_history_index + 1) % PTZ_TRACK_MEDIAN_SIZE;
  if (g_track_history_count < PTZ_TRACK_MEDIAN_SIZE) g_track_history_count++;

  float sorted_x[PTZ_TRACK_MEDIAN_SIZE];
  float sorted_y[PTZ_TRACK_MEDIAN_SIZE];
  for (uint8_t i = 0; i < g_track_history_count; i++) {
    sorted_x[i] = g_track_history_x[i];
    sorted_y[i] = g_track_history_y[i];
  }
  for (uint8_t i = 1; i < g_track_history_count; i++) {
    float value_x = sorted_x[i];
    float value_y = sorted_y[i];
    int j = i - 1;
    while (j >= 0 && sorted_x[j] > value_x) {
      sorted_x[j + 1] = sorted_x[j];
      j--;
    }
    sorted_x[j + 1] = value_x;

    j = i - 1;
    while (j >= 0 && sorted_y[j] > value_y) {
      sorted_y[j + 1] = sorted_y[j];
      j--;
    }
    sorted_y[j + 1] = value_y;
  }
  float measured_x = sorted_x[g_track_history_count / 2];
  float measured_y = sorted_y[g_track_history_count / 2];

  uint32_t box_area = (uint32_t)det.box_width * (uint32_t)det.box_height;
  if (!g_track_filter_valid) {
    g_track_filtered_x = measured_x;
    g_track_filtered_y = measured_y;
    g_track_filter_valid = true;
  } else {
    float alpha = PTZ_TRACK_FILTER_ALPHA;
    if (g_track_last_box_area > 0 && box_area > 0) {
      float area_change = fabsf((float)box_area - (float)g_track_last_box_area) /
                          (float)g_track_last_box_area;
      if (area_change > PTZ_TRACK_BOX_CHANGE_LIMIT) alpha = PTZ_TRACK_UNSTABLE_ALPHA;
    }

    float jump_x = measured_x - g_track_filtered_x;
    float jump_y = measured_y - g_track_filtered_y;
    if (fabsf(jump_x) < PTZ_TRACK_INPUT_NOISE_LIMIT) jump_x = 0.0f;
    if (fabsf(jump_y) < PTZ_TRACK_INPUT_NOISE_LIMIT) jump_y = 0.0f;
    jump_x = fmaxf(-PTZ_TRACK_MAX_COORD_JUMP, fminf(PTZ_TRACK_MAX_COORD_JUMP, jump_x));
    jump_y = fmaxf(-PTZ_TRACK_MAX_COORD_JUMP, fminf(PTZ_TRACK_MAX_COORD_JUMP, jump_y));
    g_track_filtered_x += alpha * jump_x;
    g_track_filtered_y += alpha * jump_y;
  }
  if (box_area > 0) g_track_last_box_area = box_area;

  // 中值坐标负责快速制动，低通坐标保留少量权重用于抑制边界抖动。
  float control_x = 0.75f * measured_x + 0.25f * g_track_filtered_x;
  float control_y = 0.75f * measured_y + 0.25f * g_track_filtered_y;
  float err_x = control_x - PTZ_TRACK_TARGET_CAM_X;
  float err_y = control_y - PTZ_TRACK_TARGET_CAM_Y;

  if (g_track_waiting_feedback) {
    Serial.println("[TRACK-FEEDBACK] 收到舵机移动后的新坐标，重新比较");
    g_track_waiting_feedback = false;
  }

  Serial.printf("[TRACK-COORD] class=%s(%u) score=%u raw=(%.4f,%.4f) median=(%.4f,%.4f) filtered=(%.4f,%.4f) errX=%.4f errY=%.4f pan=%d tilt=%d\n",
                className(det.class_id), det.class_id, det.score, det.cam_x, det.cam_y,
                (double)measured_x, (double)measured_y,
                (double)g_track_filtered_x, (double)g_track_filtered_y,
                (double)err_x, (double)err_y, g_pan_angle, g_tilt_angle);

  float pan_error_limit = g_track_pan_active
                            ? PTZ_TRACK_ERROR_LIMIT_X
                            : PTZ_TRACK_PAN_START_LIMIT;
  float tilt_error_limit = g_track_tilt_active
                             ? PTZ_TRACK_ERROR_LIMIT_Y
                             : PTZ_TRACK_TILT_START_LIMIT;

  int pan_delta = trackDeltaForError(err_x, pan_error_limit,
                                    PTZ_TRACK_PAN_GAIN, PTZ_TRACK_PAN_REVERSE);
  int tilt_request = trackDeltaForError(err_y, tilt_error_limit,
                                       PTZ_TRACK_TILT_GAIN, PTZ_TRACK_TILT_REVERSE);
  if (tilt_request == 0) {
    g_track_tilt_active = false;
  } else {
    g_track_tilt_active = true;
  }

  int tilt_delta = tilt_request;
  tilt_delta = clampInt(tilt_delta, -PTZ_TRACK_TILT_STEP_MAX, PTZ_TRACK_TILT_STEP_MAX);
  if (tilt_delta != 0 && now - g_track_last_tilt_command_ms < PTZ_TRACK_TILT_COMMAND_MS) {
    tilt_delta = 0;
  }

  if (pan_delta == 0) stopTrackPanServo();

  if (pan_delta != 0 || tilt_delta != 0) {
    if (movePtzByDeltaForTrack(pan_delta, tilt_delta, "快速视觉伺服")) {
      g_track_last_move_ms = millis();
      if (tilt_delta != 0) g_track_last_tilt_command_ms = g_track_last_move_ms;
      Serial.printf("[TRACK-SERVO] pan=%d tilt=%d\n", pan_delta, tilt_delta);
    }
    return;
  }

  g_track_pending_axis = 0;
  g_track_pending_direction = 0;
  g_track_pending_frames = 0;

  Serial.printf("[TRACK-OK] 坐标已满足: cam=(%.4f,%.4f), errX=%.4f errY=%.4f\n",
                g_track_filtered_x, g_track_filtered_y, (double)err_x, (double)err_y);
#else
  (void)det;
#endif
}

static bool trackPreferCandidate(const Detection_t &candidate) {
  if (!g_track_best_valid) return true;
  if (!g_track_filter_valid) return candidate.score > g_track_best_det.score;

  float candidate_dx = candidate.cam_x - g_track_filtered_x;
  float candidate_dy = candidate.cam_y - g_track_filtered_y;
  float best_dx = g_track_best_det.cam_x - g_track_filtered_x;
  float best_dy = g_track_best_det.cam_y - g_track_filtered_y;
  float candidate_distance = candidate_dx * candidate_dx + candidate_dy * candidate_dy;
  float best_distance = best_dx * best_dx + best_dy * best_dy;
  return candidate_distance < best_distance;
}

static void trackHandleDetResult(uint8_t class_id, uint8_t score, float cam_x, float cam_y,
                                 uint16_t box_width, uint16_t box_height) {
#if PTZ_TRACK_ENABLE
  if (!g_track_enabled) return;
  bool target_class = isTrackTargetClass(class_id);
  if (target_class && isfinite(cam_x) && isfinite(cam_y)) {
    g_track_last_target_ms = millis();
  }

  Detection_t det = {};
  det.class_id = class_id;
  det.score = score;
  det.cam_x = cam_x;
  det.cam_y = cam_y;
  det.box_width = box_width;
  det.box_height = box_height;
  det.local_ms = millis();

  if (g_track_has_det_count) {
    // DET_COUNT 是整帧全部检测数，即使类别不匹配也必须计数，否则该帧无法结束。
    g_track_received_count++;

    if (target_class && isfinite(cam_x) && isfinite(cam_y) && trackPreferCandidate(det)) {
      g_track_best_det = det;
      g_track_best_valid = true;
    }

    if (g_track_received_count >= g_track_expected_count) {
      if (g_track_best_valid) {
        trackFeedbackControlOnce(g_track_best_det);
      }
      trackResetDetCache();
    }
  } else {
    // 兼容 A1 没有先发 DET_COUNT 的情况。
    if (target_class) trackFeedbackControlOnce(det);
  }
#else
  (void)class_id;
  (void)score;
  (void)cam_x;
  (void)cam_y;
  (void)box_width;
  (void)box_height;
#endif
}

static void trackServoWatchdog() {
#if PTZ_TRACK_ENABLE && PTZ_PAN_CONTINUOUS_MODE
  if (!g_track_enabled) {
    g_track_pan_active = false;
    return;
  }
  if (g_track_pan_active &&
      millis() - g_track_last_target_ms > PTZ_TRACK_TARGET_TIMEOUT_MS) {
    Serial.println("[TRACK-WATCHDOG] target timeout, stop pan servo");
    stopTrackPanServo();
  }
#endif
}

// ========================== 语音模块播报 ==========================
static bool isVoiceAlarmClass(uint8_t class_id) {
  return class_id == CLASS_FIRE || class_id == CLASS_SMOKE || class_id == CLASS_FALLEN;
}

static const char *voiceClassName(uint8_t class_id) {
  switch (class_id) {
    case CLASS_FIRE:   return "fire";
    case CLASS_SMOKE:  return "smoke";
    case CLASS_FALLEN: return "fall";
    default:           return "unknown";
  }
}

// 只记录“最近收到过哪种告警”，不在这里直接发串口。
// 这样即使 A1 一秒发几十次火灾，语音模块也不会被连续指令打断。
static void voiceMarkAlarm(uint8_t class_id) {
  if (!modeVoiceEnabled()) return;
  if (!isVoiceAlarmClass(class_id) || !g_voice_mutex) return;

  uint32_t now = millis();
  if (xSemaphoreTake(g_voice_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (class_id == CLASS_FIRE) g_voice_fire_seen_ms = now;
    else if (class_id == CLASS_SMOKE) g_voice_smoke_seen_ms = now;
    else if (class_id == CLASS_FALLEN) g_voice_fall_seen_ms = now;
    xSemaphoreGive(g_voice_mutex);
  }
}

static bool voiceClassActive(uint8_t class_id, uint32_t now,
                             uint32_t fire_ms, uint32_t smoke_ms, uint32_t fall_ms) {
  if (class_id == CLASS_FIRE) return fire_ms != 0 && (now - fire_ms) <= VOICE_ALARM_HOLD_MS;
  if (class_id == CLASS_SMOKE) return smoke_ms != 0 && (now - smoke_ms) <= VOICE_ALARM_HOLD_MS;
  if (class_id == CLASS_FALLEN) return fall_ms != 0 && (now - fall_ms) <= VOICE_ALARM_HOLD_MS;
  return false;
}

// 如果多种告警同时存在，采用轮询，避免一直只播一种。
// 顺序：火灾 -> 烟雾 -> 摔倒。
static uint8_t voicePickNextAlarm() {
  if (!g_voice_mutex) return 255;

  uint32_t now = millis();
  uint32_t fire_ms = 0, smoke_ms = 0, fall_ms = 0;
  uint8_t last = 255;

  if (xSemaphoreTake(g_voice_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    fire_ms = g_voice_fire_seen_ms;
    smoke_ms = g_voice_smoke_seen_ms;
    fall_ms = g_voice_fall_seen_ms;
    last = g_voice_last_played_class;
    xSemaphoreGive(g_voice_mutex);
  } else {
    return 255;
  }

  const uint8_t order[3] = {CLASS_FIRE, CLASS_SMOKE, CLASS_FALLEN};
  int start = 0;
  for (int i = 0; i < 3; i++) {
    if (order[i] == last) {
      start = (i + 1) % 3;
      break;
    }
  }

  for (int i = 0; i < 3; i++) {
    uint8_t cls = order[(start + i) % 3];
    if (voiceClassActive(cls, now, fire_ms, smoke_ms, fall_ms)) {
      return cls;
    }
  }

  return 255;
}

static bool voiceSendCommand(uint8_t class_id) {
  const uint8_t fire_cmd[5]  = {0xAA, 0x55, 0xFF, 0x3E, 0xFB};
  const uint8_t smoke_cmd[5] = {0xAA, 0x55, 0xFF, 0x3F, 0xFB};
  const uint8_t fall_cmd[5]  = {0xAA, 0x55, 0xFF, 0x3D, 0xFB};

  const uint8_t *cmd = NULL;
  if (class_id == CLASS_FIRE) cmd = fire_cmd;
  else if (class_id == CLASS_SMOKE) cmd = smoke_cmd;
  else if (class_id == CLASS_FALLEN) cmd = fall_cmd;
  else return false;

  Serial2.write(cmd, 5);
  Serial2.flush();

  if (xSemaphoreTake(g_voice_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    g_voice_last_played_class = class_id;
    xSemaphoreGive(g_voice_mutex);
  }

  Serial.printf("[VOICE] play %s cmd=%02X %02X %02X %02X %02X\n",
                voiceClassName(class_id), cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);
  return true;
}

static void voiceTask(void *param) {
  (void)param;

  while (true) {
    if (!modeVoiceEnabled()) {
      clearVoiceState();
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    uint8_t class_id = voicePickNextAlarm();

    if (class_id != 255) {
      voiceSendCommand(class_id);
      // 关键：等待一句话播完，再允许下一次发送。
      vTaskDelay(pdMS_TO_TICKS(VOICE_PLAY_GAP_MS));
    } else {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}

// ========================== 告警缓存 ==========================
static void saveLatestDetection(uint8_t class_id, uint8_t score, float cam_x, float cam_y) {
  if (xSemaphoreTake(g_detection_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    g_latest_detection.class_id = class_id;
    g_latest_detection.score = score;
    g_latest_detection.cam_x = cam_x;
    g_latest_detection.cam_y = cam_y;
    g_latest_detection.local_ms = millis();
    g_has_detection = true;
    xSemaphoreGive(g_detection_mutex);
  }
}

static bool shouldPushAlert(uint8_t class_id) {
  uint32_t now = millis();
  uint32_t *last_ms = NULL;

  if (class_id == CLASS_FIRE) last_ms = &g_last_fire_ms;
  else if (class_id == CLASS_SMOKE) last_ms = &g_last_smoke_ms;
  else if (class_id == CLASS_FALLEN) last_ms = &g_last_fall_ms;
  else return false;

  if (*last_ms != 0 && (now - *last_ms) < ALERT_COOLDOWN_MS) return false;
  *last_ms = now;
  return true;
}

static void pushAlert(uint8_t class_id, uint8_t confidence_percent,
                      uint16_t x, uint16_t y, uint16_t box_w, uint16_t box_h,
                      bool from_alert_frame) {
  if (!isMonitorMode()) return;
  const char *event = frontendEventType(class_id);
  if (!event) return;

  if (isAlarmAcknowledged(class_id)) {
    Serial.printf("[ACK] ignore acknowledged %s, wait until danger disappears\n", className(class_id));
    return;
  }

  uint32_t now = millis();
  if (g_alert_reset_until_ms != 0 && (int32_t)(now - g_alert_reset_until_ms) < 0) {
    return;
  }

  if (!shouldPushAlert(class_id)) return;

  AlertEvent_t e = {};
  strncpy(e.event_type, event, sizeof(e.event_type) - 1);
  strncpy(e.level, frontendLevel(class_id, confidence_percent), sizeof(e.level) - 1);
  strncpy(e.message, frontendMessage(class_id, confidence_percent), sizeof(e.message) - 1);
  makeTimestamp(e.received_at, sizeof(e.received_at));
  e.class_id = class_id;
  e.confidence_percent = confidence_percent;
  e.target_x = x;
  e.target_y = y;
  e.box_w = box_w;
  e.box_h = box_h;
  e.local_ms = millis();

  if (xSemaphoreTake(g_alert_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    g_alerts[g_alert_head] = e;
    g_alert_head = (g_alert_head + 1) % ALERT_BUFFER_SIZE;
    if (g_alert_count < ALERT_BUFFER_SIZE) g_alert_count++;
    xSemaphoreGive(g_alert_mutex);
  }

  Serial.printf("[ALERT->HTTP] %s class=%s(%u) conf=%u%% center=(%u,%u) source=%s\n",
                e.event_type, className(class_id), class_id, confidence_percent,
                x, y, from_alert_frame ? "ALERT_FRAME" : "DET_RESULT_FALLBACK");
}

static String alertToJson(const AlertEvent_t &e) {
  float conf = ((float)e.confidence_percent) / 100.0f;

  char buf[640];
  snprintf(buf, sizeof(buf),
           "{\"device_id\":\"%s\"," 
           "\"event_type\":\"%s\"," 
           "\"confidence\":%.2f," 
           "\"source_timestamp\":null," 
           "\"received_at\":\"%s\"," 
           "\"level\":\"%s\"," 
           "\"message\":\"%s\"," 
           "\"class_id\":%u," 
           "\"target_x\":%u," 
           "\"target_y\":%u," 
           "\"box_width\":%u," 
           "\"box_height\":%u}",
           DEVICE_ID,
           e.event_type,
           conf,
           e.received_at,
           e.level,
           e.message,
           e.class_id,
           e.target_x,
           e.target_y,
           e.box_w,
           e.box_h);
  return String(buf);
}

static int parseLimitFromReq(httpd_req_t *req, int default_limit) {
  int limit = default_limit;
  char query[80];

  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    char value[16];
    if (httpd_query_key_value(query, "limit", value, sizeof(value)) == ESP_OK) {
      int n = atoi(value);
      if (n >= 1 && n <= 200) limit = n;
    }
  }

  if (limit > ALERT_BUFFER_SIZE) limit = ALERT_BUFFER_SIZE;
  return limit;
}

static void expireOldAlertsLocked(uint32_t now) {
  if (g_alert_count == 0) return;

  int latest_idx = (int)g_alert_head - 1;
  while (latest_idx < 0) latest_idx += ALERT_BUFFER_SIZE;
  latest_idx %= ALERT_BUFFER_SIZE;

  if (!seenRecently(g_alerts[latest_idx].local_ms, now, ALERT_FRONTEND_TTL_MS)) {
    memset(g_alerts, 0, sizeof(g_alerts));
    g_alert_head = 0;
    g_alert_count = 0;
    g_last_fire_ms = 0;
    g_last_smoke_ms = 0;
    g_last_fall_ms = 0;
    Serial.println("[ALERT] old frontend alert expired, return safe state");
  }
}

static String buildAlertsArrayJson(int limit) {
  String json = "[";

  if (xSemaphoreTake(g_alert_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    expireOldAlertsLocked(millis());
    int n = g_alert_count;
    if (n > limit) n = limit;

    for (int i = 0; i < n; i++) {
      int idx = (int)g_alert_head - 1 - i;
      while (idx < 0) idx += ALERT_BUFFER_SIZE;
      idx %= ALERT_BUFFER_SIZE;

      if (i > 0) json += ",";
      json += alertToJson(g_alerts[idx]);
    }

    xSemaphoreGive(g_alert_mutex);
  }

  json += "]";
  return json;
}

// copy the most recent alert (thread-safe). Returns true if an alert was copied.
static bool copyLatestAlert(AlertEvent_t *out) {
  if (!out) return false;
  if (xSemaphoreTake(g_alert_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;
  expireOldAlertsLocked(millis());
  if (g_alert_count == 0) {
    xSemaphoreGive(g_alert_mutex);
    return false;
  }

  int idx = (int)g_alert_head - 1;
  while (idx < 0) idx += ALERT_BUFFER_SIZE;
  idx %= ALERT_BUFFER_SIZE;
  *out = g_alerts[idx];
  xSemaphoreGive(g_alert_mutex);
  return true;
}


static void setImgLastError(const char *msg) {
  if (!msg) msg = "unknown";
  strncpy(g_img_last_error, msg, sizeof(g_img_last_error) - 1);
  g_img_last_error[sizeof(g_img_last_error) - 1] = '\0';
}

// ========================== JPEG 图片重组 ==========================
static void onJpegReady(const uint8_t *jpeg, uint32_t len, uint16_t width, uint16_t height) {
  if (!jpeg || len == 0 || len > MAX_JPEG_SIZE) return;

  if (len >= 2 && !(jpeg[0] == 0xFF && jpeg[1] == 0xD8)) {
    Serial.println("[WARN] JPEG header is not FF D8, but still cached");
  }

  if (xSemaphoreTake(g_jpeg_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    memcpy(g_latest_jpeg, jpeg, len);
    g_latest_len = len;
    g_latest_width = width;
    g_latest_height = height;
    g_latest_seq++;
    g_img_ready_count++;
    setImgLastError("ok");
    xSemaphoreGive(g_jpeg_mutex);
  }

  Serial.printf("[IMG] JPEG ready: %u bytes, %ux%u, seq=%u\n",
                (unsigned)len, width, height, (unsigned)g_latest_seq);
}

static void handleImgStart(uint8_t len, const uint8_t *data) {
  g_img_start_count++;

  if (len != 8) {
    Serial.printf("[WARN] IMG_START len error: %u\n", len);
    setImgLastError("IMG_START len error");
    g_img_drop_count++;
    g_img.active = false;
    return;
  }

  uint32_t total = u32le(&data[0]);
  uint16_t width = u16le(&data[4]);
  uint16_t height = u16le(&data[6]);

  if (total == 0 || total > MAX_JPEG_SIZE) {
    Serial.printf("[WARN] JPEG size invalid: %u, max=%u\n", (unsigned)total, (unsigned)MAX_JPEG_SIZE);
    setImgLastError("JPEG size invalid");
    g_img_drop_count++;
    g_img.active = false;
    return;
  }

  g_img.total_len = total;
  g_img.received_len = 0;
  g_img.width = width;
  g_img.height = height;
  g_img.next_seq = 0;
  g_img.active = true;
  setImgLastError("assembling");

  Serial.printf("[IMG] start total=%u width=%u height=%u\n", (unsigned)total, width, height);
}

static void handleImgData(uint8_t len, const uint8_t *data) {
  g_img_data_count++;
  if (!g_img.active) return;

  if (len < 3) {
    Serial.println("[WARN] IMG_DATA len too short");
    setImgLastError("IMG_DATA len too short");
    g_img_drop_count++;
    g_img.active = false;
    return;
  }

  // 文档写的是 seq_hi/seq_lo，但实际联调时有些 A1 代码会按小端发送。
  // 这里同时兼容大端和小端，避免序号 1 开始就被误判乱序导致整张图丢弃。
  uint16_t seq_be = u16be(&data[0]);
  uint16_t seq_le = u16le(&data[0]);
  uint16_t seq = seq_be;
  if (seq_be != g_img.next_seq && seq_le == g_img.next_seq) {
    seq = seq_le;
  }

  uint16_t chunk_len = len - 2;

  if (seq != g_img.next_seq) {
    Serial.printf("[WARN] IMG seq error: expect=%u got_be=%u got_le=%u, drop frame\n", g_img.next_seq, seq_be, seq_le);
    setImgLastError("IMG seq error");
    g_img_drop_count++;
    g_img.active = false;
    return;
  }

  if (g_img.received_len + chunk_len > g_img.total_len) {
    Serial.println("[WARN] IMG overflow, drop frame");
    setImgLastError("IMG overflow");
    g_img_drop_count++;
    g_img.active = false;
    return;
  }

  memcpy(&g_img.buf[g_img.received_len], data + 2, chunk_len);
  g_img.received_len += chunk_len;
  g_img.next_seq++;
}

static void handleImgEnd(uint8_t len, const uint8_t *data) {
  g_img_end_count++;
  if (!g_img.active) return;

  if (len != 2) {
    Serial.printf("[WARN] IMG_END len error: %u\n", len);
    setImgLastError("IMG_END len error");
    g_img_drop_count++;
    g_img.active = false;
    return;
  }

  if (g_img.received_len != g_img.total_len) {
    Serial.printf("[WARN] IMG len mismatch: expect=%u got=%u\n",
                  (unsigned)g_img.total_len, (unsigned)g_img.received_len);
    setImgLastError("IMG len mismatch");
    g_img_drop_count++;
    g_img.active = false;
    return;
  }

  uint16_t expected_be = u16be(&data[0]);
  uint16_t expected_le = u16le(&data[0]);
  uint16_t actual = 0;
  for (uint32_t i = 0; i < g_img.received_len; i++) {
    actual = (uint16_t)(actual + g_img.buf[i]);
  }

  if (expected_be != actual && expected_le != actual) {
    Serial.printf("[WARN] IMG checksum error: expect_be=0x%04X expect_le=0x%04X actual=0x%04X\n", expected_be, expected_le, actual);
    setImgLastError("IMG checksum error");
    g_img_drop_count++;
    g_img.active = false;
    return;
  }

  onJpegReady(g_img.buf, g_img.received_len, g_img.width, g_img.height);
  g_img.active = false;
}

static bool copyLatestJpeg(uint8_t *dst, uint32_t *out_len, uint32_t *out_seq, uint16_t *out_w, uint16_t *out_h) {
  bool ok = false;

  if (!dst || !out_len || !out_seq || !out_w || !out_h) return false;

  if (xSemaphoreTake(g_jpeg_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (g_latest_len > 0 && g_latest_len <= MAX_JPEG_SIZE) {
      memcpy(dst, g_latest_jpeg, g_latest_len);
      *out_len = g_latest_len;
      *out_seq = g_latest_seq;
      *out_w = g_latest_width;
      *out_h = g_latest_height;
      ok = true;
    }
    xSemaphoreGive(g_jpeg_mutex);
  }

  return ok;
}

// ========================== 检测/告警解析 ==========================
static void handleDetCount(uint8_t len, const uint8_t *data) {
  if (len != 1) {
    Serial.printf("[WARN] DET_COUNT len error: %u\n", len);
    return;
  }
  uint8_t count = data[0];
  Serial.printf("[DET] count=%u\n", count);

  // 追踪模式：记录本帧目标数量，后续多个 DET_RESULT 中只选最高置信度目标移动一次。
  trackHandleDetCount(count);

  // A1 明确告诉本帧无目标：认为危险已经消失，前端回到安全状态，重新允许下一次真实危险触发。
  if (count == 0) {
    clearAlertBuffer();
    clearVoiceState();
    clearDetectionState();
    clearAckAndDangerState("det_count_zero");
    trackResetDetCache();
  }
}

static bool pixelToCameraCoordinate(float pixel_u, float pixel_v,
                                    float *cam_x, float *cam_y) {
  if (!cam_x || !cam_y || !isfinite(pixel_u) || !isfinite(pixel_v)) return false;
  if (pixel_u < 0.0f || pixel_u >= CAMERA_IMAGE_WIDTH ||
      pixel_v < 0.0f || pixel_v >= CAMERA_IMAGE_HEIGHT) {
    return false;
  }

  *cam_x = (pixel_u - CAMERA_CX) / CAMERA_FX;
  *cam_y = (pixel_v - CAMERA_CY) / CAMERA_FY;
  return isfinite(*cam_x) && isfinite(*cam_y);
}

static void handleDetResult(uint8_t len, const uint8_t *data) {
  if (len != 10) {
    Serial.printf("[WARN] DET_RESULT len error: %u\n", len);
    return;
  }

  // 原协议：[class_id][score][center_x LE][center_y LE][width LE][height LE]
  uint8_t class_id = data[0];
  uint8_t score = data[1];
  uint16_t center_x = u16le(&data[2]);
  uint16_t center_y = u16le(&data[4]);
  uint16_t box_width = u16le(&data[6]);
  uint16_t box_height = u16le(&data[8]);
  float pixel_u = (float)center_x;
  float pixel_v = (float)center_y;

  float cam_x = 0.0f;
  float cam_y = 0.0f;
  if (!pixelToCameraCoordinate(pixel_u, pixel_v, &cam_x, &cam_y)) {
    Serial.printf("[WARN] DET pixel invalid: u=%.2f v=%.2f\n",
                  (double)pixel_u, (double)pixel_v);
    return;
  }

  saveLatestDetection(class_id, score, cam_x, cam_y);

  Serial.printf("[DET] class=%s(%u) score=%u%% center=(%u,%u) box=%ux%u cam=(%.4f,%.4f)\n",
                className(class_id), class_id, score,
                center_x, center_y, box_width, box_height,
                (double)cam_x, (double)cam_y);

  // 追踪模式：使用 ESP32 根据像素和相机内参换算出的相机坐标闭环控制。
  trackHandleDetResult(class_id, score, cam_x, cam_y, box_width, box_height);

  // 注意：DET_RESULT 只表示“检测框/坐标结果”，不能直接当成危险告警。
  // 之前这里做了兜底：class_id=1 会被当成 smoke_alarm 推给前端，
  // 所以即使 A1 没有发送 ALERT 烟雾危险包，前端也可能一直显示烟雾危险。
  // 现在改为：只有 CMD_ALERT_LOW/MID/HIGH 才触发前端告警和语音播报；
  // DET_RESULT 只用于保存坐标，供追踪模式/状态接口使用。
}

static void handleAlertFrame(uint8_t cmd, uint8_t len, const uint8_t *data) {
  if (len != 8) {
    Serial.printf("[WARN] ALERT len error: %u\n", len);
    return;
  }

  uint8_t event_type = data[0];
  uint8_t confidence = data[1];
  uint16_t duration_ms = u16le(&data[2]);
  uint16_t target_x = u16le(&data[4]);
  uint16_t target_y = u16le(&data[6]);

  Serial.printf("[ALERT-RX] cmd=0x%02X event=%s(%u) conf=%u%% duration=%u center=(%u,%u)\n",
                cmd, className(event_type), event_type, confidence,
                duration_ms, target_x, target_y);

  // 记录当前危险是否仍在持续。
  markDangerSeen(event_type);

  // 语音播报：这里只记录告警状态，真正发送给语音模块由 voiceTask 控制节奏。
  if (!isAlarmAcknowledged(event_type)) {
    voiceMarkAlarm(event_type);
  }

  // ALERT 包没有框宽高，这里填 0；坐标仍会放入 /api/alerts 的 target_x / target_y。
  pushAlert(event_type, confidence, target_x, target_y, 0, 0, true);
}

static void processValidFrame(uint8_t cmd, uint8_t len, const uint8_t *data) {
  switch (cmd) {
    case CMD_HEARTBEAT: {
      Serial.println("[UART] heartbeat received, send ACK");
      uint8_t ack = 0x01;
      uartSendFrame(CMD_ACK, &ack, 1);
      break;
    }

    case CMD_DET_COUNT:
      handleDetCount(len, data);
      break;

    case CMD_DET_RESULT:
      handleDetResult(len, data);
      break;

    case CMD_ALERT_LOW:
    case CMD_ALERT_MID:
    case CMD_ALERT_HIGH:
      handleAlertFrame(cmd, len, data);
      break;

    case CMD_IMG_START:
      handleImgStart(len, data);
      break;

    case CMD_IMG_DATA:
      handleImgData(len, data);
      break;

    case CMD_IMG_END:
      handleImgEnd(len, data);
      break;

    case CMD_SWITCH_ACK:
      handleSwitchAck(len, data);
      break;

    default:
      Serial.printf("[UART] unhandled CMD=0x%02X LEN=%u\n", cmd, len);
      break;
  }
}

// ========================== UART 状态机 ==========================
static int rx_state = 0;
static uint8_t rx_cmd = 0;
static uint8_t rx_len = 0;
static uint8_t rx_data_buf[UART_MAX_DATA_LEN];
static int rx_data_idx = 0;

static void clearA1UartRxBuffer() {
  int dropped = 0;
  while (Serial1.available() > 0) {
    Serial1.read();
    dropped++;
    if (dropped > 65535) break;
  }

  rx_state = 0;
  rx_cmd = 0;
  rx_len = 0;
  rx_data_idx = 0;
  memset(rx_data_buf, 0, sizeof(rx_data_buf));

  Serial.printf("[RESET] dropped %d bytes from A1 UART RX buffer and reset parser\n", dropped);
}

static void pollUart() {
  while (Serial1.available() > 0) {
    uint8_t b = (uint8_t)Serial1.read();

    switch (rx_state) {
      case 0:
        if (b == UART_FRAME_HEAD1) rx_state = 1;
        break;

      case 1:
        if (b == UART_FRAME_HEAD2) rx_state = 2;
        else if (b == UART_FRAME_HEAD1) rx_state = 1;
        else rx_state = 0;
        break;

      case 2:
        rx_cmd = b;
        rx_state = 3;
        break;

      case 3:
        rx_len = b;
        rx_data_idx = 0;
        if (rx_len > UART_MAX_DATA_LEN) {
          Serial.printf("[WARN] frame len too large: %u\n", rx_len);
          rx_state = 0;
        } else if (rx_len == 0) {
          rx_state = 5;
        } else {
          rx_state = 4;
        }
        break;

      case 4:
        rx_data_buf[rx_data_idx++] = b;
        if (rx_data_idx >= rx_len) rx_state = 5;
        break;

      case 5: {
        uint8_t expected = calcChecksum(rx_cmd, rx_len, rx_data_buf);
        if (b == expected) {
          g_uart_ok_frame_count++;
          processValidFrame(rx_cmd, rx_len, rx_data_buf);
        } else {
          g_uart_checksum_error_count++;
          Serial.printf("[ERROR] UART checksum failed CMD=0x%02X LEN=%u RX=0x%02X EXP=0x%02X\n",
                        rx_cmd, rx_len, b, expected);
        }
        rx_state = 0;
        break;
      }

      default:
        rx_state = 0;
        break;
    }
  }
}

static void uartRxTask(void *param) {
  (void)param;
  while (true) {
    pollUart();
    // 3Mbps 图像分包很密，串口任务不能睡太久。
    // 没数据时让出 CPU，有数据时下一轮马上继续读，减少图片包丢失概率。
    if (Serial1.available() == 0) {
      vTaskDelay(pdMS_TO_TICKS(1));
    } else {
      taskYIELD();
    }
  }
}

// ========================== HTTP 服务 ==========================
#define PART_BOUNDARY "esp32s3uartjpeg"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace; boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static void setCommonHeaders(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type, Authorization, *");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  httpd_resp_set_hdr(req, "Pragma", "no-cache");
}

static esp_err_t optionsHandler(httpd_req_t *req) {
  setCommonHeaders(req);
  httpd_resp_set_status(req, "204 No Content");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t rootHandler(httpd_req_t *req) {
  setCommonHeaders(req);
  httpd_resp_set_type(req, "text/html; charset=utf-8");

  String ip = WiFi.localIP().toString();
  String html = "<html><head><meta charset='utf-8'><title>ESP32S3 A1 Adapter</title></head><body>";
  html += "<h2>ESP32S3 A1 UART to Frontend Adapter</h2>";
  html += "<p>Stream redirect: <a href='http://" + ip + "/stream'>/stream</a></p>";
  html += "<p>Real Stream: <a href='http://" + ip + ":81/stream'>:81/stream</a></p>";
  html += "<p>Latest JPG: <a href='http://" + ip + "/latest.jpg'>/latest.jpg</a></p>";
  html += "<p>Alerts: <a href='http://" + ip + "/api/alerts?limit=1'>/api/alerts?limit=1</a></p>";
  html += "<p>Status: <a href='http://" + ip + "/api/status'>/api/status</a></p>";
  html += "<p>Ingest alert: POST http://" + ip + "/api/ingest/alert</p>";
  html += "<p style='color:#b00'>注意：当前 ESP32 提供 HTTP。如果你同学前端仍固定 HTTPS，需要把前端 src/config.ts 的 CAMERA_USE_HTTPS 改成 false。</p>";
  html += "<img src='http://" + ip + ":81/stream' style='max-width:100%;border:1px solid #ccc'>";
  html += "</body></html>";
  return httpd_resp_send(req, html.c_str(), html.length());
}


static esp_err_t streamRedirectHandler(httpd_req_t *req) {
  setCommonHeaders(req);
  String url = "http://" + WiFi.localIP().toString() + ":81/stream";
  httpd_resp_set_status(req, "302 Found");
  httpd_resp_set_hdr(req, "Location", url.c_str());
  return httpd_resp_send(req, "Stream moved to port 81", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t latestJpgHandler(httpd_req_t *req) {
  setCommonHeaders(req);

  uint32_t len = 0, seq = 0;
  uint16_t w = 0, h = 0;
  if (!copyLatestJpeg(g_http_copy, &len, &seq, &w, &h)) {
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_send(req, "No JPEG received yet", HTTPD_RESP_USE_STRLEN);
  }

  httpd_resp_set_type(req, "image/jpeg");
  return httpd_resp_send(req, (const char *)g_http_copy, len);
}

static esp_err_t streamHandler(httpd_req_t *req) {
  setCommonHeaders(req);
  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);

  Serial.println("[HTTP] client connected: /stream");

  char part_buf[96];
  uint32_t last_sent_seq = 0;

  while (true) {
    uint32_t len = 0, seq = 0;
    uint16_t w = 0, h = 0;

    bool got = copyLatestJpeg(g_http_copy, &len, &seq, &w, &h);
    if (!got || seq == last_sent_seq) {
      delay(20);
      continue;
    }

    esp_err_t res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));

    if (res == ESP_OK) {
      size_t header_len = snprintf(part_buf, sizeof(part_buf), STREAM_PART, (unsigned int)len);
      res = httpd_resp_send_chunk(req, part_buf, header_len);
    }

    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)g_http_copy, len);
    }

    if (res != ESP_OK) {
      Serial.println("[HTTP] client disconnected: /stream");
      break;
    }

    last_sent_seq = seq;
    Serial.printf("[HTTP] stream jpeg seq=%u len=%u size=%ux%u\n", (unsigned)seq, (unsigned)len, w, h);
  }

  return ESP_OK;
}

static esp_err_t apiAlertsHandler(httpd_req_t *req) {
  setCommonHeaders(req);
  httpd_resp_set_type(req, "application/json");

  int limit = parseLimitFromReq(req, 20);
  String json = buildAlertsArrayJson(limit);
  return httpd_resp_send(req, json.c_str(), json.length());
}

static esp_err_t apiAlertsLatestHandler(httpd_req_t *req) {
  setCommonHeaders(req);
  httpd_resp_set_type(req, "application/json");

  AlertEvent_t e;
  if (!copyLatestAlert(&e)) {
    return httpd_resp_send(req, "null", HTTPD_RESP_USE_STRLEN);
  }

  String json = alertToJson(e);
  return httpd_resp_send(req, json.c_str(), json.length());
}

static esp_err_t apiTelemetryHandler(httpd_req_t *req) {
  setCommonHeaders(req);
  httpd_resp_set_type(req, "application/json");

  float t = NAN;
  bool valid = false;
  if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    t = g_temperature_celsius;
    valid = g_temperature_valid && !isnan(g_temperature_celsius);
    xSemaphoreGive(g_sensor_mutex);
  }

  if (!valid) {
    return httpd_resp_send(req, "{\"temperature_celsius\":null}", HTTPD_RESP_USE_STRLEN);
  }

  String json = "{\"temperature_celsius\":" + String(t, 2) + "}";
  return httpd_resp_send(req, json.c_str(), json.length());
}

static esp_err_t apiStatusHandler(httpd_req_t *req) {
  setCommonHeaders(req);
  httpd_resp_set_type(req, "application/json");

  String ip = WiFi.localIP().toString();

  uint32_t img_len = 0;
  uint32_t img_seq = 0;
  uint16_t img_w = 0;
  uint16_t img_h = 0;

  if (xSemaphoreTake(g_jpeg_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    img_len = g_latest_len;
    img_seq = g_latest_seq;
    img_w = g_latest_width;
    img_h = g_latest_height;
    xSemaphoreGive(g_jpeg_mutex);
  }

  uint8_t alert_count = 0;
  if (xSemaphoreTake(g_alert_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    alert_count = g_alert_count;
    xSemaphoreGive(g_alert_mutex);
  }

  Detection_t det = {};
  bool has_det = false;
  if (xSemaphoreTake(g_detection_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    det = g_latest_detection;
    has_det = g_has_detection;
    xSemaphoreGive(g_detection_mutex);
  }

  SystemMode_t mode = getCurrentMode();
  uint8_t a1_model = modelIdForMode(mode);

  String json = "{";
  json += "\"status\":\"running\",";
  json += "\"service\":\"esp32s3-a1-http-center\",";
  json += "\"device_id\":\"" DEVICE_ID "\",";
  json += "\"ip\":\"" + ip + "\",";
  json += "\"ingest\":{\"uart\":true,\"http_alert\":true,\"http_sensor\":true},";
  json += "\"mode\":\"" + String(modeName(mode)) + "\",";
  json += "\"supported_modes\":[\"monitor\",\"search\",\"track\"],";
  json += "\"a1_model_id\":" + String(a1_model) + ",";
  json += "\"voice_enabled\":" + String(mode == SYS_MODE_MONITOR ? "true" : "false") + ",";
  json += "\"ptz_enabled\":" + String(mode == SYS_MODE_MONITOR ? "true" : "false") + ",";
  json += "\"track_auto_ptz_enabled\":" + String(g_track_enabled ? "true" : "false") + ",";
  uint8_t ack_mask = g_ack_alarm_mask;
  json += "\"buffer_size\":" + String(alert_count) + ",";
  json += "\"ack_alarm_mask\":" + String((unsigned int)ack_mask) + ",";
  json += "\"stream\":\"http://" + ip + ":81/stream\",";
  json += "\"stream_redirect\":\"http://" + ip + "/stream\",";
  json += "\"latest_jpg\":\"http://" + ip + "/latest.jpg\",";
  json += "\"alerts\":\"http://" + ip + "/api/alerts?limit=1\",";
  json += "\"image\":{\"seq\":" + String(img_seq) + ",\"bytes\":" + String(img_len) + ",\"width\":" + String(img_w) + ",\"height\":" + String(img_h) + "},";
  json += "\"image_debug\":{\"uart_ok_frames\":" + String((uint32_t)g_uart_ok_frame_count) + ",\"uart_checksum_errors\":" + String((uint32_t)g_uart_checksum_error_count) + ",\"img_start\":" + String((uint32_t)g_img_start_count) + ",\"img_data\":" + String((uint32_t)g_img_data_count) + ",\"img_end\":" + String((uint32_t)g_img_end_count) + ",\"img_ready\":" + String((uint32_t)g_img_ready_count) + ",\"img_drop\":" + String((uint32_t)g_img_drop_count) + ",\"last_error\":\"" + String(g_img_last_error) + "\"},";
  json += "\"latest_detection\":";
  if (has_det) {
    json += "{\"class_id\":" + String(det.class_id) + ",\"class_name\":\"" + String(className(det.class_id)) + "\",\"score\":" + String(det.score) + ",\"cam_x\":" + String(det.cam_x, 4) + ",\"cam_y\":" + String(det.cam_y, 4) + "}";
  } else {
    json += "null";
  }

  float temp = NAN;
  if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    temp = g_temperature_celsius;
    xSemaphoreGive(g_sensor_mutex);
  }
  json += ",\"temperature_celsius\":";
  if (isnan(temp)) json += "null";
  else json += String(temp, 2);
  json += ",\"ds18b20\":{";
  json += "\"data_pin\":" + String(DS18B20_DATA_PIN) + ",";
  json += "\"found\":" + String(g_ds18b20_found ? "true" : "false");
  json += "}";

  json += "}";

  return httpd_resp_send(req, json.c_str(), json.length());
}


// 给前端兜底用：有些前端会取 /api/state 或 /api/frontend。
// 这里直接复用 /api/status 的 JSON，里面已经包含 mode、image、latest_detection、temperature、stream 地址等。
static esp_err_t apiFrontendStateHandler(httpd_req_t *req) {
  return apiStatusHandler(req);
}

static String readRequestBody(httpd_req_t *req, size_t max_len = 2048) {
  String body;
  if (!req || req->content_len <= 0) return body;

  int remaining = req->content_len;
  if ((size_t)remaining > max_len) remaining = max_len;
  body.reserve(remaining + 1);

  char buf[129];
  while (remaining > 0) {
    int to_read = remaining < 128 ? remaining : 128;
    int ret = httpd_req_recv(req, buf, to_read);
    if (ret <= 0) break;
    buf[ret] = '\0';
    body += buf;
    remaining -= ret;
  }
  return body;
}

static bool jsonExtractString(const String &body, const char *key, String &out) {
  String pattern = String("\"") + key + "\"";
  int p = body.indexOf(pattern);
  if (p < 0) return false;
  int colon = body.indexOf(':', p + pattern.length());
  if (colon < 0) return false;
  int first = body.indexOf('"', colon + 1);
  if (first < 0) return false;
  int last = body.indexOf('"', first + 1);
  if (last < 0) return false;
  out = body.substring(first + 1, last);
  out.trim();
  return true;
}

static bool jsonExtractNumber(const String &body, const char *key, float &out) {
  String pattern = String("\"") + key + "\"";
  int p = body.indexOf(pattern);
  if (p < 0) return false;
  int colon = body.indexOf(':', p + pattern.length());
  if (colon < 0) return false;

  int i = colon + 1;
  while (i < (int)body.length() && (body[i] == ' ' || body[i] == '\t' || body[i] == '\r' || body[i] == '\n')) i++;
  int j = i;
  while (j < (int)body.length()) {
    char c = body[j];
    if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E') j++;
    else break;
  }
  if (j <= i) return false;
  out = body.substring(i, j).toFloat();
  return true;
}

static bool eventTypeToClassId(const String &event_type, uint8_t *class_id) {
  if (!class_id) return false;
  String e = event_type;
  e.toLowerCase();

  if (e == "fire_alarm" || e == "fire" || e == "0") {
    *class_id = CLASS_FIRE;
    return true;
  }
  if (e == "smoke_alarm" || e == "smoke" || e == "1") {
    *class_id = CLASS_SMOKE;
    return true;
  }
  if (e == "fall_detected" || e == "fallen_person" || e == "fall" || e == "3") {
    *class_id = CLASS_FALLEN;
    return true;
  }
  return false;
}

static uint8_t confidenceToPercent(float confidence) {
  if (isnan(confidence)) return 100;
  if (confidence <= 1.0f) confidence *= 100.0f;
  if (confidence < 0.0f) confidence = 0.0f;
  if (confidence > 100.0f) confidence = 100.0f;
  return (uint8_t)(confidence + 0.5f);
}

static esp_err_t apiIngestAlertHandler(httpd_req_t *req) {
  setCommonHeaders(req);
  httpd_resp_set_type(req, "application/json");

  String body = readRequestBody(req);
  String event_type;
  float conf = NAN;

  bool has_event = jsonExtractString(body, "event_type", event_type);
  jsonExtractNumber(body, "confidence", conf);

  uint8_t class_id = 255;
  if (!has_event || !eventTypeToClassId(event_type, &class_id)) {
    httpd_resp_set_status(req, "400 Bad Request");
    return httpd_resp_send(req, "{\"ok\":false,\"error\":\"event_type must be fire_alarm, smoke_alarm, or fall_detected\"}", HTTPD_RESP_USE_STRLEN);
  }

  uint8_t conf_percent = confidenceToPercent(conf);
  bool enabled = isMonitorMode();
  if (enabled) {
    // 允许用 HTTP /api/ingest/alert 测试告警时也触发语音播报。
    markDangerSeen(class_id);
    if (!isAlarmAcknowledged(class_id)) {
      voiceMarkAlarm(class_id);
    }
    pushAlert(class_id, conf_percent, 0, 0, 0, 0, true);
  }

  String json = "{\"ok\":true,\"mode\":\"";
  json += modeName(getCurrentMode());
  json += "\",\"alert_enabled\":";
  json += enabled ? "true" : "false";
  json += ",\"stored_event_type\":\"";
  json += frontendEventType(class_id);
  json += "\",\"confidence\":";
  json += String(((float)conf_percent) / 100.0f, 2);
  json += "}";
  return httpd_resp_send(req, json.c_str(), json.length());
}

static esp_err_t apiIngestSensorHandler(httpd_req_t *req) {
  setCommonHeaders(req);
  httpd_resp_set_type(req, "application/json");

  String body = readRequestBody(req);
  float t = NAN;
  bool ok = jsonExtractNumber(body, "temperature_celsius", t);
  if (!ok) ok = jsonExtractNumber(body, "temperature", t);

  if (!ok || isnan(t)) {
    httpd_resp_set_status(req, "400 Bad Request");
    return httpd_resp_send(req, "{\"ok\":false,\"error\":\"temperature_celsius or temperature is required\"}", HTTPD_RESP_USE_STRLEN);
  }

  updateTemperatureCelsius(t);

  String json = "{\"ok\":true,\"temperature_celsius\":";
  json += String(t, 2);
  json += "}";
  return httpd_resp_send(req, json.c_str(), json.length());
}


static esp_err_t apiPtzHandler(httpd_req_t *req) {
  setCommonHeaders(req);
  httpd_resp_set_type(req, "application/json");

  String body = readRequestBody(req);
  String message_type;
  float direction_value = NAN;

  jsonExtractString(body, "message_type", message_type);
  bool has_direction = jsonExtractNumber(body, "direction", direction_value);
  int direction = has_direction ? (int)(direction_value + 0.5f) : 0;

  if (!has_direction || direction < 1 || direction > 4) {
    httpd_resp_set_status(req, "400 Bad Request");
    return httpd_resp_send(req, "{\"ok\":false,\"error\":\"direction must be 1, 2, 3, or 4\"}", HTTPD_RESP_USE_STRLEN);
  }

  if (!modePtzEnabled()) {
    String json = "{\"ok\":true,\"ignored\":true,\"reason\":\"ptz disabled outside monitor mode\",\"mode\":\"";
    json += modeName(getCurrentMode());
    json += "\",\"received\":";
    if (body.length() > 0 && body[0] == '{') json += body;
    else json += "{\"direction\":" + String(direction) + "}";
    json += "}";
    return httpd_resp_send(req, json.c_str(), json.length());
  }

  // message_type 按前端文档为“移动”，这里不强制拒绝，避免中文编码差异导致误判。
  char action[16];
  bool ok = movePtzByDirection(direction, action, sizeof(action));
  if (!ok) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_send(req, "{\"ok\":false,\"error\":\"ptz mutex busy or servo write failed\"}", HTTPD_RESP_USE_STRLEN);
  }

  int pan = 0, tilt = 0;
  getPtzAngles(&pan, &tilt);

  String json = "{\"ok\":true,";
  json += "\"mode\":\"" + String(modeName(getCurrentMode())) + "\",";
  json += "\"received\":";
  if (body.length() > 0 && body[0] == '{') json += body;
  else json += "{\"direction\":" + String(direction) + "}";
  json += ",\"action\":\"" + String(action) + "\"";
  json += ",\"pan_angle\":" + String(pan);
  json += ",\"tilt_angle\":" + String(tilt);
  json += ",\"pan_pin\":" + String(PTZ_PAN_SERVO_PIN);
  json += ",\"tilt_pin\":" + String(PTZ_TILT_SERVO_PIN);
  json += "}";

  return httpd_resp_send(req, json.c_str(), json.length());
}

static esp_err_t apiModeHandler(httpd_req_t *req) {
  setCommonHeaders(req);
  httpd_resp_set_type(req, "application/json");

  String body = readRequestBody(req);
  String mode_str;
  if (!jsonExtractString(body, "mode", mode_str)) {
    httpd_resp_set_status(req, "400 Bad Request");
    return httpd_resp_send(req, "{\"ok\":false,\"error\":\"mode is required: monitor, search, or track\"}", HTTPD_RESP_USE_STRLEN);
  }

  SystemMode_t new_mode;
  if (!modeFromString(mode_str, &new_mode)) {
    httpd_resp_set_status(req, "400 Bad Request");
    return httpd_resp_send(req, "{\"ok\":false,\"error\":\"mode must be monitor, search, or track\"}", HTTPD_RESP_USE_STRLEN);
  }

  setWorkMode(new_mode, true);

  String json = "{\"ok\":true,";
  json += "\"received\":";
  if (body.length() > 0 && body[0] == '{') json += body;
  else json += "{\"mode\":\"" + String(modeName(new_mode)) + "\"}";
  json += ",\"mode\":\"" + String(modeName(new_mode)) + "\"";
  json += ",\"a1_model_id\":" + String(modelIdForMode(new_mode));
  json += ",\"voice_enabled\":" + String(new_mode == SYS_MODE_MONITOR ? "true" : "false");
  json += ",\"ptz_enabled\":" + String(new_mode == SYS_MODE_MONITOR ? "true" : "false");
  json += "}";
  return httpd_resp_send(req, json.c_str(), json.length());
}

static esp_err_t apiSearchHandler(httpd_req_t *req) {
  setCommonHeaders(req);
  httpd_resp_set_type(req, "application/json");

  String body = readRequestBody(req);
  if (getCurrentMode() != SYS_MODE_SEARCH) {
    // 即使浏览器漏发了 /api/mode，也保证“查找”不会误启用自动追踪。
    setWorkMode(SYS_MODE_SEARCH, true);
  }

  uint32_t image_seq = 0;
  uint32_t image_bytes = 0;
  if (xSemaphoreTake(g_jpeg_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    image_seq = g_latest_seq;
    image_bytes = g_latest_len;
    xSemaphoreGive(g_jpeg_mutex);
  }

  String json = "{\"ok\":true,\"mode\":\"search\"";
  if (body.length() > 0 && body[0] == '{') {
    json += String(",\"received\":") + body;
  }
  json += ",\"image_ready\":" + String(image_bytes > 0 ? "true" : "false");
  json += ",\"image_seq\":" + String(image_seq);
  json += ",\"image_bytes\":" + String(image_bytes);
  json += ",\"frame_path\":\"/api/search/frame\"}";

  return httpd_resp_send(req, json.c_str(), json.length());
}

static esp_err_t apiResetSystemHandler(httpd_req_t *req) {
  setCommonHeaders(req);
  httpd_resp_set_type(req, "application/json");

  // 读掉 body，避免 HTTP 连接里还有未消费数据；内容本身不影响重置。
  String body = readRequestBody(req);
  (void)body;

  resetFrontendSystemState("http_reset");

  String json = "{\"ok\":true";
  json += ",\"mode\":\"" + String(modeName(getCurrentMode())) + "\"";
  json += ",\"alerts_cleared\":true";
  json += ",\"voice_cleared\":true";
  json += ",\"detection_cleared\":true";
  json += ",\"safe_hold_ms\":" + String(ALERT_RESET_SAFE_HOLD_MS);
  json += ",\"alert_ttl_ms\":" + String(ALERT_FRONTEND_TTL_MS);
  json += ",\"ack_clear_ms\":" + String(ALERT_ACK_CLEAR_MS);
  json += ",\"ack_alarm_mask\":" + String((unsigned int)g_ack_alarm_mask);
  json += "}";
  return httpd_resp_send(req, json.c_str(), json.length());
}

static esp_err_t apiPostFallbackHandler(httpd_req_t *req) {
  setCommonHeaders(req);
  httpd_resp_set_type(req, "application/json");

  String uri = req->uri ? String(req->uri) : String("");
  String body = readRequestBody(req);
  String uri_lower = uri;
  uri_lower.toLowerCase();

  // 兼容不同前端写法：只要 POST 的路径里包含 reset / clear / refresh，就按“重置系统状态”处理。
  if (uri_lower.indexOf("reset") >= 0 || uri_lower.indexOf("clear") >= 0 || uri_lower.indexOf("refresh") >= 0) {
    resetFrontendSystemState("http_fallback_reset");
    String json = "{\"ok\":true,\"fallback\":true,\"uri\":\"" + uri + "\",\"alerts_cleared\":true}";
    return httpd_resp_send(req, json.c_str(), json.length());
  }

  String json;
  if (body.length() > 0 && body[0] == '{') {
    json = String("{\"ok\":true,\"fallback\":true,\"uri\":\"") + uri + "\",\"received\":" + body + "}";
  } else {
    json = String("{\"ok\":true,\"fallback\":true,\"uri\":\"") + uri + "\"}";
  }
  return httpd_resp_send(req, json.c_str(), json.length());
}

static void registerGet(httpd_handle_t server, const char *uri, esp_err_t (*handler)(httpd_req_t *)) {
  httpd_uri_t item = {};
  item.uri = uri;
  item.method = HTTP_GET;
  item.handler = handler;
  httpd_register_uri_handler(server, &item);
}

static void registerPost(httpd_handle_t server, const char *uri, esp_err_t (*handler)(httpd_req_t *)) {
  httpd_uri_t item = {};
  item.uri = uri;
  item.method = HTTP_POST;
  item.handler = handler;
  httpd_register_uri_handler(server, &item);
}

static void registerOptions(httpd_handle_t server) {
  httpd_uri_t item = {};
  item.uri = "/*";
  item.method = HTTP_OPTIONS;
  item.handler = optionsHandler;
  httpd_register_uri_handler(server, &item);
}

static void startApiServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;
  config.max_uri_handlers = 64;
  config.stack_size = 16384;
  config.lru_purge_enable = true;
  config.uri_match_fn = httpd_uri_match_wildcard;

  esp_err_t ret = httpd_start(&g_api_server, &config);
  if (ret != ESP_OK) {
    Serial.printf("[ERR] API HTTP server start failed: 0x%x\n", ret);
    return;
  }

  registerGet(g_api_server, "/", rootHandler);

  // 关键修复：80 端口不要直接跑 MJPEG 死循环，否则 /api/status 和 /api/alerts 会被堵住。
  // 浏览器访问 http://IP/stream 时，这里快速 302 到 81 端口，真正的视频流由第二个 HTTP server 处理。
  registerGet(g_api_server, "/stream", streamRedirectHandler);
  registerGet(g_api_server, "/api/stream", streamRedirectHandler);
  registerGet(g_api_server, "/video", streamRedirectHandler);
  registerGet(g_api_server, "/video_feed", streamRedirectHandler);
  registerGet(g_api_server, "/mjpeg", streamRedirectHandler);

  // 单张图片接口做多路径兼容，避免前端写成 /api/image、/image.jpg、/snapshot.jpg 时拿不到图。
  registerGet(g_api_server, "/latest.jpg", latestJpgHandler);
  registerGet(g_api_server, "/capture", latestJpgHandler);
  registerGet(g_api_server, "/image", latestJpgHandler);
  registerGet(g_api_server, "/image.jpg", latestJpgHandler);
  registerGet(g_api_server, "/frame.jpg", latestJpgHandler);
  registerGet(g_api_server, "/snapshot.jpg", latestJpgHandler);
  registerGet(g_api_server, "/api/latest.jpg", latestJpgHandler);
  registerGet(g_api_server, "/api/latest", latestJpgHandler);
  registerGet(g_api_server, "/api/capture", latestJpgHandler);
  registerGet(g_api_server, "/api/image", latestJpgHandler);
  registerGet(g_api_server, "/api/frame", latestJpgHandler);
  registerGet(g_api_server, "/api/frame.jpg", latestJpgHandler);
  registerGet(g_api_server, "/api/search/frame", latestJpgHandler);
  registerGet(g_api_server, "/api/snapshot", latestJpgHandler);
  registerGet(g_api_server, "/api/snapshot.jpg", latestJpgHandler);

  registerGet(g_api_server, "/api/alerts", apiAlertsHandler);
  registerGet(g_api_server, "/api/alerts/latest", apiAlertsLatestHandler);
  registerGet(g_api_server, "/api/telemetry", apiTelemetryHandler);
  registerGet(g_api_server, "/api/status", apiStatusHandler);
  registerGet(g_api_server, "/api/state", apiFrontendStateHandler);
  registerGet(g_api_server, "/api/frontend", apiFrontendStateHandler);
  registerGet(g_api_server, "/api/data", apiFrontendStateHandler);
  // 也支持浏览器直接 GET 测试重置接口。
  registerGet(g_api_server, "/api/reset", apiResetSystemHandler);
  registerGet(g_api_server, "/api/alerts/reset", apiResetSystemHandler);

  registerPost(g_api_server, "/api/ingest/alert", apiIngestAlertHandler);
  registerPost(g_api_server, "/api/ingest/sensor", apiIngestSensorHandler);

  registerPost(g_api_server, "/api/ptz", apiPtzHandler);
  registerPost(g_api_server, "/api/mode", apiModeHandler);
  registerPost(g_api_server, "/api/search", apiSearchHandler);

  // 前端“重置系统状态”按钮可调用下面任一接口。/api/track/refresh 也改为清空状态，兼容现有前端。
  registerPost(g_api_server, "/api/reset", apiResetSystemHandler);
  registerPost(g_api_server, "/api/system/reset", apiResetSystemHandler);
  registerPost(g_api_server, "/api/alerts/reset", apiResetSystemHandler);
  registerPost(g_api_server, "/api/track/refresh", apiResetSystemHandler);

  // 兜底：如果前端重置按钮路径不是上面几个，例如 /api/xxx/reset，也能触发 ESP32 侧清空。
  registerPost(g_api_server, "/api/*", apiPostFallbackHandler);

  registerOptions(g_api_server);

  Serial.println("[OK] API HTTP server started on port 80");
}

static void startStreamServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;
  config.ctrl_port = 32769;
  config.max_uri_handlers = 16;
  config.stack_size = 16384;
  config.lru_purge_enable = true;
  config.uri_match_fn = httpd_uri_match_wildcard;

  esp_err_t ret = httpd_start(&g_stream_server, &config);
  if (ret != ESP_OK) {
    Serial.printf("[ERR] STREAM HTTP server start failed: 0x%x\n", ret);
    return;
  }

  registerGet(g_stream_server, "/", streamHandler);
  registerGet(g_stream_server, "/stream", streamHandler);
  registerGet(g_stream_server, "/api/stream", streamHandler);
  registerGet(g_stream_server, "/video", streamHandler);
  registerGet(g_stream_server, "/video_feed", streamHandler);
  registerGet(g_stream_server, "/mjpeg", streamHandler);
  registerOptions(g_stream_server);

  Serial.println("[OK] STREAM HTTP server started on port 81");
}

static void startHttpServer() {
  startApiServer();
  startStreamServer();
}

// ========================== WiFi ==========================
static void setupTimeByNtp() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
}

static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("[WiFi] connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("[OK] WiFi connected");
  Serial.print("[OK] ESP32S3 IP: ");
  Serial.println(WiFi.localIP());

  setupTimeByNtp();

  Serial.println("================ 前端访问地址 ================");
  Serial.print("页面测试:       http://"); Serial.println(WiFi.localIP());
  Serial.print("图片视频流:     http://"); Serial.print(WiFi.localIP()); Serial.println(":81/stream");
  Serial.print("视频重定向:     http://"); Serial.print(WiFi.localIP()); Serial.println("/stream");
  Serial.print("最新单张图片:   http://"); Serial.print(WiFi.localIP()); Serial.println("/latest.jpg");
  Serial.print("兼容图片接口:   http://"); Serial.print(WiFi.localIP()); Serial.println("/api/image");
  Serial.print("统一状态接口:   http://"); Serial.print(WiFi.localIP()); Serial.println("/api/state");
  Serial.print("告警接口:       http://"); Serial.print(WiFi.localIP()); Serial.println("/api/alerts?limit=1");
  Serial.print("温度接口:       http://"); Serial.print(WiFi.localIP()); Serial.println("/api/telemetry");
  Serial.print("状态接口:       http://"); Serial.print(WiFi.localIP()); Serial.println("/api/status");
  Serial.print("云台接口:       http://"); Serial.print(WiFi.localIP()); Serial.println("/api/ptz  direction=1上 2下 3左 4右");
  Serial.print("模式接口:       http://"); Serial.print(WiFi.localIP()); Serial.println("/api/mode  mode=monitor/track");
  Serial.print("HTTP告警上报:   http://"); Serial.print(WiFi.localIP()); Serial.println("/api/ingest/alert");
  Serial.println("如果同学前端仍固定 HTTPS，请把 src/config.ts 的 CAMERA_USE_HTTPS 改成 false；视频流可填 IP 或 IP:81");
  Serial.println("=============================================");
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("ESP32S3 A1 UART -> Browser Frontend Adapter Start [monitor/track mode]");
  Serial.printf("[INFO] UART RX=IO%d TX=IO%d baud=%d\n", UART_RXD_PIN, UART_TXD_PIN, UART_BAUD_RATE);
  Serial.printf("[INFO] PTZ pan servo=IO%d tilt servo=IO%d step=%d degree\n", PTZ_PAN_SERVO_PIN, PTZ_TILT_SERVO_PIN, PTZ_STEP_ANGLE);
  Serial.printf("[INFO] TRACK target=(%.2f,%.2f), deadband=%.0fx%.0f px, step=%d..%d degree, filter=%.2f\n",
                (double)PTZ_TRACK_TARGET_CAM_X, (double)PTZ_TRACK_TARGET_CAM_Y,
                (double)PTZ_TRACK_DEADBAND_X_PIXELS, (double)PTZ_TRACK_DEADBAND_Y_PIXELS,
                PTZ_TRACK_MIN_STEP_ANGLE, PTZ_TRACK_MAX_STEP_ANGLE,
                (double)PTZ_TRACK_FILTER_ALPHA);
  Serial.printf("[INFO] Voice TX=IO%d RX=IO%d baud=%d\n", VOICE_TXD_PIN, VOICE_RXD_PIN, VOICE_UART_BAUD_RATE);
  Serial.printf("[INFO] MAX_JPEG_SIZE=%u bytes\n", (unsigned)MAX_JPEG_SIZE);

  if (!initBuffers()) {
    Serial.println("[FATAL] buffer init failed. Check PSRAM config in platformio.ini");
    while (true) delay(1000);
  }

  initDs18b20();
  xTaskCreatePinnedToCore(ds18b20Task, "ds18b20", 4096, NULL, 1, NULL, 0);
  Serial.println("[OK] DS18B20 temperature task started");

  initPtzServo();

  Serial1.setRxBufferSize(UART_RX_BUF_SIZE);
  Serial1.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RXD_PIN, UART_TXD_PIN);
  Serial.println("[OK] UART1 started");

  // 整个系统默认进入监控模式，并通知 A1 切到对应模型。
  setWorkMode(SYS_MODE_MONITOR, true);

  Serial2.setRxBufferSize(256);
  Serial2.begin(VOICE_UART_BAUD_RATE, SERIAL_8N1, VOICE_RXD_PIN, VOICE_TXD_PIN);
  Serial.printf("[OK] Voice UART2 started: TX=IO%d RX=IO%d baud=%d\n",
                VOICE_TXD_PIN, VOICE_RXD_PIN, VOICE_UART_BAUD_RATE);

  xTaskCreatePinnedToCore(uartRxTask, "uart_rx", 8192, NULL, 5, NULL, 1);
  Serial.println("[OK] UART RX task started");

  xTaskCreatePinnedToCore(voiceTask, "voice_tx", 4096, NULL, 3, NULL, 0);
  Serial.println("[OK] Voice task started");

  connectWiFi();
  startHttpServer();
}

void loop() {
  trackServoWatchdog();

  static uint32_t last_wifi_check = 0;
  if (millis() - last_wifi_check > 10000) {
    last_wifi_check = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] reconnecting...");
      WiFi.reconnect();
    }
  }

  delay(1);
}
