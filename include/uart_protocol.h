/*
 * @Filename: include/uart_protocol.h
 * @Description: ESP32-S3 侧的 UART 协议定义 (与 A1 对应)
 */
#pragma once

#include <stdint.h>

// 协议常量
#define UART_FRAME_HEAD1        0xAA
#define UART_FRAME_HEAD2        0x55
#define UART_MAX_DATA_LEN       24

// 命令字
typedef enum {
    CMD_HEARTBEAT       = 0x01,   // 心跳包
    CMD_DET_RESULT      = 0x10,   // 检测结果 (坐标框)
    CMD_DET_COUNT       = 0x11,   // 检测数量
    CMD_ALERT_LOW       = 0x20,   // [提醒级] 业务报警
    CMD_ALERT_MID       = 0x21,   // [报警级] 业务报警
    CMD_ALERT_HIGH      = 0x22,   // [紧急级] 业务报警
    CMD_IMG_START       = 0x30,   // 图像帧开始
    CMD_IMG_DATA        = 0x31,   // 图像数据分包
    CMD_IMG_END         = 0x32,   // 图像帧结束
    CMD_ACK             = 0x80,   // 应答
    CMD_SET_THRESHOLD   = 0x81    // 阈值设置
} UartCmd_t;

// 视觉框结构体 (10字节)
#pragma pack(push, 1)
typedef struct {
    uint8_t  class_id;      // 类别ID
    uint8_t  score;         // 置信度 (0~100)
    uint16_t center_x;      // 中心点X坐标
    uint16_t center_y;      // 中心点Y坐标
    uint16_t width;         // 宽度
    uint16_t height;        // 高度
} UartDetResult_t;

// 业务逻辑报警结构体 (8字节)
typedef struct {
    uint8_t  event_type;    // 事件类型
    uint8_t  confidence;    // 置信度
    uint16_t duration_ms;   // 持续时间
    uint16_t target_x;      // 目标坐标 X
    uint16_t target_y;      // 目标坐标 Y
} UartAlertPayload_t;

// 图像帧开始结构体 (8字节)
typedef struct {
    uint32_t total_len;     // JPEG数据总长度
    uint16_t width;         // 图像宽度
    uint16_t height;        // 图像高度
} UartImgStart_t;
#pragma pack(pop)
