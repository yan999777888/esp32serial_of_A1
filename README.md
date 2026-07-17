# ESP32-S3 A1 UART to Browser Frontend Adapter

ESP32-S3 作为 **A1 上位机** 与 **浏览器前端** 之间的 HTTP 中枢。

A1 通过 3Mbps 高速 UART 发送 JPEG 视频流、检测结果和目标告警，ESP32-S3 解析后通过 WiFi 提供 MJPEG 视频流和 REST API 供浏览器访问。

## 功能

- **视频流** — 接收 A1 的 JPEG 分包，重组为完整图片，支持 MJPEG 实时流和单张抓取
- **目标检测** — 解析 fire / smoke / person / fallen_person 检测结果（像素坐标 + 检测框）
- **告警推送** — fire_alarm / smoke_alarm / fall_detected 三级告警，带冷却和防抖
- **语音播报** — 通过 UART2 驱动语音模块，火灾/烟雾/摔倒自动语音提示
- **云台控制（PTZ）** — 手动方向控制 + 追踪模式下自动闭环跟踪
- **DS18B20 温度采集** — 实时温度通过 API 返回前端
- **无线热点** — ESP32-S3 连接 WiFi，浏览器直接访问 HTTP 接口

## 硬件接线

| A1 上位机 | ESP32-S3 | 说明 |
|-----------|----------|------|
| A1_TX | **GPIO6** | UART1 RX，3Mbps |
| A1_RX | **GPIO7** | UART1 TX |
| GND | **GND** | 必须共地 |

| DS18B20 | ESP32-S3 |
|---------|----------|
| VCC | **3.3V** |
| GND | **GND** |
| DQ | **GPIO4** |

| 舵机 | ESP32-S3 |
|------|----------|
| 左右 360° 信号线 | **GPIO16** (PWM, 50Hz) |
| 上下 180° 信号线 | **GPIO17** (PWM, 50Hz) |
| 电源 | 外部 5V（共地） |

| 语音模块 | ESP32-S3 |
|---------|----------|
| RX | **GPIO18** (UART2 TX, 115200) |
| TX | **GPIO15** (UART2 RX，可不接) |
| GND | **GND** |

## 快速开始

### 1. 配置 WiFi

修改 `src/main.cpp`：

```cpp
const char *WIFI_SSID     = "你的WiFi名称";
const char *WIFI_PASSWORD = "你的WiFi密码";
```

### 2. 编译上传

使用 PlatformIO：

```bash
pio run -t upload
pio device monitor
```

### 3. 访问

启动后串口会打印访问地址：

| 接口 | 地址 |
|------|------|
| 测试页面 | `http://<IP>/` |
| MJPEG 视频流 | `http://<IP>:81/stream` |
| 最新图片 | `http://<IP>/latest.jpg` |
| 告警 | `http://<IP>/api/alerts?limit=1` |
| 状态 | `http://<IP>/api/status` |

## API 接口

### GET 接口

| 路径 | 说明 |
|------|------|
| `/` | 测试页面 |
| `/stream` | 重定向到 81 端口 MJPEG 流 |
| `/latest.jpg` | 最新 JPEG 图片 |
| `/api/alerts?limit=N` | 告警列表 |
| `/api/telemetry` | DS18B20 温度 |
| `/api/status` | 系统状态（含图片、检测、温度、模式） |

### POST 接口

| 路径 | 参数 | 说明 |
|------|------|------|
| `/api/mode` | `{"mode":"monitor"}` / `{"mode":"track"}` / `{"mode":"search"}` | 切换模式 |
| `/api/ptz` | `{"direction":1}` (1=上, 2=下, 3=左, 4=右) | 云台控制 |
| `/api/reset` | - | 重置系统状态 |
| `/api/ingest/alert` | `{"event_type":"fire_alarm","confidence":0.95}` | HTTP 告警注入 |

## UART 协议

帧格式：`AA 55 CMD LEN DATA CHECKSUM`

| 命令 | 说明 |
|------|------|
| `0x01` | 心跳 |
| `0x10` | 检测结果 (10字节) |
| `0x11` | 检测数量 |
| `0x20-0x22` | 告警 (低/中/高) |
| `0x30` | 图片帧开始 |
| `0x31` | 图片数据分包 |
| `0x32` | 图片帧结束 |

检测结果帧（10 字节）：

```
[class_id][score][center_x LE][center_y LE][width LE][height LE]
```

## 模式说明

| 模式 | 告警 | 语音 | 手动云台 | 自动跟踪 |
|------|------|------|---------|---------|
| monitor | 开启 | 开启 | 开启 | 关闭 |
| search | 关闭 | 关闭 | 关闭 | 关闭 |
| track | 关闭 | 关闭 | 关闭 | 开启 |

## 项目结构

```
├── src/
│   ├── main.cpp          # 主程序
│   └── uart_protocol.h   # UART 协议定义
├── platformio.ini        # PlatformIO 配置
└── README.md
```

## 硬件平台

- **MCU**: ESP32-S3 (Xtensa LX7 dual-core)
- **框架**: Arduino (PlatformIO)
- **PSRAM**: 8MB Octal PSRAM (QIO+OPI)
