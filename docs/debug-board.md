# GD32C113 调试板接入

适用硬件：`UNIUBI-6001-MOTOR-USBCAN rev1.0`，MCU GD32C113CBT6。调试板需运行支持以下串口协议的固件。

```text
应用调用电机API
  → SDK生成64字节CAN负载
  → u6913_board_send封装81字节A8串口帧
  → 电脑串口驱动 → USB/CH343P → GD32C113 → CAN FD → 电机

电机反馈 → 调试板AC推送 → u6913_board_feed → CAN帧 → u6913_parse_feedback
板级错误 → 调试板A0回复 → u6913_board_feed → 独立板级事件
```

SDK 运行在主机应用中，通过串口与调试板通信。调试板负责串口与 CAN FD 转发；接入时将核心库和串口适配实现加入主机工程即可，无需将 SDK 烧入调试板。

## 串口与链路

- 串口：**4,000,000 baud、8N1、无硬件流控**。应用打开串口时须配置为此速率。
- 下行通道推荐1，兼容0；两者均指板上唯一CAN0。上行通道为1，不是电机ID。
- 电机链路：ISO CAN FD，标准11位ID，BRS，仲裁1 Mbps、数据5 Mbps。
- 常规请求内层64字节；外层A8头6字节 + CAN数据域73字节 + CRC2字节，共81字节。
- 外层长度、CAN ID及CRC大端；内层电机数据小端。CRC为CRC-16/MODBUS，但线上发送高字节在前。
- 帧无固定尾字节，不能用A5/5A、换行或单次串口读取长度作为分帧依据。

## 最小发送接入

将核心和 `controllers/uniubi6913_board.c` 加入工程，实现一个串口完整写出函数：

```c
#include "uniubi6913_board.h"
static u6913_board_port_t board;
static u6913_motor_t motor;

/* 返回0前，必须完整写出或复制这length字节；应用实现写超时。 */
extern int serial_write_all(void *serial, const uint8_t *data, size_t length);

int connect_motor(void *serial, uint8_t actual_motor_id)
{
    int rc = u6913_board_init(&board, 1, serial_write_all, serial);
    if (rc != U6913_OK) return rc;
    rc = u6913_init(&motor, actual_motor_id, u6913_board_send, &board);
    if (rc != U6913_OK) return rc;
    return u6913_request_version(&motor);
}
```

串口的打开/关闭、4 Mbps配置、读取线程和超时由应用或操作系统库提供。SDK不选择设备、不自动打开串口。POSIX/Windows的write接口可能返回部分字节，不能把“写入大于0”直接当作成功；应循环写到全部完成或超时。调试板端口对象与串口句柄须保持有效。

同一串口共用一个board实例分配seq；多电机句柄可指向同一个board。并发调用必须由应用在完整发送期间加锁，避免帧交错。SDK没有隐藏的线程、自动重发或自动使能。

## 接收与错误处理

```c
static u6913_board_parser_t parser = {0};

/* 应用实现这三个处理函数：板级回复、通信异常、匹配后的电机反馈。 */
extern void on_board_reply(uint8_t command_class, uint8_t seq, uint8_t error);
extern void on_transport_error(int error);
extern void on_motor_feedback(const u6913_feedback_t *feedback);

void on_serial_bytes(const uint8_t *bytes, size_t count)
{
    size_t i;
    for (i = 0; i < count; ++i) {
        u6913_board_event_t event;
        int rc = u6913_board_feed(&parser, bytes[i], &event);
        if (rc < 0) { on_transport_error(rc); continue; }
        if (rc == 0) continue;  /* 包括CRC尚未收齐的半帧 */
        if (event.kind == U6913_BOARD_REPLY) {
            on_board_reply(event.command_class, event.sequence, event.error_code);
        } else if (event.frame.can_id == motor.joint_id) {
            u6913_feedback_t feedback;
            rc = u6913_parse_feedback(motor.joint_id, event.frame.can_id,
                                      event.frame.data, event.frame.len, &feedback);
            if (rc == U6913_OK) on_motor_feedback(&feedback);
            else on_transport_error(rc);
        }
    }
}
```

`feed`返回1才有新事件；返回0不表示“电机没响应”或“CRC通过”。超时由应用计时；超时应清除未完成事务，并调用 `u6913_board_parser_reset(&parser)` 丢弃半帧。丢字节/损坏长度可能导致后续字节落入坏帧，必要时清理串口缓存并通过只读查询恢复同步。数据和CRC内出现A8/A0/AC等值都不能截断当前帧。

当前固件A1/03入队成功**没有A0成功应答**。A0为板级处理结果，其非零错误码需单独处理（4=忙、5=非法参数等）。AC来自电机接收推送，seq固定为0，不能与下行seq关联；应检查CAN ID、内层设备ID、命令及时间窗口。`u6913_request_*`返回成功只表示串口写入成功。

调试板A1/05定频测试运行时会统计CAN接收而不逐帧推送，与正常命令/反馈事务不能混用。本版SDK未启用板级定频测试。

## 报文示例与查询

构建后运行 `uniubi6913_board_demo`，可打印 ID6 的版本和标准状态查询帧。该示例不访问串口；连接设备时，请使用上面的收发回调接入应用，或运行 [只读检查脚本](../README.md#usb-调试板只读检查)。

SN 查询使用 64 字节 CAN 载荷，对应 81 字节 A8 请求；32 字节身份反馈对应 49 字节 AC 包。自动标定和标定状态查询使用相同六槽请求格式；E1 回复应按 `U6913_FEEDBACK_CALIBRATION` 读取完成标志，不能按普通状态解析。
