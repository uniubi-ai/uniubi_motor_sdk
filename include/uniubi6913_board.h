#ifndef UNIUBI6913_BOARD_H
#define UNIUBI6913_BOARD_H
#include "uniubi6913.h"

#ifdef __cplusplus
extern "C" {
#endif

#define U6913_BOARD_BAUDRATE 4000000u
#define U6913_BOARD_PACKET_MAX 128u
#define U6913_BOARD_REQUEST_SIZE 81u

/* 主机侧 USB 串口封装；不依赖 GD32 或操作系统头文件。
 * write 必须复制或完整写出 length 字节后返回0；部分写入/超时返回非0。
 * 不得保存 data 指针。部分写入后应用应恢复串口流同步，不要直接重发。 */
typedef int (*u6913_board_write_fn)(void *user, const uint8_t *data, size_t length);
typedef struct {
    u6913_board_write_fn write;
    void *user;
    uint8_t channel;       /* 0/1均对应物理CAN0；推荐1 */
    uint8_t next_sequence; /* 单个串口由同一实例/任务分配序号，应用负责锁 */
} u6913_board_port_t;

typedef struct {
    size_t length;
    uint8_t data[U6913_BOARD_PACKET_MAX];
} u6913_board_packet_t;

typedef enum { U6913_BOARD_CAN_RECEIVED, U6913_BOARD_REPLY } u6913_board_event_kind_t;
typedef struct {
    u6913_board_event_kind_t kind;
    uint8_t command_class;
    uint8_t sequence;      /* A0回显请求seq；AC固定0，不能用于匹配电机事务 */
    uint8_t error_code;    /* A0板级错误码，与电机故障无关 */
    uint8_t channel;       /* 仅CAN事件有效，上行通常为1 */
    u6913_frame_t frame;   /* 仅CAN事件有效，仅返回标准 CAN FD 数据帧 */
    uint8_t reply_length; /* A0数据域，其他事件为0 */
    uint8_t reply_data[U6913_BOARD_PACKET_MAX];
} u6913_board_event_t;

typedef struct {
    size_t used;
    size_t expected;
    uint8_t data[U6913_BOARD_PACKET_MAX];
} u6913_board_parser_t;

int u6913_board_init(u6913_board_port_t *port, uint8_t channel,
                     u6913_board_write_fn write, void *user);
/* 作为 u6913_init 的发送回调。只表示写串口成功，不等待 A0/AC。 */
int u6913_board_send(void *port, const u6913_frame_t *frame);
/* 固定启用CRC，外层大端，CAN负载原样保留。失败时out不变。 */
int u6913_board_wrap(const u6913_frame_t *frame, uint8_t channel, uint8_t sequence,
                     u6913_board_packet_t *out);
/* 一次解析一条完整A0或AC；len必须恰好等于帧长，失败时out不变。
 * A0非零错误码也会作为有效事件返回，由应用检查event.error_code。 */
int u6913_board_decode(const uint8_t *data, size_t len, u6913_board_event_t *out);
void u6913_board_parser_reset(u6913_board_parser_t *parser);
/* 逐字节输入：1=输出一条事件，0=等待/跳过帧前噪声，负值=丢弃坏帧。
 * 不按负载中的A0/AC字节切帧。应用发现半帧超时后须显式reset；
 * 损坏长度可能吞掉后续字节，reset后应重新发送只读查询恢复同步。
 * parser初始化为{0}。out仅在返回1时更新。 */
int u6913_board_feed(u6913_board_parser_t *parser, uint8_t byte, u6913_board_event_t *out);

#ifdef __cplusplus
}
#endif
#endif
