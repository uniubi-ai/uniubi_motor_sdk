#include "uniubi6913_board.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "line %d: %s\n", __LINE__, #x); exit(1); } } while (0)

/* 独立测试向量CRC实现；额外通过真实板级源码做互操作验证。 */
static void add_crc(uint8_t *data, size_t len)
{
    unsigned crc = 0xFFFF;
    size_t i;
    unsigned bit;
    for (i = 0; i < len; ++i) {
        crc ^= data[i];
        for (bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ ((crc & 1) ? 0xA001 : 0);
    }
    data[len] = (uint8_t)(crc >> 8); data[len + 1] = (uint8_t)crc;
}
static int failure;
static uint8_t last_seq;
static int writer(void *user, const uint8_t *data, size_t length)
{
    CHECK(user == &failure && length == 81 && data[0] == 0xA8 && data[7] == 1);
    last_seq = data[3];
    return failure;
}
int main(void)
{
    u6913_frame_t frame;
    u6913_board_packet_t packet;
    u6913_board_port_t port;
    u6913_motor_t motor;
    u6913_board_parser_t parser = {0};
    u6913_board_event_t event, before;
    u6913_feedback_t feedback;
    uint8_t ac[81] = {0xAC,3,0xA1,0,0,21,4,1,1,0,0,0,0,6,12,
                      6,0xCB,0,0x80,0,0x80,0,0x80,65,0,48,6};
    uint8_t a0[10] = {0xA0,1,0xA1,4,0xA8,0,1,4,0,0};
    uint8_t no_crc[] = {0xA0,0,0xA1,5,0xAC,0,0};
    const uint8_t bad_header[] = {0xAC,3,0xA1,0,0xFF,0xFF};
    unsigned i, repeat;
    CHECK(u6913_build_simple(&frame, 32, U6913_CMD_STANDARD_STATUS) == 0);
    CHECK(frame.data[1] == 0xCB && frame.data[61] == 0xCB);
    CHECK(u6913_board_wrap(&frame, 1, 0xA8, &packet) == 0);
    CHECK(packet.length == 81 && packet.data[3] == 0xA8 && packet.data[5] == 73);
    CHECK(memcmp(packet.data + 15, frame.data, 64) == 0);
    CHECK(u6913_board_wrap(&frame, 1, 1, &packet) == 0);
    CHECK(packet.data[79] == 0x98 && packet.data[80] == 0xEA); /* ID6 CB查询黄金向量 */
    CHECK(u6913_board_wrap(&frame, 2, 0, &packet) == U6913_EINVAL);
    frame.len = 12;
    CHECK(u6913_board_wrap(&frame, 1, 0, &packet) == U6913_EFRAME);
    CHECK(u6913_board_init(&port, 1, writer, &failure) == 0);
    CHECK(u6913_init(&motor, 6, u6913_board_send, &port) == 0);
    port.next_sequence = 255;
    CHECK(u6913_request_standard_status(&motor) == 0 && last_seq == 255 && port.next_sequence == 0);
    failure = 1;
    CHECK(u6913_request_version(&motor) == U6913_EIO && last_seq == 0 && port.next_sequence == 1);

    add_crc(ac, 27); add_crc(a0, 8);
    CHECK(u6913_board_decode(ac, 29, &event) == 0);
    CHECK(event.kind == U6913_BOARD_CAN_RECEIVED && event.channel == 1);
    CHECK(event.frame.can_id == 6 && event.frame.len == 12 && event.sequence == 0);
    CHECK(u6913_parse_feedback(6, event.frame.can_id, event.frame.data, event.frame.len, &feedback) == 0);
    CHECK(feedback.value.status.predicted_temperature_c == 25);
    for (i = 0; i < 29; ++i) CHECK(u6913_board_decode(ac, i, &event) == U6913_EFRAME);
    CHECK(u6913_board_decode(ac, 30, &event) == U6913_EFRAME);
    CHECK(u6913_board_decode(a0, 10, &event) == 0 && event.kind == U6913_BOARD_REPLY);
    CHECK(event.error_code == 4 && event.sequence == 0xA8 && event.reply_data[0] == 4);
    CHECK(u6913_board_decode(no_crc, sizeof(no_crc), &event) == 0 && event.sequence == 0xAC);
    before = event;
    ac[28] ^= 1;
    CHECK(u6913_board_decode(ac, 29, &event) == U6913_ECRC);
    CHECK(memcmp(&before, &event, sizeof(event)) == 0);
    ac[28] ^= 1;
    /* 逐字节、粘包、负载内帧头、CRC跨读取边界。 */
    ac[17] = 0xA8; ac[18] = 0xAC; ac[19] = 0xA0; ac[20] = 0x5A; add_crc(ac, 27);
    for (repeat = 0; repeat < 20; ++repeat) {
        for (i = 0; i < 29; ++i) CHECK(u6913_board_feed(&parser, ac[i], &event) == (i == 28 ? 1 : 0));
        for (i = 0; i < 10; ++i) CHECK(u6913_board_feed(&parser, a0[i], &event) == (i == 9 ? 1 : 0));
    }
    for (i = 0; i < 6; ++i) CHECK(u6913_board_feed(&parser, bad_header[i], &event) == (i == 5 ? U6913_EFRAME : 0));
    for (i = 0; i < 29; ++i) CHECK(u6913_board_feed(&parser, ac[i], &event) == (i == 28 ? 1 : 0));
    for (i = 0; i < 28; ++i) CHECK(u6913_board_feed(&parser, ac[i], &event) == 0);
    CHECK(parser.used == 28); /* 不把缺失的最后CRC字节降级为成功 */
    u6913_board_parser_reset(&parser);
    CHECK(parser.used == 0);
    ac[1] = 2; /* 此板AC必须带CRC */
    CHECK(u6913_board_decode(ac, 29, &event) == U6913_EFRAME);
    ac[1] = 3; ac[7] = 2; add_crc(ac, 27);
    CHECK(u6913_board_decode(ac, 29, &event) == U6913_EFRAME);
    ac[7] = 1; ac[9] = 1; add_crc(ac, 27);
    CHECK(u6913_board_decode(ac, 29, &event) == U6913_EUNSUPPORTED);
    /* E8的32B CAN反馈经AC封装后49B；逐字节接收再交给电机解析。 */
    memset(ac + 15, 0, 64); ac[9] = 0; ac[5] = 41; ac[14] = 32;
    ac[15] = 6; ac[16] = 0xE8; ac[25] = 'S'; ac[26] = 'N'; ac[46] = 0xE8;
    add_crc(ac, 47);
    for (i = 0; i < 49; ++i) CHECK(u6913_board_feed(&parser, ac[i], &event) == (i == 48 ? 1 : 0));
    CHECK(u6913_parse_feedback(6, event.frame.can_id, event.frame.data, event.frame.len, &feedback) == 0);
    CHECK(feedback.kind == U6913_FEEDBACK_IDENTITY && strcmp(feedback.value.identity.serial_number, "SN") == 0);
    /* 非法长度、随机输入在ASan/UBSan下不应越界。 */
    for (i = 0; i < 100000; ++i) (void)u6913_board_feed(&parser, (uint8_t)rand(), &event);
    puts("board framing, CRC, A0 errors, stream boundaries and timeout reset: PASS");
    return 0;
}
