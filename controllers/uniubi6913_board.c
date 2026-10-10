#include "uniubi6913_board.h"
#include <string.h>

static uint16_t crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    size_t i;
    unsigned bit;
    for (i = 0; i < length; ++i) {
        crc ^= data[i];
        for (bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc & 1u) ? (crc >> 1) ^ 0xA001u : crc >> 1);
    }
    return crc;
}

static uint16_t be16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] << 8 | p[1]); }
static int valid_channel(uint8_t channel) { return channel <= 1; }
static int valid_length(uint8_t n)
{
    return n <= 8 || n == 12 || n == 16 || n == 20 || n == 24 || n == 32 || n == 48 || n == 64;
}

int u6913_board_wrap(const u6913_frame_t *frame, uint8_t channel, uint8_t sequence,
                     u6913_board_packet_t *out)
{
    u6913_board_packet_t packet;
    uint16_t crc;
    if (!frame || !out || !valid_channel(channel)) return U6913_EINVAL;
    if (frame->can_id != U6913_HOST_CAN_ID || frame->len != 64 || frame->data[0] != 0)
        return U6913_EFRAME;
    memset(&packet, 0, sizeof(packet));
    packet.length = U6913_BOARD_REQUEST_SIZE;
    packet.data[0] = 0xA8; packet.data[1] = 1; packet.data[2] = 0xA1;
    packet.data[3] = sequence; packet.data[5] = 73;
    packet.data[6] = 3; packet.data[7] = channel; packet.data[8] = 1;
    packet.data[14] = 64;
    memcpy(packet.data + 15, frame->data, 64);
    crc = crc16(packet.data, 79);
    packet.data[79] = (uint8_t)(crc >> 8); packet.data[80] = (uint8_t)crc;
    *out = packet;
    return U6913_OK;
}

int u6913_board_init(u6913_board_port_t *port, uint8_t channel,
                     u6913_board_write_fn write, void *user)
{
    if (!port || !write || !valid_channel(channel)) return U6913_EINVAL;
    port->write = write; port->user = user; port->channel = channel; port->next_sequence = 0;
    return U6913_OK;
}

int u6913_board_send(void *context, const u6913_frame_t *frame)
{
    u6913_board_port_t *port = (u6913_board_port_t *)context;
    u6913_board_packet_t packet;
    int rc;
    if (!port || !port->write) return U6913_EINVAL;
    rc = u6913_board_wrap(frame, port->channel, port->next_sequence, &packet);
    if (rc != U6913_OK) return rc;
    /* 写入失败也消耗序号：部分发送的请求仍有可能被调试板接收。 */
    ++port->next_sequence;
    return port->write(port->user, packet.data, packet.length) == 0 ? U6913_OK : U6913_EIO;
}

/* 返回0等待头部，1头部有效，负值为非法。 */
static int header_length(const uint8_t *data, size_t len, size_t *header, size_t *total)
{
    size_t body, crc;
    if (len == 0) return 0;
    if (data[0] != 0xA0 && data[0] != 0xAC) return U6913_EFRAME;
    *header = data[0] == 0xA0 ? 7u : 6u;
    if (len < 2) return 0;
    /* 此板当前AC必须带CRC和push位。A0可随请求省略CRC。 */
    if ((data[0] == 0xAC && data[1] != 3) || (data[0] == 0xA0 && data[1] > 1))
        return U6913_EFRAME;
    if (len < *header) return 0;
    body = be16(data + *header - 2);
    crc = (data[1] & 1u) ? 2u : 0u;
    *total = *header + body + crc;
    if (*total > U6913_BOARD_PACKET_MAX) return U6913_EFRAME;
    return 1;
}

int u6913_board_decode(const uint8_t *data, size_t len, u6913_board_event_t *out)
{
    u6913_board_event_t event;
    const uint8_t *body;
    size_t header = 0, total = 0, body_len;
    int rc;
    if (!data || !out) return U6913_EINVAL;
    rc = header_length(data, len, &header, &total);
    if (rc <= 0 || total != len) return U6913_EFRAME;
    if ((data[1] & 1u) && crc16(data, len - 2) != be16(data + len - 2)) return U6913_ECRC;
    memset(&event, 0, sizeof(event));
    event.command_class = data[2];
    body = data + header;
    body_len = be16(data + header - 2);
    if (data[0] == 0xA0) {
        event.kind = U6913_BOARD_REPLY;
        event.error_code = data[3]; event.sequence = data[4];
        event.reply_length = (uint8_t)body_len;
        memcpy(event.reply_data, body, body_len);
    } else {
        uint8_t n;
        if (data[2] != 0xA1 || data[3] != 0) return U6913_EUNSUPPORTED;
        if (body_len < 9) return U6913_EFRAME;
        if (body[0] != 4 || body[2] != 1 || body[3] != 0) return U6913_EUNSUPPORTED;
        n = body[8];
        if (!valid_channel(body[1]) || !valid_length(n) || body_len != 9u + n) return U6913_EFRAME;
        event.kind = U6913_BOARD_CAN_RECEIVED;
        event.channel = body[1];
        event.frame.can_id = (uint32_t)body[4] << 24 | (uint32_t)body[5] << 16 |
                             (uint32_t)body[6] << 8 | body[7];
        if (event.frame.can_id > 0x7FFu) return U6913_EFRAME;
        event.frame.len = n;
        memcpy(event.frame.data, body + 9, n);
    }
    *out = event;
    return U6913_OK;
}

void u6913_board_parser_reset(u6913_board_parser_t *parser)
{
    if (parser) { parser->used = 0; parser->expected = 0; }
}

int u6913_board_feed(u6913_board_parser_t *parser, uint8_t byte, u6913_board_event_t *out)
{
    size_t header = 0, total = 0;
    int rc;
    if (!parser || !out) return U6913_EINVAL;
    if (parser->used >= sizeof(parser->data)) {
        u6913_board_parser_reset(parser);
        return U6913_EFRAME;
    }
    if (parser->used == 0 && byte != 0xA0 && byte != 0xAC) return 0;
    parser->data[parser->used++] = byte;
    rc = header_length(parser->data, parser->used, &header, &total);
    if (rc < 0) { u6913_board_parser_reset(parser); return rc; }
    if (rc == 0) return 0;
    parser->expected = total;
    if (parser->used < total) return 0;
    rc = u6913_board_decode(parser->data, parser->used, out);
    u6913_board_parser_reset(parser);
    return rc == U6913_OK ? 1 : rc;
}
