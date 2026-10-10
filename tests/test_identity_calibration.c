#include "uniubi6913.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); exit(1); } } while (0)
static u6913_frame_t sent;
static int capture(void *user, const u6913_frame_t *frame) { (void)user; sent = *frame; return 0; }
int main(void)
{
    const uint8_t commands[] = {0xAF,0xE1,0xE8};
    u6913_frame_t frame;
    u6913_motor_t motor;
    u6913_feedback_t out, before;
    uint8_t data[64] = {6,0xE8,7,1,2,193,0xEA,7,10,10};
    unsigned c, mask, i, j;
    for (c = 0; c < sizeof(commands); ++c) {
        for (mask = 1; mask < 64; ++mask) {
            CHECK(u6913_build_simple(&frame, (uint8_t)mask, (u6913_command_t)commands[c]) == 0);
            CHECK(frame.can_id == 0 && frame.len == 64 && frame.data[0] == 0 && frame.data[1] == commands[c]);
            for (i = 0; i < 6; ++i) {
                for (j = 0; j < 9; ++j) CHECK(frame.data[2 + 10*i + j] == ((mask & (1u << i)) ? 0xFF : 0));
                CHECK(frame.data[11 + 10*i] == ((mask & (1u << i)) ? commands[c] : 0));
            }
            CHECK(frame.data[62] == 0 && frame.data[63] == 0);
        }
    }
    CHECK(u6913_init(&motor, 6, capture, NULL) == 0);
    CHECK(u6913_start_auto_calibration(&motor) == 0 && sent.data[61] == 0xAF);
    CHECK(u6913_request_calibration_status(&motor) == 0 && sent.data[61] == 0xE1);
    CHECK(u6913_request_serial_number(&motor) == 0 && sent.data[61] == 0xE8 && sent.data[11] == 0);
    CHECK(u6913_start_auto_calibration(NULL) == U6913_EINVAL);
    memcpy(data + 10, "1234567890ABCDEF", 16); data[26] = 2; data[31] = 0xE8;
    CHECK(u6913_parse_feedback(6, 6, data, 32, &out) == 0 && out.kind == U6913_FEEDBACK_IDENTITY);
    CHECK(strcmp(out.value.identity.serial_number, "1234567890ABCDEF") == 0);
    CHECK(out.value.identity.serial_length == 16 && out.value.identity.product_type == 7);
    CHECK(out.value.identity.production_year == 2026 && out.value.identity.production_month == 10);
    CHECK(out.value.identity.production_day == 10 && out.value.identity.mcu_type == 2);
    CHECK(out.value.identity.major == 1 && out.value.identity.minor == 2 && out.value.identity.patch == 193);
    before = out;
    for (i = 0; i <= 64; ++i) if (i != 32) {
        CHECK(u6913_parse_feedback(6, 6, data, i, &out) == U6913_EFRAME);
        CHECK(memcmp(&out, &before, sizeof(out)) == 0);
    }
    data[31] = 0;
    CHECK(u6913_parse_feedback(6, 6, data, 32, &out) == U6913_EFRAME);
    data[31] = 0xE8; data[0] = 0xE8; /* 旧注释E8 E8不能绕过节点归属校验 */
    CHECK(u6913_parse_feedback(6, 6, data, 32, &out) == U6913_EFRAME);
    data[0] = 6; memset(data + 10, 0, 16);
    CHECK(u6913_parse_feedback(6, 6, data, 32, &out) == 0 && out.value.identity.serial_length == 0);
    memcpy(data + 10, "SN0123", 6);
    CHECK(u6913_parse_feedback(6, 6, data, 32, &out) == 0 && out.value.identity.serial_length == 6);
    CHECK(strcmp(out.value.identity.serial_number, "SN0123") == 0);
    memset(data, 0, sizeof(data)); data[0] = 6; data[1] = 0xE1; data[8] = 0x51; data[9] = 3;
    CHECK(u6913_parse_feedback(6, 6, data, 48, &out) == 0 && out.kind == U6913_FEEDBACK_CALIBRATION);
    CHECK(out.value.calibration.flags == 0x0351);
    CHECK(u6913_parse_feedback(6, 6, data, 12, &out) == U6913_EFRAME);
    before = out;
    for (i = 2; i <= 64; ++i) if (i != 48) {
        CHECK(u6913_parse_feedback(6, 6, data, i, &out) == U6913_EFRAME);
        CHECK(memcmp(&out, &before, sizeof(out)) == 0);
    }
    data[1] = 0xD0;
    CHECK(u6913_parse_feedback(6, 6, data, 12, &out) == U6913_EUNSUPPORTED);
    CHECK(u6913_parse_feedback(6, 6, data, 48, &out) == U6913_EUNSUPPORTED);
    CHECK(memcmp(&out, &before, sizeof(out)) == 0);
    data[1] = 0xAF;
    CHECK(u6913_parse_feedback(6, 6, data, 12, &out) == 0 && out.kind == U6913_FEEDBACK_STATUS);
    puts("identity, calibration requests and feedback: PASS");
    return 0;
}
