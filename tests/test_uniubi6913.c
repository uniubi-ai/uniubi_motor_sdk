#include "uniubi6913.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "line %d: %s\n", __LINE__, #x); exit(1); } } while (0)

static unsigned sent;
static int send_error;
static u6913_frame_t captured;
static int capture(void *user, const u6913_frame_t *frame)
{
    CHECK(user == &sent);
    ++sent;
    captured = *frame;
    return send_error;
}

static void test_frames(void)
{
    u6913_frame_t f, before;
    u6913_pd_target_t pd[6] = {{0}};
    const uint8_t neutral[10] = {0, 0x80, 0, 0x80, 0, 0, 0, 0, 0, 0x80};
    unsigned i;
    CHECK(u6913_build_simple(&f, 0x21, U6913_CMD_ENABLE) == 0);
    CHECK(f.can_id == 0 && f.len == 64 && f.data[0] == 0 && f.data[1] == 0xFC);
    for (i = 2; i <= 10; ++i) CHECK(f.data[i] == 0xFF && f.data[i + 50] == 0xFF);
    CHECK(f.data[11] == 0xFC && f.data[61] == 0xFC);
    for (i = 12; i < 52; ++i) CHECK(f.data[i] == 0);
    CHECK(f.data[62] == 0 && f.data[63] == 0);
    CHECK(u6913_build_simple(&f, U6913_ALL_JOINTS, U6913_CMD_DISABLE) == 0);
    for (i = 0; i < 6; ++i) CHECK(f.data[11 + i * 10] == 0xFD);
    CHECK(u6913_build_speed(&f, 2, -65, 100) == 0);
    CHECK(f.data[1] == 0xF4 && f.data[12] == 0 && f.data[13] == 0);
    CHECK(f.data[14] == 0xFF && f.data[15] == 0xFF && f.data[21] == 0xF4);
    CHECK(u6913_build_position(&f, 1, 12.5f, 50) == 0);
    CHECK(f.data[2] == 0xFF && f.data[3] == 0xFF);
    CHECK(f.data[4] == 0xFF && f.data[5] == 0x7F); /* limit 截断 */
    CHECK(u6913_build_torque(&f, 1, 50) == 0);
    CHECK(f.data[2] == 0xFF && f.data[3] == 0xBF && f.data[4] == 0 && f.data[5] == 0x80);
    CHECK(u6913_build_pd(&f, pd) == 0);
    for (i = 0; i < 6; ++i) CHECK(memcmp(f.data + 2 + i * 10, neutral, 10) == 0);
    pd[5].position_rad = -12.5f; pd[5].velocity_rad_s = 65;
    pd[5].kp = 500; pd[5].kd = 5; pd[5].feedforward_nm = 100;
    CHECK(u6913_build_pd(&f, pd) == 0 && f.data[1] == 0xAE);
    CHECK(f.data[52] == 0 && f.data[53] == 0);
    for (i = 54; i < 62; ++i) CHECK(f.data[i] == 0xFF); /* 第10字节仍为数据 */
    CHECK(u6913_build_mode(&f, 4, U6913_MODE_TORQUE) == 0);
    CHECK(f.data[22] == 2 && f.data[23] == 0xFF && f.data[31] == 0xF0);
    CHECK(u6913_build_set_can_id(&f, 6, 2) == 0 && f.data[52] == 2 && f.data[61] == 0xFB);
    before = f;
    CHECK(u6913_build_speed(&f, 1, NAN, 1) == U6913_ERANGE);
    CHECK(u6913_build_torque(&f, 1, INFINITY) == U6913_ERANGE);
    CHECK(u6913_build_position(&f, 1, 12.6f, 1) == U6913_ERANGE);
    CHECK(u6913_build_speed(&f, 1, 0, -1) == U6913_ERANGE);
    pd[5].kd = -1;
    CHECK(u6913_build_pd(&f, pd) == U6913_ERANGE);
    CHECK(u6913_build_pd(&f, NULL) == U6913_EINVAL);
    CHECK(u6913_build_mode(&f, 1, (u6913_mode_t)4) == U6913_EINVAL);
    CHECK(u6913_build_simple(&f, 0, U6913_CMD_ENABLE) == U6913_EINVAL);
    CHECK(u6913_build_simple(&f, 0x80, U6913_CMD_ENABLE) == U6913_EINVAL);
    CHECK(u6913_build_simple(&f, 1, U6913_CMD_PD) == U6913_EUNSUPPORTED);
    CHECK(u6913_build_set_can_id(&f, 1, 7) == U6913_EINVAL);
    CHECK(memcmp(&f, &before, sizeof(f)) == 0);
    CHECK(u6913_build_simple(NULL, 1, U6913_CMD_ENABLE) == U6913_EINVAL);
}

static void test_send(void)
{
    u6913_motor_t m;
    CHECK(u6913_init(&m, 0, capture, &sent) == U6913_EINVAL);
    CHECK(u6913_init(&m, 1, NULL, &sent) == U6913_EINVAL);
    CHECK(u6913_init(&m, 6, capture, &sent) == 0 && sent == 0);
    CHECK(u6913_enable(&m) == 0 && sent == 1 && captured.data[61] == 0xFC);
    CHECK(u6913_set_mode(&m, U6913_MODE_SPEED) == 0);
    CHECK(u6913_speed_control(&m, 1, 1) == 0);
    CHECK(u6913_position_control(&m, 1, 1) == 0);
    CHECK(u6913_torque_control(&m, 0) == 0);
    CHECK(u6913_set_can_id(&m, 2) == 0 && m.joint_id == 6);
    CHECK(u6913_set_zero(&m) == 0);
    CHECK(u6913_request_status(&m) == 0);
    CHECK(u6913_request_version(&m) == 0);
    CHECK(u6913_request_standard_status(&m) == 0);
    CHECK(u6913_disable(&m) == 0 && sent == 11);
    CHECK(u6913_torque_control(&m, 101) == U6913_ERANGE && sent == 11);
    CHECK(u6913_enable(NULL) == U6913_EINVAL);
    captured.len = 8;
    CHECK(u6913_send_frame(&m, &captured) == U6913_EFRAME && sent == 11);
    send_error = 1;
    CHECK(u6913_disable(&m) == U6913_EIO);
}

static void test_feedback(void)
{
    uint8_t status[12] = {2, 0xF4, 0, 0x80, 0xFF, 0xFF, 0, 0, 65, 1, 48, 0};
    uint8_t full[48] = {2, 0xFE};
    u6913_feedback_t f, before;
    size_t n;
    CHECK(u6913_parse_feedback(2, 2, status, 12, &f) == 0);
    CHECK(f.kind == U6913_FEEDBACK_STATUS && f.value.status.position_rad == 0);
    CHECK(f.value.status.velocity_rad_s == 65 && f.value.status.torque_nm == -100);
    CHECK(f.value.status.predicted_temperature_c == 25 && f.value.status.bus_voltage_v == 48);
    status[1] = 0xCB;
    CHECK(u6913_parse_feedback(2, 2, status, 12, &f) == 0 && f.command == 0xCB);
    full[4] = 0x80; full[5] = 0x3F; /* Iq target = 1.0f, little endian */
    full[32] = 0x80; full[33] = 0x3F; /* 输出轴速度 = 1 rad/s，不乘/除21 */
    full[44] = 0xC8; full[45] = 0x41; /* NTC = 25 C */
    full[46] = 3; full[47] = 5;
    CHECK(u6913_parse_feedback(2, 2, full, 48, &f) == 0);
    CHECK(f.kind == U6913_FEEDBACK_FULL_STATUS && f.value.full_status.iq_target_a == 1);
    CHECK(f.value.full_status.output_velocity_rad_s == 1 && f.value.full_status.ntc_temperature_c == 25);
    CHECK(f.value.full_status.mode == 3 && f.value.full_status.compensation_bits == 5);
    before = f;
    for (n = 0; n < 48; ++n) CHECK(u6913_parse_feedback(2, 2, full, n, &f) == U6913_EFRAME);
    full[4] = 0xC0; full[5] = 0x7F; /* NaN */
    CHECK(u6913_parse_feedback(2, 2, full, 48, &f) == U6913_EFRAME);
    CHECK(u6913_parse_feedback(1, 2, status, 12, &f) == U6913_EFRAME);
    CHECK(u6913_parse_feedback(2, 1, status, 12, &f) == U6913_EFRAME);
    CHECK(u6913_parse_feedback(0, 2, status, 12, &f) == U6913_EINVAL);
    CHECK(u6913_parse_feedback(2, 2, NULL, 12, &f) == U6913_EINVAL);
    status[1] = 0xB5;
    CHECK(u6913_parse_feedback(2, 2, status, 12, &f) == U6913_EUNSUPPORTED);
    CHECK(memcmp(&f, &before, sizeof(f)) == 0);
    memset(status, 0, sizeof(status));
    status[0] = 2; status[1] = 0xE9; status[2] = 0x10; status[3] = 2; status[4] = 1;
    status[5] = 3;
    CHECK(u6913_parse_feedback(2, 2, status, 12, &f) == 0);
    CHECK(f.value.version.patch == 16 && f.value.version.major == 1);
    CHECK(f.value.version.encoder_type == 3);

}

static void test_removed_commands(void)
{
    const uint8_t commands[] = {0xF9, 0xF8, 0xF6, 0xF7, 0xC6, 0xD0};
    u6913_motor_t motor;
    u6913_frame_t frame, before;
    u6913_feedback_t feedback, previous;
    uint8_t reply[12] = {1};
    unsigned i, count = sent;
    CHECK(u6913_init(&motor, 1, capture, &sent) == 0);
    CHECK(u6913_build_simple(&frame, 1, U6913_CMD_ENABLE) == 0);
    memset(&feedback, 0xA5, sizeof(feedback));
    previous = feedback;
    for (i = 0; i < sizeof(commands); ++i) {
        before = frame;
        CHECK(u6913_build_simple(&frame, 1, (u6913_command_t)commands[i]) == U6913_EUNSUPPORTED);
        CHECK(memcmp(&frame, &before, sizeof(frame)) == 0);
        frame.data[1] = frame.data[11] = commands[i];
        CHECK(u6913_send_frame(&motor, &frame) == U6913_EUNSUPPORTED);
        CHECK(sent == count); /* 禁止通过通用SDK发送入口发送已移除指令。 */
        reply[1] = commands[i];
        CHECK(u6913_parse_feedback(1, 1, reply, sizeof(reply), &feedback) == U6913_EUNSUPPORTED);
        CHECK(memcmp(&feedback, &previous, sizeof(feedback)) == 0);
    }
}

int main(void)
{
    test_frames(); test_send(); test_feedback(); test_removed_commands();
    puts("protocol, validation, transport errors and feedback: PASS");
    return 0;
}
