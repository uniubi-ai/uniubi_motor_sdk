#include "uniubi6913.h"

#include <float.h>
#include <math.h>
#include <string.h>

#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128
#error "UNIUBI6913 requires IEEE-754 binary32 float"
#endif
typedef char u6913_float_size_check[(sizeof(float) == 4) ? 1 : -1];

static int valid_id(uint8_t id) { return id >= 1 && id <= U6913_JOINT_COUNT; }
static int valid_mask(uint8_t mask) { return mask != 0 && (mask & ~U6913_ALL_JOINTS) == 0; }
static uint8_t id_mask(uint8_t id) { return (uint8_t)(1u << (id - 1)); }
static int in_range(float x, float lo, float hi) { return isfinite(x) && x >= lo && x <= hi; }
static void put16(uint8_t *p, uint16_t x) { p[0] = (uint8_t)x; p[1] = (uint8_t)(x >> 8); }
static uint16_t get16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | (uint16_t)p[1] << 8); }

static float get_float(const uint8_t *p)
{
    uint32_t bits = (uint32_t)p[0] | (uint32_t)p[1] << 8 |
                    (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
    float x;
    memcpy(&x, &bits, sizeof(x));
    return x;
}

/* 使用 double 中间量和 ties-to-even，与当前 Python map_* 的 round 对齐。
 * limit 字段例外：按固件线性映射向零截断。拒绝越界由上层完成。 */
static uint16_t quantize(float x, float lo, float hi, uint16_t zero, int truncate)
{
    double raw, fraction;
    uint32_t base;
    if (x == 0.0f) return zero;
    raw = ((double)x - lo) / ((double)hi - lo) * 65535.0;
    base = (uint32_t)raw;
    fraction = raw - base;
    if (!truncate && (fraction > 0.5 || (fraction == 0.5 && (base & 1u)))) ++base;
    return (uint16_t)base;
}

static float decode(uint16_t raw, float max)
{
    return raw == 0x8000 ? 0.0f : (float)(-max + (double)raw / 65535.0 * (2.0 * max));
}

static void start_frame(u6913_frame_t *f, uint8_t command)
{
    memset(f, 0, sizeof(*f));
    f->can_id = U6913_HOST_CAN_ID;
    f->len = U6913_FRAME_SIZE;
    f->data[1] = command;
}

static int special(u6913_frame_t *out, uint8_t mask, uint8_t command, const uint8_t slot[9])
{
    u6913_frame_t f;
    unsigned i;
    if (!out || !valid_mask(mask)) return U6913_EINVAL;
    start_frame(&f, command);
    for (i = 0; i < U6913_JOINT_COUNT; ++i) {
        if (mask & (1u << i)) {
            memcpy(f.data + 2 + 10 * i, slot, 9);
            f.data[11 + 10 * i] = command;
        }
    }
    *out = f;
    return U6913_OK;
}

int u6913_build_simple(u6913_frame_t *out, uint8_t mask, u6913_command_t command)
{
    uint8_t slot[9];
    switch (command) {
    case U6913_CMD_ENABLE: case U6913_CMD_DISABLE: case U6913_CMD_SET_ZERO:
    case U6913_CMD_AUTO_CALIBRATION: case U6913_CMD_CALIBRATION_STATUS:
    case U6913_CMD_SERIAL_NUMBER:
    case U6913_CMD_FULL_STATUS: case U6913_CMD_STANDARD_STATUS: case U6913_CMD_VERSION: break;
    default: return U6913_EUNSUPPORTED;
    }
    memset(slot, 0xFF, sizeof(slot));
    return special(out, mask, (uint8_t)command, slot);
}

int u6913_build_mode(u6913_frame_t *out, uint8_t mask, u6913_mode_t mode)
{
    uint8_t slot[9];
    if (mode < U6913_MODE_SPEED || mode > U6913_MODE_PD) return U6913_EINVAL;
    memset(slot, 0xFF, sizeof(slot));
    slot[0] = (uint8_t)mode;
    return special(out, mask, U6913_CMD_SET_MODE, slot);
}

static int build_limited(u6913_frame_t *out, uint8_t mask, uint8_t command,
                         float value, float max, float limit)
{
    uint8_t slot[9];
    if (!in_range(value, -max, max) || !in_range(limit, 0, U6913_LIMIT_MAX)) return U6913_ERANGE;
    memset(slot, 0xFF, sizeof(slot));
    put16(slot, quantize(value, -max, max, 0x8000, 0));
    put16(slot + 2, quantize(limit, 0, U6913_LIMIT_MAX, 0, 1));
    return special(out, mask, command, slot);
}

int u6913_build_speed(u6913_frame_t *out, uint8_t mask, float rad_s, float torque_limit_nm)
{
    return build_limited(out, mask, U6913_CMD_SPEED, rad_s, U6913_VELOCITY_MAX, torque_limit_nm);
}

int u6913_build_position(u6913_frame_t *out, uint8_t mask, float rad, float velocity_limit_rad_s)
{
    return build_limited(out, mask, U6913_CMD_POSITION, rad, U6913_POSITION_MAX, velocity_limit_rad_s);
}

int u6913_build_torque(u6913_frame_t *out, uint8_t mask, float torque_nm)
{
    uint8_t slot[9];
    if (!in_range(torque_nm, -U6913_TORQUE_MAX, U6913_TORQUE_MAX)) return U6913_ERANGE;
    memset(slot, 0xFF, sizeof(slot));
    put16(slot, quantize(torque_nm, -U6913_TORQUE_MAX, U6913_TORQUE_MAX, 0x8000, 0));
    put16(slot + 2, 0x8000); /* D 轴零目标 */
    return special(out, mask, U6913_CMD_TORQUE, slot);
}

int u6913_build_pd(u6913_frame_t *out, const u6913_pd_target_t targets[U6913_JOINT_COUNT])
{
    u6913_frame_t f;
    unsigned i;
    if (!out || !targets) return U6913_EINVAL;
    start_frame(&f, U6913_CMD_PD);
    for (i = 0; i < U6913_JOINT_COUNT; ++i) {
        const u6913_pd_target_t *t = &targets[i];
        uint8_t *p = f.data + 2 + 10 * i;
        if (!in_range(t->position_rad, -U6913_POSITION_MAX, U6913_POSITION_MAX) ||
            !in_range(t->velocity_rad_s, -U6913_VELOCITY_MAX, U6913_VELOCITY_MAX) ||
            !in_range(t->kp, 0, U6913_KP_MAX) || !in_range(t->kd, 0, U6913_KD_MAX) ||
            !in_range(t->feedforward_nm, -U6913_FF_TORQUE_MAX, U6913_FF_TORQUE_MAX)) return U6913_ERANGE;
        put16(p, quantize(t->position_rad, -U6913_POSITION_MAX, U6913_POSITION_MAX, 0x8000, 0));
        put16(p + 2, quantize(t->velocity_rad_s, -U6913_VELOCITY_MAX, U6913_VELOCITY_MAX, 0x8000, 0));
        put16(p + 4, quantize(t->kp, 0, U6913_KP_MAX, 0, 0));
        put16(p + 6, quantize(t->kd, 0, U6913_KD_MAX, 0, 0));
        put16(p + 8, quantize(t->feedforward_nm, -U6913_FF_TORQUE_MAX, U6913_FF_TORQUE_MAX, 0x8000, 0));
    }
    *out = f;
    return U6913_OK;
}

int u6913_build_set_can_id(u6913_frame_t *out, uint8_t joint_id, uint8_t new_id)
{
    uint8_t slot[9];
    if (!valid_id(joint_id) || !valid_id(new_id)) return U6913_EINVAL;
    memset(slot, 0xFF, sizeof(slot));
    slot[0] = new_id;
    return special(out, id_mask(joint_id), U6913_CMD_SET_CAN_ID, slot);
}

static int valid_motor(const u6913_motor_t *motor)
{
    return motor && valid_id(motor->joint_id) && motor->send;
}

int u6913_init(u6913_motor_t *motor, uint8_t joint_id, u6913_send_fn send, void *user)
{
    if (!motor || !valid_id(joint_id) || !send) return U6913_EINVAL;
    motor->joint_id = joint_id;
    motor->send = send;
    motor->user = user;
    return U6913_OK;
}

int u6913_send_frame(const u6913_motor_t *motor, const u6913_frame_t *frame)
{
    if (!valid_motor(motor) || !frame) return U6913_EINVAL;
    if (frame->can_id != U6913_HOST_CAN_ID || frame->len != U6913_FRAME_SIZE || frame->data[0] != 0)
        return U6913_EFRAME;
    switch (frame->data[1]) {
    case U6913_CMD_AUTO_CALIBRATION: case U6913_CMD_CALIBRATION_STATUS:
    case U6913_CMD_SERIAL_NUMBER:
    case U6913_CMD_PD: case U6913_CMD_STANDARD_STATUS: case U6913_CMD_VERSION:
    case U6913_CMD_SET_ZERO: case U6913_CMD_SET_MODE: case U6913_CMD_TORQUE:
    case U6913_CMD_SPEED: case U6913_CMD_POSITION: case U6913_CMD_SET_CAN_ID:
    case U6913_CMD_ENABLE: case U6913_CMD_DISABLE: case U6913_CMD_FULL_STATUS: break;
    default: return U6913_EUNSUPPORTED;
    }
    return motor->send(motor->user, frame) == 0 ? U6913_OK : U6913_EIO;
}

static int send_simple(const u6913_motor_t *motor, u6913_command_t command)
{
    u6913_frame_t f;
    int rc;
    if (!valid_motor(motor)) return U6913_EINVAL;
    rc = u6913_build_simple(&f, id_mask(motor->joint_id), command);
    return rc == U6913_OK ? u6913_send_frame(motor, &f) : rc;
}

int u6913_enable(const u6913_motor_t *m) { return send_simple(m, U6913_CMD_ENABLE); }
int u6913_disable(const u6913_motor_t *m) { return send_simple(m, U6913_CMD_DISABLE); }
int u6913_set_zero(const u6913_motor_t *m) { return send_simple(m, U6913_CMD_SET_ZERO); }
int u6913_request_status(const u6913_motor_t *m) { return send_simple(m, U6913_CMD_FULL_STATUS); }
int u6913_request_standard_status(const u6913_motor_t *m) { return send_simple(m, U6913_CMD_STANDARD_STATUS); }
int u6913_request_version(const u6913_motor_t *m) { return send_simple(m, U6913_CMD_VERSION); }
int u6913_request_serial_number(const u6913_motor_t *m) { return send_simple(m, U6913_CMD_SERIAL_NUMBER); }
int u6913_request_calibration_status(const u6913_motor_t *m) { return send_simple(m, U6913_CMD_CALIBRATION_STATUS); }
int u6913_start_auto_calibration(const u6913_motor_t *m) { return send_simple(m, U6913_CMD_AUTO_CALIBRATION); }

int u6913_set_mode(const u6913_motor_t *motor, u6913_mode_t mode)
{
    u6913_frame_t f;
    int rc;
    if (!valid_motor(motor)) return U6913_EINVAL;
    rc = u6913_build_mode(&f, id_mask(motor->joint_id), mode);
    return rc == U6913_OK ? u6913_send_frame(motor, &f) : rc;
}

int u6913_speed_control(const u6913_motor_t *motor, float rad_s, float torque_limit_nm)
{
    u6913_frame_t f;
    int rc;
    if (!valid_motor(motor)) return U6913_EINVAL;
    rc = u6913_build_speed(&f, id_mask(motor->joint_id), rad_s, torque_limit_nm);
    return rc == U6913_OK ? u6913_send_frame(motor, &f) : rc;
}

int u6913_position_control(const u6913_motor_t *motor, float rad, float velocity_limit_rad_s)
{
    u6913_frame_t f;
    int rc;
    if (!valid_motor(motor)) return U6913_EINVAL;
    rc = u6913_build_position(&f, id_mask(motor->joint_id), rad, velocity_limit_rad_s);
    return rc == U6913_OK ? u6913_send_frame(motor, &f) : rc;
}

int u6913_torque_control(const u6913_motor_t *motor, float torque_nm)
{
    u6913_frame_t f;
    int rc;
    if (!valid_motor(motor)) return U6913_EINVAL;
    rc = u6913_build_torque(&f, id_mask(motor->joint_id), torque_nm);
    return rc == U6913_OK ? u6913_send_frame(motor, &f) : rc;
}

int u6913_set_can_id(const u6913_motor_t *motor, uint8_t new_id)
{
    u6913_frame_t f;
    int rc;
    if (!valid_motor(motor)) return U6913_EINVAL;
    rc = u6913_build_set_can_id(&f, motor->joint_id, new_id);
    return rc == U6913_OK ? u6913_send_frame(motor, &f) : rc;
}

static int is_standard_status(uint8_t cmd)
{
    switch (cmd) {
    case 0x00: case U6913_CMD_PD: case U6913_CMD_ENABLE: case U6913_CMD_DISABLE:
    case U6913_CMD_SET_ZERO: case U6913_CMD_SET_MODE: case U6913_CMD_TORQUE:
    case U6913_CMD_SPEED: case U6913_CMD_POSITION: case U6913_CMD_SET_CAN_ID:
    case U6913_CMD_AUTO_CALIBRATION:
    case U6913_CMD_STANDARD_STATUS: return 1;
    default: return 0;
    }
}

int u6913_parse_feedback(uint8_t expected_id, uint32_t can_id, const uint8_t *data,
                         size_t len, u6913_feedback_t *out)
{
    u6913_feedback_t result;
    if (!out || !data || !valid_id(expected_id)) return U6913_EINVAL;
    if (len < 2 || can_id != expected_id || data[0] != expected_id) return U6913_EFRAME;
    memset(&result, 0, sizeof(result));
    result.joint_id = expected_id;
    result.command = data[1];
    if (data[1] == U6913_CMD_FULL_STATUS) {
        float values[11];
        unsigned i;
        u6913_full_status_t *s = &result.value.full_status;
        if (len != 48) return U6913_EFRAME;
        for (i = 0; i < 11; ++i) {
            values[i] = get_float(data + 2 + i * 4);
            if (!isfinite(values[i])) return U6913_EFRAME;
        }
        result.kind = U6913_FEEDBACK_FULL_STATUS;
        s->iq_target_a = values[0]; s->iq_measured_a = values[1];
        s->id_target_a = values[2]; s->id_measured_a = values[3];
        s->position_target_rad = values[4]; s->phase_b_a = values[5]; s->phase_c_a = values[6];
        s->output_velocity_rad_s = values[7]; s->mechanical_angle_rad = values[8];
        s->output_position_rad = values[9]; s->ntc_temperature_c = values[10];
        s->mode = data[46]; s->compensation_bits = data[47];
    } else if (data[1] == U6913_CMD_VERSION) {
        if (len != 12) return U6913_EFRAME;
        result.kind = U6913_FEEDBACK_VERSION;
        result.value.version.patch = data[2];
        result.value.version.minor = data[3];
        result.value.version.major = data[4];
        result.value.version.encoder_type = data[5];
    } else if (data[1] == U6913_CMD_SERIAL_NUMBER) {
        u6913_identity_t *s = &result.value.identity;
        if (len != 32 || data[31] != U6913_CMD_SERIAL_NUMBER) return U6913_EFRAME;
        result.kind = U6913_FEEDBACK_IDENTITY;
        s->product_type = data[2];
        s->major = data[3]; s->minor = data[4]; s->patch = data[5];
        s->production_year = get16(data + 6);
        s->production_month = data[8]; s->production_day = data[9];
        memcpy(s->serial_number, data + 10, 16);
        s->serial_number[16] = '\0';
        s->serial_length = 16;
        while (s->serial_length && s->serial_number[s->serial_length - 1] == '\0') --s->serial_length;
        s->mcu_type = data[26];
    } else if (data[1] == U6913_CMD_CALIBRATION_STATUS) {
        if (len != 48) return U6913_EFRAME;
        result.value.calibration.flags = get16(data + 8);
        result.kind = U6913_FEEDBACK_CALIBRATION;
    } else if (is_standard_status(data[1])) {
        u6913_status_t *s = &result.value.status;
        if (len != 12) return U6913_EFRAME;
        result.kind = U6913_FEEDBACK_STATUS;
        s->position_rad = decode(get16(data + 2), U6913_POSITION_MAX);
        s->velocity_rad_s = decode(get16(data + 4), U6913_VELOCITY_MAX);
        s->torque_nm = decode(get16(data + 6), U6913_TORQUE_MAX);
        s->predicted_temperature_c = (int16_t)data[8] - 40;
        s->status_bits = data[9]; s->bus_voltage_v = data[10];
    } else {
        return U6913_EUNSUPPORTED;
    }
    *out = result;
    return U6913_OK;
}
