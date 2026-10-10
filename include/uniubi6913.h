#ifndef UNIUBI6913_H
#define UNIUBI6913_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define U6913_SDK_VERSION "0.5.0"
#define U6913_JOINT_COUNT 6u
#define U6913_FRAME_SIZE 64u
#define U6913_ALL_JOINTS 0x3Fu
#define U6913_HOST_CAN_ID 0u

/* 协议编码范围，不是电机额定值或允许运行范围。单位为输出轴 rad、rad/s、Nm。 */
#define U6913_POSITION_MAX 12.5f
#define U6913_VELOCITY_MAX 65.0f
#define U6913_TORQUE_MAX 100.0f
#define U6913_FF_TORQUE_MAX 100.0f
#define U6913_KP_MAX 500.0f
#define U6913_KD_MAX 5.0f
#define U6913_LIMIT_MAX 100.0f

typedef enum {
    U6913_OK = 0,
    U6913_EINVAL = -1,       /* 空指针、非法 ID/mask/模式/长度 */
    U6913_ERANGE = -2,      /* NaN、Inf 或超协议量程；不会发送 */
    U6913_EIO = -3,         /* 发送回调失败 */
    U6913_EFRAME = -4,      /* 接收长度、CAN ID 或设备 ID 不匹配 */
    U6913_EUNSUPPORTED = -5, /* 未支持的命令/反馈 */
    U6913_ECRC = -6         /* 调试板串口 CRC 不匹配 */
} u6913_result_t;

typedef enum {
    U6913_MODE_SPEED = 0,
    U6913_MODE_POSITION = 1,
    U6913_MODE_TORQUE = 2,   /* 固件历史名称 Current，F1 实际编码 Nm */
    U6913_MODE_PD = 3
} u6913_mode_t;

typedef enum {
    U6913_CMD_PD = 0xAE,
    U6913_CMD_AUTO_CALIBRATION = 0xAF,
    U6913_CMD_STANDARD_STATUS = 0xCB,
    U6913_CMD_CALIBRATION_STATUS = 0xE1,
    U6913_CMD_SERIAL_NUMBER = 0xE8,
    U6913_CMD_VERSION = 0xE9,
    U6913_CMD_SET_ZERO = 0xEF,
    U6913_CMD_SET_MODE = 0xF0,
    U6913_CMD_TORQUE = 0xF1,
    U6913_CMD_SPEED = 0xF4,
    U6913_CMD_POSITION = 0xF5,
    U6913_CMD_SET_CAN_ID = 0xFB,
    U6913_CMD_ENABLE = 0xFC,
    U6913_CMD_DISABLE = 0xFD,
    U6913_CMD_FULL_STATUS = 0xFE
} u6913_command_t;

/* 始终为标准 ID、CAN FD 数据帧。len 是字节数，不是 HAL 的 DLC 常量。 */
typedef struct {
    uint32_t can_id;
    uint8_t len;
    uint8_t data[U6913_FRAME_SIZE];
} u6913_frame_t;

/* 返回 0 表示已复制/入队，非 0 表示失败。不得保留临时 frame 指针。 */
typedef int (*u6913_send_fn)(void *user, const u6913_frame_t *frame);
typedef struct {
    uint8_t joint_id;       /* 1..6；更改设备 ID 后由调用方确认再重新 init */
    u6913_send_fn send;
    void *user;
} u6913_motor_t;

typedef struct {
    float position_rad;
    float velocity_rad_s;
    float kp;
    float kd;
    float feedforward_nm;
} u6913_pd_target_t;

typedef struct {
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    int16_t predicted_temperature_c; /* 普通状态为预测温度，不是 NTC 测量值 */
    uint8_t status_bits;
    uint8_t bus_voltage_v;
} u6913_status_t;

typedef struct {
    float iq_target_a, iq_measured_a;
    float id_target_a, id_measured_a;
    float position_target_rad;
    float phase_b_a, phase_c_a;
    float output_velocity_rad_s, mechanical_angle_rad, output_position_rad;
    float ntc_temperature_c;
    uint8_t mode;
    uint8_t compensation_bits;
} u6913_full_status_t;

typedef struct {
    uint8_t major, minor, patch;
    uint8_t encoder_type; /* 0未知、1 MT6701、2 AS5047P、3 MT6826S */
} u6913_version_t;

typedef struct {
    uint8_t product_type;
    uint8_t major, minor, patch; /* E8顺序与E9不同 */
    uint16_t production_year;
    uint8_t production_month, production_day;
    char serial_number[17]; /* 保留16字节SN区，额外保证末尾NUL */
    uint8_t serial_length;  /* 去掉尾部零填充后的字节数；0表示未配置 */
    uint8_t mcu_type;
} u6913_identity_t;

typedef struct {
    uint16_t flags; /* E1完成位图；未完成位不能区分未标定、进行中或失败 */
} u6913_calibration_status_t;

#define U6913_STATUS_ENABLED       0x01u
#define U6913_STATUS_BUS_VOLTAGE   0x02u
#define U6913_STATUS_OVERCURRENT   0x04u
#define U6913_STATUS_OVERTEMP      0x08u
#define U6913_STATUS_OVERSPEED     0x10u
#define U6913_STATUS_ENCODER       0x20u
#define U6913_STATUS_GATE_DRIVER   0x40u
#define U6913_STATUS_BUS_TIMEOUT   0x80u
typedef enum {
    U6913_FEEDBACK_STATUS,
    U6913_FEEDBACK_FULL_STATUS,
    U6913_FEEDBACK_VERSION,
    U6913_FEEDBACK_IDENTITY,
    U6913_FEEDBACK_CALIBRATION
} u6913_feedback_kind_t;
typedef struct {
    uint8_t joint_id;
    uint8_t command;
    u6913_feedback_kind_t kind;
    union {
        u6913_status_t status;
        u6913_full_status_t full_status;
        u6913_version_t version;
        u6913_identity_t identity;
        u6913_calibration_status_t calibration;
    } value;
} u6913_feedback_t;

/* 构帧函数不访问硬件。失败时 out 不变；mask 的 bit0..5 对应 ID1..6。 */
int u6913_build_simple(u6913_frame_t *out, uint8_t mask, u6913_command_t command);
int u6913_build_mode(u6913_frame_t *out, uint8_t mask, u6913_mode_t mode);
int u6913_build_speed(u6913_frame_t *out, uint8_t mask, float rad_s, float torque_limit_nm);
int u6913_build_position(u6913_frame_t *out, uint8_t mask, float rad, float velocity_limit_rad_s);
/* D 轴字段的历史语义不稳定，对外固定编码零；此 API 只设置输出扭矩。 */
int u6913_build_torque(u6913_frame_t *out, uint8_t mask, float torque_nm);
/* AE 无忽略槽：必须提供全部六槽，包括未接电机的槽。全零结构编码为零目标。 */
int u6913_build_pd(u6913_frame_t *out, const u6913_pd_target_t targets[U6913_JOINT_COUNT]);
int u6913_build_set_can_id(u6913_frame_t *out, uint8_t joint_id, uint8_t new_id);

/* 无动态分配、全局状态、内部线程或延时。发送成功不等于电机已执行。 */
int u6913_init(u6913_motor_t *motor, uint8_t joint_id, u6913_send_fn send, void *user);
/* 仅发送当前支持的15类电机命令；其他命令返回EUNSUPPORTED。 */
int u6913_send_frame(const u6913_motor_t *motor, const u6913_frame_t *frame);
int u6913_enable(const u6913_motor_t *motor);
int u6913_disable(const u6913_motor_t *motor);
int u6913_set_zero(const u6913_motor_t *motor);
int u6913_request_status(const u6913_motor_t *motor);
int u6913_request_standard_status(const u6913_motor_t *motor);
int u6913_request_version(const u6913_motor_t *motor);
int u6913_request_serial_number(const u6913_motor_t *motor);
int u6913_request_calibration_status(const u6913_motor_t *motor);
/* AF会由固件自动驱动电机并可能保存标定数据；ACK不表示标定成功。
 * 仅在满足标定工装/完整行程要求时显式调用，不能套用普通小幅运动范围。 */
int u6913_start_auto_calibration(const u6913_motor_t *motor);
int u6913_set_mode(const u6913_motor_t *motor, u6913_mode_t mode);
int u6913_speed_control(const u6913_motor_t *motor, float rad_s, float torque_limit_nm);
int u6913_position_control(const u6913_motor_t *motor, float rad, float velocity_limit_rad_s);
int u6913_torque_control(const u6913_motor_t *motor, float torque_nm);
int u6913_set_can_id(const u6913_motor_t *motor, uint8_t new_id);

/* 调用方先过滤标准 CAN FD 数据帧，再传入实际字节数。
 * 同时校验仲裁 ID 和 payload[0]。失败时 out 不变。
 * 事务匹配（设备ID/cmd）、超时和线程同步由调用方管理。 */
int u6913_parse_feedback(uint8_t expected_id, uint32_t can_id, const uint8_t *data,
                         size_t len, u6913_feedback_t *out);

#ifdef __cplusplus
}
#endif
#endif
