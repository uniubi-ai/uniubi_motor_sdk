/* 仅输出帧，不访问硬件。编译和运行方法见 README。 */
#include "uniubi6913.h"
#include <stdio.h>

static int print_frame(void *user, const u6913_frame_t *frame)
{
    unsigned i;
    (void)user;
    printf("CAN FD STD ID=%03X len=%u:", (unsigned)frame->can_id, frame->len);
    for (i = 0; i < frame->len; ++i) printf(" %02X", frame->data[i]);
    putchar('\n');
    return 0;
}

int main(void)
{
    u6913_motor_t motor;
    u6913_feedback_t feedback;
    const uint8_t simulated_version[12] = {1, 0xE9, 179, 2, 1};
    int rc = u6913_init(&motor, 1, print_frame, NULL);
    if (rc != U6913_OK) return 1;
    if (u6913_request_version(&motor) != U6913_OK) return 1;
    if (u6913_request_status(&motor) != U6913_OK) return 1;
    if (u6913_request_serial_number(&motor) != U6913_OK) return 1;
    if (u6913_request_calibration_status(&motor) != U6913_OK) return 1;
    rc = u6913_parse_feedback(1, 1, simulated_version, sizeof(simulated_version), &feedback);
    if (rc != U6913_OK) return 1;
    printf("Simulated reply: V%u.%u.%u\n", feedback.value.version.major,
           feedback.value.version.minor, feedback.value.version.patch);
    return 0;
}
