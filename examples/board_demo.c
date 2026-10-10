/* 离线示例：只打印81字节串口请求，不访问硬件、不使能电机。 */
#include "uniubi6913_board.h"
#include <stdio.h>

static int print_serial(void *user, const uint8_t *data, size_t length)
{
    size_t i;
    (void)user;
    printf("UART %u baud, 8N1, %zu bytes:", U6913_BOARD_BAUDRATE, length);
    for (i = 0; i < length; ++i) printf(" %02X", data[i]);
    putchar('\n');
    return 0;
}

int main(void)
{
    u6913_board_port_t port;
    u6913_motor_t motor;
    if (u6913_board_init(&port, 1, print_serial, NULL) != U6913_OK) return 1;
    /* 手册默认ID6；实际集成时换成已确认的设备ID。 */
    if (u6913_init(&motor, 6, u6913_board_send, &port) != U6913_OK) return 1;
    if (u6913_request_version(&motor) != U6913_OK) return 1;
    if (u6913_request_standard_status(&motor) != U6913_OK) return 1;
    return 0;
}
