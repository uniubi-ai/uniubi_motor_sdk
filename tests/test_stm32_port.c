#include "uniubi6913_stm32.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "line %d: %s\n", __LINE__, #x); exit(1); } } while (0)
static int hal_result;
static unsigned calls;
static FDCAN_TxHeaderTypeDef last_header;
static uint8_t last_data[64];
int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h, FDCAN_TxHeaderTypeDef *header, uint8_t *data)
{
    CHECK(h != NULL);
    ++calls;
    last_header = *header;
    memcpy(last_data, data, 64);
    return hal_result;
}
int main(void)
{
    FDCAN_HandleTypeDef hal = {0};
    u6913_stm32_port_t port = {&hal, FDCAN_BRS_ON};
    u6913_motor_t m;
    u6913_feedback_t out;
    FDCAN_RxHeaderTypeDef rx = {1, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_12, FDCAN_FD_CAN};
    uint8_t data[64] = {1, 0xE9, 179, 2, 1};
    CHECK(u6913_init(&m, 1, u6913_stm32_send, &port) == 0);
    CHECK(u6913_request_version(&m) == 0 && calls == 1);
    CHECK(last_header.Identifier == 0 && last_header.IdType == FDCAN_STANDARD_ID);
    CHECK(last_header.DataLength == FDCAN_DLC_BYTES_64 && last_header.FDFormat == FDCAN_FD_CAN);
    CHECK(last_header.BitRateSwitch == FDCAN_BRS_ON && last_data[1] == 0xE9 && last_data[11] == 0xE9);
    hal_result = 1;
    CHECK(u6913_disable(&m) == U6913_EIO);
    CHECK(u6913_stm32_parse(1, &rx, data, &out) == 0 && out.value.version.patch == 179);
    rx.DataLength = 12; /* 原始数字12并不是 HAL DLC */
    CHECK(u6913_stm32_parse(1, &rx, data, &out) == U6913_EFRAME);
    rx.DataLength = FDCAN_DLC_BYTES_48;
    data[1] = 0xFE; memset(data + 2, 0, 62);
    CHECK(u6913_stm32_parse(1, &rx, data, &out) == 0 && out.kind == U6913_FEEDBACK_FULL_STATUS);
    rx.DataLength = FDCAN_DLC_BYTES_32;
    data[1] = 0xE8; data[10] = 'S'; data[11] = 'N'; data[31] = 0xE8;
    CHECK(u6913_stm32_parse(1, &rx, data, &out) == 0 && out.kind == U6913_FEEDBACK_IDENTITY);
    CHECK(strcmp(out.value.identity.serial_number, "SN") == 0);
    rx.FDFormat = 0;
    CHECK(u6913_stm32_parse(1, &rx, data, &out) == U6913_EFRAME);
    CHECK(u6913_stm32_parse(1, NULL, data, &out) == U6913_EINVAL);
    puts("STM32 FDCAN adapter contract: PASS (mock HAL)");
    return 0;
}
