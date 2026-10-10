/* 测试替身，不是 STM32 HAL 实现，不用于产品固件。 */
#ifndef MOCK_HAL_H
#define MOCK_HAL_H
#include <stdint.h>
typedef struct { int unused; } FDCAN_HandleTypeDef;
typedef struct {
    uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator;
    uint32_t BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef struct {
    uint32_t Identifier, IdType, RxFrameType, DataLength, FDFormat;
} FDCAN_RxHeaderTypeDef;
#define FDCAN_STANDARD_ID 0u
#define FDCAN_DATA_FRAME 0u
#define FDCAN_DLC_BYTES_12 0x00090000u
#define FDCAN_DLC_BYTES_32 0x000D0000u
#define FDCAN_DLC_BYTES_48 0x000E0000u
#define FDCAN_DLC_BYTES_64 0x000F0000u
#define FDCAN_ESI_ACTIVE 0u
#define FDCAN_BRS_ON 0x00100000u
#define FDCAN_BRS_OFF 0u
#define FDCAN_FD_CAN 0x00200000u
#define FDCAN_NO_TX_EVENTS 0u
#define HAL_OK 0
int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *, FDCAN_TxHeaderTypeDef *, uint8_t *);
#endif
