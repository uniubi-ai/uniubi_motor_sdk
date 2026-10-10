#include "uniubi6913_stm32.h"
#include <string.h>

int u6913_stm32_send(void *user, const u6913_frame_t *frame)
{
    u6913_stm32_port_t *port = (u6913_stm32_port_t *)user;
    FDCAN_TxHeaderTypeDef header = {0};
    uint8_t data[64];
    if (!port || !port->hfdcan || !frame || frame->can_id != U6913_HOST_CAN_ID ||
        frame->len != 64 || frame->data[0] != 0 ||
        (port->bit_rate_switch != FDCAN_BRS_ON && port->bit_rate_switch != FDCAN_BRS_OFF)) return -1;
    header.Identifier = frame->can_id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_64;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = port->bit_rate_switch;
    header.FDFormat = FDCAN_FD_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0;
    memcpy(data, frame->data, sizeof(data));
    return HAL_FDCAN_AddMessageToTxFifoQ(port->hfdcan, &header, data) == HAL_OK ? 0 : -1;
}

int u6913_stm32_parse(uint8_t expected_id, const FDCAN_RxHeaderTypeDef *header,
                      const uint8_t *data, u6913_feedback_t *out)
{
    size_t len;
    if (!header || !data || !out) return U6913_EINVAL;
    if (header->IdType != FDCAN_STANDARD_ID || header->RxFrameType != FDCAN_DATA_FRAME ||
        header->FDFormat != FDCAN_FD_CAN) return U6913_EFRAME;
    switch (header->DataLength) {
    case FDCAN_DLC_BYTES_12: len = 12; break;
    case FDCAN_DLC_BYTES_32: len = 32; break;
    case FDCAN_DLC_BYTES_48: len = 48; break;
    default: return U6913_EFRAME;
    }
    return u6913_parse_feedback(expected_id, header->Identifier, data, len, out);
}
