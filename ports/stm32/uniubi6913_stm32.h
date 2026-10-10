#ifndef UNIUBI6913_STM32_H
#define UNIUBI6913_STM32_H
#include "uniubi6913.h"
/* 可在编译器中定义为对应系列 HAL 头文件，例如 stm32h7xx_hal.h。 */
#ifndef U6913_STM32_HAL_HEADER
#define U6913_STM32_HAL_HEADER "stm32g4xx_hal.h"
#endif
#include U6913_STM32_HAL_HEADER

#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    FDCAN_HandleTypeDef *hfdcan;
    uint32_t bit_rate_switch; /* FDCAN_BRS_ON/OFF，与设备配置一致 */
} u6913_stm32_port_t;

/* hfdcan 由 CubeMX/应用配置并启动，端口对象生命周期须覆盖 motor。 */
int u6913_stm32_send(void *user, const u6913_frame_t *frame);
/* HAL_FDCAN_GetRxMessage 后调用；header 的 DLC 由本函数转为真实字节数。
 * data 指向至少 DLC 所表示长度的缓冲区，HAL 接收建议分配 64 字节。 */
int u6913_stm32_parse(uint8_t expected_id, const FDCAN_RxHeaderTypeDef *header,
                      const uint8_t *data, u6913_feedback_t *out);
#ifdef __cplusplus
}
#endif
#endif
