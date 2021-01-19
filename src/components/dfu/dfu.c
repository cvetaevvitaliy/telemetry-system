#include "dfu.h"
#include "stm32f4xx_hal.h"
#include "cli.h"

static CLI_Result_t _dfu(void);
typedef void (*ptr)(void);

uint8_t dfu_add_cli_cmd(void)
{
    return cli_add_new_cmd("dfu", _dfu, 0, 0, "Reboot to DFU");
}


extern RTC_HandleTypeDef hrtc;

CLI_Result_t _dfu(void)
{
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, MAGIC_DFU_VALUE);
    HAL_Delay(10);
    NVIC_SystemReset();

    return CLI_OK;
}


inline void dfu_check(void)
{
    /** If magic value is found in BKP register - reboot to DFU */
    if (MAGIC_DFU_VALUE == HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0)) {
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0); // clear bkp register
        HAL_Delay(10);
        HAL_DeInit();
        HAL_RCC_DeInit();
        ptr jump_to_dfu = (ptr) *(volatile uint32_t *) (0x1FFF0000 + 0x04); // set pointer to jump DFU
        __set_MSP((*(volatile uint32_t *) 0x1FFF0000)); // Set new main Stack Pointer
        jump_to_dfu();
    }

}

