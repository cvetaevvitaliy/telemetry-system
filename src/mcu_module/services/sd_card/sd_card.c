#include "sd_card.h"
#include "sd_cli_cmd.h"
#include "cli.h"


SD_Card_State_t SD_Card_State = {0};

extern SD_HandleTypeDef hsd;

static char const card_type[3][15] = {"CARD_SDSC\0","CARD_SDHC_SDXC\0","CARD_SECURED\0"};
static char const card_ver [2][10] = {"CARD_V1_0\0","CARD_V1_1\0"};


void sd_card_init(void)
{
    uint32_t wbytes; /* File write counts */

    sd_card_enable_pin_detect(false);

    if (!HAL_GPIO_ReadPin(SD_PRESET_GPIO_Port, SD_PRESET_Pin))
    {
        SD_Card_State.insert = true;

        FRESULT fres;

        if (BSP_SD_Init() == MSD_OK)
        {
            ULOG_DEBUG("init SD card succeed!\n");
            fres = f_mount(&SD_Card_State.fatfs, "", 1);

            HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B);

            if (fres != FR_OK)
            {
                ULOG_DEBUG("SD card is not formatted\n");
            } else
            {
                ULOG_DEBUG("Mount SD card successfully\n");
                sd_card_info();
                SD_Card_State.initialized = true;

                if(f_open(&SD_Card_State.fd, "tmp.txt", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
                {
                    uint8_t wtext[] = "Temporary file";
                    if(f_write(&SD_Card_State.fd, wtext, sizeof(wtext), (void *)&wbytes) == FR_OK);
                    {
                        f_close(&SD_Card_State.fd);
                    }
                }
            }
        }
        else
        {
            SD_Card_State.initialized = false;
            ULOG_DEBUG("Failed init SD card\n");
        }
    }
    else
    {
        SD_Card_State.initialized = false;
        SD_Card_State.insert = false;
        ULOG_DEBUG("SD card not inserted\n");
    }

    sd_card_enable_pin_detect(true);

}


void sd_card_deinit(void)
{
    SD_Card_State.initialized = false;
    BSP_SD_DeInit();
    sd_card_enable_pin_detect(true);
}


void sd_card_format(void)
{
    FRESULT fres;

    ULOG_DEBUG("Start formatting SD card\n");

    BSP_SD_DeInit();
    BSP_SD_Init();

    f_mount(&SD_Card_State.fatfs, "", 1);
    HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B);
    fres = f_mkfs("", FM_FAT32, 0, SD_Card_State.work, sizeof SD_Card_State.work);
    if (fres)
    {
        ULOG_ERROR("Formatting fall, error: %d\n ", fres);
    } else
    {
        ULOG_INFO("Formatting OK\n ");
    }

}

void sd_card_info(void)
{
    BSP_SD_GetCardInfo(&hsd.SdCard);
    ULOG_DEBUG("Card Type\t\t-> %s\n", card_type[hsd.SdCard.CardType]);
    ULOG_DEBUG("Card Version\t\t-> %s\n", card_ver[hsd.SdCard.CardVersion]);
    ULOG_DEBUG("Card Class\t\t-> 0x%X\n", hsd.SdCard.Class);
    ULOG_DEBUG("Block Size\t\t-> %dbyte\n", hsd.SdCard.BlockSize);
    ULOG_DEBUG("Card Capacity\t\t-> %uGB\n",
               (uint32_t) ((((float) hsd.SdCard.BlockNbr / 1000) * (float) hsd.SdCard.BlockSize / 1000000) + 0.5));

}

void sd_card_insert_event(void)
{
    sd_card_enable_pin_detect(false);

    if (SD_Card_State.insert)
    {
        SD_Card_State.insert = false;
    }
    else
    {
        SD_Card_State.insert = true;
    }

    ULOG_INFO("%s\n", SD_Card_State.insert ? "SD inserted" : "SD removed");

}

SD_Card_State_t *sd_card_get_state(void)
{
    return &SD_Card_State;
}

void sd_card_enable_pin_detect(bool state)
{
    if (state)
    {
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = SD_PRESET_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
        GPIO_InitStruct.Pull = GPIO_PULLUP;

        HAL_GPIO_Init(SD_PRESET_GPIO_Port, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    }
    else
    {
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
        HAL_GPIO_DeInit(SD_PRESET_GPIO_Port, (uint32_t)SD_PRESET_Pin);
    }

}
