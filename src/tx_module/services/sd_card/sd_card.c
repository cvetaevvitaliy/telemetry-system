#include "sd_card.h"
#include "sd_cli_cmd.h"
#include "cli.h"


SD_Card_State_t SD_Card_State = {0};

extern SD_HandleTypeDef hsd;


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
            ULOG_INFO("init SD card succeed!\n");
            fres = f_mount(&SD_Card_State.fatfs, "", 1);

            HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B);

            if (fres != FR_OK)
            {
                ULOG_INFO("SD card is not formatted\n");
            } else
            {
                ULOG_INFO("Mount SD card successfully\n");
                sd_card_info();
                SD_Card_State.initialized = true;

                if(f_open(&SD_Card_State.fd, "version.txt", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
                {
                    f_printf(&SD_Card_State.fd, "%s\n", BUILD_NAME);
                    f_printf(&SD_Card_State.fd, "SW  ver: v%s.%s.%s\n", MINOR, MAJOR, PATCH);
                    f_printf(&SD_Card_State.fd, "CLI ver: %s\n", _TERM_VER_);
                    f_printf(&SD_Card_State.fd, "MCU: %s\n", MCU);
                    f_printf(&SD_Card_State.fd, "Build Date: %s %s Where: %s\n", __DATE__, __TIME__, _WHERE_BUILD);
                    f_printf(&SD_Card_State.fd, "Branch: %s GIT-HASH: %s\n", GIT_BRANCH, GIT_HASH);
                    f_sync(&SD_Card_State.fd);
                    f_close(&SD_Card_State.fd);
                }
            }
        }
        else
        {
            SD_Card_State.initialized = false;
            ULOG_ERROR("Failed init SD card\n");
        }
    }
    else
    {
        SD_Card_State.initialized = false;
        SD_Card_State.insert = false;
        ULOG_INFO("SD card not inserted\n");
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

    ULOG_DEBUG("\nStart formatting SD card\n");

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


char *_manID_to_str(uint8_t man_id)
{
    static char manufacturer[10];

    switch (man_id)
    {
        case 0x01:
            sprintf(manufacturer,"Panasonic");
            break;

        case 0x02:
            sprintf(manufacturer,"Toshiba");
            break;

        case 0x03:
            sprintf(manufacturer,"SanDisk");
            break;

        case 0x1b:
            sprintf(manufacturer,"Samsung");
            break;

        case 0x1d:
            sprintf(manufacturer,"AData");
            break;

        case 0x27:
            sprintf(manufacturer,"Phison");
            break;

        case 0x28:
            sprintf(manufacturer,"Lexar");
            break;

        case 0x31:
            sprintf(manufacturer,"Silicon Power");
            break;

        case 0x41:
            sprintf(manufacturer,"Kingston");
            break;

        case 0x74:
            sprintf(manufacturer,"Transcend");
            break;

        default:
            sprintf(manufacturer,"Unknown");
            break;

    }

    return manufacturer;

}

char *_speed_class_to_str(uint8_t class)
{
    static char speed_class[8];
    switch (class)
    {
        case 0x00:
            sprintf(speed_class,"Class 0");
            break;
        case 0x01:
            sprintf(speed_class,"Class 2");
            break;
        case 0x02:
            sprintf(speed_class,"Class 4");
            break;
        case 0x03:
            sprintf(speed_class,"Class 6");
            break;
        case 0x04:
            sprintf(speed_class,"Class 10");
            break;

        default:
            sprintf(speed_class,"Unknown");
            break;

    }

    return speed_class;

}


void sd_card_info(void)
{
    HAL_SD_CardCIDTypeDef CID;
    HAL_SD_CardStatusTypeDef card_status;
    BSP_SD_GetCardInfo(&hsd.SdCard);
    HAL_SD_GetCardCID(&hsd, &CID);
    HAL_SD_GetCardStatus(&hsd, &card_status);

    ULOG_INFO("\nManufacturer\t\t-> %s\n", _manID_to_str (CID.ManufacturerID));
    ULOG_INFO("Speed Class\t\t-> %s\n", _speed_class_to_str (card_status.SpeedClass));
    ULOG_INFO("Serial Number\t\t-> %04X\n", CID.ProdSN);
    ULOG_DEBUG("Block Size\t\t-> %dbyte\n", hsd.SdCard.BlockSize);
    ULOG_INFO("Card Capacity\t\t-> %uGB\n",
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
