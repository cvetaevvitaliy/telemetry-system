/*
 * This file is part of "Telemetry system" project.
 *
 * "Telemetry system" are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * "Telemetry system" are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
#include "sd_cli_cmd.h"
#include "sd_card.h"
#include "cli.h"


static CLI_Result_t sd_cli_format(void);
static CLI_Result_t sd_cli_info(void);
static CLI_Result_t sd_cli_list_dir(void);
static CLI_Result_t sd_cli_cat_file(void);
static CLI_Result_t sd_cli_log(void);

static char readBuff[512] = {0};

uint8_t sd_card_cli_cmd_init(void)
{
    uint8_t res;
    /**
     * @brief Add command
     * @param name - input name
     * @param fcn - callback function
     * @param argc - min count arguments
     * @param mode - execute mode
     * @param descr - description
     * @return result append command
    * */
    res = cli_add_new_cmd("sd_format", sd_cli_format, 1, 0, "Format SD card");
    res = cli_add_new_cmd("sd_info", sd_cli_info, 1, 0, "Info SD card");
    res = cli_add_new_cmd("sd_list", sd_cli_list_dir, 1, 0, "List dir in the SD card");
    res = cli_add_new_cmd("sd_cat", sd_cli_cat_file, 1, 0, "Display file on screen");
    res = cli_add_new_cmd("sd_log", sd_cli_log, 1, 0, "Enable/Disable save log to SD card");

    return res;
}

void _sd_not_initialized(void)
{
    CLI_PRINTF("\nSD not initialized\n")
}


CLI_Result_t sd_cli_format(void)
{
    SD_Card_State_t *sd = sd_card_get_state();

    if (sd->insert)
    {
        sd->mount_fs = false;
        CLI_PRINTF("\nSD format\n");

        if (FR_OK == sd_card_format())
            sd->mount_fs = true;
    }
    else
    {
        _sd_not_initialized();
    }

    return CLI_OK;
}

CLI_Result_t sd_cli_info(void)
{
    SD_Card_State_t *sd = sd_card_get_state();

    if (sd->initialized)
    {
        CLI_PRINTF("\nSD info\n");
        sd_card_info();
    }
    else
    {
        _sd_not_initialized();
    }

    return CLI_OK;
}


FRESULT scan_files (char* path)
{
    CLI_PRINTF("\nList of files in the directory: %s\n\n", path);
    FRESULT res;
    SD_Card_State_t *sd_state = sd_card_get_state();

    res = f_opendir(&sd_state->dir, path); /* Open the directory */
    if (res == FR_OK)
    {
        for (;;)
        {
            res = f_readdir(&sd_state->dir, &sd_state->finfo); /* Read a directory item */
            if (res != FR_OK || sd_state->finfo.fname[0] == 0)
                break;

            if (!(sd_state->finfo.fattrib & AM_HID)) /* skip hidden file & directory */
            {
                if (sd_state->finfo.fattrib & AM_DIR)  /* It is a directory */
                {
                    CLI_PRINTF("\tDIR\t\t%s\n", sd_state->finfo.fname);
                } else
                {
                    CLI_PRINTF(" %10lu bytes\t%s\n", sd_state->finfo.fsize, sd_state->finfo.fname);
                }
                HAL_Delay(10);
            }
        }
        f_closedir(&sd_state->dir);
    }

    return res;
}

CLI_Result_t sd_cli_list_dir(void)
{

    SD_Card_State_t *sd = sd_card_get_state();
    //osMutexTake(sd->lock_file, osWaitForever);

    if (!sd->initialized)
    {
        _sd_not_initialized();
        return CLI_OK;
    }
    FRESULT res;


    char *arg = cli_get_arg(0);

    res = scan_files(arg);

    if (res == FR_OK)
    {
        return CLI_OK;
    }
    else if (res == FR_NO_PATH)
    {
        CLI_PRINTF("\nDirectory not found\n");
    }
    //osMutexRelease(sd->lock_file);

    return CLI_OK;
}


CLI_Result_t sd_cli_cat_file(void)
{
    static FIL fd;

    SD_Card_State_t *sd_state = sd_card_get_state();

    if (!sd_state->initialized)
    {
        _sd_not_initialized();
        return CLI_OK;
    }

    CLI_PRINTF("\n")
    FRESULT res;

    char *arg = cli_get_arg(0);

    osMutexTake(sd_state->lock_file, osWaitForever);

    res = f_open(&fd, arg, FA_READ);

    if (res != FR_OK)
    {
        CLI_PRINTF("\nError open file: %d\n", res);
        osMutexRelease(sd_state->lock_file);
        return CLI_OK;
    }

    memset(readBuff, 0, sizeof(readBuff));

    uint16_t lines;
    for (lines = 0; (f_eof(&fd) == 0); lines++)
    {
        f_gets((char*)readBuff, sizeof(readBuff), &fd);
        CLI_PRINTF("%s", readBuff);
        HAL_Delay(50);
    }
    CLI_PRINTF("\nRead %d lines\n", lines);


    f_close(&fd);
    osMutexRelease(sd_state->lock_file);

    return CLI_OK;
}


CLI_Result_t sd_cli_log(void)
{

    return CLI_OK;
}
