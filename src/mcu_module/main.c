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

#include <stdint.h>
#include "hardware_init.h"
#include "cli.h"
#include "range_test.h"
#include "cmsis_os.h"
#include "gps_service.h"
#include "data_struct_def.h"
#include "dfu.h"

#define QUEUE_SIZE                      (10)

osThreadId cli_task_handle;
osThreadId range_test_task_handle;
osThreadId gps_task_handle;
osThreadId transceiver_task_handle;

void cli_task(void const * argument);
void range_test_task(void const * argument);
void gps_task(void const * argument);
void transceiver_task(void const * argument);

osPoolDef(gps_pool, QUEUE_SIZE, GPS_Data_t);
osPoolId  gps_pool;
osMessageQDef(MsgBox_GPS, QUEUE_SIZE, sizeof(GPS_Data_t));
osMessageQId  MsgBox_GPS;

int main(void)
{
    hardware_init();

    cli_init();
    dfu_add_cli_cmd();

    osThreadDef(CLI_Task, cli_task, osPriorityLow, 0, 128);
    cli_task_handle = osThreadCreate(osThread(CLI_Task), NULL);

    osThreadDef(RangeTest_Task, range_test_task, osPriorityNormal, 0, 128);
    range_test_task_handle = osThreadCreate(osThread(RangeTest_Task), NULL);

    osThreadDef(GPS_Task, gps_task, osPriorityHigh, 0, 256);
    gps_task_handle = osThreadCreate(osThread(GPS_Task), NULL);

    osThreadDef(transceiver_task, transceiver_task, osPriorityHigh, 0, 128);
    transceiver_task_handle = osThreadCreate(osThread(transceiver_task), NULL);

    gps_pool = osPoolCreate(osPool(gps_pool));
    MsgBox_GPS = osMessageCreate(osMessageQ(MsgBox_GPS), NULL);


    osKernelStart();

    ULOG_ERROR("Kernel down\n");

    while (1) {


    }

}

void cli_task(void const * argument)
{

    while (1)
    {
        cli_loop_service();

        osDelay(100);
    }
}


void range_test_task(void const * argument)
{
    range_test_init();

    while (1)
    {
        //range_test_execute();

        osDelay(100);
    }
}



void gps_task(void const *argument)
{
    gps_service_init();


    while (1)
    {

        GPS_Data_t *GPS_Data = gps_service_execute();

        if (GPS_Data != NULL){
            GPS_Data_t  *queue;

            queue = osPoolAlloc(gps_pool);
            memcpy(queue, GPS_Data, sizeof(GPS_Data_t));

            osMessagePut(MsgBox_GPS, (uint32_t)queue, osWaitForever);
        }

        osDelay(100);
    }
}


void transceiver_task(void const * argument)
{
    osEvent  evt;
    GPS_Data_t  *receive_queue;

    while (1)
    {
        evt = osMessageGet(MsgBox_GPS, osWaitForever);
        if (evt.status == osEventMessage) {
            receive_queue = evt.value.p;
            // for debug only todo: need implement sending data to air via protocol
            //ULOG_DEBUG("latitude: %f\n", receive_queue->latitude);
            //ULOG_DEBUG("longitude: %f\n", receive_queue->longitude);
            osPoolFree(gps_pool, receive_queue);
        }

    }

}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
    /** Error_Handler_Debug */
    ULOG_ERROR("Error in: %s %d", file, line);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

