#include "main.h"
#include "cli.h"

static CLI_Result_t test_led(void);

void add_test_cli_cmd(void)
{
    /**
     * @brief Add command
     * @param name - input name
     * @param fcn - callback function
     * @param argc - min count arguments
     * @param mode - execute mode
     * @param descr - description
     * @return result append command
    * */
    cli_add_new_cmd("led", test_led, 1, 0, "enable LED_2 - led 0 or led 1");

}


CLI_Result_t test_led(void)
{
    uint8_t value;
    uint8_t argv_0 = cli_get_arg_dec (0);
    if (argv_0 != 0 && argv_0 != 1) {
        CLI_PRINTF("\nled <arg>: 1 or 0");
        return CLI_ArgErr;
    }

    if (argv_0)
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);

    return CLI_OK;
}
