/**
 * @brief Compact & Simple Command Line Interface for microcontrollers
 *
 * Copyright (c) 2018 Vitaliy Nimych - vitaliy.nimych@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Modified 14 January 2019 - Vitaliy Nimych
 * add cli input log commands
 *
 * Modified 20 April 2019 - Vitaliy Nimych
 * add pars buttons: UP, DOWN, LEFT, RIGHT
 *
 * Modified 1 September 2019 - Vitaliy Nimych
 * add pars buttons: TAB, ESC, Ctrl + C
 *
 * Modified 27 December 2020 - Vitaliy Nimych
 * add pars buttons for clear screen : Ctrl + L, small code refactoring
 *
 * */

#include "cli.h"

#include "cli_queue.h"
#include "tinystring.h"
#include "cli_time.h"
#include "cli_log.h"
#include "cli_input.h"


#define PRINT_ARROW()                   {CLI_PRINTF("%s%s",STRING_TERM_ENTER, STRING_TERM_ARROW);}	// Output of input line
#define SEPARATOR_ASTERISK              ("\r\n***********************************************************")
#define SEPARATOR                       ("\r\n-----------------------------------------------------------")

// todo: need refactor this variable, maybe put to struct
char output_print_buffer[128];
volatile uint64_t _tick;
// need change address for another MCU, see datasheet for MCU (for stm32f4 0x1FFF7A10, stm32F1 0x1FFFF7E8)
static volatile uint32_t *UniqueID = (uint32_t *) 0x1FFF7A10;

/** @brief Command settings */
typedef struct {
    uint8_t (*fcn)();                   // callback function command
    const char *name;                   // name command
    uint8_t argc;                       // min count argument
    uint16_t mode;                      // mode execute command
    const char *description;            // description command
} CLI_Cmd_t;

typedef struct{
    uint8_t argc;
    char** argv;
} CLI_Params_t;

/** @brief CLI State */
struct {
    CLI_Cmd_t cmds[CLI_SIZE_MAX_CMD];   // list commands
    uint8_t countCommand;               // count commands
    uint8_t executeState;               // state terminal
    volatile CLI_Params_t inputArgs;    // args current execute command
    bool isEntered;
    bool first_in;
} CLI_State_s;

// **************** Callback for included in CLI commands *****************
static uint8_t help_cmd();              // print help
static uint8_t reboot_mcu();            // reboot mcu
static uint8_t print_cli_w(void);       // print welcome screen
// ************************************************************************

// ************************** static function *****************************
static CLI_Cmd_t *_find_cli_command(const char *cmdName);
static void _clear_screen(void);
static void _print_result_exec(uint8_t code);
static void _print_result_add_cmd(uint8_t code);
static void _print_boot_time(CLI_Time_t *time);
static int8_t _index_of_flag(const char *flag);
static void _cli_print_time();
static void _arg_destroy(CLI_Params_t* src);
static uint8_t _strPartCmp(const char* str1, const char* str2);
// ************************************************************************



// ************************* interrupt function ***************************

static bool _interrupt_operation = false; // todo: need implement - abort run current job
/** @brief Checking the status of the start of the operation (return, stator-on) */
inline bool cli_get_int_state(void) {

    bool res = _interrupt_operation;
    _interrupt_operation = false;
    return res;
}

// ************************************************************************

// ************************** CLI function ********************************

static void cli_welcome(void) {

    CLI_PRINTF(SEPARATOR_ASTERISK);
    CLI_PRINTF("\r\n| %s\t\t\t\t\t  |", BUILD_NAME);
    CLI_PRINTF("\r\n| SW  ver: v%s.%s.%s    MCU: %s \t\t\t  |", MINOR, MAJOR, PATCH, MCU);
    CLI_PRINTF("\r\n| CLI ver: %s    UID: %X-%X-%X\t  |", _TERM_VER_, UniqueID[0], UniqueID[1], UniqueID[2]);
    CLI_PRINTF("\r\n| Build Date: %s %s Where: %s\t  |", __DATE__, __TIME__, _WHERE_BUILD);
    CLI_PRINTF("\r\n| Branch: %s GIT-HASH: %s\t\t\t  |", GIT_BRANCH, GIT_HASH);
    CLI_PRINTF(SEPARATOR_ASTERISK);
    CLI_PRINTF("\r\n");
}

uint8_t sys_uptime(void)
{
    return CLI_OK;
}

/** @brief Terminal initialize */
void cli_init(void)
{

    cli_input_init();

    CLI_State_s.countCommand = 0;
    CLI_State_s.executeState = 0;
    CLI_State_s.isEntered = false;

    cli_add_new_cmd("help", help_cmd, 0, CLI_PrintNone, "help by CLI command");
    cli_add_new_cmd("welcome", print_cli_w, 0, CLI_PrintNone, "CLI welcome message");
    cli_add_new_cmd("boottime", sys_uptime, 0, CLI_PrintStartTime, "System BootTime");
    cli_add_new_cmd("reboot", reboot_mcu, 0, CLI_PrintNone, "reboot MCU");

    CLI_PRINTF("\r\n");

    CLI_State_s.inputArgs.argv = (char **) cli_malloc(sizeof(char *) * CLI_ARGS_BUF_SIZE);
    for (uint8_t i = 0; i < CLI_ARGS_BUF_SIZE; i++)
        CLI_State_s.inputArgs.argv[i] = cli_malloc(sizeof(char) * (CLI_ARG_SIZE + 1));

    cli_log_init();

    PRINT_ARROW();
}

/**
 * @brief Get index coinciding args string or -1
 * @param flag - searched string
* */
int8_t _index_of_flag(const char *flag)
{

    for (uint8_t i = 0; i < CLI_State_s.inputArgs.argc; i++) {
        if ( _strcmp(CLI_State_s.inputArgs.argv[i], flag)) {
            return i;
        }
    }

    return -1;
}


uint8_t _strPartCmp(const char* str1, const char* str2)
{
    uint8_t co = 0;
    while(((str1 + co) != NULL) && (*(str1 + co) != '\0') &&
          ((str2 + co) != NULL) && (*(str2 + co) != '\0')){

        if (str1[co] != str2[co])
            return 0;

        co++;
    }

    return 1;
}


uint16_t _getCountSeparator(const char* strSrc, const char* separator)
{
    uint8_t i = 0;
    uint16_t result = 0;
    uint16_t count_separator = _strlen((char*) separator);

    for(; strSrc[i] != '\0'; i++)
    {
        uint16_t j = 0;
        for(; j < count_separator; j++)
            if (strSrc[i] == separator[j]){
                result++;
                break;
            }
    }

    return result + 1;
}


void _arg_destroy(CLI_Params_t* src)
{
    src->argc = 0;
}


void _split(char* strSrc, const char* separator, CLI_Params_t* dst)
{
    uint8_t count_sep = _getCountSeparator(strSrc, separator);
    uint16_t lenSep = _strlen((char*)separator);

    dst->argc = count_sep;

    int start_index = 0;
    int size_word = 0;

    int co = 0;
    int co_arg = 0;
    while(1)
    {

        uint16_t s = 0;

        for(; s < lenSep; s++){
            if((strSrc[co] == separator[s]) || (strSrc[co] == '\0'))
            {
                size_word = co - start_index;

                dst->argv[co_arg][size_word] = '\0';
                _strcpy(strSrc, start_index, dst->argv[co_arg], 0, size_word);

                start_index = co + 1;
                co_arg++;
                break;
            }
        }

        if (strSrc[co] == '\0')
            break;

        co++;
    }
}

#define CLI_GetDecString(str) ((uint32_t)strtol((const char *)str, NULL, 10))
#define CLI_GetHexString(str) ((uint32_t)strtol((const char *)str, NULL, 16))
#define CLI_GetArgString(str) ((uint32_t)_strcmp((const char *)str, NULL, 16))

/** @brief Get argument in Dec */
inline uint32_t cli_get_arg_dec(uint8_t index)
{
    return CLI_GetDecString(CLI_State_s.inputArgs.argv[index + 1]);
}

/** @brief Get argument in Hex */
inline uint32_t cli_get_arg_hex(uint8_t index)
{
    return CLI_GetHexString(CLI_State_s.inputArgs.argv[index + 1]);
}

/** @brief Get argument in str */
int cli_get_arg_str(uint8_t index, char *str)
{
    return _strcmp(CLI_State_s.inputArgs.argv[index + 1], str);
}

bool cli_get_arg_dec_by_flag(const char *flag, uint32_t *outValue)
{

    int8_t w = _index_of_flag(flag);

    if ((w > 0) && (w + 1 < CLI_State_s.inputArgs.argc)) {
        *outValue = cli_get_arg_dec(w);
        return true;
    }

    return false;
}

bool cli_get_arg_hex_by_flag(const char *flag, uint32_t *outValue)
{

    int8_t w = _index_of_flag(flag);

    if ((w > 0) && (w + 1 < CLI_State_s.inputArgs.argc)) {
        *outValue = cli_get_arg_hex(w);
        return true;
    }

    return false;
}

bool cli_is_arg_flag(const char *flag)
{
    return _index_of_flag(flag) >= 0;
}

/**
 * @brief Execute command
 * @param argv argument strings
 * @param argc count argument strings
 * @return result execute command
* */
CLI_Result_t _execute_cli_cmd(char **argv, uint8_t argc)
{

    if ( argc < 1 )
        return CLI_ArgErr;

    CLI_Cmd_t *cmd = _find_cli_command(argv[0]);

    if ( cmd != NULL) {

        if ((argc - 1) < cmd->argc )
            return CLI_ArgErr;

        CLI_State_s.executeState = 1;

        if ( cmd->mode & CLI_PrintStartTime )
            _cli_print_time();

        uint32_t startMs = CLI_GETMS();
        CLI_Result_t result = cmd->fcn(argv, argc);
        uint32_t stopMs = CLI_GETMS();

        if ( cmd->mode & CLI_PrintStopTime )
            _cli_print_time();

        if ( cmd->mode & CLI_PrintDiffTime ) {
            CLI_Time_t t = cli_time_get_time_ms(stopMs - startMs);
            _print_boot_time(&t);
        }

        CLI_State_s.executeState = 0;

        return result;
    }

    return CLI_NotFound;
}

/**
 * @brief Execute command
 * @param str command const string include arguments
 * @return result execute command
* */
CLI_Result_t ExecuteString(const char *str)
{

    _split((char *) str, " ", (CLI_Params_t *) &CLI_State_s.inputArgs);

    CLI_Result_t result = _execute_cli_cmd(CLI_State_s.inputArgs.argv, CLI_State_s.inputArgs.argc);

    _arg_destroy((CLI_Params_t *) &CLI_State_s.inputArgs);

#if (CLI_PRINT_ERROR_EXEC_EN == 1)
    _print_result_exec(result);
#endif

    PRINT_ARROW();

    return result;
}

/** @brief Execute CLI */
bool cli_loop_service()
{

    if (CLI_State_s.isEntered == true ) {
        ExecuteString((const char *) cli_input_get_buffer(TransitBuffer));
        CLI_State_s.isEntered = false;

        return true;
    }

    return false;
}

/**
 * @brief Add command
 * @param name - input name
 * @param fcn - callback function
 * @param argc - min count arguments
 * @param mode - execute mode
 * @param descr - description
 * @return result append command
* */
CLI_Add_Result_t cli_add_new_cmd(const char *name, uint8_t (*fcn)(), uint8_t argc, uint16_t mode, const char *descr)
{

    if (CLI_State_s.countCommand >= CLI_SIZE_MAX_CMD) {
        _print_result_add_cmd(ADD_CMD_MaxCmd);
        return ADD_CMD_MaxCmd;
    }

    if ( fcn == NULL) {
        _print_result_add_cmd(ADD_CMD_FcnNull);
        return ADD_CMD_FcnNull;
    }

    if ( _strlen((char *) name) == 0 ) {
        _print_result_add_cmd(ADD_CMD_EmptyName);
        return ADD_CMD_EmptyName;
    }

    uint8_t i = 0;
    for (; i < CLI_State_s.countCommand; i++)
        if ( _strcmp((char *) CLI_State_s.cmds[i].name, (char *) name)) {
            _print_result_add_cmd(ADD_CMD_RetryName);
            return ADD_CMD_RetryName;
        }

    uint8_t countCmd = CLI_State_s.countCommand;
    CLI_State_s.cmds[countCmd].fcn = fcn;
    CLI_State_s.cmds[countCmd].name = name;
    CLI_State_s.cmds[countCmd].argc = argc;
    CLI_State_s.cmds[countCmd].mode = mode;
    CLI_State_s.cmds[countCmd].description = descr;
    CLI_State_s.countCommand++;

    return ADD_CMD_OK;
}

/**
 * @brief Print result execute command
 * @param code - result code
 * @return none
 * */
static void _print_result_exec(uint8_t code)
{
    switch (code) {
        case CLI_NotFound:
            CLI_PRINTF ("\n\rCommand not found");
            break;
        case CLI_ArgErr:
            CLI_PRINTF ("\n\rFault argument");
            break;
        case CLI_ExecErr:
            CLI_PRINTF ("\n\rExecute functions");
            break;
        case CLI_WorkInt:
            CLI_PRINTF ("\n\rCommand abort");
            break;
        default:
            break;
    }
}

/**
 * @brief Print result add command action
 * @param code - result code
 * @return none
 * */
static void _print_result_add_cmd(uint8_t code)
{
#if (CLI_PRINT_ERROR_ADD_CMD_EN == 1)
    switch (code) {
        case ADD_CMD_MaxCmd:
            CLI_PRINTF ("\n\rAdd cmd err: Memory is full");
            break;
        case ADD_CMD_FcnNull:
            CLI_PRINTF ("\n\rAdd cmd err: Function callback is NULL");
            break;
        case ADD_CMD_EmptyName:
            CLI_PRINTF ("\n\rAdd cmd err: Empty name command");
            break;
        case ADD_CMD_RetryName:
            CLI_PRINTF ("\n\rAdd cmd err: Retry name command");
            break;
        default:
            break;
    }
#endif
}

/**
 * @brief Search command by name
 * @return Command pointer or NULL
 * */
CLI_Cmd_t *_find_cli_command(const char *cmdName)
{
    uint8_t i = 0;
    for (; i < CLI_State_s.countCommand; i++) {
        char *name1 = (char *) CLI_State_s.cmds[i].name;
        char *name2 = (char *) cmdName;

        int res = _strcmp(name1, name2);

        if ( res )
            return &CLI_State_s.cmds[i];
    }

    return NULL;
}

/**
 * @brief Search command by name
 * @return Command pointer or NULL
* */
CLI_Cmd_t *_find_part_term_cmd(const char *cmdName)
{
    CLI_Cmd_t *result = NULL;

    uint8_t i = 0;
    for (; i < CLI_State_s.countCommand; i++) {
        char *name1 = (char *) CLI_State_s.cmds[i].name;
        char *name2 = (char *) cmdName;

        int res = _strPartCmp(name1, name2);

        if ( res ) {
            if ( result != NULL)
                return NULL;

            result = &CLI_State_s.cmds[i];
        }
    }

    return result;
}

// ************************************************************************

// *************************   sys cmd CLI    *****************************

uint8_t help_cmd()
{
    CLI_PRINTF("\r\nCount command: %d", (int) CLI_State_s.countCommand ); // -1 HELP
    CLI_PRINTF(SEPARATOR);

    for (uint16_t i = 0; i < CLI_State_s.countCommand; i++) {
        CLI_PRINTF("\r\n%-10s - %s", CLI_State_s.cmds[i].name, CLI_State_s.cmds[i].description);
        CLI_PRINTF(SEPARATOR);
    }

    return CLI_OK;
}

uint8_t reboot_mcu(void)
{
    CLI_PRINTF("\r\nreset MCU\r\n")
    HAL_Delay(1000);
    HAL_NVIC_SystemReset();
    return CLI_OK;
}

uint8_t print_cli_w(void)
{
    cli_welcome();
    return CLI_OK;
}

// ************************************************************************

void _print_boot_time(CLI_Time_t *time)
{
    CLI_PRINTF("\r\n%02dh:%02dm:%02ds.%03dms", (int) time->hour, (int) time->minute, (int) time->second, (int) time->msec);
}

/**
 * @brief CLI Print Time
 * @return none
* */
void _cli_print_time()
{
#if (CLI_TIMELEFT_EN == 1)
    uint32_t ms = CLI_GETMS();
    CLI_Time_t tv = cli_time_get_plus_time_ms(ms);
    _print_boot_time(&tv);
#endif
}

void cli_set_first_in_cli(bool set)
{
    CLI_State_s.first_in = set;
}

void _clear_screen(void)
{
    CLI_PRINTF("\033[2J");      /* Clear the entire screen. */
    CLI_PRINTF("\033[0;0f");    /* Move cursor to the top left hand corner */
}

/** @brief Append new symbols */
CLI_Append_Result_t cli_append_char(char ch)
{
    static bool rstUnlock = false;
    if ( rstUnlock )
        rstUnlock = false;

    CLI_InputValue_t iv = cli_input_put_char(ch);
    char c = iv.keyCode;

    if ( iv.isValid ) {
        switch (c) {
            case CLI_KEY_ENTER: {
                if (CLI_State_s.first_in == false ) {
                    cli_welcome();
                    CLI_State_s.first_in = true;
                }
                if ( cli_input_is_empty()) {
                    PRINT_ARROW();
                    return CLI_APPEND_Ignore;
                }
                CLI_State_s.isEntered = true;

                cli_input_cache();

                cli_log_cmd_push(cli_input_get_buffer(MainBuffer));
                cli_log_cur_reset();

                cli_input_reset();

                return CLI_APPEND_Enter;
            }
                break;

            case CLI_KEY_ESCAPE:
            case CHAR_INTERRUPT:
                _interrupt_operation = true;
                CLI_PRINTF("\r\nINTERRUPT\n");
                break;

            case CLI_KEY_BACKSPACE:
                cli_input_backspace();
                break;

            case CLI_KEY_TAB: {
                if ((!cli_input_is_empty()) && (cli_input_get_last_char() != ' ')) {
                    char *buf = cli_input_get_buffer(MainBuffer);
                    CLI_Cmd_t *cmd = _find_part_term_cmd(buf);

                    if ( cmd != NULL) {
                        uint8_t len = _strlen(cmd->name);

                        cli_input_set_buffer(MainBuffer, (char *) cmd->name, len + 1);
                        cli_input_refresh(strcat(buf, " ")); // todo: need implement light strcat func
                    }
                }
            }
                break;

            case CLI_KEY_DOWN: {
                const char *ptrCmd = cli_log_get_next_cmd();
                if ( ptrCmd != NULL)
                    cli_input_refresh(ptrCmd);
            }
                break;

            case CLI_KEY_UP: {
                const char *ptrCmd = cli_log_get_last_cmd();
                if ( ptrCmd != NULL)
                    cli_input_refresh(ptrCmd);
            }
                break;

            case CLI_KEY_LEFT:
                cli_input_cursor_to_left();
                break;

            case CLI_KEY_RIGHT:
                cli_input_cursor_to_right();
                break;

            case CLI_KEY_DEL:
                cli_input_delete();
                break;

            case CLI_KEY_HOME:
                cli_input_cursor_to_home();
                break;

            case CLI_KEY_END:
                cli_input_cursor_to_end();
                break;

            case CLI_KEY_CLEAR_SCR:
                _clear_screen();
                CLI_PRINTF(STRING_TERM_ARROW);
                break;

            default: {
                if ( iv.isAlphaBet && !cli_input_is_full())
                    cli_input_add_char(c);
            }
                break;
        }
    } else {
        return CLI_APPEND_BufFull;
    }

    return CLI_APPEND_OK;
}


void SysTick_CLI(void)
{
    _tick++;
}