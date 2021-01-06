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
 * add pars buttons: Ctrl + L, small code refactoring
 *
 * */

#include "cli.h"
#include "cli_input.h"
#include "cli_queue.h"
#include "tinystring.h"

#define INPUT_COUNT_BUFFER      (2)

typedef struct
{
    char Data[CLI_CMD_BUF_SIZE + 1];            // buffer
    int16_t CursorInBuffer;                     // cursor position
    int16_t BufferCount;                        // count entered symbols
}Buffer_t;

struct
{
    Buffer_t Buffers[INPUT_COUNT_BUFFER];       // buffers commands
    Buffer_t* CurBuffer;                        // processing buffer
    CLI_Queue_t Symbols;                        // queue symbols input
    CLI_InputBufferType_t CurrentBuffer;        // current processing buffer
}CLI_Input_s;

static void _add_char(char c)
{
    CLI_PUT_CHAR(c);
    
    CLI_Input_s.CurBuffer->Data[CLI_Input_s.CurBuffer->CursorInBuffer] = c;
    CLI_Input_s.CurBuffer->BufferCount++;
    CLI_Input_s.CurBuffer->CursorInBuffer++;
    CLI_Input_s.CurBuffer->Data[CLI_Input_s.CurBuffer->BufferCount] = '\0';
}

static void _rem_char(void)
{
    CLI_PUT_CHAR(CLI_KEY_BACKSPACE);
    CLI_PUT_CHAR(' ');
    CLI_PUT_CHAR(CLI_KEY_BACKSPACE);
    
    CLI_Input_s.CurBuffer->CursorInBuffer--;
    CLI_Input_s.CurBuffer->BufferCount--;
    CLI_Input_s.CurBuffer->Data[CLI_Input_s.CurBuffer->BufferCount] = '\0';
}

void cli_input_refresh(const char* newCmd)
{
    CLI_PUT_CHAR('\r');
    CLI_PRINTF(STRING_TERM_ARROW);

    if (CLI_Input_s.CurBuffer->Data != newCmd)
    {
        uint32_t lenNewCmd = _strlen(newCmd);
        uint32_t lenCurCmd = CLI_Input_s.CurBuffer->BufferCount;
        memcpy(CLI_Input_s.CurBuffer->Data, newCmd, lenNewCmd);
        
        CLI_Input_s.CurBuffer->BufferCount = lenNewCmd;
        CLI_Input_s.CurBuffer->CursorInBuffer = lenNewCmd;

        for(uint8_t i = 0; i < lenNewCmd; i++)
        {
            CLI_PUT_CHAR(CLI_Input_s.CurBuffer->Data[i]);
        }

        uint8_t cntSpcChar = 0;
        for(uint8_t i = lenNewCmd; i < lenCurCmd; i++)
        {
            CLI_PUT_CHAR(' ');
            cntSpcChar++;
        }
        
        for(uint8_t i = 0; i < cntSpcChar; i++)
            {CLI_PUT_CHAR(CLI_KEY_BACKSPACE);}
        
    }
    else
    {
        for(uint8_t i = 0; i < CLI_Input_s.CurBuffer->BufferCount; i++)
        {
            CLI_PUT_CHAR(CLI_Input_s.CurBuffer->Data[i]);
        }
    }
}

bool cli_input_is_empty(void)
{
    return CLI_Input_s.CurBuffer->BufferCount == 0;
}

bool cli_input_is_full(void)
{
    return CLI_Input_s.CurBuffer->BufferCount >= CLI_CMD_BUF_SIZE;
}

void cli_input_rem_char(void)
{

    if (CLI_Input_s.CurBuffer->CursorInBuffer != CLI_Input_s.CurBuffer->BufferCount)
    {
        // save current position cursor
        uint8_t tmpPos = CLI_Input_s.CurBuffer->CursorInBuffer - 1;

        memcpy(CLI_Input_s.Buffers[TransitBuffer].Data, CLI_Input_s.CurBuffer->Data, tmpPos);
        memcpy(CLI_Input_s.Buffers[TransitBuffer].Data + tmpPos, CLI_Input_s.CurBuffer->Data + tmpPos + 1, CLI_Input_s.CurBuffer->BufferCount - tmpPos);
        
        CLI_Input_s.Buffers[TransitBuffer].Data[CLI_Input_s.CurBuffer->BufferCount - 1] = '\0';
        
        cli_input_refresh(CLI_Input_s.Buffers[TransitBuffer].Data);

        for(uint8_t pos = 0; pos < CLI_Input_s.CurBuffer->BufferCount - tmpPos; pos++)
        {
            CLI_PUT_CHAR(CLI_KEY_LSHIFT);
            CLI_Input_s.CurBuffer->CursorInBuffer--;
        }
    }
    else
    {
        _rem_char();
    }
}

void cli_input_add_char(char c)
{
    if (CLI_Input_s.CurBuffer->CursorInBuffer != CLI_Input_s.CurBuffer->BufferCount)
    {
        uint8_t tmpPos = CLI_Input_s.CurBuffer->CursorInBuffer;
        memcpy(CLI_Input_s.Buffers[TransitBuffer].Data, CLI_Input_s.CurBuffer->Data, tmpPos);
        memcpy(CLI_Input_s.Buffers[TransitBuffer].Data + tmpPos, &c, 1);
        memcpy(CLI_Input_s.Buffers[TransitBuffer].Data + tmpPos + 1, CLI_Input_s.CurBuffer->Data + tmpPos, CLI_Input_s.CurBuffer->BufferCount - tmpPos);
        CLI_Input_s.Buffers[TransitBuffer].Data[CLI_Input_s.CurBuffer->BufferCount + 1] = '\0';
        
        CLI_Input_s.CurBuffer->BufferCount++;
        CLI_Input_s.CurBuffer->CursorInBuffer++;
        CLI_Input_s.CurBuffer->Data[CLI_Input_s.CurBuffer->BufferCount] = '\0';

        tmpPos++;
        cli_input_refresh(CLI_Input_s.Buffers[TransitBuffer].Data);

        for(uint8_t pos = 0; pos < CLI_Input_s.CurBuffer->BufferCount - tmpPos; pos++)
        {
            CLI_PUT_CHAR(CLI_KEY_LSHIFT);
            CLI_Input_s.CurBuffer->CursorInBuffer--;
        }
    }
    else
    {
        _add_char(c);
    }

}

void cli_input_init(void)
{
    for(uint32_t i = 0; i < INPUT_COUNT_BUFFER; i++)
    {
        CLI_Input_s.Buffers[i].Data[0] = '\0';
        CLI_Input_s.Buffers[i].BufferCount = 0;
        CLI_Input_s.Buffers[i].CursorInBuffer = 0;
    }
    
    CLI_Input_s.CurBuffer = &CLI_Input_s.Buffers[MainBuffer];
    
    cli_queue_init(&CLI_Input_s.Symbols, 3, sizeof(char), QUEUE_FORCED_PUSH_POP_Msk);
    
    for(uint8_t i = 0; i < 3; i++)
    {
        char c = 0;
        cli_queue_push(&CLI_Input_s.Symbols, &c);
    }
}

static const uint8_t arr_up[]       = {0x1B, 0x5B, 0x41}; // Key code button "UP"
static const uint8_t arr_down[]     = {0x1B, 0x5B, 0x42}; // Key code button "DOWN"
static const uint8_t arr_right[]    = {0x1B, 0x5B, 0x43}; // Key code button "RIGHT"
static const uint8_t arr_left[]     = {0x1B, 0x5B, 0x44}; // Key code button "LEFT"
static const uint8_t arr_esc[]      = {0x1B, 0x1B, 0x1B}; // Key code button "ESC"
static const uint8_t del[]          = {0x1B, 0x5B, 0x33}; // Key code button "Backspace"
static const uint8_t home[]         = {0x1B, 0x5B, 0x31}; // Key code button "HOME"
static const uint8_t end[]          = {0x1B, 0x5B, 0x34}; // Key code button "END"
    
CLI_InputValue_t cli_input_put_char(char c)
{
    CLI_InputValue_t iv;
    
    cli_queue_push(&CLI_Input_s.Symbols, &c);
    
    if (cli_queue_is_equal(&CLI_Input_s.Symbols, arr_up, 3))            {c = CLI_KEY_UP;}
    else if (cli_queue_is_equal(&CLI_Input_s.Symbols, arr_down, 3))     {c = CLI_KEY_DOWN;}
    else if (cli_queue_is_equal(&CLI_Input_s.Symbols, arr_right, 3))    {c = CLI_KEY_RIGHT;}
    else if (cli_queue_is_equal(&CLI_Input_s.Symbols, arr_left, 3))     {c = CLI_KEY_LEFT;}
    else if (cli_queue_is_equal(&CLI_Input_s.Symbols, arr_esc, 3))      {c = CLI_KEY_ESCAPE;}
    else if (cli_queue_is_equal(&CLI_Input_s.Symbols, del, 3))          {c = CLI_KEY_DEL;}
    else if (cli_queue_is_equal(&CLI_Input_s.Symbols, home, 3))         {c = CLI_KEY_HOME;}
    else if (cli_queue_is_equal(&CLI_Input_s.Symbols, end, 3))          {c = CLI_KEY_END;}
    
    iv.isValid = ((CLI_Input_s.CurBuffer->BufferCount < CLI_CMD_BUF_SIZE) ||
                        (c == CLI_KEY_BACKSPACE) ||
                        (c == CLI_KEY_ENTER)	||
                        (c == CHAR_INTERRUPT));
    if ( (c != '\n') && (c != '\r')) // drop characters \r or \n
        iv.isAlphaBet = c;
                    
    iv.keyCode = c;
    return iv;
}

void cli_input_cache(void)
{
    CLI_Input_s.CurBuffer->Data[CLI_Input_s.CurBuffer->BufferCount] = '\0';
    memcpy(CLI_Input_s.Buffers[TransitBuffer].Data, CLI_Input_s.CurBuffer->Data, CLI_Input_s.CurBuffer->BufferCount + 1);
}

void cli_input_reset(void)
{
    CLI_Input_s.CurBuffer->CursorInBuffer = CLI_Input_s.CurBuffer->BufferCount = 0;
    CLI_Input_s.CurBuffer->Data[CLI_Input_s.CurBuffer->BufferCount] = '\0';
}

char cli_input_get_last_char(){ return CLI_Input_s.CurBuffer->Data[CLI_Input_s.CurBuffer->BufferCount - 1];	}

void cli_input_cursor_to(uint16_t pos){ CLI_Input_s.CurBuffer->CursorInBuffer = pos; }

void cli_input_cursor_shift(int16_t shift){ CLI_Input_s.CurBuffer->CursorInBuffer += shift; }

char* cli_input_get_buffer(CLI_InputBufferType_t type){ return CLI_Input_s.Buffers[type].Data; }

void cli_input_set_buffer(CLI_InputBufferType_t type, char* buffer, uint32_t len)
{
    memcpy(CLI_Input_s.Buffers[type].Data, buffer, len);
    CLI_Input_s.CurBuffer->BufferCount = CLI_Input_s.CurBuffer->CursorInBuffer = len;
}

void cli_input_cursor_to_home(void)
{
    while(CLI_Input_s.CurBuffer->CursorInBuffer > 0)
    {
        CLI_PUT_CHAR(CLI_KEY_LSHIFT);
        cli_input_cursor_shift(-1);
    }
}

void cli_input_cursor_to_end(void)
{
    while(CLI_Input_s.CurBuffer->CursorInBuffer < CLI_Input_s.CurBuffer->BufferCount)
    {
        CLI_PUT_CHAR(CLI_Input_s.CurBuffer->Data[CLI_Input_s.CurBuffer->CursorInBuffer]);
        cli_input_cursor_shift(1);
    }
}

void cli_input_cursor_to_left(void)
{
    if (CLI_Input_s.CurBuffer->CursorInBuffer > 0)
    {
        cli_input_cursor_shift(-1);
        CLI_PUT_CHAR(CLI_KEY_LSHIFT);
    }
}

void cli_input_cursor_to_right(void)
{
    if (CLI_Input_s.CurBuffer->CursorInBuffer < CLI_Input_s.CurBuffer->BufferCount)
    {
        CLI_PUT_CHAR(CLI_Input_s.CurBuffer->Data[CLI_Input_s.CurBuffer->CursorInBuffer]);
        cli_input_cursor_shift(1);
    }	
}

void cli_input_delete(void)
{
    if ((CLI_Input_s.CurBuffer->CursorInBuffer != CLI_Input_s.CurBuffer->BufferCount) && (!cli_input_is_empty()))
    {
        cli_input_cursor_shift(1);
        if(CLI_Input_s.CurBuffer->CursorInBuffer != CLI_Input_s.CurBuffer->BufferCount)
        {
            CLI_PUT_CHAR(CLI_Input_s.CurBuffer->Data[CLI_Input_s.CurBuffer->CursorInBuffer - 1]);
        }
        cli_input_rem_char();
    }	
}

void cli_input_backspace(void)
{
    if (!cli_input_is_empty() && (CLI_Input_s.CurBuffer->CursorInBuffer > 0))
        cli_input_rem_char();
}
