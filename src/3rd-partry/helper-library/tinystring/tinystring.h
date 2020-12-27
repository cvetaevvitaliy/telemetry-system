#ifndef _CLI_STRING_A_H
#define _CLI_STRING_A_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>


void _strcpy(const char* src, uint16_t offsetSrc, char* dst, uint16_t offsetDst, uint16_t length);
uint8_t _strcmp(const char* str1, const char* str2);
uint32_t _strlen(const char* strSrc);



#endif // _CLI_STRING_A_H
