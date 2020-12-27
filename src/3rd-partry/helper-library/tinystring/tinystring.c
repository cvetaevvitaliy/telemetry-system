#include "tinystring.h"


void _strcpy(const char* src, uint16_t offsetSrc, char* dst, uint16_t offsetDst, uint16_t length)
{
    uint16_t i = 0;
    for(; i < length; i++)
        dst[i + offsetDst] = src[i + offsetSrc];
}


uint32_t _strlen(const char* strSrc)
{
    int32_t co = 0;
    while((strSrc != NULL) && (*strSrc != '\0')){
        strSrc++;
        co++;
    }

    return co;
}


uint8_t _strcmp(const char* str1, const char* str2)
{
    uint16_t co = 0;

	if (_strlen(str1) != _strlen(str2))
		return 0;

    while(((str1 + co) != NULL) && (*(str1 + co) != '\0') &&
          ((str2 + co) != NULL) && (*(str2 + co) != '\0')){

        if (str1[co] != str2[co])
            return 0;

        co++;
    }

    if   (
            (((str1 + co) == NULL) && ((str2 + co) != NULL)) ||
            (((str2 + co) == NULL) && ((str1 + co) != NULL)) ||
            (((str1 + co) != NULL) && ((str2 + co) != NULL) && (*(str2 + co) != *(str1 + co)))
         )
        return 0;


    return 1;
}
