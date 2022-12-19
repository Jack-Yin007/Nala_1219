#ifndef __ACC_TYPES_H__
#define __ACC_TYPES_H__

#include "stdint.h"

typedef struct
{
    int16_t     x;
    int16_t     y;
    int16_t     z;
} accValues_t;

typedef struct
{
    int32_t     x;
    int32_t     y;
    int32_t     z;
} accValuesCalc_t;

#endif	// #ifndef __ACC_TYPES_H__
