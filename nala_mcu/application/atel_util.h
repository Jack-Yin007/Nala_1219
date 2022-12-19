/*
 * atel_util.h
 *
 *  Created on: Nov 12, 2019
 *      Author: G
 */

#ifndef __ATEL_UTIL_H_
#define __ATEL_UTIL_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

//#define ATEL_RING_BUFF_SIZE_BYTE (1024 + 128)
#define ATEL_RING_BUFF_SIZE_BYTE (1024 * 2)

typedef struct {
    uint8_t            Buffer[ATEL_RING_BUFF_SIZE_BYTE];
    uint32_t            Head;
    uint32_t            Tail;
    uint32_t            Size;
} atel_ring_buff_t;

void AtelWriteRingBuff(atel_ring_buff_t *Queue, uint8_t value);
void AtelWriteRingBuffEscape(atel_ring_buff_t *pQueue, uint8_t value);
uint8_t AtelReadRingBuff(atel_ring_buff_t *Queue);

#ifdef __cplusplus
}
#endif

#endif /* __ATEL_UTIL_H_ */
