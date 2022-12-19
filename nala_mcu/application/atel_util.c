/*
 * atel_util.c
 *
 *  Created on: Nov 12, 2019
 *      Author: G
 */

#include "atel_util.h"

uint8_t AtelReadRingBuff(atel_ring_buff_t *pQueue)
{
    uint8_t value;

    if (pQueue->Size == 0)
    {
        return 0;
    }

    value = pQueue->Buffer[pQueue->Head];
    pQueue->Head = (uint32_t)((pQueue->Head + 1) % ATEL_RING_BUFF_SIZE_BYTE);

    if (pQueue->Head <= pQueue->Tail)
    {
        pQueue->Size = (pQueue->Tail - pQueue->Head);
    }
    else
    {
        pQueue->Size = (ATEL_RING_BUFF_SIZE_BYTE + pQueue->Tail - pQueue->Head);
    }

    return value;
}

void AtelWriteRingBuff(atel_ring_buff_t *pQueue, uint8_t value)
{
    if (pQueue->Size >= ATEL_RING_BUFF_SIZE_BYTE)
    {
        return;
    }

    pQueue->Buffer[pQueue->Tail] = value;
    pQueue->Tail = (uint32_t)((pQueue->Tail + 1) % ATEL_RING_BUFF_SIZE_BYTE);

    if (pQueue->Head < pQueue->Tail)
    {
        pQueue->Size = (pQueue->Tail - pQueue->Head);
    }
    else
    {
        pQueue->Size = (ATEL_RING_BUFF_SIZE_BYTE + pQueue->Tail - pQueue->Head);
    }
}

void AtelWriteRingBuffEscape(atel_ring_buff_t *pQueue, uint8_t value)
{
    // if (pQueue->Size >= (ATEL_RING_BUFF_SIZE_BYTE - 1))
    //     return;

    if(value=='$')
    {
        AtelWriteRingBuff(pQueue, '!');
        AtelWriteRingBuff(pQueue, '0');
    }
    else if(value == '!')
    {
        AtelWriteRingBuff(pQueue, '!');
        AtelWriteRingBuff(pQueue, '1');
    }
    else
    {
        AtelWriteRingBuff(pQueue, value);
    }
}
