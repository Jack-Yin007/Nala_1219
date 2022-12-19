/*
 * util.h
 *
 *  Created on: Dec 12, 2015
 *      Author: J
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <stddef.h>
#include <GenericTypeDefs.h>
//#include "em_int.h"
#include "user.h"

#define IRQ_OFF  INT_Disable()

#define IRQ_ON   INT_Enable()

//#define USE_LEUART
#define K_BYTES(n)      ((n) * 1024)

#ifndef MIN
#define MIN(x, y)               (((x) < (y)) ? (x) : (y))
#endif

#ifndef MAX
#define MAX(x, y)               (((x) > (y)) ? (x) : (y))
#endif

#ifndef	ARRAY_SIZE
#define	ARRAY_SIZE(x)   ((sizeof(x)) / (sizeof(x[0])))
#endif

// PUMAMCU-62
#ifndef UART1_RX_BUFFER_SIZE
#define UART1_RX_BUFFER_SIZE     280
#endif
#ifndef UART1_TX_BUFFER_SIZE
#define UART1_TX_BUFFER_SIZE     280
#endif
#define UART2_RX_BUFFER_SIZE     280
#define UART2_TX_BUFFER_SIZE     280
#ifdef _SIMBA2                   // SIMBAMCU-19
#define BLE_RX_BUFFER_SIZE    50 // Max BLE packet size is 512 
#define BLE_TX_BUFFER_SIZE    50 // Max BLE packet size is 512 
#endif

#define ARS_QUEUE_RESET(q)      ((q).head = (q).tail = -1)
#define ARS_QUEUE_INIT(q, s)    (ARS_QUEUE_RESET(q), (q).size = MIN((s), ARRAY_SIZE((q).queue)))
#define ARS_QUEUE_CLEAR(q)      ARS_QUEUE_INIT(q)
#define ARS_QUEUE_COUNT(q)      ((size_t)((q).tail - (q).head))
#define ARS_QUEUE_SIZE(q)       ((q).size)
#define ARS_QUEUE_IS_EMPTY(q)   ((q).head == (q).tail)  // or (0 == ARS_QUEUE_COUNT(q))
#define ARS_QUEUE_IS_FULL(q)    ((q).size == ARS_QUEUE_COUNT(q))
#define ARS_QUEUE_PUSH(q, e)    (ARS_QUEUE_IS_FULL(q) ? false : ((q).queue[++(q).tail % (q).size] = (e), true))
#define ARS_QUEUE_PUSH_CYCLIC(q, e) \
                                ((ARS_QUEUE_IS_FULL(q) ? ++(q).head : 0), (q).queue[++q.tail % (q).size] = (e), true)
/*  Safe version of macro to avoid GCC warning when the target buffer is on the stack (local variable):
warning: the comparison will always evaluate as 'false' for the address of 'data' will never be NULL [-Waddress]
*/
#define ARS_QUEUE_POP_SAFE(q, pe)    (ARS_QUEUE_IS_EMPTY(q) ? \
                                    false : \
                                    (*(pe) = (q).queue[++(q).head % (q).size], true) \
                                )
#define ARS_QUEUE_POP(q, pe)    ((NULL == (pe)) ? false : ARS_QUEUE_POP_SAFE(q, pe))

#define ARS_QUEUE_DEFINE_TYPE(qType, elType, max)  \
    typedef struct {                                \
        int       head;                             \
        int       tail;                             \
        size_t    size;                             \
        elType    queue[(max)];                     \
    } qType
#define ARS_QUEUE_DEFINE(qType, name)  \
    qType name

ARS_QUEUE_DEFINE_TYPE(uartGenQ_t, BYTE, K_BYTES(2));           // Generic type to be used as a prototype in the generix IRQ handler, size is not important here
ARS_QUEUE_DEFINE_TYPE(uartAppRxQ_t, BYTE, UART1_RX_BUFFER_SIZE);
ARS_QUEUE_DEFINE_TYPE(uartAppTxQ_t, BYTE, UART1_TX_BUFFER_SIZE);
ARS_QUEUE_DEFINE_TYPE(uartPeriRxQ_t, BYTE, UART2_RX_BUFFER_SIZE);
ARS_QUEUE_DEFINE_TYPE(uartPeriTxQ_t, BYTE, UART2_TX_BUFFER_SIZE);

extern volatile uartAppRxQ_t uartAppRxQ;
extern volatile uartAppTxQ_t uartAppTxQ;
extern volatile uartPeriRxQ_t uartPeriRxQ;
extern volatile uartPeriTxQ_t uartPeriTxQ;

#ifdef BLE_SUPPORTED // SIMBAMCU-19
ARS_QUEUE_DEFINE_TYPE(uartBleRxQ_t, BYTE, BLE_RX_BUFFER_SIZE);
ARS_QUEUE_DEFINE_TYPE(uartBleTxQ_t, BYTE, BLE_TX_BUFFER_SIZE);
extern volatile uartBleRxQ_t uartBleRxQ;
extern volatile uartBleTxQ_t uartBleTxQ;
void monet_GetBleData(size_t tempU2Size);
size_t md_BleWrite(uint8_t * buffer, size_t size);
size_t leuartWrite(LEUART_TypeDef * pUsart, uartGenQ_t * pQ, uint8_t * buffer, size_t size); // SIMBAMCU-19
#endif

void WriteQueue(uartGenQ_t * pQueue, BYTE value);
void WriteQueueEscape(uartGenQ_t * pQueue, BYTE value);
//size_t md_PeriphWrite(uint8_t * buffer, size_t size);
//size_t md_AppWrite(uint8_t * buffer, size_t size);
//size_t usartWrite(USART_TypeDef * pUsart, uartGenQ_t * pQ, uint8_t * buffer, size_t size);

BYTE ReadQueue(queue_struct *Queue);
void monet_TxData(queue_struct *pQueue, BYTE uart);
//void monet_GetPeriphData(size_t tempU2Size);
//uint8 isOtherPowerSource();
//__attribute((noreturn)) __attribute__((optimize("-O0"))) void BOOT_jump(uint32_t sp, uint32_t pc);
//__attribute((noreturn)) void BOOT_boot(uint32_t ssp, uint32_t spc);
//void StopWatchDog(void);
//void StartWatchDog(void);
void mnt_SendToApp(void);
void SysTick_Handler(void);

#endif /* UTIL_H_ */
