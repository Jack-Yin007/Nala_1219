/*
 * util.c
 *
 *  Created on: Dec 12, 2015
 *      Author: J
 */

#include <stdint.h>          /* For uint32_t definition */
#include <stdbool.h>         /* For true/false definition */


#include "user.h"            /*  */
#include "util.h"            /*  */
//#include "em_leuart.h"
//#include "em_usart.h"
//#include "em_wdog.h"

BYTE ReadQueue(queue_struct *pQueue)
{
    BYTE value;

    if (pQueue->Size == 0)
        return 0;

    value = pQueue->Buffer[pQueue->Head];
    pQueue->Head =  (pQueue->Head + 1) % QUEUE_SIZE;
    pQueue->Size = pQueue->Size - 1;
    return value;
}

void WriteQueue(uartGenQ_t * pQueue, BYTE value)
{
    (void)ARS_QUEUE_PUSH_CYCLIC((*pQueue), value);
}


void WriteQueueEscape(uartGenQ_t * pQueue, BYTE value)
{
    if (ARS_QUEUE_COUNT((*pQueue)) >= ARS_QUEUE_SIZE((*pQueue)) - 1)
        return;
    if(value=='$')
    {
    	WriteQueue(pQueue, '!');
    	WriteQueue(pQueue, '0');
    }
    else if(value == '!')
    {
       	WriteQueue(pQueue, '!');
        WriteQueue(pQueue, '1');

    }
    else
    {
    	WriteQueue(pQueue, value);

    }

}


//void monet_TxData(queue_struct *pQueue, BYTE uart)
//{
//   if (uart == 1) {
//#ifdef USE_LEUART
//       USART_Tx(USART0, ReadQueue(pQueue));
//#else
//	   USART_Tx(USART1, ReadQueue(pQueue));
//#endif
//    } else {
//#ifndef USE_LEUART
//       USART_Tx(USART0, ReadQueue(pQueue));
//#else
//    	USART_Tx(USART1, ReadQueue(pQueue));
//#endif
//    }
//}

//void monet_GetPeriphData(size_t tempU2Size) {
//    size_t  i;
//    size_t  nSize = tempU2Size;
//    BYTE  pdata[255];

//    for(i=0; i<nSize && i < sizeof(pdata); i++) {
//        ARS_QUEUE_POP(uartPeriRxQ, &pdata[i]);
//    }
//    BuildFrame('#', pdata, i);
//}

#ifdef BLE_SUPPORTED // SIMBAMCU-19
void monet_GetBleData(size_t tempU2Size) {
    size_t  i;
    size_t  nSize = tempU2Size;
    BYTE  pdata[255];

    for(i=0; i<nSize && i < sizeof(pdata); i++) {
        ARS_QUEUE_POP(uartBleRxQ, &pdata[i]);
    }
    monet_AlphaResponse(pdata, i);
}

#endif
///**************************************************************************//**
// * @brief This function sets up the Cortex M-3 with a new SP and PC.
// *****************************************************************************/
//__attribute((noreturn)) __attribute__((optimize("-O0"))) void BOOT_jump(uint32_t sp, uint32_t pc)
//{
//	(void) sp;
//	(void) pc;
//	/* Set new MSP, PSP based on SP (r0)*/
//	__asm("msr msp, r0");
//	__asm("msr psp, r0");

//	/* Jump to PC (r1)*/
//	__asm("mov pc, r1");
//	// Remove warning
//	while (1) {}
//}

///**************************************************************************//**
// * @brief Boots the application
// *****************************************************************************/
//// PUMAMCU-161, reset the mcu 
//__attribute((noreturn)) void BOOT_boot(uint32_t ssp, uint32_t spc)
//{
//  uint32_t pc, sp;

//	// Stop the watchdog
//	StopWatchDog();

//	/* Reset registers */
//#ifdef USART_OVERLAPS_WITH_BOOTLOADER
//  CMU->LFBCLKEN0    = _CMU_LFBCLKEN0_RESETVALUE;
//  GPIO->ROUTE       = _GPIO_ROUTE_RESETVALUE;
//  GPIO->EXTIPSELL   = _GPIO_EXTIPSELL_RESETVALUE;
//  GPIO->EXTIFALL    = _GPIO_EXTIFALL_RESETVALUE;
//  GPIO->IEN         = _GPIO_IEN_RESETVALUE;
//  GPIO->IFC         = 0xFFFFFFFF;
//#endif

//  /* Clear all interrupts set. */
//  NVIC->ICER[0]     = 0xFFFFFFFF;
//#if ( __CORTEX_M != 0 )
//  NVIC->ICER[1]     = 0xFFFFFFFF;
//#endif
//  RTC->CTRL         = _RTC_CTRL_RESETVALUE;
//  RTC->COMP0        = _RTC_COMP0_RESETVALUE;
//  RTC->IEN          = _RTC_IEN_RESETVALUE;

//  /* Disable RTC clock */
//  CMU->LFACLKEN0    = _CMU_LFACLKEN0_RESETVALUE;
//  CMU->LFCLKSEL     = _CMU_LFCLKSEL_RESETVALUE;
//  /* Disable LFRCO */
//  CMU->OSCENCMD     = CMU_OSCENCMD_LFRCODIS;
//  /* Disable LE interface */
//  CMU->HFCORECLKEN0 = _CMU_HFCORECLKEN0_RESETVALUE;
//  /* Reset clocks */
//  CMU->HFPERCLKDIV  = _CMU_HFPERCLKDIV_RESETVALUE;
//  CMU->HFPERCLKEN0  = _CMU_HFPERCLKEN0_RESETVALUE;

//  /* Set new vector table */
//  SCB->VTOR = (uint32_t) ssp;
//  /* Read new SP and PC from vector table */
//  sp = *((volatile uint32_t *) ssp);
//  pc = *((volatile uint32_t *) spc + 1);

//  BOOT_jump(sp, pc);
//}

//void StartWatchDog(void)
//{
//	while (WDOG->SYNCBUSY & WDOG_SYNCBUSY_CTRL) {
//		// Wait for synchronization
//	}
//	WDOG->CTRL =	WDOG_CTRL_EN | 
//					(wdogClkSelLFXO << _WDOG_CTRL_CLKSEL_SHIFT) | 
//					(wdogPeriod_256k << _WDOG_CTRL_PERSEL_SHIFT);
//	  
//}

//void StopWatchDog(void)
//{
//	while (WDOG->SYNCBUSY & WDOG_SYNCBUSY_CTRL) {
//		// Wait for synchronization
//	}
//	WDOG->CTRL =	0;
//}

//size_t md_PeriphWrite(uint8_t * buffer, size_t size) {
//    return usartWrite(USART1, (uartGenQ_t *)&uartPeriTxQ, buffer, size);
//}

//size_t md_AppWrite(uint8_t * buffer, size_t size) {
//    return usartWrite(USART0, (uartGenQ_t *)&uartAppTxQ, buffer, size);
//}

#ifdef BLE_SUPPORTED // SIMBAMCU-19
size_t md_BleWrite(uint8_t * buffer, size_t size) {
    return leuartWrite(LEUART0, (uartGenQ_t *)&uartBleTxQ, buffer, size);
}
#endif
