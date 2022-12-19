#ifndef __UART_DRV_EXT_H__
#define __UART_DRV_EXT_H__

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#include "nrf_drv_uart.h"

typedef enum
{
	UART0_DRV_EX = 0,
	UART1_DRV_EX,
} uart_num_t;

// Macro for initialize UART. Based on APP_UART_INIT()
#define UART_DRV_EX_INIT(UART_NUM, P_COMM_PARAMS, RX_BUF_SIZE, TX_BUF_SIZE, EVT_HANDLER, IRQ_PRIO, ERR_CODE) \
			do                                                                                             \
			{                                                                                              \
				app_uart_buffers_t buffers;                                                                \
				static uint8_t     CONCAT_2(rx_buf, UART_NUM)[RX_BUF_SIZE];                                \
				static uint8_t     CONCAT_2(tx_buf, UART_NUM)[TX_BUF_SIZE];                                \
																										   \
				buffers.rx_buf      = CONCAT_2(rx_buf, UART_NUM);                                          \
				buffers.rx_buf_size = sizeof (CONCAT_2(rx_buf, UART_NUM));                                 \
				buffers.tx_buf      = CONCAT_2(tx_buf, UART_NUM);                                          \
				buffers.tx_buf_size = sizeof (CONCAT_2(tx_buf, UART_NUM));                                 \
				ERR_CODE = uart_drv_ext_init(UART_NUM, P_COMM_PARAMS, &buffers, EVT_HANDLER, IRQ_PRIO);    \
			} while (0)


uint32_t uart_drv_ext_init(uart_num_t uart_num,
							const app_uart_comm_params_t * p_comm_params,
							app_uart_buffers_t *           p_buffers,
							app_uart_event_handler_t       error_handler,
							app_irq_priority_t             irq_priority);
uint32_t uart_drv_ext_get(uart_num_t uart_num, uint8_t * p_byte);
uint32_t uart_drv_ext_put(uart_num_t uart_num, uint8_t byte);
uint32_t uart_drv_ext_flush(uart_num_t uart_num);
uint32_t uart_drv_ext_close(uart_num_t uart_num);

bool uart_drv_ext_tx_fifo_is_empty(uart_num_t uart_num);
bool uart_drv_ext_rx_fifo_is_empty(uart_num_t uart_num);

#endif	// #ifndef __UART_DRV_EXT_H__
