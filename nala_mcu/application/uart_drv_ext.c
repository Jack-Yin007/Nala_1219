#include "uart_drv_ext.h"
#include "app_uart.h"
#include "app_fifo.h"
#include "sdk_common.h"

/*
Description: 
	Extented UART driver code.
	The code surpports 2 UARTs and uses FIFO for data sending and receiving.
	The code is based on Nordic API APP_UART module.
	See APP_UART_FIFO_INIT() for more description.
*/

nrf_drv_uart_t uart_inst0 = NRF_DRV_UART_INSTANCE(0);		// UART0
nrf_drv_uart_t uart_inst1 = NRF_DRV_UART_INSTANCE(1);		// UART1

static app_uart_event_handler_t   m_event_handler_uart0;            /**< Event handler function. */
static app_uart_event_handler_t   m_event_handler_uart1;            /**< Event handler function. */
static uint8_t uart0_tx_buffer[1];
static uint8_t uart0_rx_buffer[1];
static uint8_t uart1_tx_buffer[1];
static uint8_t uart1_rx_buffer[1];
static bool m_uart0_rx_ovf;
static bool m_uart1_rx_ovf;

static app_fifo_t                  m_rx_fifo_uart0;                               /**< RX FIFO buffer for storing data received on the UART until the application fetches them using app_uart_get(). */
static app_fifo_t                  m_tx_fifo_uart0;

static app_fifo_t                  m_rx_fifo_uart1;                               /**< RX FIFO buffer for storing data received on the UART until the application fetches them using app_uart_get(). */
static app_fifo_t                  m_tx_fifo_uart1;

static __INLINE uint32_t fifo_length(app_fifo_t * const fifo)
{
  uint32_t tmp = fifo->read_pos;
  return fifo->write_pos - tmp;
}

#define FIFO_LENGTH(F) fifo_length(&F)

static void uart0_event_handler(nrf_drv_uart_event_t * p_event, void* p_context)
{
	app_uart_evt_t app_uart_event;
    uint32_t err_code;

    switch (p_event->type)
    {
        case NRF_DRV_UART_EVT_RX_DONE:
            // Write received byte to FIFO.
            err_code = app_fifo_put(&m_rx_fifo_uart0, p_event->data.rxtx.p_data[0]);
            if (err_code != NRF_SUCCESS)
            {
                app_uart_event.evt_type          = APP_UART_FIFO_ERROR;
                app_uart_event.data.error_code   = err_code;
                m_event_handler_uart0(&app_uart_event);
            }
            // Notify that there are data available.
            else if (FIFO_LENGTH(m_rx_fifo_uart0) != 0)
            {
                app_uart_event.evt_type = APP_UART_DATA_READY;
                m_event_handler_uart0(&app_uart_event);
            }

            // Start new RX if size in buffer.
            if (FIFO_LENGTH(m_rx_fifo_uart0) <= m_rx_fifo_uart0.buf_size_mask)
            {
                (void)nrf_drv_uart_rx(&uart_inst0, uart0_rx_buffer, 1);
            }
            else
            {
                // Overflow in RX FIFO.
                m_uart0_rx_ovf = true;
            }

            break;

        case NRF_DRV_UART_EVT_ERROR:
            app_uart_event.evt_type                 = APP_UART_COMMUNICATION_ERROR;
            app_uart_event.data.error_communication = p_event->data.error.error_mask;
            (void)nrf_drv_uart_rx(&uart_inst0, uart0_rx_buffer, 1);
            m_event_handler_uart0(&app_uart_event);
            break;

        case NRF_DRV_UART_EVT_TX_DONE:
            // Get next byte from FIFO.
            if (app_fifo_get(&m_tx_fifo_uart0, uart0_tx_buffer) == NRF_SUCCESS)
            {
                (void)nrf_drv_uart_tx(&uart_inst0, uart0_tx_buffer, 1);
            }
            else
            {
                // Last byte from FIFO transmitted, notify the application.
                app_uart_event.evt_type = APP_UART_TX_EMPTY;
                m_event_handler_uart0(&app_uart_event);
            }
            break;

        default:
            break;
    }
}

static void uart1_event_handler(nrf_drv_uart_event_t * p_event, void* p_context)
{
	app_uart_evt_t app_uart_event;
    uint32_t err_code;

    switch (p_event->type)
    {
        case NRF_DRV_UART_EVT_RX_DONE:
            // Write received byte to FIFO.
            err_code = app_fifo_put(&m_rx_fifo_uart1, p_event->data.rxtx.p_data[0]);
            if (err_code != NRF_SUCCESS)
            {
                app_uart_event.evt_type          = APP_UART_FIFO_ERROR;
                app_uart_event.data.error_code   = err_code;
                m_event_handler_uart1(&app_uart_event);
            }
            // Notify that there are data available.
            else if (FIFO_LENGTH(m_rx_fifo_uart1) != 0)
            {
                app_uart_event.evt_type = APP_UART_DATA_READY;
                m_event_handler_uart1(&app_uart_event);
            }

            // Start new RX if size in buffer.
            if (FIFO_LENGTH(m_rx_fifo_uart1) <= m_rx_fifo_uart1.buf_size_mask)
            {
                (void)nrf_drv_uart_rx(&uart_inst1, uart1_rx_buffer, 1);
            }
            else
            {
                // Overflow in RX FIFO.
                m_uart1_rx_ovf = true;
            }

            break;

        case NRF_DRV_UART_EVT_ERROR:
            app_uart_event.evt_type                 = APP_UART_COMMUNICATION_ERROR;
            app_uart_event.data.error_communication = p_event->data.error.error_mask;
            (void)nrf_drv_uart_rx(&uart_inst1, uart1_rx_buffer, 1);
            m_event_handler_uart1(&app_uart_event);
            break;

        case NRF_DRV_UART_EVT_TX_DONE:
            // Get next byte from FIFO.
            if (app_fifo_get(&m_tx_fifo_uart1, uart1_tx_buffer) == NRF_SUCCESS)
            {
                (void)nrf_drv_uart_tx(&uart_inst1, uart1_tx_buffer, 1);
            }
            else
            {
                // Last byte from FIFO transmitted, notify the application.
                app_uart_event.evt_type = APP_UART_TX_EMPTY;
                m_event_handler_uart1(&app_uart_event);
            }
            break;

        default:
            break;
    }
}

// UART init function
// Based on app_uart_init()
uint32_t uart_drv_ext_init(uart_num_t uart_num,
							const app_uart_comm_params_t * p_comm_params,
							app_uart_buffers_t *           p_buffers,
							app_uart_event_handler_t       event_handler,
							app_irq_priority_t             irq_priority)
{
	uint32_t err_code;

	if(uart_num == UART0_DRV_EX)
		m_event_handler_uart0 = event_handler;
	else	//(uart_num == UART1_DRV_FIFO)
		m_event_handler_uart1 = event_handler;

    if (p_buffers == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    // Configure buffer RX buffer.
    err_code = app_fifo_init((uart_num == UART0_DRV_EX)? (&m_rx_fifo_uart0) : (&m_rx_fifo_uart1), p_buffers->rx_buf, p_buffers->rx_buf_size);
    VERIFY_SUCCESS(err_code);

    // Configure buffer TX buffer.
    err_code = app_fifo_init((uart_num == UART0_DRV_EX)? (&m_tx_fifo_uart0) : (&m_tx_fifo_uart1), p_buffers->tx_buf, p_buffers->tx_buf_size);
    VERIFY_SUCCESS(err_code);

    nrf_drv_uart_config_t config = NRF_DRV_UART_DEFAULT_CONFIG;
    config.baudrate = (nrf_uart_baudrate_t)p_comm_params->baud_rate;
    config.hwfc = (p_comm_params->flow_control == APP_UART_FLOW_CONTROL_DISABLED) ?
            NRF_UART_HWFC_DISABLED : NRF_UART_HWFC_ENABLED;
    config.interrupt_priority = irq_priority;
    config.parity = p_comm_params->use_parity ? NRF_UART_PARITY_INCLUDED : NRF_UART_PARITY_EXCLUDED;
    config.pselcts = p_comm_params->cts_pin_no;
    config.pselrts = p_comm_params->rts_pin_no;
    config.pselrxd = p_comm_params->rx_pin_no;
    config.pseltxd = p_comm_params->tx_pin_no;

	if(uart_num == UART0_DRV_EX)
	{
		err_code = nrf_drv_uart_init(&uart_inst0, &config, uart0_event_handler);
		m_uart0_rx_ovf = false;	
	}
	else	//(uart_num == UART1_DRV_EX)
	{
		err_code = nrf_drv_uart_init(&uart_inst1, &config, uart1_event_handler);
		m_uart1_rx_ovf = false;	
	}
	VERIFY_SUCCESS(err_code);

    // Turn on receiver if RX pin is connected
    if (p_comm_params->rx_pin_no != UART_PIN_DISCONNECTED)
    {
		if(uart_num == UART0_DRV_EX)
			return nrf_drv_uart_rx(&uart_inst0, uart0_rx_buffer,1);
		else
			return nrf_drv_uart_rx(&uart_inst1, uart1_rx_buffer,1);
    }
    else
    {
        return NRF_SUCCESS;
    }
}

// Get a byte from the UART. Based on app_uart_get()
uint32_t uart_drv_ext_get(uart_num_t uart_num, uint8_t * p_byte)
{
    ASSERT(p_byte);
    bool *p_rx_ovf = (uart_num == UART0_DRV_EX)? &m_uart0_rx_ovf: &m_uart1_rx_ovf;
	app_fifo_t *p_rx_fifo = (uart_num == UART0_DRV_EX)? (&m_rx_fifo_uart0): (&m_rx_fifo_uart1);
	nrf_drv_uart_t * p_instance = (uart_num == UART0_DRV_EX)? (&uart_inst0): (&uart_inst1);
	uint8_t *p_rx_buf = (uart_num == UART0_DRV_EX)? (uart0_rx_buffer): (uart1_rx_buffer);
	
    ret_code_t err_code =  app_fifo_get(p_rx_fifo, p_byte);

    // If FIFO was full new request to receive one byte was not scheduled. Must be done here.
    if (*p_rx_ovf)
    {
        *p_rx_ovf = false;
        uint32_t uart_err_code = nrf_drv_uart_rx(p_instance, p_rx_buf, 1);

        // RX resume should never fail.
        APP_ERROR_CHECK(uart_err_code);
    }

    return err_code;
}

// Put a byte on the UART. Based on app_uart_put()
uint32_t uart_drv_ext_put(uart_num_t uart_num, uint8_t byte)
{
	app_fifo_t *p_tx_fifo = (uart_num == UART0_DRV_EX)? (&m_tx_fifo_uart0): (&m_tx_fifo_uart1);
	nrf_drv_uart_t * p_instance = (uart_num == UART0_DRV_EX)? (&uart_inst0): (&uart_inst1);
	uint8_t *p_tx_buf = (uart_num == UART0_DRV_EX)? (uart0_tx_buffer): (uart1_tx_buffer);
    uint32_t err_code;
	
    err_code = app_fifo_put(p_tx_fifo, byte);
    if (err_code == NRF_SUCCESS)
    {
        // The new byte has been added to FIFO. It will be picked up from there
        // (in 'uart_event_handler') when all preceding bytes are transmitted.
        // But if UART is not transmitting anything at the moment, we must start
        // a new transmission here.
        if (!nrf_drv_uart_tx_in_progress(p_instance))
        {
            // This operation should be almost always successful, since we've
            // just added a byte to FIFO, but if some bigger delay occurred
            // (some heavy interrupt handler routine has been executed) since
            // that time, FIFO might be empty already.
            if (app_fifo_get(p_tx_fifo, p_tx_buf) == NRF_SUCCESS)
            {
                err_code = nrf_drv_uart_tx(p_instance, p_tx_buf, 1);
            }
        }
    }
    return err_code;
}

// Flush the RX and TX FIFOs. Based on app_uart_flush()
uint32_t uart_drv_ext_flush(uart_num_t uart_num)
{
    uint32_t err_code;

    err_code = app_fifo_flush((uart_num == UART0_DRV_EX)? (&m_rx_fifo_uart0) : (&m_rx_fifo_uart1));
    VERIFY_SUCCESS(err_code);

    err_code = app_fifo_flush((uart_num == UART0_DRV_EX)? (&m_tx_fifo_uart0) : (&m_tx_fifo_uart1));
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

// Close the UART module. Based on app_uart_close()
uint32_t uart_drv_ext_close(uart_num_t uart_num)
{
	nrf_drv_uart_uninit((uart_num == UART0_DRV_EX)? (&uart_inst0) : (&uart_inst1));
    return NRF_SUCCESS;
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

bool uart_drv_ext_tx_fifo_is_empty(uart_num_t uart_num)
{
	app_fifo_t *p_tx_fifo = (uart_num == UART0_DRV_EX)? (&m_tx_fifo_uart0): (&m_tx_fifo_uart1);
	uint8_t buf_byte;
	
	return (app_fifo_peek(p_tx_fifo, 0, &buf_byte) == NRF_ERROR_NOT_FOUND)? true: false;	// If the #0 element in FIFO cannot be peeked, the FIFO is empty
}

bool uart_drv_ext_rx_fifo_is_empty(uart_num_t uart_num)
{
	app_fifo_t *p_rx_fifo = (uart_num == UART0_DRV_EX)? (&m_rx_fifo_uart0): (&m_rx_fifo_uart1);
	uint8_t buf_byte;
	
	return (app_fifo_peek(p_rx_fifo, 0, &buf_byte) == NRF_ERROR_NOT_FOUND)? true: false;	// If the #0 element in FIFO cannot be peeked, the FIFO is empty
}
