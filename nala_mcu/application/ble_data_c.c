
/*
 * ble_data_c.c
 *
 *  Created on: Apr 27, 2020
 *      Author: Yangjie Gu
 */

#include "user.h"
#include "ble_data_c.h"
#include "mux_mcu.h"

extern uint16_t get_ble_aus_max_data_len(void);
extern bool ble_aus_ready_state_get_c(void);
extern volatile uint32_t g_ble_data_error;
extern uint32_t io_tx_count;

typedef struct {
    uint8_t * p_data;
    uint16_t length;
    uint8_t st;            // Sensor Type
    uint8_t id;            // Intance ID
} buffer_t;

buffer_t m_ble_send_buf;

typedef struct
{
  buffer_t *p_buf;
  uint16_t in;
  uint16_t out;
  uint16_t num;
  uint16_t max;
} ble_data_queue_t;

buffer_t ble_data_queue_tx[BLE_DATA_SEND_BUFFER_CELL_NUM] = {0};
uint8_t ble_data_queue_tx_buf[BLE_DATA_SEND_BUFFER_CELL_NUM][BLE_DATA_SEND_BUFFER_CELL_LEN] = {0};
ble_data_queue_t ble_data_tx_queue = {
  .p_buf = ble_data_queue_tx,
  .in = 0,
  .out = 0,
  .num = 0,
  .max = BLE_DATA_SEND_BUFFER_CELL_NUM
};

buffer_t ble_data_queue_rx[BLE_DATA_RECV_BUFFER_CELL_NUM] = {0};
uint8_t ble_data_queue_rx_buf[BLE_DATA_RECV_BUFFER_CELL_NUM][BLE_DATA_RECV_BUFFER_CELL_LEN] = {0};
ble_data_queue_t ble_data_rx_queue = {
  .p_buf = ble_data_queue_rx,
  .in = 0,
  .out = 0,
  .num = 0,
  .max = BLE_DATA_RECV_BUFFER_CELL_NUM
};

uint32_t m_len_sent;
uint32_t m_len_recv;
uint32_t m_cnt_nms;

APP_TIMER_DEF(m_timer_ble_send);
uint8_t ble_send_timer_started = 0;

uint32_t ble_data_throughput_count = 0;
uint8_t data_throughput_index = 0;

void ble_data_queue_init(void)
{
    uint32_t i = 0;

    for (i = 0; i < BLE_DATA_SEND_BUFFER_CELL_NUM; i++)
    {
        ble_data_queue_tx[i].p_data = ble_data_queue_tx_buf[i];
        ble_data_queue_tx[i].length = 0;
        ble_data_queue_tx[i].st = 0xff;
        ble_data_queue_tx[i].id = 0xff;
    }

    for (i = 0; i < BLE_DATA_RECV_BUFFER_CELL_NUM; i++)
    {
        ble_data_queue_rx[i].p_data = ble_data_queue_rx_buf[i];
        ble_data_queue_rx[i].length = 0;
        ble_data_queue_rx[i].st = 0xff;
        ble_data_queue_rx[i].id = 0xff;
    }
}

int8_t ble_data_queue_in(ble_data_queue_t *p_queue, buffer_t buf)
{
    if ((buf.p_data == NULL) || (buf.length == 0) || (p_queue == NULL))
    {
        return -1;
    }

    if (p_queue->num == p_queue->max)
    {
        return -2;
    }

    memcpy(p_queue->p_buf[p_queue->in].p_data, buf.p_data, buf.length);
    p_queue->p_buf[p_queue->in].length = buf.length;
    p_queue->p_buf[p_queue->in].st = buf.st;
    p_queue->p_buf[p_queue->in].id = buf.id;
    p_queue->in++;
    p_queue->in %= p_queue->max;

    if (p_queue->in > p_queue->out)
    {
        p_queue->num = p_queue->in - p_queue->out;
    }
    else
    {
        p_queue->num = p_queue->max + p_queue->in - p_queue->out;
    }

    return 0;
}

buffer_t* ble_data_queue_get_one(ble_data_queue_t *p_queue)
{
    buffer_t *tmp = NULL;

    if (p_queue == NULL)
    {
        return NULL;
    }

    if (p_queue->num == p_queue->max)
    {
        return NULL;
    }

    tmp = &(p_queue->p_buf[p_queue->in]);
    tmp->length = 0;
    tmp->st = 0xff;
    tmp->id = 0xff;

    p_queue->in++;
    p_queue->in %= p_queue->max;

    if (p_queue->in >= p_queue->out)
    {
        p_queue->num = p_queue->in - p_queue->out;
    }
    else
    {
        p_queue->num = p_queue->max + p_queue->in - p_queue->out;
    }

    return tmp;
}

int8_t ble_data_queue_out(ble_data_queue_t *p_queue, buffer_t *p_buf)
{
    if ((p_buf == NULL) || (p_queue == NULL))
    {
        return -1;
    }

    if (p_queue->num == 0)
    {
        return -2;
    }

    p_buf->p_data = p_queue->p_buf[p_queue->out].p_data;
    p_buf->length = p_queue->p_buf[p_queue->out].length;
    p_buf->st = p_queue->p_buf[p_queue->out].st;
    p_buf->id = p_queue->p_buf[p_queue->out].id;

    return 0;
}

int8_t ble_data_queue_delete_one(ble_data_queue_t *p_queue)
{
    if (p_queue == NULL)
    {
        return -1;
    }

    if (p_queue->num == 0)
    {
        return -2;
    }

    p_queue->p_buf[p_queue->out].st = 0xff;
    p_queue->p_buf[p_queue->out].id = 0xff;
    p_queue->p_buf[p_queue->out].length = 0;
    p_queue->out++;
    p_queue->out %= p_queue->max;

    if (p_queue->in >= p_queue->out)
    {
        p_queue->num = p_queue->in - p_queue->out;
    }
    else
    {
        p_queue->num = p_queue->max + p_queue->in - p_queue->out;
    }

    return 0;
}

int8_t is_ble_data_queue_empty(ble_data_queue_t *p_queue)
{
    if (p_queue->num == 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int8_t is_ble_data_queue_full(ble_data_queue_t *p_queue)
{
    if (p_queue->num == p_queue->max)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void ble_data_send_with_queue_c(void)
{
    uint32_t err_code;
    uint16_t length = 0;
    uint8_t st = 0;
    uint8_t id = 0;
    static bool retry = false;

    if (retry)
    {
        length = m_ble_send_buf.length;
        st = m_ble_send_buf.st;
        id = m_ble_send_buf.id;
        err_code = ble_aus_data_send_central(m_ble_send_buf.p_data, length, st, id);
        //NRF_LOG_INFO("Data2: %d", m_ble_send_buf.p_data[0]);
        if ((err_code != NRF_SUCCESS)
            && (err_code != NRF_ERROR_BUSY)
            && (err_code != NRF_ERROR_INVALID_PARAM)
            && (err_code != NRF_ERROR_INVALID_STATE)
            && (err_code != NRF_ERROR_NO_MEM)
            && (err_code != NRF_ERROR_RESOURCES))
        {
            APP_ERROR_CHECK(err_code);
        }
        if ((err_code == NRF_SUCCESS) && (g_ble_data_error == 0))
        {
            if (m_ble_send_buf.p_data)
            {
                ble_data_queue_delete_one(&ble_data_tx_queue);
                m_ble_send_buf.p_data = NULL;
                m_ble_send_buf.length = 0;
                m_ble_send_buf.st = 0xff;
                m_ble_send_buf.id = 0xff;
            }
            m_len_sent += length;
            retry = false;
        }
        else
        {
            g_ble_data_error = 0;
            return;
        }
    }

    while (!is_ble_data_queue_empty(&ble_data_tx_queue) && !retry)
    {
        err_code = ble_data_queue_out(&ble_data_tx_queue, &m_ble_send_buf);
        length = m_ble_send_buf.length;
        st = m_ble_send_buf.st;
        id = m_ble_send_buf.id;

        err_code = ble_aus_data_send_central(m_ble_send_buf.p_data, length, st, id);
        // NRF_LOG_INFO("Data: %d", m_ble_send_buf.p_data[0]);
        if ((err_code != NRF_SUCCESS)
            && (err_code != NRF_ERROR_BUSY)
            && (err_code != NRF_ERROR_INVALID_PARAM)
            && (err_code != NRF_ERROR_INVALID_STATE)
            && (err_code != NRF_ERROR_NO_MEM)
            && (err_code != NRF_ERROR_RESOURCES))
        {
            APP_ERROR_CHECK(err_code);
        }

        if ((err_code == NRF_SUCCESS) && (g_ble_data_error == 0))
        {
            if (m_ble_send_buf.p_data)
            {
                ble_data_queue_delete_one(&ble_data_tx_queue);
                m_ble_send_buf.p_data = NULL;
                m_ble_send_buf.length = 0;
                m_ble_send_buf.st = 0xff;
                m_ble_send_buf.id = 0xff;
            }
            m_len_sent += length;
            retry = false;
        }
        else
        {
            g_ble_data_error = 0;
            retry = true;
            break;
        }
    }
}

// Write value to space p_data
// Return value: the data bytes written
uint32_t AtelWriteBuff(uint8_t *p_data, uint8_t value)
{
	*p_data = value;
	return 1;
}

uint32_t AtelWriteBuffEscape(uint8_t *p_data, uint8_t value)
{
	uint32_t len = 0;
	
    if(value == '$')
    {
        len += AtelWriteBuff(&p_data[len], '!');
        len += AtelWriteBuff(&p_data[len], '0');
    }
    else if(value == '!')
    {
        len += AtelWriteBuff(&p_data[len], '!');
        len += AtelWriteBuff(&p_data[len], '1');
    }
    else
    {
        len += AtelWriteBuff(&p_data[len], value);
    }
	return len;
}

#define CHAR_I2C_SEND	'5'

bool i2c_data_send(uint8_t * p_data, uint16_t data_len, uint16_t channel)
{
	uint8_t arr[MUX_MCU_PACK_SIZE];
	uint32_t arr_len = 0;
	uint8_t len_non_payload = 5;//1+1+1+1+1;	// [I2C_len ['$' len ['5' ch [data ...]] chksum]]
	int i = 0;
	uint8_t sum = 0;
	
	if (data_len > (MUX_MCU_PACK_SIZE - len_non_payload))
	{
		NRF_LOG_RAW_INFO("i2c_data_send() len(%u) err\r\n", data_len);
		NRF_LOG_FLUSH();
		return false;
	}
	memset(arr, 0, sizeof(arr));
	arr[0] = '$';
	arr_len++;
	arr_len += AtelWriteBuffEscape(&arr[arr_len], data_len+1+1);
	arr_len += AtelWriteBuffEscape(&arr[arr_len], IO_CMD_CHAR_I2C_SEND);
	sum += CHAR_I2C_SEND;
	arr[arr_len] = channel;
	arr_len++;
	sum += channel;
	for (i = 0; i < data_len; i++)
	{
		arr_len += AtelWriteBuffEscape(&arr[arr_len], p_data[i]);
		sum += p_data[i];
	}
	arr_len += AtelWriteBuffEscape(&arr[arr_len], sum ^ 0xff);
	if (arr_len > MUX_MCU_PACK_SIZE)
	{
		NRF_LOG_RAW_INFO("i2c_data_send() buffer overflow err %u\r\n", arr_len);
		NRF_LOG_FLUSH();
		return false;
	}

	NRF_LOG_RAW_INFO("i2c_data_send() data content, len %u, ch %u\r", data_len, channel);
	for (i = 0; i < arr_len; i++)
		NRF_LOG_RAW_INFO("%0x ", arr[i]);
	NRF_LOG_RAW_INFO("\r");
	NRF_LOG_FLUSH();
//	return true;
	
	return mux_mcu_write(arr, arr_len);
}

bool is_tx_queue_empty(void)
{
	if (is_ble_data_queue_empty(&ble_data_tx_queue) == 1)
		return true;
	else
		return false;
}

void i2c_data_send_with_queue(void)
{
    uint32_t err_code;
    uint16_t length = 0;
    uint8_t st = 0xff;
    uint8_t id = 0xff;
    static bool retry = false;
	bool ret = true;
	
    if (retry)
    {
        length = m_ble_send_buf.length;
        st = m_ble_send_buf.st;
        id = m_ble_send_buf.id;
		ret = i2c_data_send(m_ble_send_buf.p_data, length, (uint16_t)id);
		NRF_LOG_RAW_INFO("i2c_data_send_with_queue() retry, ret %u, length %u, st %u id %u\r", ret, length, st, id);
		NRF_LOG_FLUSH();
		if (ret == true)
		{
			if (m_ble_send_buf.p_data)
            {
                ble_data_queue_delete_one(&ble_data_tx_queue);
                m_ble_send_buf.p_data = NULL;
                m_ble_send_buf.length = 0;
                m_ble_send_buf.st = 0xff;
                m_ble_send_buf.id = 0xff;
            }
            m_len_sent += length;
            retry = false;
		}
		else
		{
			NRF_LOG_RAW_INFO("i2c_data_send_with_queue() i2c send retry err\r\n");
			NRF_LOG_FLUSH();
			return;
		}
    }

//    while (!is_ble_data_queue_empty(&ble_data_tx_queue) && !retry)
    if (!is_ble_data_queue_empty(&ble_data_tx_queue) && !retry)
    {
        err_code = ble_data_queue_out(&ble_data_tx_queue, &m_ble_send_buf);
		err_code = err_code;	// To avoid compiler warning
        length = m_ble_send_buf.length;
        st = m_ble_send_buf.st;
        id = m_ble_send_buf.id;
		ret = i2c_data_send(m_ble_send_buf.p_data, length, (uint16_t)id);
		NRF_LOG_RAW_INFO("i2c_data_send_with_queue(), ble_data_queue_out() ret %u\r", err_code);
		NRF_LOG_RAW_INFO("i2c_data_send_with_queue(), ret %u, length %u, st %u id %u\r", ret, length, st, id);
		NRF_LOG_FLUSH();
		if (ret == true)
		{
			if (m_ble_send_buf.p_data)
            {
                ble_data_queue_delete_one(&ble_data_tx_queue);
                m_ble_send_buf.p_data = NULL;
                m_ble_send_buf.length = 0;
                m_ble_send_buf.st = 0xff;
                m_ble_send_buf.id = 0xff;
            }
            m_len_sent += length;
            retry = false;
		}
		else
		{
			NRF_LOG_RAW_INFO("i2c_data_send_with_queue() i2c send err\r\n");
			NRF_LOG_FLUSH();
			return;
		}
    }
}

void ble_send_data_rate_show(uint8_t force)
{
    static uint16_t index = 0;
    static uint32_t data_send_rec[5] = {0};
    static uint32_t data_recv_rec[5] = {0};
    static uint32_t io_tx_rec[5] = {0};

    //calculate speed every 1 second
    if ((m_cnt_nms >= (1000 / BLE_DATA_SEND_TIMER_PERIOD_MS)) ||
         force)
    {
        ble_data_throughput_count++;

        data_send_rec[index] = m_len_sent;
        data_recv_rec[index] = m_len_recv;
        io_tx_rec[index] = io_tx_count;

        index++;

        if ((index >= 5) || force)
        {
            uint16_t i = 0;
            for (i = 0; i < index; i++)
            {
                pf_log_raw(atel_log_ctl.platform_en,"==**Speed Tx: %d B/s Rx: %d B/s IO Tx: %d B/s (%d)**==\r", 
                           data_send_rec[i], data_recv_rec[i], io_tx_rec[i], ble_data_throughput_count - index + i);
            }
            index = 0;
        }

        m_cnt_nms = 0;
        m_len_sent = 0;
        m_len_recv = 0;
        io_tx_count = 0;
    }
}

static void ble_send_timer_c_handler(void * p_context)
{
    #if BLE_DATA_THROUGHPUT_TEST_EN
    ret_code_t err_code;
    buffer_t *p_tmp;
    buffer_t buf;
    uint16_t i = 0;
    #endif /* BLE_DATA_THROUGHPUT_TEST_EN */

    #if BLE_BYPASS_TEST_ONEWAY_TX
    if (is_ble_send_queue_full() == 0)
    {
        static uint32_t onway_tx_count = 0;
        uint8_t buf[128] = {0};
        memset(buf, (onway_tx_count % 26) + 'a', 128);
        buf[126] = '\r';
        buf[127] = '\n';
        memcpy(buf, &onway_tx_count, 4);
        if (ble_aus_ready_state_get_c())
        {
            ble_send_data_push(buf, 128, 0);
            onway_tx_count++;
            // nrf_delay_ms(20);
        }
    }
    #endif /* BLE_BYPASS_TEST_ONEWAY_TX */

	if (com_method_zazu_get() == COM_METHOD_ZAZU_BLE)
		ble_data_send_with_queue_c();
	m_cnt_nms++;
	mux_com_timer_set();

//    pf_status_led_indicate(BLE_DATA_SEND_TIMER_PERIOD_MS);

#if BLE_DATA_THROUGHPUT_TEST_EN
    //the snippet simulate a real application scenairo. Queue is involved.
    //produce the data irregard of BLE activity
    //put the data into a queue to cache them
    p_tmp = ble_data_queue_get_one(&ble_data_tx_queue);
    if (p_tmp != NULL)
    {
        buf = *(p_tmp);
    }
    else
    {
        return;
    }

    buf.length = MIN(get_ble_aus_max_data_len(), BLE_DATA_SEND_BUFFER_CELL_LEN);
    if (buf.p_data)
    {
        buf.p_data[0] = data_throughput_index++;
        for (i = 1; i < buf.length; i++)
        {
            buf.p_data[i] = i;
        }
        buf.p_data[buf.length - 1] = my_checksum_8(buf.p_data, buf.length - 1);
    }
    else
    {
        NRF_LOG_INFO("ble_send_timer_c_handler Drop.");
        return;
    }
#endif /* BLE_DATA_THROUGHPUT_TEST_EN */
}

void ble_send_recv_init_c(void)
{
    ret_code_t err_code;

    ble_data_queue_init();

    err_code = app_timer_create(&m_timer_ble_send, APP_TIMER_MODE_REPEATED, ble_send_timer_c_handler);
    APP_ERROR_CHECK(err_code);
}

void ble_send_timer_start_c(void)
{
    ret_code_t err_code;
    if (ble_send_timer_started)
    {
        return;
    }
    err_code = app_timer_start(m_timer_ble_send, APP_TIMER_TICKS(BLE_DATA_SEND_TIMER_PERIOD_MS),NULL);
    APP_ERROR_CHECK(err_code);
    ble_send_timer_started = 1;
}

void ble_send_timer_stop_c(void)
{
    ret_code_t err_code;
    if (ble_send_timer_started == 0)
    {
        return;
    }
    err_code = app_timer_stop(m_timer_ble_send);
    APP_ERROR_CHECK(err_code);

    #if BLE_DATA_THROUGHPUT_TEST_EN
    ble_data_throughput_count = 0;
    #endif /* BLE_DATA_THROUGHPUT_TEST_EN */

    ble_send_timer_started = 0;

    ble_send_data_rate_show(1);
}

int8_t is_ble_send_queue_full(void)
{
    return is_ble_data_queue_full(&ble_data_tx_queue);
}

int8_t is_ble_send_queue_empty(void)
{
    return is_ble_data_queue_empty(&ble_data_tx_queue);
}

int8_t ble_send_data_push(uint8_t *p_data, uint16_t len, uint8_t st, uint8_t id)
{
    int8_t err_code;
    buffer_t buf = {0};

    if ((p_data == NULL) || (len == 0))
    {
        NRF_LOG_ERROR("ble_send_data_push Param.");
        return -1;
    }

    buf.st = st;
    buf.id = id;
    buf.length = len;
    buf.p_data = p_data;

    err_code = ble_data_queue_in(&ble_data_tx_queue, buf);
    if (err_code)
    {
        NRF_LOG_ERROR("ble_send_data_push Drop(E:%d N:%d M:%d)", err_code, ble_data_tx_queue.num, ble_data_tx_queue.max);
        return -2;
    }

    return 0;
}

int8_t ble_recv_data_push(uint8_t *p_data, uint16_t len, uint8_t st, uint8_t id)
{
    int8_t err_code;
    buffer_t buf = {0};

    if ((p_data == NULL) || (len == 0))
    {
        NRF_LOG_ERROR("ble_recv_data_push Param.");
        return -1;
    }

    if (len > BLE_DATA_RECV_BUFFER_CELL_LEN)
    {
        NRF_LOG_ERROR("ble_recv_data_push(len: %d > %d).", len, BLE_DATA_RECV_BUFFER_CELL_LEN);
        return -2;
    }

    buf.st = st;
    buf.id = id;
    buf.length = len;
    buf.p_data = p_data;
    if (buf.p_data)
    {
        err_code = ble_data_queue_in(&ble_data_rx_queue, buf);
        if (err_code)
        {
            NRF_LOG_ERROR("ble_recv_data_push Drop(E:%d N:%d M:%d)", err_code, ble_data_rx_queue.num, ble_data_rx_queue.max);
            return -2;
        }

        return 0;
    }
    else
    {
        return -3;
    }
}

int8_t is_ble_recv_queue_empty(void)
{
    return is_ble_data_queue_empty(&ble_data_rx_queue);
}

int8_t ble_recv_data_pop(uint8_t **p_data, uint16_t *p_len, uint8_t *p_st, uint8_t *p_id)
{
    int8_t err_code;
    buffer_t buf = {0};

    if ((p_data == NULL) || (p_len == NULL))
    {
        return -1;
    }

    err_code = ble_data_queue_out(&ble_data_rx_queue, &buf);
    if (err_code)
    {
        NRF_LOG_ERROR("ble_recv_data_pop pop fail.");
        return -2;
    }

    *p_data = buf.p_data;

    if (*p_data)
    {
        *p_len = buf.length;
        *p_st = buf.st;
        *p_id = buf.id;
        // ble_data_queue_delete_one(&ble_data_rx_queue);
        // NRF_LOG_ERROR("ble_recv_data_pop pop(N:%d M:%d).", ble_data_rx_queue.num, ble_data_rx_queue.max);
    }
    else
    {
        return -3;
    }

    return 0;
}

void ble_recv_data_delete_one(void)
{
    ble_data_queue_delete_one(&ble_data_rx_queue);
}

uint8_t my_checksum_8(uint8_t *p_data, uint16_t len)
{
    uint8_t sum = 0;

    while (len)
    {
        sum += *p_data;
        p_data++;
        len--;
    }

    return sum;
}
