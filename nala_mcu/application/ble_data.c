/*
 * ble_data.c
 *
 *  Created on: Apr 27, 2020
 *      Author: Yangjie Gu
 */

#include <stdint.h>
#include <string.h>
#include "platform_hal_drv.h"

#include "ble_data.h"

extern ret_code_t ble_aus_data_send_periheral(uint8_t * p_data, uint16_t data_len, uint16_t channel);
extern uint16_t get_ble_aus_max_data_len(void);

typedef struct {
    uint8_t * p_data;
    uint16_t length;
    uint16_t channel;
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
        ble_data_queue_tx[i].channel = 0xffff;
    }

    for (i = 0; i < BLE_DATA_RECV_BUFFER_CELL_NUM; i++)
    {
        ble_data_queue_rx[i].p_data = ble_data_queue_rx_buf[i];
        ble_data_queue_rx[i].length = 0;
        ble_data_queue_rx[i].channel = 0xffff;
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
    p_queue->p_buf[p_queue->in].channel = buf.channel;
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
    tmp->channel = 0;

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
    p_buf->channel = p_queue->p_buf[p_queue->out].channel;

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

    p_queue->p_buf[p_queue->out].channel = 0xffff;
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

void ble_data_send_with_queue(void)
{
    uint32_t err_code;
    uint16_t length = 0;
    uint16_t channel = 0;
    static bool retry = false;

    if (retry)
    {
        length = m_ble_send_buf.length;
        channel = m_ble_send_buf.channel;
        err_code = ble_aus_data_send_periheral(m_ble_send_buf.p_data, length, channel);
        //NRF_LOG_INFO("Data2: %d", m_ble_send_buf.p_data[0]);
        if ((err_code != NRF_SUCCESS) &&
            (err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_NOT_FOUND))
        {
            NRF_LOG_ERROR("ble_s_0(%d)", err_code);
            // APP_ERROR_CHECK(err_code);
        }
        if (err_code == NRF_SUCCESS)
        {
            if (m_ble_send_buf.p_data)
            {
                ble_data_queue_delete_one(&ble_data_tx_queue);
                m_ble_send_buf.p_data = NULL;
                m_ble_send_buf.length = 0;
                m_ble_send_buf.channel = 0xffff;
            }
            m_len_sent += length;
            retry = false;
        }
    }

    while (!is_ble_data_queue_empty(&ble_data_tx_queue) && !retry)
    {
        err_code = ble_data_queue_out(&ble_data_tx_queue, &m_ble_send_buf);
        length = m_ble_send_buf.length;
        channel = m_ble_send_buf.channel;

        err_code = ble_aus_data_send_periheral(m_ble_send_buf.p_data, length, channel);
        //NRF_LOG_INFO("Data: %d", m_ble_send_buf.p_data[0]);
        if ((err_code != NRF_SUCCESS) &&
            (err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_NOT_FOUND))
        {
            NRF_LOG_ERROR("ble_s_1(%d)", err_code);
            // APP_ERROR_CHECK(err_code);
        }
        if (err_code == NRF_SUCCESS)
        {
            if (m_ble_send_buf.p_data)
            {
                ble_data_queue_delete_one(&ble_data_tx_queue);
                m_ble_send_buf.p_data = NULL;
                m_ble_send_buf.length = 0;
                m_ble_send_buf.channel = 0xffff;
            }
            m_len_sent += length;
            retry = false;
        }
        else
        {
            retry = true;
            break;
        }
    }
}

void ble_send_data_rate_show(void)
{
    static uint16_t index = 0;
    static uint32_t data_send_rec[5] = {0};
    static uint32_t data_recv_rec[5] = {0};

    //calculate speed every 1 second
    if (m_cnt_nms >= (1000 / BLE_DATA_SEND_TIMER_PERIOD_MS))
    {
        ble_data_throughput_count++;

        data_send_rec[index] = m_len_sent;
        data_recv_rec[index] = m_len_recv;
        index++;

        if (index >= 5)
        {
            uint16_t i = 0;
            index = 0;

            for (i = 0; i < 5; i++)
            {
                NRF_LOG_RAW_INFO("==**Speed Tx: %d B/s Rx: %d B/s(%d)**==\r", 
                                 data_send_rec[i],
                                 data_recv_rec[i],
                                 ble_data_throughput_count - 5 + i);
                NRF_LOG_FLUSH();
            }
        }

        m_cnt_nms = 0;
        m_len_sent = 0;
        m_len_recv = 0;
    }
}

static void ble_send_timer_handler(void * p_context)
{
    #if BLE_DATA_THROUGHPUT_TEST_EN
    ret_code_t err_code;
    buffer_t *p_tmp;
    buffer_t buf;
    uint16_t i = 0;
    #endif /* BLE_DATA_THROUGHPUT_TEST_EN */

    ble_data_send_with_queue();

    m_cnt_nms++;

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
        NRF_LOG_INFO("ble_send_timer_handler Drop.");
        return;
    }
#endif /* BLE_DATA_THROUGHPUT_TEST_EN */
}

void ble_send_recv_init(void)
{
    ret_code_t err_code;

    ble_data_queue_init();

    err_code = app_timer_create(&m_timer_ble_send, APP_TIMER_MODE_REPEATED, ble_send_timer_handler);
    APP_ERROR_CHECK(err_code);
}

void ble_send_timer_start(void)
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

void ble_send_timer_stop(void)
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
}

int8_t is_ble_send_queue_full(void)
{
    return is_ble_data_queue_full(&ble_data_tx_queue);
}

int8_t ble_send_data_push(uint8_t *p_data, uint16_t len, uint16_t channel)
{
    int8_t err_code;
    buffer_t buf = {0};

    if ((p_data == NULL) || (len == 0))
    {
        NRF_LOG_ERROR("ble_send_data_push Param.");
        return -1;
    }

    buf.channel = channel;
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

int8_t ble_recv_data_push(uint8_t *p_data, uint16_t len, uint16_t channel)
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

    buf.channel = channel;
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

int8_t ble_recv_data_pop(uint8_t **p_data, uint16_t *p_len, uint16_t *p_ch)
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
        *p_ch = buf.channel;
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

#include "user.h"
void ble_data_test(uint32_t ms)
{
    static uint32_t count_ms = 0;
    uint8_t addr[BLE_GAP_ADDR_LEN * 8] = {0};
    // ble_aus_conn_param_t ble_conn_param = {7.5, 7.5, 0, 2000};

    // Evaluate Board MAC
    addr[0] = 0x5E;
    addr[1] = 0x3A;
    addr[2] = 0x30;
    addr[3] = 0x99;
    addr[4] = 0x91;
    addr[5] = 0xC5;
    // addr[6] = 0x12;
    // addr[7] = 0xE1;
    // addr[8] = 0x20;
    // addr[9] = 0xE7;
    // addr[10] = 0x4E;
    // addr[11] = 0xD8;


    // AC61 Developing Board MAC
    // addr[0] = 0x12;
    // addr[1] = 0xE1;
    // addr[2] = 0x20;
    // addr[3] = 0xE7;
    // addr[4] = 0x4E;
    // addr[5] = 0xD8;

    // AC61_SW Developing Board MAC
    // addr[0] = 0x67;
    // addr[1] = 0x45;
    // addr[2] = 0x22;
    // addr[3] = 0xCD;
    // addr[4] = 0x17;
    // addr[5] = 0xC3;

    memcpy(monet_data.ble_info[0].mac_addr, addr, BLE_MAC_ADDRESS_LEN);
    monet_data.ble_info[0].connect_status = BLE_CONNECTION_STATUS_MAC_SET;
    monet_data.ble_info[0].handler = 0xffff;

    // AC61_CMY Developing Board MAC
    // addr[0] = 0x97;
    // addr[1] = 0x39;
    // addr[2] = 0x56;
    // addr[3] = 0xF1;
    // addr[4] = 0x9C;
    // addr[5] = 0xE9;

    count_ms += ms;

    if ((count_ms % 10000) == 0)
    {
        ble_aus_advertising_stop();
        if (((count_ms / 10000) % 2) == 1)
        {
            addr[0] += 1;
        }

        ble_aus_white_list_set();
        ble_aus_advertising_start();
    }

    // monet_data.ble_conn_param[0].min_100us = 75;
    // monet_data.ble_conn_param[0].max_100us = 75;
    // monet_data.ble_conn_param[0].latency = 0;
    // monet_data.ble_conn_param[0].timeout_100us = 20000;
    // monet_data.ble_conn_param_update_ok[0] = ble_aus_change_change_conn_params(0, ble_conn_param);
}

#if 0
NRF_LOG_FLUSH();

if (is_ble_recv_queue_empty() == 0)
{
    uint8_t *tmp_buf;
    uint16_t tmp_len = 0;

    if (is_ble_send_queue_full() == 0)
    {
        ble_recv_data_pop(&tmp_buf, &tmp_len);
        ble_send_data_push(tmp_buf, tmp_len);
        ble_recv_data_delete_one();
    }
}
#endif
