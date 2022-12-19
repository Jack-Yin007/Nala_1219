/*
 * ble_data.h
 *
 *  Created on: Apr 27, 2020
 *      Author: Yangjie Gu
 */

#ifndef __ATEL_BLE_DATA_PERIPHERAL_H
#define __ATEL_BLE_DATA_PERIPHERAL_H
#ifdef __cplusplus
 extern "C" {
#endif

#define BLE_DATA_SEND_TIMER_PERIOD_MS (20)

#define BLE_DATA_SEND_BUFFER_CELL_NUM (10)
#define BLE_DATA_SEND_BUFFER_CELL_LEN (160)

#define BLE_DATA_RECV_BUFFER_CELL_NUM (10)
#define BLE_DATA_RECV_BUFFER_CELL_LEN (160)

#define BLE_DATA_THROUGHPUT_TEST_EN (0)
#define BLE_DATA_THROUGHPUT_ERROR_CHECK_EN (0)

extern uint32_t m_len_recv;
extern uint8_t data_throughput_index;

extern void ble_data_send_with_queue(void);
extern void ble_send_data_rate_show(void);
extern void ble_send_recv_init(void);
extern void ble_send_timer_start(void);
extern void ble_send_timer_stop(void);
extern int8_t is_ble_send_queue_full(void);
extern int8_t ble_send_data_push(uint8_t *p_data, uint16_t len, uint16_t channel);
extern int8_t ble_recv_data_push(uint8_t *p_data, uint16_t len, uint16_t channel);
extern int8_t is_ble_recv_queue_empty(void);
extern int8_t ble_recv_data_pop(uint8_t **p_data, uint16_t *p_len, uint16_t *p_ch);
extern void ble_recv_data_delete_one(void);
extern uint8_t my_checksum_8(uint8_t *p_data, uint16_t len);
extern void ble_data_test(uint32_t ms);

extern void ble_aus_white_list_set(void);
extern void ble_aus_advertising_stop(void);
extern uint32_t ble_aus_advertising_start(void);
extern uint32_t ble_aus_change_change_conn_params_disconnect(uint16_t channel);

typedef struct
{
    float min_ms;
    float max_ms;
    uint16_t latency;
    uint16_t timeout_ms;
} ble_aus_conn_param_t;

uint32_t ble_aus_change_change_conn_params(uint16_t channel, ble_aus_conn_param_t ble_conn_param);

#ifdef __cplusplus
}
#endif
#endif /* __ATEL_BLE_DATA_PERIPHERAL_H */


