/*
 * ble_data_c.h
 *
 *  Created on: Apr 27, 2020
 *      Author: Yangjie Gu
 */

#ifndef __ATEL_BLE_DATA_CENTRAL_H
#define __ATEL_BLE_DATA_CENTRAL_H
#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

#define BLE_DATA_SEND_TIMER_PERIOD_MS (9)//(10)//(20)

#define BLE_DATA_SEND_BUFFER_CELL_NUM (100)//(80) // cell_num = BLE_DATA_SEND_BUFFER_CELL_NUM - 1
#define BLE_DATA_SEND_BUFFER_CELL_LEN (168)

#define BLE_DATA_RECV_BUFFER_CELL_NUM (500) //(700) // cell_num = BLE_DATA_RECV_BUFFER_CELL_NUM - 1
#define BLE_DATA_RECV_BUFFER_CELL_LEN (168)

#define BLE_DATA_THROUGHPUT_TEST_EN (0)
#define BLE_DATA_THROUGHPUT_ERROR_CHECK_EN (0)

extern uint32_t m_len_recv;
extern uint8_t data_throughput_index;

extern uint32_t ble_aus_data_send_central(uint8_t * p_data, uint16_t data_len, uint8_t st, uint8_t id);
extern void ble_aus_set_scan_filter(uint8_t *p_addr);
extern void ble_aus_set_scan_start(void);
extern void ble_aus_set_scan_mode(uint8_t mode);

extern void ble_send_data_rate_show(uint8_t force);
extern void ble_send_recv_init_c(void);
extern void ble_send_timer_start_c(void);
extern void ble_send_timer_stop_c(void);
extern int8_t is_ble_send_queue_full(void);
extern int8_t is_ble_send_queue_empty(void);
extern int8_t ble_send_data_push(uint8_t *p_data, uint16_t len, uint8_t st, uint8_t id);
extern int8_t ble_recv_data_push(uint8_t *p_data, uint16_t len, uint8_t st, uint8_t id);
extern int8_t is_ble_recv_queue_empty(void);
extern int8_t ble_recv_data_pop(uint8_t **p_data, uint16_t *p_len, uint8_t *p_st, uint8_t *p_id);
extern void ble_recv_data_delete_one(void);

extern uint8_t my_checksum_8(uint8_t *p_data, uint16_t len);

#ifdef __cplusplus
}
#endif
#endif /* __ATEL_BLE_DATA_CENTRAL_H */


