#ifndef __BLE_OPER_H__
#define __BLE_OPER_H__

// BLE operation code. Device scan, pair...

#include "stdint.h"
#include "platform_hal_drv.h"

// @BLE_state_definition
#define BLE_STATE_NULL					0		// Null state
#define BLE_STATE_SCAN_STARTED			1		// Scan started
#define BLE_STATE_SCAN_DONE				2		// Scan done
#define BLE_STATE_TO_PAIR				3		// To pair
#define BLE_STATE_PAIRING				4		// Pairing 
#define BLE_STATE_WT_FOR_ZAZU_CONN		5		// Wait for Zazu connection
#define BLE_STATE_WT_FOR_ZAZU_RESP		6		// Wait for Zazu response
#define BLE_STATE_WT_FOR_ZAZU_DISC		7		// Wait for Zazu disconnection

#define SCAN_RESP_SIZE	32

// Return value for function scan_list_device_pick()
#define DEV_PICK_RET_OK					0	// OK
#define DEV_PICK_RET_SCAN_LIST_EMPTY	1	// Scan list is empty (scanned no devices)
#define DEV_PICK_RET_SM_ALL_NOT_MATCH	2	// Sensor mask all not match
#define DEV_PICK_RET_NO_UNPAIRED_DEV	3	// No unpaired device

#define SCAN_LIST_INDEX_INVALID			0xffffffff

#define BLE_DG_LOG_HEADER				"Nala BLE: "	// Header for BLE diagnostic log

typedef struct
{
	uint8_t data[SCAN_RESP_SIZE];
	uint8_t len;
} scan_response_buf_t;

typedef struct
{
	ble_gap_evt_adv_report_t adv_report;
	scan_response_buf_t scan_response;
    uint32_t count;
} scan_list_t;

void ble_state_set(uint32_t state);
uint32_t ble_state_get(void);

void scan_list_clear(void);
void scan_list_add(ble_gap_evt_adv_report_t const * p_adv_report);
void scan_list_scn_resp_add(const ble_gap_addr_t *p_ble_addr, const ble_data_t *p_scan_response);
uint8_t scan_list_len_get(void);
scan_list_t * scan_list_elem_get(uint8_t index);
void scan_list_mask_set(uint32_t mask);
uint32_t scan_list_mask_get(void);
void scan_list_mask_deinit(void);

uint32_t pairing_device_index_get(void);
void pairing_device_index_set(uint32_t index);

void leds_state_recover(void);
void ble_scan_mode_enter(void);
void ble_scan_mode_exit(void);
bool is_in_ble_scan_mode(void);
void scan_mode_timeout_proc(void);

void scan_now_set(bool setting);
bool is_scan_now_enabled(void);

void pair_mode_set(bool setting);
bool is_in_pair_mode(void);
void dev_paired_cnt_add_one(void);

void ble_proc(void);

void device_in_pairing_set(bool setting);
bool is_in_pairing_state(void);
#endif
