#include "ble_oper.h"
#include "app_timer.h"
#include "user.h"
#include "peer_manager.h"

// BLE operation related code

static uint32_t ble_state = BLE_STATE_NULL;

APP_TIMER_DEF(scan_mode_timer);				// Timer for scan mode
#define SCAN_MODE_DUARATION			20000	// Unit in ms
static bool scan_mode = false;
static bool scan_mode_timeout = false;
//static bool no_pair_after_scan = true;

#define SCAN_LIST_SIZE	30
static scan_list_t scan_list[SCAN_LIST_SIZE] = {0};
static uint8_t scan_list_len = 0;
static uint32_t scan_list_mask = MASK_FOR_BIT(SENSOR_MASK_BPOS_ANY);

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

// Set BLE state
// Param. state: See @BLE_state_definition for value selection
void ble_state_set(uint32_t state)
{
	ble_state = state;
}

// Set BLE state
// Return value: See @BLE_state_definition for value definition
uint32_t ble_state_get(void)
{
	return ble_state;
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

// Clear scan list
void scan_list_clear(void)
{
	memset((void *)scan_list, 0, sizeof(scan_list));
	scan_list_len = 0;
}

// Add element to scan list
void scan_list_add(ble_gap_evt_adv_report_t const * p_adv_report)
{
	int i = 0;
	
	for (i = 0; i < scan_list_len; i++)
	{
		if (memcmp(p_adv_report->peer_addr.addr, scan_list[i].adv_report.peer_addr.addr, BLE_GAP_ADDR_LEN) == 0)	// Repeated value, ignore it
		{
            int8_t rssi = 0;
            rssi = scan_list[i].adv_report.rssi / 2 + p_adv_report->rssi / 2;
            scan_list[i].adv_report.rssi = rssi;
            scan_list[i].count++;
			NRF_LOG_RAW_INFO("scan_list_add() repeated data MeanRssi(%d) Count(%d)\r",
                             scan_list[i].adv_report.rssi, scan_list[i].count);
			return;
		}
	}
	if (scan_list_len < SCAN_LIST_SIZE)	// To avoid array overflow
	{
		memcpy(&scan_list[scan_list_len].adv_report, p_adv_report, sizeof(ble_gap_evt_adv_report_t));
		scan_list_len++;
	}
	else
	{
		NRF_LOG_RAW_INFO("scan_list_add() scan list is full\r");
	}
}

// Add the scan response data to the scan_list[] whose mac addr is same with the given one
void scan_list_scn_resp_add(const ble_gap_addr_t *p_ble_addr, const ble_data_t *p_scan_response)
{
    int i = 0;
    scan_list_t *p_element = NULL;

    for (i = 0; i < scan_list_len_get(); i++)
    {
        p_element = scan_list_elem_get(i);
        if (p_element->scan_response.len > 0) // There is already scan_response linked to the address
        {
        }
        else
        {
            if (memcmp(p_element->adv_report.peer_addr.addr, p_ble_addr->addr, BLE_GAP_ADDR_LEN) == 0)
            {
                p_element->scan_response.len = MIN(p_scan_response->len, SCAN_RESP_SIZE); // To avoid array overflow
                memcpy(p_element->scan_response.data, p_scan_response->p_data, p_element->scan_response.len);
            }
            else
            {
                // If address not matched, ignore the scan_response
            }
        }
    }
}

// Get scan list length
uint8_t scan_list_len_get(void)
{
	return scan_list_len;
}

// Get scan list element address
// Param. index: index of scan list, starts from 0
// Return value: address of scan_list[index]
scan_list_t * scan_list_elem_get(uint8_t index)
{
	if (scan_list_len == 0 || index >= scan_list_len)
		return NULL;
	return &scan_list[index];
}

uint32_t scan_list_mask_get(void)
{
	return scan_list_mask;
}

void scan_list_mask_set(uint32_t mask)
{
	scan_list_mask = mask;
}

void scan_list_mask_deinit(void)
{
	scan_list_mask = MASK_FOR_BIT(SENSOR_MASK_BPOS_ANY);
}

static uint32_t index_start = 0;
static uint32_t paired_cnt = 0;
static uint32_t pairing_device_index = SCAN_LIST_INDEX_INVALID; // Index in scan_list which currently is pairing with
static bool first_time_run = true;

// Initialize the parameters used in pair mode
static void scan_list_pair_param_init(void)
{
	index_start = 0;
	paired_cnt = 0;
	pairing_device_index = SCAN_LIST_INDEX_INVALID;
	first_time_run = true;
}

void dev_paired_cnt_add_one(void)
{
	paired_cnt++;
}

uint32_t pairing_device_index_get(void)
{
	return pairing_device_index;
}

void pairing_device_index_set(uint32_t index)
{
	pairing_device_index = index;
}

static bool is_paired_addr(const ble_gap_addr_t *p_addr)
{
	pm_peer_id_t current_peer_id = PM_PEER_ID_INVALID;
	pm_peer_data_bonding_t bonding_data;
	
	current_peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
	while (current_peer_id != PM_PEER_ID_INVALID)
	{
		pm_peer_data_bonding_load(current_peer_id, &bonding_data);
		if (memcmp(p_addr->addr, bonding_data.peer_ble_id.id_addr_info.addr, BLE_GAP_ADDR_LEN) == 0)	// Address matches
			return true;
		current_peer_id = pm_next_peer_id_get(current_peer_id);
	}
	return false;
}

// Pick one device in scan_list which matches the condition ((sensor_mask OK) AND not_paired)
// Param. start: the index number started to search in scan_list.
// Param. sensor_mask: sensor mask. Type of the device should matches sensor mask
// Param. p_index[out]: the index of finded devices would be stored in the space pointed by it
// Return value: true, the device is found and index is stored in p_index
//               false, the device can not be found in scan_list
static bool scan_list_device_pick(uint32_t start, uint32_t sensor_mask, uint32_t *p_index)
{
	int i = 0, j = 0, k = 0;
	const scan_list_t *p_element = NULL;
    const scan_list_t *p_element_m = NULL;
	uint32_t sensor_type_target = 0;
    uint8_t index_array[SCAN_LIST_SIZE] = {0};
    uint8_t index = 0;

    for (k = 0; k < scan_list_len_get(); k++)
    {
        #define BLE_PARING_MODE_INTERVAL_FACTOR (10)
        uint32_t interval = 0;
        p_element = scan_list_elem_get(k);
        interval = scan_param_dur_get() * BLE_PARING_MODE_INTERVAL_FACTOR / p_element->count;
        NRF_LOG_RAW_INFO("scan_list_device_pick() k(%d/%d) interval(%d) Count(%d) Rssi(%d) MAC LSB:\r", k, scan_list_len_get(), interval, p_element->count, p_element->adv_report.rssi);
        NRF_LOG_RAW_HEXDUMP_INFO(p_element->adv_report.peer_addr.addr, BLE_GAP_ADDR_LEN);
        NRF_LOG_FLUSH();
        if (interval >= (500 * BLE_PARING_MODE_INTERVAL_FACTOR))
        {
            index_array[k] = 1;
        }
    }
    NRF_LOG_RAW_INFO("scan_list_device_pick() index_array[%d]:\r", SCAN_LIST_SIZE);
    NRF_LOG_RAW_HEXDUMP_INFO(index_array, SCAN_LIST_SIZE);
    NRF_LOG_FLUSH();

	// for (i = start; i < scan_list_len_get(); i++)
    for (i = 0; i < scan_list_len_get(); i++)
	{
        index = 0;
        p_element_m = scan_list_elem_get(index);
        for (j = 0; j < scan_list_len_get(); j++)
        {
            if (index_array[j] == 1)
            {
                if (index == j)
                {
                    index++;
                    if (index < scan_list_len_get())
                    {
                        p_element_m = scan_list_elem_get(index);
                    }
                    else
                    {
                        NRF_LOG_RAW_INFO("scan_list_device_pick() index error\r");
                        return false;
                    }
                }
            }
            else
            {
                p_element = scan_list_elem_get(j);

                if (p_element->adv_report.rssi > p_element_m->adv_report.rssi)
                {
                    p_element_m = p_element;
                    index = j;
                }
            }
        }
        index_array[index] = 1;
		p_element = p_element_m;
        NRF_LOG_RAW_INFO("scan_list_device_pick() i(%d) index(%d) count(%d) rssi(%d) MAC LSB:\r", i, index, p_element->count, p_element->adv_report.rssi);
        NRF_LOG_RAW_HEXDUMP_INFO(p_element->adv_report.peer_addr.addr, BLE_GAP_ADDR_LEN);
        NRF_LOG_FLUSH();

        // Warning Every sensor will be paired 
        if (p_element->scan_response.len > 0) // The scan response was received
            sensor_type_target = p_element->scan_response.data[ST_POS_IN_SCN_RESP];

        NRF_LOG_RAW_INFO("scan_list_device_pick() target(%d) len(%d) mask(0x%08x)\r", sensor_type_target, p_element->scan_response.len, sensor_mask);

        if ((sensor_mask & MASK_FOR_BIT(SENSOR_MASK_BPOS_ANY)) || ((p_element->scan_response.len > 0) && (sensor_mask & MASK_FOR_BIT(sensor_type_target))))
        {
			if (is_paired_addr(&p_element->adv_report.peer_addr) == false)
			{
				*p_index = index;
                NRF_LOG_RAW_INFO("scan_list_device_pick() index(%d) NotPaired\r", index);
                NRF_LOG_FLUSH();
				return true;
			}
            else
            {
                NRF_LOG_RAW_INFO("scan_list_device_pick() index(%d) Paired\r", index);
                NRF_LOG_FLUSH();
                // Warning: Only One Device will be paired
                return false;
            }
		}
	}
	return false;
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

static void timer_handler_scan_mode(void * p_context)
{
	if (p_context == (void *)scan_mode_timer)
	{
		scan_mode_timeout = true;
	}
}

// Find the element with max RSSI and match the sensor mask in scan_list[]
// If sensor_mask bit SENSOR_MASK_BPOS_ANY is set, just find the element with max rssi
// Else, find the element with max RSSI within specific sensor type.
scan_list_t const * find_ele_with_max_rssi(uint32_t sensor_mask)
{
	int i = 0;
	int index_max = 0;
	scan_list_t const * p_element = NULL;
	uint32_t sensor_type = 0;
	bool fisrt_ele = true;
	
	for (i = 0; i < scan_list_len_get(); i++)
	{
		p_element = scan_list_elem_get(i);
		if (sensor_mask & MASK_FOR_BIT(SENSOR_MASK_BPOS_ANY))
		{
			if (p_element->adv_report.rssi > scan_list_elem_get(index_max)->adv_report.rssi)
				index_max = i;
		}
		else
		{
			if (p_element->scan_response.len > 0)
				sensor_type = p_element->scan_response.data[ST_POS_IN_SCN_RESP];
			else
				continue;	// There is no scan_response field in scan_list[i]
			if (MASK_FOR_BIT(sensor_type) & sensor_mask)
			{
				if (fisrt_ele == true)
				{
					fisrt_ele = false;
					index_max = i;
				}
				else
				{
					if (p_element->adv_report.rssi > scan_list_elem_get(index_max)->adv_report.rssi)
						index_max = i;
				}
			}
		}
	}
	
	if (sensor_mask & MASK_FOR_BIT(SENSOR_MASK_BPOS_ANY))
		return (p_element == NULL) ? NULL : scan_list_elem_get(index_max);
	else
		return (fisrt_ele == true) ? NULL : scan_list_elem_get(index_max);
}

void leds_state_recover(void)
{
	leds_blink(0, 0);
	leds_ctrl_release();
	inform_mdm_to_restore_leds_state();
}

void scan_mode_timeout_proc(void)
{
//	int i = 0;
//	const scan_list_t *p_element = NULL;

	if (scan_mode_timeout == true)
	{
		scan_mode_timeout = false;
		scan_stop();
		ble_scan_mode_exit();
		ble_state_set(BLE_STATE_SCAN_DONE);
		NRF_LOG_RAW_INFO("scan_mode_timeout_proc() scan done\r");
		ble_dg_printf(BLE_DG_LOG_HEADER "scan done\r\n");
		
//		if (no_pair_after_scan == true)	// Only output the scan result, no pair. Scan Now feature
//		{
//			ble_dg_printf(BLE_DG_LOG_HEADER "scanned device count %u\r\n", scan_list_len_get());
//			for (i = 0; i < scan_list_len_get(); i++)
//			{
//				p_element = scan_list_elem_get(i);
//				ble_dg_printf("Device %u/%u, ", i+1, scan_list_len_get());
//				if (p_element != NULL)
//					scan_result_report(p_element);
//			}
//			scan_list_mask_set(MASK_FOR_BIT(SENSOR_MASK_BPOS_ANY));
//			scan_list_clear();
//			ble_scan_mode_exit();
//			return;
//		}
//		
//		if (scan_list_len_get() > 0)
//		{
//			p_element = find_ele_with_max_rssi(scan_list_mask_get());
//			if (p_element != NULL)
//			{
//				scan_init_with_param(true, false, p_element->adv_report.peer_addr.addr);
//				scan_list_mask_set(MASK_FOR_BIT(SENSOR_MASK_BPOS_ANY));
//				
//				ble_dg_printf(BLE_DG_LOG_HEADER "paired device, type %u, ", p_element->scan_response.data[ST_POS_IN_SCN_RESP]);
//				ble_dg_printf("address: ");
//				for (i = 0; i < BLE_GAP_ADDR_LEN; i++)
//					ble_dg_printf("%02X", p_element->adv_report.peer_addr.addr[i]);
//				ble_dg_printf("\r\n");
//				pair_resp_to_mdm_send(1, 0, (uint8_t *)&sensor_mask_old, sizeof(sensor_mask_old), &p_element->adv_report.peer_addr);
//				
//				scan_start();	// The device would be paired after scan is done
//				scan_list_clear();
//				ble_scan_mode_exit();
//				leds_blink(LEDS_BLINK_INTERVAL_SLOW, LEDS_BLINK_DUR_10000MS);
//				leds_blink_callback_enable(leds_state_recover);
//				return;
//			}
//		}
//		// scan_list is empty OR no matched devices
//		ble_dg_printf(BLE_DG_LOG_HEADER "paired no device\r\n");
//		pair_resp_to_mdm_send(0, 0, (uint8_t *)&sensor_mask_old, sizeof(sensor_mask_old), NULL);
//		scan_list_mask_set(MASK_FOR_BIT(SENSOR_MASK_BPOS_ANY));
//		scan_list_clear();
//		ble_scan_mode_exit();
//		leds_blink(0, 0);
//		leds_ctrl_release();
//		inform_mdm_to_restore_leds_state();
	}
}

void ble_scan_mode_enter(void)
{
	uint32_t scan_dur = scan_param_dur_get();
	if (scan_dur == 0)
		scan_dur = SCAN_MODE_DUARATION;
	scan_mode = true;
	app_timer_create(&scan_mode_timer, APP_TIMER_MODE_SINGLE_SHOT, timer_handler_scan_mode);
	app_timer_start(scan_mode_timer, APP_TIMER_TICKS(scan_dur + 200), (void *)scan_mode_timer);
}

void ble_scan_mode_exit(void)
{
	scan_mode = false;
	app_timer_stop(scan_mode_timer);
	scan_mode_timeout = false;
}

bool is_in_ble_scan_mode(void)
{
	return scan_mode;
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

static bool scan_now_enabled = false;

void scan_now_set(bool setting)
{
	scan_now_enabled = setting;
}

bool is_scan_now_enabled(void)
{
	return scan_now_enabled;
}

static bool pair_mode = false;

void pair_mode_set(bool setting)
{
	pair_mode = setting;
}

bool is_in_pair_mode(void)
{
	return pair_mode;
}

static bool in_pairing = false;
void device_in_pairing_set(bool setting)
{
	in_pairing = setting;
}

bool is_in_pairing_state(void)
{	
	return in_pairing;
}

void scan_result_report(const scan_list_t *p_element)
{
    uint8_t Param[16] = {0};
    int16_t sig_level = p_element->adv_report.rssi;	// Signed value
	uint8_t sensor_type = SENSOR_MASK_BPOS_ANY;
	int i = 0;
	
	if (p_element->scan_response.len > 0)	// The scan_response related to the BLE address is received
    {
        sensor_type = p_element->scan_response.data[ST_POS_IN_SCN_RESP];
    }
    else
    {
        NRF_LOG_RAW_INFO("scan_result_report() scan response error.\r");
        return;
    }
	
    Param[0] = 'n';
    Param[1] = sensor_type;	// Sensor Type
	memcpy(&Param[2], p_element->adv_report.peer_addr.addr, BLE_GAP_ADDR_LEN);	// BLE mac Address
	Param[8] = sig_level & 0xff;				 	// signal strength value coded on 2 bytes in little endian format
	Param[9] = (sig_level >> 8) & 0xff;

	ble_dg_printf("sensor type %u, address (LSB):", sensor_type);
	for (i = 0; i < BLE_GAP_ADDR_LEN; i++)
		ble_dg_printf("%02X", p_element->adv_report.peer_addr.addr[i]);
	ble_dg_printf("\r\n");
	
    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, 10);
}

static void scan_now_on_scan_done(void)
{
	int i = 0;
	const scan_list_t *p_element = NULL;
	
	ble_dg_printf(BLE_DG_LOG_HEADER "scanned device count %u\r\n", scan_list_len_get());
	for (i = 0; i < scan_list_len_get(); i++)
	{
		p_element = scan_list_elem_get(i);
		ble_dg_printf("Device %u/%u, ", i+1, scan_list_len_get());
		if (p_element != NULL)
			scan_result_report(p_element);
	}
	scan_list_mask_set(MASK_FOR_BIT(SENSOR_MASK_BPOS_ANY));
	ble_state_set(BLE_STATE_NULL);
	scan_now_set(false);
}

// There are no devices in scan_list matching the supplied sensor_mask
static bool sensor_type_all_not_matched(uint32_t sensor_mask)
{
	int i = 0;
	const scan_list_t *p_element = NULL;
	uint8_t sensor_type = 0;
	
	if (sensor_mask & MASK_FOR_BIT(SENSOR_MASK_BPOS_ANY))
	{
		NRF_LOG_RAW_INFO("sensor_type_all_not_matched() return false\r");
		return false;
	}
	
	for (i = 0; i < scan_list_len_get(); i++)
	{
		p_element = scan_list_elem_get(i);
		if ((p_element != NULL) && (p_element->scan_response.len > 0))
		{
			sensor_type = p_element->scan_response.data[ST_POS_IN_SCN_RESP];
			if (MASK_FOR_BIT(sensor_type) & sensor_mask)
			{
				NRF_LOG_RAW_INFO("sensor_type_all_not_matched() return false\r");
				return false;
			}
		}
	}
	NRF_LOG_RAW_INFO("sensor_type_all_not_matched() return true\r");
	return true;
}

static bool is_ever_paired_addr(const ble_gap_addr_t *p_addr)
{
	pm_peer_id_t current_peer_id = PM_PEER_ID_INVALID;
	pm_peer_data_bonding_t bonding_data;
	
	current_peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
	while (current_peer_id != PM_PEER_ID_INVALID)
	{
		pm_peer_data_bonding_load(current_peer_id, &bonding_data);
		if (memcmp(p_addr->addr, bonding_data.peer_ble_id.id_addr_info.addr, BLE_GAP_ADDR_LEN) == 0)	// Address matches
		{
			NRF_LOG_RAW_INFO("is_ever_paired_addr() return true\r");
			return true;
		}
		current_peer_id = pm_next_peer_id_get(current_peer_id);
	}
	NRF_LOG_RAW_INFO("is_ever_paired_addr() return false\r");
	return false;
}

// The devices in scan_list are all ever paired
static bool dev_all_paired(void)
{
	int i = 0;
	const scan_list_t *p_element = NULL;
		
	for (i = 0; i < scan_list_len_get(); i++)
	{
		p_element = scan_list_elem_get(i);
		if (is_ever_paired_addr(&p_element->adv_report.peer_addr) == false)
			return false;
	}
	return true;
}

// Pair mode on event "To Pair"
static void pair_mode_on_to_pair(void)
{
	uint32_t sensor_mask = scan_list_mask_get();
	uint32_t dev_index = 0;
	const scan_list_t *p_element = NULL;

	if (first_time_run == true)
	{
		first_time_run = false;
		if (scan_list_len_get() == 0				// Scan list is empty (scanned no devices)
			|| sensor_type_all_not_matched(sensor_mask) == true	// Sensor type all not matched
			|| dev_all_paired() == true)			// The devices in scan_list are all paired
		{
			scan_list_clear();
			scan_list_mask_set(MASK_FOR_BIT(SENSOR_MASK_BPOS_ANY));
			pair_resp_to_mdm_send(0, 0, (uint8_t *)&sensor_mask, sizeof(sensor_mask), NULL);
			ble_dg_printf(BLE_DG_LOG_HEADER "no valid devices scanned\r\n");
			NRF_LOG_RAW_INFO("pair_mode_on_to_pair() no valid devices scanned\r");
			
			leds_state_recover();
			
			pair_mode_set(false);
			ble_state_set(BLE_STATE_NULL);
			return;
		}
	}
		
	if (scan_list_device_pick(index_start, sensor_mask, &dev_index) == true)
	{
		index_start++;
		p_element = scan_list_elem_get(dev_index);
		pairing_device_index_set(dev_index);
		scan_init_with_param(true, false, p_element->adv_report.peer_addr.addr);
		scan_start();
		leds_blink(LEDS_BLINK_INTERVAL_FAST, LEDS_BLINK_DUR_UNLIMITED);
		ble_dg_printf(BLE_DG_LOG_HEADER "pairing with address (LSB): %02X%02X%02X%02X%02X%02X\r\n", 
												p_element->adv_report.peer_addr.addr[0], 
												p_element->adv_report.peer_addr.addr[1], 
												p_element->adv_report.peer_addr.addr[2], 
												p_element->adv_report.peer_addr.addr[3], 
												p_element->adv_report.peer_addr.addr[4], 
												p_element->adv_report.peer_addr.addr[5]);
		NRF_LOG_RAW_INFO("pair_mode_on_to_pair() pairing with address (LSB): ");
		NRF_LOG_RAW_HEXDUMP_INFO(p_element->adv_report.peer_addr.addr, BLE_GAP_ADDR_LEN);
		ble_state_set(BLE_STATE_PAIRING);
	}
	else
	{
		if (paired_cnt > 0)	// Ever paired with some devices
		{
			if (are_leds_blinking() == true)
				leds_blink_callback_enable(leds_state_recover);
			else
				leds_state_recover();
			ble_dg_printf(BLE_DG_LOG_HEADER "pair ends\r\n");
			NRF_LOG_RAW_INFO("pair_mode_on_to_pair() pair ends\r");
		}
		else	// Paired with no devices
		{
			pair_resp_to_mdm_send(0, 0, (uint8_t *)&sensor_mask, sizeof(sensor_mask), NULL);
			ble_dg_printf(BLE_DG_LOG_HEADER "paired no devices\r\n");
			NRF_LOG_RAW_INFO("pair_mode_on_to_pair() paired no devices\r");
			
			leds_state_recover();
		}
		pair_mode_set(false);
		ble_state_set(BLE_STATE_NULL);
	}
}

// Pair mode on event "Wait for Zazu Connection"
static void pair_mode_on_wait_for_zazu_conn(void)
{
	if (is_in_pair_mode() != true)
		return;
	if (ble_aus_ready_state_get_c() == true)	// Get true when connected with Zazu
	{
		uint8_t param[8] = {0};
		
		// TODO: instance ID
		//       add suitable instance ID number
		param[0] = 0xAA;	// IO command
		param[1] = COMP_ZAZU_BLE;	// Component
		param[2] = 0;		// instance ID (channel)
		param[3] = 0x7f;	// Bitmask for info type. If 1 byte bitmask, 0x7f means requesting all info types
		monet_bleScommand(param, 4, SENSOR_MASK_BPOS_CMR, 0);

        // ble_state_set(BLE_STATE_WT_FOR_ZAZU_RESP);
		ble_state_set(BLE_STATE_TO_PAIR);
	}
}

// Pair mode on event "Wait for Zazu Disconnection"
// static void pair_mode_on_wait_for_zazu_disc(void)
// {
// 	if (is_in_pair_mode() != true)
// 		return;
// 	if (ble_aus_ready_state_get_c() != true)	// Get true when connected with Zazu, get false when disconnected
// 	{
// 		scan_stop();
// 		ble_state_set(BLE_STATE_TO_PAIR);
// 	}
// }

// BLE operation process
// This function is designed to be called in main loop
void ble_proc(void)
{
	switch (ble_state_get())
	{
		case BLE_STATE_NULL:
		{
			device_in_pairing_set(false);
		}
			break;
		case BLE_STATE_SCAN_STARTED:
		{
			device_in_pairing_set(false);
		}
			break;
		case BLE_STATE_SCAN_DONE:
		{
			if (is_scan_now_enabled() == true)
				scan_now_on_scan_done();
			if (is_in_pair_mode() == true)
			{
				ble_state_set(BLE_STATE_TO_PAIR);
				scan_list_pair_param_init();
			}
		}
			break;
		case BLE_STATE_TO_PAIR:
		{
			if (is_in_pair_mode() == true)
			{
				device_in_pairing_set(true);
				pair_mode_on_to_pair();
			}
		}
			break;
		case BLE_STATE_PAIRING:		// 
		{
			device_in_pairing_set(true);
		}
			break;
		case BLE_STATE_WT_FOR_ZAZU_CONN:
			pair_mode_on_wait_for_zazu_conn();
			break;
		// case BLE_STATE_WT_FOR_ZAZU_RESP:
		// 	break;
		// case BLE_STATE_WT_FOR_ZAZU_DISC:
		// 	pair_mode_on_wait_for_zazu_disc();
		// 	break;
		default:
			break;
	}
}
