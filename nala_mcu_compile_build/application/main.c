/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include "ble_data_c.h"
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_db_discovery.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_fstorage_sd.h"
//#include "ble_nus_c.h"
#include "ble_aus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "nrf_ble_gatt.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "acc_simba_lis2dh12.h"
#include "mp2762a_drv.h"
#include "solar_mppt.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "platform_hal_drv.h"
#include "nrf_delay.h"
#include "user.h"
//#include "ble_data.h"
#include "version.h"
#include "mux_mcu.h"


#if  (SUPPORT_BLE_CENTRAL_AND_PERIPH == 1)
#include "ble_conn_params.h"

#define DEVICE_NAME                     "miyue"                                     /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(30, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(5000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    0                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_ADV_INTERVAL                906                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 800 ms). */
#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BEACON_INFO_LENGTH          0x17                               /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                               /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x01                               /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xCC  // 0xC3                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_BEACON_UUID                 0xFB, 0xDD, 0x00, 0x01, \
                                        0xAA, 0x76, 0x42, 0x3A, \
                                        0xB5, 0x2F, 0x6F, 0x74, \
                                        0xEC, 0xDE, 0x9E, 0x3C            /**< Proprietary UUID for Beacon. */
                                        

//#define MODULE_SN_LENGTH      (8u)

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_PERIPHERAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
//NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    //{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
    {0x0001,NUS_SERVICE_UUID_TYPE}
};





uint8_t m_sn_info[MODULE_SN_LENGTH + 1] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, '\0'};

uint32_t beacon_adv_interval = APP_ADV_INTERVAL;
uint16_t beacon_adv_duration = APP_ADV_DURATION;

static int8_t tx_power_level_default = 8;                                      // The code from simba.In nala change the default value to 8dBm.
tx_power_struct tx_pwr_rssi[TX_LEVEL_LAST] = {
    [TX_LEVEL_P8].txRssi = 0xD0,
    [TX_LEVEL_P4].txRssi = 0xCC,
    [TX_LEVEL_0].txRssi = 0xC7,
    [TX_LEVEL_N4].txRssi = 0xC3,
    [TX_LEVEL_N8].txRssi = 0xBD,
    [TX_LEVEL_N12].txRssi = 0xB9,
    [TX_LEVEL_N16].txRssi = 0xB5,
    [TX_LEVEL_N20].txRssi = 0xB0,
};

static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    MNT_MAJOR,
    MNT_MINOR,
    MNT_REVISION,
    MNT_BUILD,
    // APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    // APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};

static bool ble_evt_is_advertising_timeout(ble_evt_t const * p_ble_evt);
ble_advertising_init_t init = {0}; 
ble_advdata_manuf_data_t p_m_manuf_specific_data = {0};
static ble_advdata_manuf_data_t m_manuf_specific_data = {0}; 
static ble_advdata_manuf_data_t manuf_specific_data = {0}; 
uint8_t tx_1m_rssi = APP_MEASURED_RSSI;


 //static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
 //static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
 static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */
 static uint8_t              m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};


#endif /* (SUPPORT_BLE_CENTRAL_AND_PERIPH == 1) */


#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

#define SEC_PARAM_BOND              1                                   /**< Perform bonding. */
#define SEC_PARAM_MITM              0                                   /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC              1                                   /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS          0                                   /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES   BLE_GAP_IO_CAPS_NONE                /**< No I/O capabilities. */
#define SEC_PARAM_OOB               0                                   /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE      7                                   /**< Minimum encryption key size in octets. */
#define SEC_PARAM_MAX_KEY_SIZE      16                                  /**< Maximum encryption key size in octets. */

#define SCAN_DURATION_WITELIST      3000                                /**< Duration of the scanning in units of 10 milliseconds. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define ECHOBACK_BLE_UART_DATA  0                                       /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */


BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/**@brief NUS UUID. */
static ble_uuid_t const m_nus_uuid =
{
    .uuid = BLE_UUID_NUS_SERVICE,
    .type = NUS_SERVICE_UUID_TYPE
};

#define BLE_POWER_LEVEL_DEFAULT (8)

volatile bool ble_aus_ready_c = false;		// If true, connected, else, not connected
volatile uint32_t g_ble_data_error = 0;
ble_gap_addr_t central_addr = {0};
static void on_ble_central_evt(ble_evt_t const* p_ble_evt);


static ble_gap_addr_t m_target_periph_addr =
{
    /* Possible values for addr_type:
       BLE_GAP_ADDR_TYPE_PUBLIC,
       BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
       BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE,
       BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE. */
    .addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
    .addr      = {0, 0, 0, 0, 0, 0}
};


// Default scan parameters
//static ble_gap_scan_params_t const m_scan_param_deflt =
//{
//    .active        = 0x01,
//#if (NRF_SD_BLE_API_VERSION > 7)
//    .interval_us   = NRF_BLE_SCAN_SCAN_INTERVAL * UNIT_0_625_MS,
//    .window_us     = NRF_BLE_SCAN_SCAN_WINDOW * UNIT_0_625_MS,
//#else
//    .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
//    .window        = NRF_BLE_SCAN_SCAN_WINDOW,
//#endif // (NRF_SD_BLE_API_VERSION > 7)
//    .timeout       = NRF_BLE_SCAN_SCAN_DURATION,
//    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
//    .scan_phys     = BLE_GAP_PHY_1MBPS,
//};

static ble_gap_scan_params_t m_scan_param =
{
    .active        = 0x01,
#if (NRF_SD_BLE_API_VERSION > 7)
    .interval_us   = NRF_BLE_SCAN_SCAN_INTERVAL * UNIT_0_625_MS,
    .window_us     = NRF_BLE_SCAN_SCAN_WINDOW * UNIT_0_625_MS,
#else
    .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
    .window        = NRF_BLE_SCAN_SCAN_WINDOW,
#endif // (NRF_SD_BLE_API_VERSION > 7)
    .timeout       = NRF_BLE_SCAN_SCAN_DURATION,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .scan_phys     = BLE_GAP_PHY_1MBPS,
};

uint8_t pair_success = 0;
uint32_t sensor_mask_old_r = 0;

// Set scan parameters.
// Unit of parameters is in ms
bool scan_param_set(uint32_t scan_window_ms, uint32_t scan_interval_ms, uint32_t scan_duration_ms)
{
	if (!scan_window_ms || !scan_interval_ms)
		return false;
	m_scan_param.window = scan_window_ms / 0.625;
	m_scan_param.interval = scan_interval_ms / 0.625;
	m_scan_param.timeout = scan_duration_ms / 10;	// "If set to 0x0000, the scanning continues until it is explicitly disabled". See NRF_BLE_SCAN_SCAN_DURATION
	return true;
}

// Get duration in scan parameters. Unit in ms.
// "If set to 0x0000, the scanning continues until it is explicitly disabled". See NRF_BLE_SCAN_SCAN_DURATION
uint32_t scan_param_dur_get(void)
{
	return m_scan_param.timeout * 10;
}

static uint16_t m_conn_handle = 0xffff;

#if BLE_DATA_THROUGHPUT_ERROR_CHECK_EN
uint8_t data_throughput_last = 0;
bool data_throughput_wait_index0 = true;
uint32_t data_throughput_total_count = 0, data_throughput_total_lost = 0;
#endif /* BLE_DATA_THROUGHPUT_ERROR_CHECK_EN */

/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nus_error_handler(uint32_t nrf_error)
{
    g_ble_data_error = nrf_error;
    // APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function to start scanning. */
/*static*/ uint32_t scan_start(void)
{
    ret_code_t ret;

    ret = nrf_ble_scan_start(&m_scan);
    NRF_LOG_RAW_INFO("nrf_ble_scan_start error(%d)\r", ret);
	NRF_LOG_FLUSH();
	return ret;
    // APP_ERROR_CHECK(ret);

    // ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    // APP_ERROR_CHECK(ret);
}

/**@brief Function for stopping scanning.
 */
void scan_stop(void)
{
    nrf_ble_scan_stop();
}

void monet_bleCcommand_C_resp_lite(void);

/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch (p_scan_evt->scan_evt_id)
    {
		 case NRF_BLE_SCAN_EVT_FILTER_MATCH:
		 {
			 ble_gap_evt_adv_report_t const * p_adv_report = p_scan_evt->params.filter_match.p_adv_report;
			 
			 if (is_in_ble_scan_mode() == true)
				scan_list_add(p_adv_report);
			 
			 NRF_LOG_INFO("Filter matched mac addr %02x%02x%02x%02x%02x%02x",
                      p_adv_report->peer_addr.addr[0],
                      p_adv_report->peer_addr.addr[1],
                      p_adv_report->peer_addr.addr[2],
                      p_adv_report->peer_addr.addr[3],
                      p_adv_report->peer_addr.addr[4],
                      p_adv_report->peer_addr.addr[5] );
			 NRF_LOG_INFO("type %02x, tx_p %d, rssi %d", 
							*( (uint16_t *) (&p_adv_report->type) ),
							p_adv_report->tx_power, 
							p_adv_report->rssi);
            if (ble_link_target.action == 1)
            {
                paired_ble_txrx_power[0].tx = 8;
                paired_ble_txrx_power[0].rx = (int16_t)p_adv_report->rssi;
            }
			//  NRF_LOG_INFO("p_adv_report len %u", p_adv_report->data.len);
			//  NRF_LOG_HEXDUMP_INFO(p_adv_report->data.p_data, p_adv_report->data.len);
		 } break;
		 
         case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
         {
             err_code = p_scan_evt->params.connecting_err.err_code;
			 NRF_LOG_INFO("CONNECTING_ERROR %d \r", err_code);
            //  APP_ERROR_CHECK(err_code);
         } break;

         case NRF_BLE_SCAN_EVT_CONNECTED:
         {
              ble_gap_evt_connected_t const * p_connected =
                               p_scan_evt->params.connected.p_connected;
			  
//			  monet_bleCcommand_C_resp_lite();
			  
             // Scan is automatically stopped by the connection.
             monet_data.ble_peer_mac_addr[0] = p_connected->peer_addr.addr[0];
             monet_data.ble_peer_mac_addr[1] = p_connected->peer_addr.addr[1];
             monet_data.ble_peer_mac_addr[2] = p_connected->peer_addr.addr[2];
             monet_data.ble_peer_mac_addr[3] = p_connected->peer_addr.addr[3];
             monet_data.ble_peer_mac_addr[4] = p_connected->peer_addr.addr[4];
             monet_data.ble_peer_mac_addr[5] = p_connected->peer_addr.addr[5];
             NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                      p_connected->peer_addr.addr[0],
                      p_connected->peer_addr.addr[1],
                      p_connected->peer_addr.addr[2],
                      p_connected->peer_addr.addr[3],
                      p_connected->peer_addr.addr[4],
                      p_connected->peer_addr.addr[5]
                      );
         } break;

         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
         {
             NRF_LOG_RAW_INFO("<<<<<<<<<<Scan timed out>>>>>>>>>>\r");
             if (ble_link_target.action == 1)
             {
                 ble_link_target.result = 0;
                 ble_link_target.noreported = 1;
             }
//             scan_start();
         } break;
		 
         default:
             break;
    }
}

/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    NRF_LOG_RAW_INFO("scan_init\r");
    NRF_LOG_FLUSH();

	init_scan.p_scan_param = &m_scan_param;
	init_scan.connect_if_match = true;
//	init_scan.connect_if_match = false;
	init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
    APP_ERROR_CHECK(err_code);
}

void scan_init_with_param(bool connect_if_match, bool no_pair, const uint8_t *p_ble_addr)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    NRF_LOG_RAW_INFO("scan_init_with_param\r");
    NRF_LOG_FLUSH();

    memset(&init_scan, 0, sizeof(init_scan));

	init_scan.p_scan_param = &m_scan_param;
//	init_scan.connect_if_match = true;
	init_scan.connect_if_match = connect_if_match;
	init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

	nrf_ble_scan_all_filter_remove(&m_scan);	// Clear filter before set new filter
	
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
    APP_ERROR_CHECK(err_code);

	if (p_ble_addr == NULL)
	{
		err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
		APP_ERROR_CHECK(err_code);
	}
	else
	{
		err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_ADDR_FILTER, p_ble_addr);
		APP_ERROR_CHECK(err_code);
		err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER | SCAN_ADDR_FILTER, true);
		APP_ERROR_CHECK(err_code);
	}
	
//	no_pair_after_scan = no_pair;
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}

#if BLE_DATA_THROUGHPUT_ERROR_CHECK_EN
__STATIC_INLINE void ble_data_throughput_check_correction_reset(void)
{
    data_throughput_index = 0;
    data_throughput_last = 0;
    data_throughput_total_lost = 0;
    data_throughput_total_count = 0;
    data_throughput_wait_index0 = true;
}

__STATIC_INLINE void ble_data_throughput_check_correction(uint8_t *p_data, uint8_t len)
{
    uint8_t delta = 0;

    if (data_throughput_wait_index0)
    {
        NRF_LOG_RAW_INFO(">>>>wait_index0 p_data[0]: %u\r", p_data[0]);
        NRF_LOG_FLUSH();
        data_throughput_last = 0;
        data_throughput_total_lost = 0;
        data_throughput_total_count = 0;

        if (p_data[0] != 0)
        {
            return;
        }
        data_throughput_wait_index0 = false;
    }

    delta = p_data[0] - data_throughput_last;
    data_throughput_last = p_data[0];
    // NRF_LOG_RAW_INFO(">>>>p_data[0]: %u, last: %u, delta: %u\r", p_data[0], data_throughput_last, delta);
    data_throughput_total_count += delta;
    if (delta > 1)
    {
        NRF_LOG_RAW_INFO(">>>>Central detected data lost: %u\r", data_throughput_total_lost);
        NRF_LOG_FLUSH();
        data_throughput_total_lost += (delta - 1);
    }

    if (my_checksum_8(p_data, len - 1) != p_data[len - 1])
    {
        NRF_LOG_RAW_INFO(">>>>Central checksum_8 fail(%u:%u)\r", data_throughput_total_count, data_throughput_total_lost);
        NRF_LOG_FLUSH();
    }

    if ((data_throughput_total_count % 256) == 255)
    {
        NRF_LOG_RAW_INFO(">>>>Central AUS(%u) (%u:%u) lost:"NRF_LOG_FLOAT_MARKER"\r",
                    len,
                    data_throughput_total_count,
                    data_throughput_total_lost,
                    NRF_LOG_FLOAT((double)data_throughput_total_lost / (double)data_throughput_total_count));
        NRF_LOG_FLUSH();
    }
}
#endif /* BLE_DATA_THROUGHPUT_ERROR_CHECK_EN */

/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
//    ret_code_t ret_val;

    m_len_recv += data_len;

    if (monet_data.bleConnectionStatus == 0)
    {
        monet_data.bleConnectionEvent = 1;
        monet_data.bleConnectionEventDelay = 10;
        ble_aus_ready_c = true;
        if (monet_data.ble_scan_mode == 1)
        {
            monet_data.ble_scan_mode = 0;
            NRF_LOG_INFO(">>>BLE Connection change scan_mode(%d).\r", monet_data.ble_scan_mode);
            ble_aus_set_scan_mode(monet_data.ble_scan_mode);
            monet_data.bleDisconnectTimeMs = 0;
            monet_data.ble_scan_reset_ms = 0;
        }
    }

//    if ((p_data[0] == 0x66) && (p_data[1] == 0x00))
//    {
////        pf_gpio_pattern_set(p_data + 2, data_len - 2);
//        NRF_LOG_INFO("AUS GPIO Control(%d:%d).", p_data[2], p_data[3]);
//        return;
//    }

    monet_data.bleRecvDataEvent = 1;

	if (com_method_zazu_get() == COM_METHOD_ZAZU_BLE)
		ble_recv_data_push(p_data, data_len, SENSOR_MASK_BPOS_CMR, 0);
//    camera_poweroff_delay_refresh();
    #if BLE_DATA_THROUGHPUT_ERROR_CHECK_EN
    ble_data_throughput_check_correction(p_data, data_len);
    #endif /* BLE_DATA_THROUGHPUT_ERROR_CHECK_EN */
    // NRF_LOG_HEXDUMP_INFO(p_data, data_len);

    // for (uint32_t i = 0; i < data_len; i++)
    // {
    //     do
    //     {
    //         ret_val = app_uart_put(p_data[i]);
    //         if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
    //         {
    //             NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
    //             APP_ERROR_CHECK(ret_val);
    //         }
    //     } while (ret_val == NRF_ERROR_BUSY);
    // }
    // if (p_data[data_len-1] == '\r')
    // {
    //     while (app_uart_put('\n') == NRF_ERROR_BUSY);
    // }
//    if (ECHOBACK_BLE_UART_DATA)
//    {
//        // Send data back to the peripheral.
//        do
//        {
//            ret_val = ble_nus_c_string_send(&m_ble_nus_c, p_data, data_len);
//            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
//            {
//                NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
//                APP_ERROR_CHECK(ret_val);
//            }
//        } while (ret_val == NRF_ERROR_BUSY);
//    }
}

// Find the positiion of key in p_data[]
// Return value: position, starts from 0. if <0, not find; else, return the position
int32_t find_pos(uint8_t key, const uint8_t * p_data, uint32_t len)
{
	int i = 0;
	
	for (i = 0; i < len; i++)
	{
		if (p_data[i] == key)
			return i;
	}
	return -1;
}

#define CHAR_ESP			'!'
#define CHAR_ESP_COMP1		'0'
#define CHAR_ESP_COMP2		'1'

#define CHAR_ESP_DECODE1	'$'
#define CHAR_ESP_DECODE2	'!'

// Decode the given data
// Param. p_dest: space to store the decoded data
// Param. p_d_len: [in]length of space. [out]length of decoded data
// Param. p_src: data to be decoded
// Param. s_len: length of data to be decoded
// Return value: if true, OK; else, meet error
bool decode_esp(uint8_t * p_dest, uint32_t *p_d_len, const uint8_t * p_src, uint32_t s_len)
{
	int buf_len = *p_d_len;
	int dest_len = 0;
	int i = 0;
	
	for (i = 0; i < s_len; /*i++*/)
	{
		if (dest_len >= (buf_len-1))	// There is no enough space in destination buffer
			return false;
		
		if (p_src[i] == CHAR_ESP && ((i+1) < s_len))
		{
			if (p_src[i+1] == CHAR_ESP_COMP1)
			{
				p_dest[dest_len++] = CHAR_ESP_DECODE1;
				i += 2;
			}
			else if (p_src[i+1] == CHAR_ESP_COMP2)
			{
				p_dest[dest_len++] = CHAR_ESP_DECODE2;
				i += 2;
			}
			else
			{
				p_dest[dest_len++] = p_src[i];	// Jump 1 byte while not 2. This would not make mistake when meet sequence like "!!0"
				i++;
			}
		}
		else
		{
			p_dest[dest_len++] = p_src[i];
			i++;
		}
	}
	*p_d_len = dest_len;
	return true;
}

uint8_t checksum_cal(const uint8_t *p_data, uint32_t len)
{
	uint8_t sum = 0;
	int i = 0;
	
	if (len == 0 || p_data == NULL)
		return 0;
	for (i = 0; i < len; i++)
		sum += p_data[i];
	return (sum ^ 0xff);
}

// Data format: [I2C_len ['$' len ['4' ch [data ...]] chksum]]
void i2c_received_data_proc(uint8_t * p_data, uint16_t data_len)
{
	uint8_t buf[MUX_MCU_PACK_SIZE];
	uint32_t buf_len = MUX_MCU_PACK_SIZE;
	int16_t pos_dollar = 0;
	bool ret = false; 
	uint32_t len_payload = 0;
	uint8_t chksum = 0;
	uint8_t cmd = 0;

    m_len_recv += data_len;		// Update data Rx speed
	
//	int i = 0;
//	NRF_LOG_RAW_INFO("i2c_received_data_proc() len %u\r\n", data_len);
//	for (i = 0; i < data_len; i++)
//	{
//		NRF_LOG_RAW_INFO("%02x ", p_data[i]);
//		if (i % 10 == 9)
//		{
//			NRF_LOG_RAW_INFO("\r");
//			NRF_LOG_FLUSH();
//		}
//	}
//	NRF_LOG_RAW_INFO("\r\n");
//	NRF_LOG_FLUSH();
	
	if (data_len > MUX_MCU_PACK_SIZE)
	{
		NRF_LOG_RAW_INFO("i2c_received_data_proc() err, data len %u\r\n", data_len);
		NRF_LOG_FLUSH();
		return;
	}
	pos_dollar = find_pos('$', p_data, data_len);
	if (pos_dollar < 0)
	{
		NRF_LOG_RAW_INFO("i2c_received_data_proc() err, no $ sign %u\r\n");
		NRF_LOG_FLUSH();
		return;
	}
	ret = decode_esp(&buf[0], &buf_len, &p_data[pos_dollar], data_len-pos_dollar);
	if (ret != true || buf[0] != '$')
	{
		NRF_LOG_RAW_INFO("i2c_received_data_proc() err, decode_esp\r\n");
		NRF_LOG_FLUSH();
		return;
	}
	len_payload = buf[1];
	if (len_payload < 2)
	{
		NRF_LOG_RAW_INFO("i2c_received_data_proc() err, payload len %u\r\n", len_payload);
		NRF_LOG_FLUSH();
		return;
	}
	chksum = checksum_cal(&buf[2], len_payload);
	if (chksum != buf[1+1+len_payload])
	{
		NRF_LOG_RAW_INFO("i2c_received_data_proc() err, chksum cal(%02x) read(%02x)\r\n", chksum, buf[1+1+len_payload]);
		NRF_LOG_FLUSH();
		return;
	}
	cmd = buf[2];
	if (cmd == IO_CMD_CHAR_I2C_RECV)
	{
		ble_recv_data_push(&buf[4], len_payload-2, SENSOR_MASK_BPOS_CMR, buf[3]);
	}
	else
	{
		NRF_LOG_RAW_INFO("i2c_received_data_proc() err, unknown cmd %02x\r\n", cmd);
		NRF_LOG_FLUSH();
		return;
	}
}

/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;

    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete.");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            m_conn_handle = p_ble_nus_evt->conn_handle;

            ble_link_target.conn_handle_valid = 1;
            ble_link_target.conn_handle = p_ble_nus_c->conn_handle;
            ble_link_target.ready = 1;

            // Initiate bonding.
            err_code = pm_conn_secure(p_ble_nus_c->conn_handle, false);
            if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }
			
            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Atel UART Service.");
//            if (monet_data.ble_scan_mode == 1)
//            {
//                monet_data.ble_scan_mode = 0;
//                NRF_LOG_INFO(">>>BLE Connection change scan_mode(%d).\r", monet_data.ble_scan_mode);
//                ble_aus_set_scan_mode(monet_data.ble_scan_mode);
//                monet_data.bleDisconnectTimeMs = 0;
//                monet_data.ble_scan_reset_ms = 0;
//            }

            #if BLE_DATA_THROUGHPUT_ERROR_CHECK_EN
            ble_data_throughput_check_correction_reset();
            #endif /* BLE_DATA_THROUGHPUT_ERROR_CHECK_EN */
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("BLE_NUS_C_EVT_DISCONNECTED Disconnected.");
//            scan_start();		// Disconnecting with the device meets failure if this line is not commented
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */


/**
 * @brief Function for handling shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
// static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
// {
//     ret_code_t err_code;

//     err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//     APP_ERROR_CHECK(err_code);

//     switch (event)
//     {
//         case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
//             // Prepare wakeup buttons.
//             err_code = bsp_btn_ble_sleep_mode_prepare();
//             APP_ERROR_CHECK(err_code);
//             break;

//         default:
//             break;
//     }

//     return true;
// }

// NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);

char const * phy_str(ble_gap_phys_t phys)
{
    static char const * str[] =
    {
        "1 Mbps",
        "2 Mbps",
        "Coded",
        "Unknown"
    };

    switch (phys.tx_phys)
    {
        case BLE_GAP_PHY_1MBPS:
            return str[0];

        case BLE_GAP_PHY_2MBPS:
        case BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS:
        case BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED:
            return str[1];

        case BLE_GAP_PHY_CODED:
            return str[2];

        default:
            return str[3];
    }
}


#if (SUPPORT_BLE_CENTRAL_AND_PERIPH == 1)

static bool ble_evt_is_advertising_timeout(ble_evt_t const * p_ble_evt)
{
    return (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_SET_TERMINATED);
}

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    // uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            // err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            // APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            // sleep_mode_enter();
            break;
        case BLE_ADV_EVT_FAST_WHITELIST:
            NRF_LOG_INFO("Fast advertising with Whitelist");
            break;
        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            // Apply the whitelist.
            //APP_ERROR_CHECK(ble_advertising_whitelist_reply(&m_advertising, whitelist_addrs, whitelist_count, NULL, 0));
        }
        break;
        default:
            break;
    }
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    // cp_init.evt_handler                 = on_conn_params_evt;
    cp_init.evt_handler                    = NULL;
    // cp_init.error_handler               = conn_params_error_handler;
    cp_init.error_handler                  = NULL;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    uint16_t channel = 0xffff;
   // uint16_t s_len_recv = 0;

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
//         uint8_t *p_data = (uint8_t *)p_evt->params.rx_data.p_data;
// //        uint8_t data_len = (uint8_t)p_evt->params.rx_data.length;
//         s_len_recv += p_evt->params.rx_data.length;

//         for (uint32_t i = 0; i < s_len_recv; i++)
//         {
//             GetRxCommandEsp(p_data[i]);
//             //NRF_LOG_RAW_INFO(":%0x",p_data[i]);
//         }

//         // as slave transfer to master.
//         ble_nus_data_send(&m_nus, p_data, &s_len_recv, p_evt->conn_handle);  // 0803wfyadd. for a test, loopback the data to master.

        // uint8_t sendbuff[10] = {0};
        // sendbuff[0] = 'Z';
        // sendbuff[1] = 0x12;
        // sendbuff[2] = 0x13;
        // sendbuff[3] = 0x14;
        // sendbuff[4] = 0x15;
        // sendbuff[5] = 0x16;
        // sendbuff[6] = 0x17;
        // // sendbuff[7] = 0x18;
        // // sendbuff[8] = 0x19;
        // uint8_t len = 7;
        // BuildFrame(IO_CMD_CHAR_BLE_CTRL, sendbuff, len);


        //channel = ble_channel_get_from_handler(p_evt->conn_handle);
        // NRF_LOG_INFO("BLE_AUS_EVT_RX_DATA CH: %d len: %d.", channel, p_evt->params.rx_data.length);
        // NRF_LOG_FLUSH();
//        #if BLE_CONNECTION_SUPPORT_HEARTBEAT
//        if ((((uint8_t *)p_evt->params.rx_data.p_data)[0] == 0x65) &&
//            (((uint8_t *)p_evt->params.rx_data.p_data)[1] == 0x00))
//        {
//            uint32_t beat = (((uint8_t *)p_evt->params.rx_data.p_data)[2] << 0) |
//                             (((uint8_t *)p_evt->params.rx_data.p_data)[3] << 8) |
//                             (((uint8_t *)p_evt->params.rx_data.p_data)[4] << 16)|
//                             (((uint8_t *)p_evt->params.rx_data.p_data)[5] << 24);
//            uint16_t min_100us = (((uint8_t *)p_evt->params.rx_data.p_data)[12] << 0) |
//                                 (((uint8_t *)p_evt->params.rx_data.p_data)[13] << 8);
//            uint32_t timeout_100us = (((uint8_t *)p_evt->params.rx_data.p_data)[18] << 0) |
//                                     (((uint8_t *)p_evt->params.rx_data.p_data)[19] << 8) |
//                                     (((uint8_t *)p_evt->params.rx_data.p_data)[20] << 16)|
//                                     (((uint8_t *)p_evt->params.rx_data.p_data)[21] << 24);
//            uint8_t camera_on = ((uint8_t *)p_evt->params.rx_data.p_data)[22];
//            uint8_t camera_act = ((uint8_t *)p_evt->params.rx_data.p_data)[23];
//            channel = ble_channel_get_from_mac((uint8_t *)(p_evt->params.rx_data.p_data + 6));

//            if (channel != 0xffff)
//            {
//                if (monet_data.ble_info[channel].connect_status != BLE_CONNECTION_STATUS_CONNECTED)
//                {
//                    monet_data.bleConnectionEvent = 1 + channel;
//                    monet_data.ble_info[channel].connect_status = BLE_CONNECTION_STATUS_CONNECTED;
//                    monet_data.ble_info[channel].handler = p_evt->conn_handle;
//                    if ((min_100us != monet_data.ble_conn_param[channel].min_100us) &&
//                        (monet_data.ble_conn_param[channel].min_100us))
//                    {
//                        (monet_data.ble_conn_param_check_in_heartbeat[channel])++;
//                        if (monet_data.ble_conn_param_check_in_heartbeat[channel] >= BLE_CONN_PARM_CHECK_IN_HEARTBEAT_COUNT)
//                        {
//                            monet_data.ble_conn_param_check_in_heartbeat[channel] = 0;
//                            monet_data.ble_conn_param_update_ok[channel] = 0xff;
//                            NRF_LOG_RAW_INFO(">>>ble_conn_param_check_in_heartbeat.\r");
//                        }
//                    }
//                    else
//                    {
//                        monet_data.ble_conn_param_check_in_heartbeat[channel] = 0;
//                    }
//                }
//            }

//            NRF_LOG_RAW_INFO(">>>CH: %d HB(%d) Min(%d) Out(%d) On(%d) Act(%d).\r",
//                            channel, beat, min_100us, timeout_100us, camera_on, camera_act);
//            return;
//        }
//        #endif /* BLE_CONNECTION_SUPPORT_HEARTBEAT */

//        if ((p_data[0] == 0x66) && (p_data[1] == 0x00))
//        {
//            pf_gpio_pattern_set(p_data + 2, data_len - 2);
//            NRF_LOG_INFO("AUS GPIO Control(%d:%d).", p_data[2], p_data[3]);
//            return;
//        }
        // if (p_data[0] == 0xDD)
        // {
        //     NRF_LOG_RAW_INFO("Receive nala 0xDD command 1 \r");
        // }
//        else
//        {
//            if (p_data[0] == 0xAA)
//            {
//                CRITICAL_REGION_ENTER();
//                monet_data.mcuDataHandleMs = MCU_DATA_HANDLE_TIMEOUT_MS;		
//                CRITICAL_REGION_EXIT();
//                NRF_LOG_RAW_INFO("AUS AA Com(%x) InstID(%d) BM_Len(%d) ", 
//                                p_data[1], 
//                                p_data[2],
//                                data_len);
//                NRF_LOG_RAW_INFO("BM(%02x:%02x:%02x:%02x)\r", 
//                                p_data[3], 
//                                p_data[4], 
//                                p_data[5], 
//                                p_data[6]);
//            }
//            else
//            {
//                CRITICAL_REGION_ENTER();
//                monet_data.RecvDataEvent = 1;
//                CRITICAL_REGION_EXIT();
//            }
//        }

//        ble_recv_data_push((uint8_t *)p_evt->params.rx_data.p_data, p_evt->params.rx_data.length, channel);
//        camera_poweroff_delay_refresh();
//        #if BLE_DATA_THROUGHPUT_ERROR_CHECK_EN
//        ble_data_throughput_check_correction((uint8_t *)p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
//        #endif /* BLE_DATA_THROUGHPUT_ERROR_CHECK_EN */

//        // NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
//        // NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
    }
    else if (p_evt->type == BLE_NUS_EVT_COMM_STARTED)
    {
        #if BLE_DATA_THROUGHPUT_ERROR_CHECK_EN
        ble_data_throughput_check_correction_reset();
        #endif /* BLE_DATA_THROUGHPUT_ERROR_CHECK_EN */

        channel = ble_channel_get_from_handler(p_evt->conn_handle);
        NRF_LOG_RAW_INFO("AUS_EVT_COMM_STARTED CH: %d.\r", channel);
        NRF_LOG_FLUSH();

        if (channel != 0xffff)
        {
//            CRITICAL_REGION_ENTER();
//            ble_connection_status_refresh();
//            monet_data.bleConnectionEvent = 1 + channel;
//            monet_data.ble_info[channel].connect_status = BLE_CONNECTION_STATUS_CONNECTED;
//            CRITICAL_REGION_EXIT();
        }
    }
    else if (p_evt->type == BLE_NUS_EVT_TX_RDY)
    {
//        ble_data_send_with_queue();
    }
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // for (uint32_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)  // for more than one queue
    // {
    //     err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init);
    //     APP_ERROR_CHECK(err_code);
    // }


    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

void ble_adv_start(void)
{
    ret_code_t   err_code;
    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    adv_control.startAdvErrcode = err_code;
    if (adv_control.bleAdvertiseStatus == NRF_SUCCESS)
    {
        adv_control.bleAdvertiseStatus = 1;
    }

    pf_log_raw(atel_log_ctl.core_en, "ble_adv_start called %d \r", err_code);
}


#if (SUPPORT_BLE_BEACON == 1)
void ble_beacon_start(void)
{
    ret_code_t err_code;
    err_code = sd_ble_gap_adv_start(m_advertising.adv_handle, APP_BLE_CONN_CFG_TAG);
    adv_control.startAdvErrcode = err_code;
    if (adv_control.bleAdvertiseStatus == NRF_SUCCESS)
    {
        adv_control.bleAdvertiseStatus = 1;
    }
    ble_advertisement_status_inform(0);
    // err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    // APP_ERROR_CHECK(err_code);
    pf_log_raw(atel_log_ctl.core_en, "ble_beacon_start called %d \r",err_code);

}

void beacon_advertising_init(uint8_t mode)
{

    uint32_t        err_code;
    ble_advdata_t   advdata;
    uint8_t         flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    ble_advdata_t   srdata;

    ble_advdata_manuf_data_t     bea_manuf_specific_sr_data;
    ble_advdata_manuf_data_t     bea_manuf_specific_data = {0};
    bea_manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

    #if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
    // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
    // UICR instead of using the default values. The major and minor values obtained from the UICR
    // are encoded into advertising data in big endian order (MSB First).
    // To set the UICR used by this example to a desired value, write to the address 0x10001080
    // using the nrfjprog tool. The command to be used is as follows.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
    // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
    // the following command should be used.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
    uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
    uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

    uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;

    m_beacon_info[index++] = MSB_16(major_value);
    m_beacon_info[index++] = LSB_16(major_value);

    m_beacon_info[index++] = MSB_16(minor_value);
    m_beacon_info[index++] = LSB_16(minor_value);
    #endif

    #if APP_ADV_BEANDATA
        bea_manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
        bea_manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;
    #endif /* APP_ADV_BEANDATA */

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.include_appearance    = false;
    #if APP_ADV_BEANDATA
        advdata.p_manuf_specific_data = &bea_manuf_specific_data;
    #else
        advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
        advdata.uuids_complete.p_uuids  = m_adv_uuids;
    #endif /* APP_ADV_BEANDATA */
    
    if (bea_manuf_specific_data.data.size)
    {
        //Avoid compile warning
    }

    // bea_manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    // bea_manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;
    // advdata.p_manuf_specific_data = &bea_manuf_specific_data;

    // memset(&m_adv_params, 0, sizeof(m_adv_params));
    // m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;//BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    // m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    // m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    // m_adv_params.interval        = APP_ADV_INTERVAL;
    // m_adv_params.duration        = APP_ADV_DURATION;       // Never time out. 10ms unit

    memset(&m_advertising.adv_params, 0, sizeof(m_advertising.adv_params));

    if (mode == BEACON_MODE)
    {
        m_advertising.adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;//BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    }
    else if (mode == NORMAL_MODE)
    {
        m_advertising.adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    }
 
    m_advertising.adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_advertising.adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_advertising.adv_params.interval        = beacon_adv_interval;
    m_advertising.adv_params.duration        = beacon_adv_duration;       // Never time out. 10ms unit
    //m_advertising.adv_data.adv_data.len      = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
    m_advertising.adv_data                   = m_adv_data;
    
    //err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    err_code = ble_advdata_encode(&advdata, m_advertising.adv_data.adv_data.p_data, &m_advertising.adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    memset(&srdata, 0, sizeof(srdata));
    //srdata.flags                 = flags;
    //srdata.name_type = BLE_ADVDATA_SHORT_NAME; //BLE_ADVDATA_NO_NAME
    //srdata.short_name_len = 9;
    bea_manuf_specific_sr_data.company_identifier = APP_COMPANY_IDENTIFIER;
    bea_manuf_specific_sr_data.data.p_data = (uint8_t *) m_sn_info;
    bea_manuf_specific_sr_data.data.p_data[MODULE_SN_LENGTH] = tx_1m_rssi; //APP_MEASURED_RSSI
    bea_manuf_specific_sr_data.data.size   = MODULE_SN_LENGTH+1;
    srdata.p_manuf_specific_data = &bea_manuf_specific_sr_data;


    // srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    // srdata.uuids_complete.p_uuids  = m_adv_uuids;

    #if APP_ADV_BEANDATA
        srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
        srdata.uuids_complete.p_uuids  = m_adv_uuids;
    #endif /* APP_ADV_BEANDATA */

 
    m_advertising.adv_modes_config.ble_adv_on_disconnect_disabled = true;
    m_advertising.adv_modes_config.ble_adv_whitelist_enabled = true;
    m_advertising.adv_modes_config.ble_adv_fast_enabled = true;
    m_advertising.adv_modes_config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    m_advertising.adv_modes_config.ble_adv_fast_timeout = APP_ADV_DURATION;



    //err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    err_code = ble_advdata_encode(&srdata, m_advertising.adv_data.scan_rsp_data.p_data, &m_advertising.adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    m_advertising.adv_handle                 = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
    //err_code = sd_ble_gap_adv_set_configure(&m_advertising.adv_handle, &m_adv_data, &m_adv_params);
    err_code = sd_ble_gap_adv_set_configure(&m_advertising.adv_handle, &m_advertising.adv_data, &m_advertising.adv_params);
    APP_ERROR_CHECK(err_code);

}

void beacon_advertising_update(uint32_t interval, uint16_t duration, uint8_t update, uint8_t mode)
{
    uint32_t err_code;
    //ble_advertising_init_t init;
    ble_advdata_t srdata;
    ble_advdata_manuf_data_t manuf_specific_sr_data;
    // ble_advdata_uuid_list_t* p_uuids_complete = NULL;
    // ble_adv_modes_config_t*   p_config = NULL;
    // p_config = &(init.config);
    if (update)
    {
        //Initialize advertising parameters(used when starting advertising)
        // memset(&(*p_config), 0, sizeof(*p_config));
 
        // p_config->ble_adv_fast_interval = interval;
        // p_config->ble_adv_fast_timeout = duration;
        // p_config->ble_adv_on_disconnect_disabled = true;
        // p_config->ble_adv_whitelist_enabled = true;
        // p_config->ble_adv_fast_enabled = true;
        // NRF_LOG_RAW_INFO("Update the info,interval %d, duration %d \r,",init.config.ble_adv_fast_interval,init.config.ble_adv_fast_timeout);
        // err_code = ble_advertising_init(&m_advertising, &init);
        // APP_ERROR_CHECK(err_code);

        // ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

        memset(&m_advertising.adv_params, 0 , sizeof(m_advertising.adv_params));
        //m_advertising.adv_params.properties.type =  BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;
        m_advertising.adv_params.p_peer_addr = NULL;  //Undirected advertisement.
        m_advertising.adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
        m_advertising.adv_params.interval = interval;
        m_advertising.adv_params.duration = duration;
        if (mode == BEACON_MODE)
        {
            m_advertising.adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;//BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
        }
        else if (mode == NORMAL_MODE)
        {
            m_advertising.adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
        }


        err_code = sd_ble_gap_adv_set_configure(&m_advertising.adv_handle, &m_advertising.adv_data, &m_advertising.adv_params);
        NRF_LOG_RAW_INFO("Update the info,interval %d, duration %d, Err %d \r,",m_advertising.adv_params.interval,m_advertising.adv_params.duration,err_code);

        //APP_ERROR_CHECK(err_code);
    }
    else
    {
        //Build and set scan response data
        // memset(&p_m_manuf_specific_data, 0, sizeof(p_m_manuf_specific_data));
        // memset(&(*p_uuids_complete), 0, sizeof(*p_uuids_complete));
        // p_uuids_complete = &(init.srdata.uuids_complete);  // Keep sync in sima
        // p_uuids_complete->uuid_cnt = 0;
        // p_uuids_complete->p_uuids = 0;
        // p_m_manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
        // p_m_manuf_specific_data.data.p_data = (uint8_t *)m_sn_info;
        // p_m_manuf_specific_data.data.p_data[MODULE_SN_LENGTH] = tx_1m_rssi;  // APP_MEASURED_RSSI
        // p_m_manuf_specific_data.data.size = MODULE_SN_LENGTH + 1;
        // init.srdata.p_manuf_specific_data = (ble_advdata_manuf_data_t *)&p_m_manuf_specific_data;
       
        // err_code = ble_advertising_init(&m_advertising, &init);
        // APP_ERROR_CHECK(err_code);

        // ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

        memset(&srdata, 0, sizeof(srdata));
    
        //srdata.flags                 = flags;
        //srdata.name_type = BLE_ADVDATA_NO_NAME;
        //srdata.short_name_len = 9;
        manuf_specific_sr_data.company_identifier = APP_COMPANY_IDENTIFIER;
        manuf_specific_sr_data.data.p_data = (uint8_t *) m_sn_info;
        manuf_specific_sr_data.data.p_data[MODULE_SN_LENGTH] = tx_1m_rssi; //APP_MEASURED_RSSI
        manuf_specific_sr_data.data.size   = MODULE_SN_LENGTH+1;
        srdata.p_manuf_specific_data = &manuf_specific_sr_data;
    
        //srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
        //srdata.uuids_complete.p_uuids  = m_adv_uuids;
        
        err_code = ble_advdata_encode(&srdata, m_advertising.adv_data.scan_rsp_data.p_data, &m_advertising.adv_data.scan_rsp_data.len);
        //APP_ERROR_CHECK(err_code);

    }
}

#endif /*(SUPPORT_BLE_BEACON == 1)*/

/**@brief Function for initializing the Advertising functionality.
 */

void normal_advertising_init(bool adv_on_disconnect, uint32_t adv_interval)
{
    uint32_t               err_code;
    // ble_advertising_init_t init;                 
    // int8_t tx_power_level = tx_power_level_default;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_NO_NAME;
    init.advdata.include_appearance = false;
    // init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    // init.advdata.p_tx_power_level   = &tx_power_level;


    // init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    // init.advdata.uuids_complete.p_uuids  = m_adv_uuids;          //072022

    m_manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    m_manuf_specific_data.data.size = APP_BEACON_INFO_LENGTH;
    m_manuf_specific_data.data.p_data = (uint8_t *)m_beacon_info;

    init.advdata.p_manuf_specific_data = (ble_advdata_manuf_data_t*)&m_manuf_specific_data;


   // init.srdata.p_manuf_specific_data = (ble_advdata_manuf_data_t *)&m_manuf_specific_data;
    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.size = MODULE_SN_LENGTH + 1;
    manuf_specific_data.data.p_data = (uint8_t *)m_sn_info;
    manuf_specific_data.data.p_data[MODULE_SN_LENGTH] = tx_1m_rssi; //APP_MEASURED_RSSI
    init.srdata.p_manuf_specific_data =(ble_advdata_manuf_data_t*)&manuf_specific_data;
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_on_disconnect_disabled = adv_on_disconnect;
    init.config.ble_adv_whitelist_enabled = true;
    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = adv_interval;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;


    //init.config.ble_adv_fast_timeout  = 0;
    // init.config.ble_adv_primary_phy   = BLE_GAP_PHY_1MBPS; // Must be changed to connect in long range. (BLE_GAP_PHY_CODED)
    // init.config.ble_adv_secondary_phy = BLE_GAP_PHY_1MBPS;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


void normal_advertising_data_update(uint32_t interval, uint16_t duration, uint8_t update)
{
    uint32_t err_code;
    //ble_advertising_init_t init;
    //ble_advdata_t srdata;
    //ble_advdata_manuf_data_t manuf_specific_sr_data;
    ble_advdata_uuid_list_t* p_uuids_complete = NULL;
    ble_adv_modes_config_t*   p_config = NULL;
    p_config = &(init.config);
    if (update)
    {
        //Initialize advertising parameters(used when starting advertising)
        memset(&(*p_config), 0, sizeof(ble_adv_modes_config_t));
 
        p_config->ble_adv_fast_interval = interval;
        p_config->ble_adv_fast_timeout = duration;
        p_config->ble_adv_on_disconnect_disabled = true;
        p_config->ble_adv_whitelist_enabled = true;
        p_config->ble_adv_fast_enabled = true;
        NRF_LOG_RAW_INFO("Update the info,interval %d, duration %d \r,",init.config.ble_adv_fast_interval,init.config.ble_adv_fast_timeout);
        err_code = ble_advertising_init(&m_advertising, &init);
        APP_ERROR_CHECK(err_code);

        ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

        // memset(&m_advertising.adv_params, 0 , sizeof(m_advertising.adv_params));
        // m_advertising.adv_params.properties.type =  BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;
        // m_advertising.adv_params.p_peer_addr = NULL;  //Undirected advertisement.
        // m_advertising.adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
        // m_advertising.adv_params.interval = interval;
        // m_advertising.adv_params.duration = duration;
        
        // err_code = sd_ble_gap_adv_set_configure(&m_advertising.adv_handle, &m_advertising.adv_data, &m_advertising.adv_params);

        APP_ERROR_CHECK(err_code);
    }
    else
    {
        //Build and set scan response data
        p_uuids_complete = &(init.srdata.uuids_complete);  // Keep sync in sima
        memset(&p_m_manuf_specific_data, 0, sizeof(p_m_manuf_specific_data));
        //memset(&(*p_uuids_complete), 0, sizeof(*p_uuids_complete));
        memset(&(*p_uuids_complete), 0, sizeof(ble_advdata_uuid_list_t));
        p_uuids_complete->uuid_cnt = 0;
        p_uuids_complete->p_uuids = 0;
        p_m_manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
        p_m_manuf_specific_data.data.p_data = (uint8_t *)m_sn_info;
        p_m_manuf_specific_data.data.p_data[MODULE_SN_LENGTH] = tx_1m_rssi;  // APP_MEASURED_RSSI
        p_m_manuf_specific_data.data.size = MODULE_SN_LENGTH + 1;
        init.srdata.p_manuf_specific_data = (ble_advdata_manuf_data_t *)&p_m_manuf_specific_data;
       
        err_code = ble_advertising_init(&m_advertising, &init);
        APP_ERROR_CHECK(err_code);

        ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

        // memset(&srdata, 0, sizeof(srdata));
    
        // //srdata.flags                 = flags;
        // //srdata.name_type = BLE_ADVDATA_NO_NAME;
        // //srdata.short_name_len = 9;
        // manuf_specific_sr_data.company_identifier = APP_COMPANY_IDENTIFIER;
        // manuf_specific_sr_data.data.p_data = (uint8_t *) m_sn_info;
        // manuf_specific_sr_data.data.p_data[MODULE_SN_LENGTH] = tx_1m_rssi; //APP_MEASURED_RSSI
        // manuf_specific_sr_data.data.size   = MODULE_SN_LENGTH+1;
        // srdata.p_manuf_specific_data = &manuf_specific_sr_data;
    
        // //srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
        // //srdata.uuids_complete.p_uuids  = m_adv_uuids;
        
        // err_code = ble_advdata_encode(&srdata, m_advertising.adv_data.scan_rsp_data.p_data, &m_advertising.adv_data.scan_rsp_data.len);
        // APP_ERROR_CHECK(err_code);

    }
}

#if (SUPPORT_BLE_BEACON == 1)
/**@brief Function for supporting centeral and peripheral work together.
 */
#define BEACON_MODE   0
#define NORMAL_MODE   1

void pf_adv_start(uint8_t adv_mode)
{
    // ret_code_t err_code;
    // // if (!nrf_fstorage_is_busy(NULL))
    // // {
    // //     scan_start(); // start scan
    // // }
    // err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    // err_code = adv_control.startAdvErrcode;
    // adv_control.bleAdvertiseStatus = 1;
    // pf_log_raw(atel_log_ctl.core_en, "pf_adv_start called %d \r", err_code);
    if (adv_mode == BEACON_MODE)
    {
        ble_beacon_start();
    }
    else if (adv_mode == NORMAL_MODE)
    ble_adv_start();

    //APP_ERROR_CHECK(err_code);  
}

void ble_aus_advertising_stop(void)
{
#if (SUPPORT_BLE_CENTRAL_AND_PERIPH == 1)

    uint32_t err_code = 0;
    // if (mode == BEACON_MODE)
    // {
    //     err_code = sd_ble_gap_adv_stop(m_adv_handle);
    // }
    // else if (mode == NORMAL_MODE)
    {
        err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
    }

    APP_ERROR_CHECK(err_code);

    adv_control.bleAdvertiseStatus = 0;

    pf_log_raw(atel_log_ctl.core_en, "ble_aus_advertising_stop called %d \r", err_code);

#else

    uint32_t err_code = 0;
    uint16_t i = 0, handler = 0xffff;

    for (i = 0; i < BLE_CHANNEL_NUM_MAX; i++)
    {
        handler = ble_connected_handler_get_from_channel(i);
        if (handler != 0xffff)
        {
            err_code = sd_ble_gap_disconnect(handler, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            pf_log_raw(atel_log_ctl.core_en, "ble_aus_advertising_stop CH(%d) Handler(%d) Err(%d).\r", i, handler, err_code);
        }
    }

    // TODO: optimize advertising stop
    // if (m_advertising.adv_mode_current != BLE_ADV_MODE_IDLE)
    // {
    //     sd_ble_gap_adv_stop(m_advertising.adv_handle);
    // }

    pf_log_raw(atel_log_ctl.core_en, "ble_aus_advertising_stop called\r");
    ble_connected_channel_clear();

    // TODO: disconnect all
    monet_data.bleDisconnectEvent = 1;

#endif /* (SUPPORT_BLE_CENTRAL_AND_PERIPH == 1) */
}

uint32_t pf_tx_power_set(int8_t tx_level)
{
    uint32_t err_code;

    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, tx_level);
    APP_ERROR_CHECK(err_code);

    return err_code;
}
#endif /*(SUPPORT_BLE_BEACON == 1)*/

static void on_ble_peripheral_evt(ble_evt_t const* p_ble_evt)
{
    uint32_t err_code;
//    uint16_t channel = 0xffff;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            adv_control.bleAdvertiseStatus = 0;
            NRF_LOG_RAW_INFO("on_ble_peripheral_evt connected \r");
            ble_advertisement_status_inform(0);
            // err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            // APP_ERROR_CHECK(err_code);
            // CRITICAL_REGION_ENTER();
            // m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            // CRITICAL_REGION_EXIT();
            // #if (BLE_FUNCTION_ONOFF == BLE_FUNCTION_ON)
            // channel = ble_information_set(p_ble_evt->evt.gap_evt.conn_handle,
            //                     BLE_CONNECTION_STATUS_CONNECTED,
            //                     (uint8_t *)(p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr));
            // NRF_LOG_RAW_INFO("Connected CH: %d.\r", channel);
            // monet_data.is_advertising = 0;
            // #endif /* BLE_FUNCTION_ONOFF */
            // err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            // APP_ERROR_CHECK(err_code);
            // err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, tx_power_level_default);
            // APP_ERROR_CHECK(err_code);
            // {
            //     NRF_LOG_RAW_INFO("PHY update connected.\r");
            //     ble_gap_phys_t const phys =
            //     {
            //         .rx_phys = BLE_GAP_PHY_2MBPS,
            //         .tx_phys = BLE_GAP_PHY_2MBPS,
            //     };
            //     err_code = sd_ble_gap_phy_update(m_conn_handle, &phys);
            //     APP_ERROR_CHECK(err_code);
            // }
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            // #if (BLE_FUNCTION_ONOFF == BLE_FUNCTION_ON)
            // channel = ble_channel_get_from_handler(p_ble_evt->evt.gap_evt.conn_handle);
            // NRF_LOG_RAW_INFO("Disconnected CH: %d\r", channel);
            // monet_data.is_advertising = 0;
            // monet_data.to_advertising = 1;
            // #endif /* BLE_FUNCTION_ONOFF */
            // // LED indication will be changed when advertising starts.
            // CRITICAL_REGION_ENTER();
            // m_conn_handle = BLE_CONN_HANDLE_INVALID;

            // #if (BLE_FUNCTION_ONOFF == BLE_FUNCTION_ON)
            // if (channel != 0xffff)
            // {
            //     monet_data.bleDisconnectEvent = 1 + channel;
            //     monet_data.ble_info[channel].connect_status = BLE_CONNECTION_STATUS_NOT_CONNECTED;
            //     monet_data.ble_info[channel].handler = 0xffff;
            // }
            // #endif /* BLE_FUNCTION_ONOFF */
            // CRITICAL_REGION_EXIT();
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
            // #if (BLE_FUNCTION_ONOFF == BLE_FUNCTION_ON)
            // channel = ble_channel_get_from_handler(p_ble_evt->evt.gap_evt.conn_handle);
            // NRF_LOG_RAW_INFO(">>>EVT_CONN_PARAM_UPDATE CH(%d) Min(%d) Max(%d) Late(%d) Timeout(%d).\r",
            //              channel,
            //              p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval,
            //              p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval,
            //              p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.slave_latency,
            //              p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.conn_sup_timeout);
            // #endif /* BLE_FUNCTION_ONOFF */
            break;
        
        case BLE_GAP_EVT_PHY_UPDATE:
        // {
        //     ble_gap_evt_phy_update_t const * p_phy_evt = &p_ble_evt->evt.gap_evt.params.phy_update;

        //     if (p_phy_evt->status == BLE_HCI_STATUS_CODE_LMP_ERROR_TRANSACTION_COLLISION)
        //     {
        //         // Ignore LL collisions.
        //         NRF_LOG_RAW_INFO("LL transaction collision during PHY update.\r");
        //         break;
        //     }

        //     ble_gap_phys_t phys = {0};
        //     phys.tx_phys = p_phy_evt->tx_phy;
        //     phys.rx_phys = p_phy_evt->rx_phy;
        //     NRF_LOG_RAW_INFO("PHY update %s. PHY set to %s.\r",
        //                      (p_phy_evt->status == BLE_HCI_STATUS_CODE_SUCCESS) ?
        //                      "accepted" : "rejected",
        //                      phy_str(phys));
        // }
        break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            // NRF_LOG_RAW_INFO("PHY update request.\r");
            // ble_gap_phys_t const phys =
            // {
            //     .rx_phys = BLE_GAP_PHY_AUTO,
            //     .tx_phys = BLE_GAP_PHY_AUTO,
            // };
            // err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            // APP_ERROR_CHECK(err_code);
        } break;

        // case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        //     // Pairing not supported
        //     err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
        //     APP_ERROR_CHECK(err_code);
        //     break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            // err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            // APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_RAW_INFO("GATT Client Timeout.\r");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_RAW_INFO("GATT Server Timeout.\r");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_RAW_INFO("BLE_GAP_EVT_SEC_PARAMS_REQUEST\r");
            break;

        // case BLE_GAP_EVT_PASSKEY_DISPLAY:
        // {
        //     char passkey[BLE_GAP_PASSKEY_LEN + 1];
        //     memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, BLE_GAP_PASSKEY_LEN);
        //     passkey[BLE_GAP_PASSKEY_LEN] = 0x00;
        //     NRF_LOG_INFO("=== PASSKEY: %s =====", nrf_log_push(passkey));
        // }
        // break;

        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
            //  NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
            //               p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
            //               p_ble_evt->evt.gap_evt.params.auth_status.bonded,
            //               p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
            //               *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
            //               *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            // // WARNING: If pair fail will it still advertise?
            // monet_data.in_pairing_mode = (p_ble_evt->evt.gap_evt.params.auth_status.bonded + 1) % 2;
            // monet_data.pairing_success = p_ble_evt->evt.gap_evt.params.auth_status.bonded;
            // if (monet_data.pairing_success)
            // {
            //     CRITICAL_REGION_ENTER();
            //     reset_info.ble_peripheral_paired = 1;
            //     CRITICAL_REGION_EXIT();
            // }
            break;
        case BLE_GAP_EVT_ADV_SET_TERMINATED:
            if (p_ble_evt->evt.gap_evt.params.adv_set_terminated.reason == BLE_GAP_EVT_ADV_SET_TERMINATED_REASON_TIMEOUT)
            {
                NRF_LOG_INFO("on_ble_peripheral_evt:Advertise timeout \r");
                ble_advertisement_status_inform(0);
                adv_control.bleAdvertiseStatus = 0;    // in nala, it can't reach.
            }
            break;

        default:
            // No implementation needed.
            break;
    }

}

#endif /* (SUPPORT_BLE_CENTRAL_AND_PERIPH == 1) */


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
//     ret_code_t            err_code;
//     ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

//     switch (p_ble_evt->header.evt_id)
//     {
//         case BLE_GAP_EVT_CONNECTED:
//         NRF_LOG_RAW_INFO("ble_evt_handler BLE_GAP_EVT_CONNECTED\r");
//         NRF_LOG_FLUSH();
// //            err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
// //            APP_ERROR_CHECK(err_code);

//             // err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
//             // APP_ERROR_CHECK(err_code);

//             // start discovery of services. The NUS Client waits for a discovery result
//             err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
//             APP_ERROR_CHECK(err_code);
//             monet_data.ble_conn_param.min_100us = NRF_BLE_SCAN_MIN_CONNECTION_INTERVAL * 10;
//             monet_data.ble_conn_param.max_100us = NRF_BLE_SCAN_MAX_CONNECTION_INTERVAL * 10;
//             monet_data.ble_conn_param.latency = NRF_BLE_SCAN_SLAVE_LATENCY;
//             monet_data.ble_conn_param.timeout_100us = NRF_BLE_SCAN_SUPERVISION_TIMEOUT * 10;
//             break;

//         case BLE_GAP_EVT_DISCONNECTED:
//             ble_aus_ready_c = false;
//             monet_data.bleDisconnectEvent = 1;
// 			monet_data.bleConnectionStatus = 0;

//             // Only APP have the right to change ble_link_target.action value
//             if (ble_link_target.action == 1)
//             {
//                 ble_link_target.disconnected = 1;
//             }

//             monet_data.ble_peer_mac_addr[0] = 0;
//             monet_data.ble_peer_mac_addr[1] = 0;
//             monet_data.ble_peer_mac_addr[2] = 0;
//             monet_data.ble_peer_mac_addr[3] = 0;
//             monet_data.ble_peer_mac_addr[4] = 0;
//             monet_data.ble_peer_mac_addr[5] = 0;
            
// 			m_conn_handle = 0xffff;
		
//             NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
//                          p_gap_evt->conn_handle,
//                          p_gap_evt->params.disconnected.reason);

//             if (is_in_pair_mode() == true && ble_state_get() == BLE_STATE_PAIRING)
// 			{
// 				uint32_t sensor_mask = scan_list_mask_get();
// 				const scan_list_t *p_element = NULL;
				
// 				// The event message here does not contain address info, so it is needed to be given manually.
// 				p_element = scan_list_elem_get(pairing_device_index_get());
// 				scan_stop();
// 				leds_blink(0, 0);
// 				ble_dg_printf(BLE_DG_LOG_HEADER "disconnected with address (LSB) %02X%02X%02X%02X%02X%02X\r\n", 
// 											p_element->adv_report.peer_addr.addr[0], 
// 											p_element->adv_report.peer_addr.addr[1], 
// 											p_element->adv_report.peer_addr.addr[2], 
// 											p_element->adv_report.peer_addr.addr[3], 
// 											p_element->adv_report.peer_addr.addr[4], 
// 											p_element->adv_report.peer_addr.addr[5] );
// 				NRF_LOG_RAW_INFO("ble_evt_handler() BLE_GAP_EVT_DISCONNECTED event\r");
// 				NRF_LOG_RAW_INFO("ble_evt_handler() BLE_GAP_EVT_DISCONNECTED failed to connect with address (LSB): ");
// 				NRF_LOG_RAW_HEXDUMP_INFO(p_element->adv_report.peer_addr.addr, BLE_GAP_ADDR_LEN);
// 				ble_state_set(BLE_STATE_TO_PAIR);
// 			}
//             break;

//         case BLE_GAP_EVT_TIMEOUT:
//             if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
//             {
//                 NRF_LOG_INFO("Connection Request timed out.");
//             }
//             break;
		
// //        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
// //            // Pairing not supported.
// //            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
// //            APP_ERROR_CHECK(err_code);
// //            break;

//         case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
//             // Accepting parameters requested by peer.
//             NRF_LOG_INFO("Connection Param update Min(%d) Max(%d) Late(%d) Time(%d).",
//                           p_gap_evt->params.conn_param_update_request.conn_params.min_conn_interval,
//                           p_gap_evt->params.conn_param_update_request.conn_params.max_conn_interval,
//                           p_gap_evt->params.conn_param_update_request.conn_params.slave_latency,
//                           p_gap_evt->params.conn_param_update_request.conn_params.conn_sup_timeout);
//             monet_data.ble_conn_param.min_100us = p_gap_evt->params.conn_param_update_request.conn_params.min_conn_interval * 100 / 8;
//             monet_data.ble_conn_param.max_100us = p_gap_evt->params.conn_param_update_request.conn_params.max_conn_interval * 100 / 8;
//             monet_data.ble_conn_param.latency = p_gap_evt->params.conn_param_update_request.conn_params.slave_latency;
//             monet_data.ble_conn_param.timeout_100us = p_gap_evt->params.conn_param_update_request.conn_params.conn_sup_timeout * 100;
//             err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
//                                                     &p_gap_evt->params.conn_param_update_request.conn_params);
//             APP_ERROR_CHECK(err_code);
//             break;

//         case BLE_GAP_EVT_PHY_UPDATE:
//         {
//             ble_gap_evt_phy_update_t const * p_phy_evt = &p_ble_evt->evt.gap_evt.params.phy_update;

//             if (p_phy_evt->status == BLE_HCI_STATUS_CODE_LMP_ERROR_TRANSACTION_COLLISION)
//             {
//                 // Ignore LL collisions.
//                 NRF_LOG_RAW_INFO("LL transaction collision during PHY update.\r");
//                 break;
//             }

//             ble_gap_phys_t phys = {0};
//             phys.tx_phys = p_phy_evt->tx_phy;
//             phys.rx_phys = p_phy_evt->rx_phy;
//             NRF_LOG_RAW_INFO("PHY update %s. PHY set to %s.\r",
//                              (p_phy_evt->status == BLE_HCI_STATUS_CODE_SUCCESS) ?
//                              "accepted" : "rejected",
//                              phy_str(phys));
//         }
//         break;

//         case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
//         {
//             NRF_LOG_RAW_INFO("PHY update request.\r");
//             NRF_LOG_FLUSH();
//             ble_gap_phys_t const phys =
//             {
//                 .rx_phys = BLE_GAP_PHY_AUTO,
//                 .tx_phys = BLE_GAP_PHY_AUTO,
//             };
//             err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
//             APP_ERROR_CHECK(err_code);
//         } break;

//         case BLE_GATTC_EVT_TIMEOUT:
//             // Disconnect on GATT Client timeout event.
//             NRF_LOG_RAW_INFO("GATT Client Timeout.\r");
//             err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
//                                              BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//             APP_ERROR_CHECK(err_code);
//             break;

//         case BLE_GATTS_EVT_TIMEOUT:
//             // Disconnect on GATT Server timeout event.
//             NRF_LOG_RAW_INFO("GATT Server Timeout.\r");
//             err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
//                                              BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//             APP_ERROR_CHECK(err_code);
//             break;
		
//         case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
//             NRF_LOG_RAW_INFO("BLE_GAP_EVT_SEC_PARAMS_REQUEST\r");
//             break;

//         case BLE_GAP_EVT_AUTH_KEY_REQUEST:
//             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
//             break;

//         case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
//             NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
//             break;
		
// 		case BLE_GAP_EVT_ADV_REPORT:
// 		{
// 			const ble_gap_evt_adv_report_t *p_adv_report = &p_ble_evt->evt.gap_evt.params.adv_report;
// 			if (is_in_ble_scan_mode() == true)
// 			{
// 				if (p_adv_report->type.scan_response == 1 		// Current report is scan_response
// 					&& p_adv_report->data.len > 5				// It should contain at least 5 bytes (Len(1) 0xFF cmpny_id(2) sensor_mask(1)). Length of Tested Manufacture Specific Data (MSD) is about 27
// 					&& p_adv_report->data.p_data[2] == (APP_COMPANY_ID & 0xff) 			// MSD[2-3] is App Company Idendifier.
// 					&& p_adv_report->data.p_data[3] == ((APP_COMPANY_ID >> 8) & 0xff) )
// 				{
// 					scan_list_scn_resp_add(&p_adv_report->peer_addr, &p_adv_report->data);
// 				}
// 			}
// 		} break;
		
// 		case BLE_GAP_EVT_AUTH_STATUS:
//         {
// 			NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
//                           p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
//                           p_ble_evt->evt.gap_evt.params.auth_status.bonded,
//                           p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
//                           *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
//                           *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
// 			if (is_in_pair_mode() == true && ble_state_get() == BLE_STATE_PAIRING)
// 			{
// 				if (p_ble_evt->evt.gap_evt.params.auth_status.bonded > 0)	// Device is bonded
// 				{
// 					// Warning: Move to device_ble_status_report()
// 					// pair_resp_to_mdm_send(1, 0, (uint8_t *)&sensor_mask, sizeof(sensor_mask), &p_element->adv_report.peer_addr);
//                     monet_data.ble_pairok_notreported = 1;
//                     NRF_LOG_RAW_INFO("ble_evt_handler() BLE_GAP_EVT_AUTH_STATUS paired ok\r");
// 				}
// 				else	// Failed to bond with the device
// 				{
//                     uint32_t sensor_mask = scan_list_mask_get();
//                     const scan_list_t *p_element = NULL;

//                     // The event message here does not contain address info, so it is needed to be given manually.
//                     p_element = scan_list_elem_get(pairing_device_index_get());

//                     scan_stop();
// 					leds_blink(0, 0);
// 					ble_dg_printf(BLE_DG_LOG_HEADER "failed to pair with address (LSB) %02X%02X%02X%02X%02X%02X\r\n", 
// 												p_element->adv_report.peer_addr.addr[0], 
// 												p_element->adv_report.peer_addr.addr[1], 
// 												p_element->adv_report.peer_addr.addr[2], 
// 												p_element->adv_report.peer_addr.addr[3], 
// 												p_element->adv_report.peer_addr.addr[4], 
// 												p_element->adv_report.peer_addr.addr[5] );
// 					NRF_LOG_RAW_INFO("ble_evt_handler() BLE_GAP_EVT_AUTH_STATUS failed to pair with address (LSB): ");
// 					NRF_LOG_RAW_HEXDUMP_INFO(p_element->adv_report.peer_addr.addr, BLE_GAP_ADDR_LEN);
// 					ble_state_set(BLE_STATE_TO_PAIR);
// 				}
// 			}
// 		}
//             break;
		
//         default:
//             break;
//     }
    uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    uint16_t role = ble_conn_state_role(conn_handle);
    if ((role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT))
    {
        on_ble_central_evt(p_ble_evt);
    }

#if (SUPPORT_BLE_CENTRAL_AND_PERIPH == 1)

    if ((role == BLE_GAP_ROLE_PERIPH) || (ble_evt_is_advertising_timeout(p_ble_evt)))
    {
        on_ble_peripheral_evt(p_ble_evt);
    }

#endif /* (SUPPORT_BLE_CENTRAL_AND_PERIPH == 1) */    
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    static uint8_t fail = 0;
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        NRF_LOG_RAW_INFO("pm_evt_handler(), PM_EVT_PEERS_DELETE_SUCCEEDED.\r");
        {
            fail = 0;
            // Bonds are deleted. Start scanning.
            // scan_start();
        }
            break;

		case PM_EVT_BONDED_PEER_CONNECTED:
		{
			NRF_LOG_RAW_INFO("pm_evt_handler(), PM_EVT_BONDED_PEER_CONNECTED, peer id %x\r", p_evt->peer_id);
		} break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            fail = 0;

            if (ble_link_target.ready == 1)
            {
                monet_data.bleConnectionEvent = 1;
                monet_data.bleConnectionEventDelay = 10;
                monet_data.bleConnectionStatus = 1;

                if (ble_link_target.action == 1)
                {
                    ble_link_target.noreported = 1;
                    ble_link_target.result = 1;
                }

                if (ble_link_target.report_delay == 1)
                {
                    ble_link_target.report_delay = 0;
                    ble_link_target.result = 1;
                }

                ble_aus_ready_c = true;
            }
            NRF_LOG_RAW_INFO("pm_evt_handler: PM_EVT_CONN_SEC_SUCCEEDED Ready(%d).\r", ble_link_target.ready);
        }
        break;

        case PM_EVT_CONN_SEC_FAILED:
        //     NRF_LOG_RAW_INFO("pm_evt_handler: PM_EVT_CONN_SEC_FAILED(%d)\r", 
        //                      pm_conn_secure(p_evt->conn_handle, true));
            fail++;
            if (fail >= 1)
            {
               // pm_peers_delete();
            }
            else
            {
                if (ble_link_target.conn_handle_valid == 1)
                {
                    pm_conn_secure(ble_link_target.conn_handle, false);
                }
            }
            NRF_LOG_RAW_INFO("pm_evt_handler: PM_EVT_CONN_SEC_FAILED Fail(%d)\r", fail);

            if (ble_link_target.action == 1)
            {
                ble_link_target.noreported = 1;
                ble_link_target.result = 0;
                ble_link_target.report_delay = 1;  //yj logic is right,but app does not comply the protool.here will look like is a bug. //wfyadd
                ble_link_target.report_intime = 1; //we shouldn't change the old logic.In generally app should change.
                monet_data.bleConnectionEvent = 0;
            }
            break;
        
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
            NRF_LOG_RAW_INFO("pm_evt_handler: PM_EVT_PEER_DATA_UPDATE_SUCCEEDED.\r");
            break;
		
		case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            pm_conn_sec_config_t cfg;
            cfg.allow_repairing = true;
            pm_conn_sec_config_reply(p_evt->conn_handle, &cfg);
        }
            break;
		
        default:
            break;
    }
	NRF_LOG_RAW_INFO("pm_evt_handler() EVT_ID %u\r", p_evt->evt_id);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for the Peer Manager initialization.
 */
static void erase_fds_areas(void);
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();

    if (err_code != NRF_SUCCESS)
    {
        erase_fds_areas();
    }

    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
//    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
//    {
//        NRF_LOG_INFO("ATT MTU exchange completed.");

//        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
//        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
//    }
	
	switch (p_evt->evt_id)
    {
        case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
        {
            NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                         p_evt->conn_handle,
                         p_evt->params.att_mtu_effective);
        } break;

        case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED:
        {
            NRF_LOG_INFO("Data length for connection 0x%x updated to %d.",
                         p_evt->conn_handle,
                         p_evt->params.data_length);
        } break;

        default:
            break;
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

//    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
//    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
// void bsp_event_handler(bsp_event_t event)
// {
//     ret_code_t err_code;

//     switch (event)
//     {
//         case BSP_EVENT_SLEEP:
//             nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
//             break;

//         case BSP_EVENT_DISCONNECT:
//             err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle,
//                                              BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//             if (err_code != NRF_ERROR_INVALID_STATE)
//             {
//                 APP_ERROR_CHECK(err_code);
//             }
//             break;

//         default:
//             break;
//     }
// }

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler   = ble_nus_c_evt_handler;
    init.error_handler = nus_error_handler;
    init.p_gatt_queue  = &m_ble_gatt_queue;

    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
    APP_ERROR_CHECK(err_code);
}

// Get the NUS service handle
const ble_nus_c_t *ble_nus_handle_get(void)
{
	return &m_ble_nus_c;
}

/**@brief Function for initializing buttons and leds. */
// static void buttons_leds_init(void)
// {
//     ret_code_t err_code;
//     bsp_event_t startup_event;

//     err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
//     APP_ERROR_CHECK(err_code);

//     err_code = bsp_btn_ble_init(NULL, &startup_event);
//     APP_ERROR_CHECK(err_code);
// }



static void on_ble_central_evt(ble_evt_t const* p_ble_evt)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_RAW_INFO("on_ble_central_evt BLE_GAP_EVT_CONNECTED\r");
        NRF_LOG_FLUSH();
//            err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
//            APP_ERROR_CHECK(err_code);

            // err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            // APP_ERROR_CHECK(err_code);

            // start discovery of services. The NUS Client waits for a discovery result
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
            monet_data.ble_conn_param.min_100us = NRF_BLE_SCAN_MIN_CONNECTION_INTERVAL * 10;
            monet_data.ble_conn_param.max_100us = NRF_BLE_SCAN_MAX_CONNECTION_INTERVAL * 10;
            monet_data.ble_conn_param.latency = NRF_BLE_SCAN_SLAVE_LATENCY;
            monet_data.ble_conn_param.timeout_100us = NRF_BLE_SCAN_SUPERVISION_TIMEOUT * 10;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            ble_aus_ready_c = false;
            monet_data.bleDisconnectEvent = 1;
			monet_data.bleConnectionStatus = 0;

            // Only APP have the right to change ble_link_target.action value
            if (ble_link_target.action == 1)
            {
                ble_link_target.disconnected = 1;
            }

            monet_data.ble_peer_mac_addr[0] = 0;
            monet_data.ble_peer_mac_addr[1] = 0;
            monet_data.ble_peer_mac_addr[2] = 0;
            monet_data.ble_peer_mac_addr[3] = 0;
            monet_data.ble_peer_mac_addr[4] = 0;
            monet_data.ble_peer_mac_addr[5] = 0;
            
			m_conn_handle = 0xffff;
		
            NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);

            if (is_in_pair_mode() == true && ble_state_get() == BLE_STATE_PAIRING)
			{
				uint32_t sensor_mask = scan_list_mask_get();
				const scan_list_t *p_element = NULL;
				
				// The event message here does not contain address info, so it is needed to be given manually.
				p_element = scan_list_elem_get(pairing_device_index_get());
				scan_stop();
				leds_blink(0, 0);
				ble_dg_printf(BLE_DG_LOG_HEADER "disconnected with address (LSB) %02X%02X%02X%02X%02X%02X\r\n", 
											p_element->adv_report.peer_addr.addr[0], 
											p_element->adv_report.peer_addr.addr[1], 
											p_element->adv_report.peer_addr.addr[2], 
											p_element->adv_report.peer_addr.addr[3], 
											p_element->adv_report.peer_addr.addr[4], 
											p_element->adv_report.peer_addr.addr[5] );
				NRF_LOG_RAW_INFO("ble_evt_handler() BLE_GAP_EVT_DISCONNECTED event ssr_mask %d\r", sensor_mask);
				NRF_LOG_RAW_INFO("ble_evt_handler() BLE_GAP_EVT_DISCONNECTED failed to connect with address (LSB): ");
				NRF_LOG_RAW_HEXDUMP_INFO(p_element->adv_report.peer_addr.addr, BLE_GAP_ADDR_LEN);
				ble_state_set(BLE_STATE_TO_PAIR);
			}
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;
		
//        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
//            // Pairing not supported.
//            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
//            APP_ERROR_CHECK(err_code);
//            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            NRF_LOG_INFO("Connection Param update Min(%d) Max(%d) Late(%d) Time(%d).",
                          p_gap_evt->params.conn_param_update_request.conn_params.min_conn_interval,
                          p_gap_evt->params.conn_param_update_request.conn_params.max_conn_interval,
                          p_gap_evt->params.conn_param_update_request.conn_params.slave_latency,
                          p_gap_evt->params.conn_param_update_request.conn_params.conn_sup_timeout);
            monet_data.ble_conn_param.min_100us = p_gap_evt->params.conn_param_update_request.conn_params.min_conn_interval * 100 / 8;
            monet_data.ble_conn_param.max_100us = p_gap_evt->params.conn_param_update_request.conn_params.max_conn_interval * 100 / 8;
            monet_data.ble_conn_param.latency = p_gap_evt->params.conn_param_update_request.conn_params.slave_latency;
            monet_data.ble_conn_param.timeout_100us = p_gap_evt->params.conn_param_update_request.conn_params.conn_sup_timeout * 100;
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE:
        {
            ble_gap_evt_phy_update_t const * p_phy_evt = &p_ble_evt->evt.gap_evt.params.phy_update;

            if (p_phy_evt->status == BLE_HCI_STATUS_CODE_LMP_ERROR_TRANSACTION_COLLISION)
            {
                // Ignore LL collisions.
                NRF_LOG_RAW_INFO("LL transaction collision during PHY update.\r");
                break;
            }

            ble_gap_phys_t phys = {0};
            phys.tx_phys = p_phy_evt->tx_phy;
            phys.rx_phys = p_phy_evt->rx_phy;
            NRF_LOG_RAW_INFO("PHY update %s. PHY set to %s.\r",
                             (p_phy_evt->status == BLE_HCI_STATUS_CODE_SUCCESS) ?
                             "accepted" : "rejected",
                             phy_str(phys));
        }
        break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_RAW_INFO("PHY update request.\r");
            NRF_LOG_FLUSH();
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_RAW_INFO("GATT Client Timeout.\r");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_RAW_INFO("GATT Server Timeout.\r");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
		
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_RAW_INFO("BLE_GAP_EVT_SEC_PARAMS_REQUEST\r");
            break;

        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;
		
		case BLE_GAP_EVT_ADV_REPORT:
		{
			const ble_gap_evt_adv_report_t *p_adv_report = &p_ble_evt->evt.gap_evt.params.adv_report;
			if (is_in_ble_scan_mode() == true)
			{
				if (p_adv_report->type.scan_response == 1 		// Current report is scan_response
					&& p_adv_report->data.len > 5				// It should contain at least 5 bytes (Len(1) 0xFF cmpny_id(2) sensor_mask(1)). Length of Tested Manufacture Specific Data (MSD) is about 27
					&& p_adv_report->data.p_data[2] == (APP_COMPANY_ID & 0xff) 			// MSD[2-3] is App Company Idendifier.
					&& p_adv_report->data.p_data[3] == ((APP_COMPANY_ID >> 8) & 0xff) )
				{
					scan_list_scn_resp_add(&p_adv_report->peer_addr, &p_adv_report->data);
				}
			}
		} break;
		
		case BLE_GAP_EVT_AUTH_STATUS:
        {
			NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));

            if (((p_ble_evt->evt.gap_evt.params.auth_status.bonded + 1) % 2) == 0)
            {
                device_in_pairing_set(false);
            }                         
			if (is_in_pair_mode() == true && ble_state_get() == BLE_STATE_PAIRING)
			{
				if (p_ble_evt->evt.gap_evt.params.auth_status.bonded > 0)	// Device is bonded
				{
					// Warning: Move to device_ble_status_report()
					// pair_resp_to_mdm_send(1, 0, (uint8_t *)&sensor_mask, sizeof(sensor_mask), &p_element->adv_report.peer_addr);
                    monet_data.ble_pairok_notreported = 1;
                    NRF_LOG_RAW_INFO("ble_evt_handler() BLE_GAP_EVT_AUTH_STATUS paired ok\r");
				}
				else	// Failed to bond with the device
				{
                   // uint32_t sensor_mask = scan_list_mask_get();
                    const scan_list_t *p_element = NULL;

                    // The event message here does not contain address info, so it is needed to be given manually.
                    p_element = scan_list_elem_get(pairing_device_index_get());

                    scan_stop();
					leds_blink(0, 0);
					ble_dg_printf(BLE_DG_LOG_HEADER "failed to pair with address (LSB) %02X%02X%02X%02X%02X%02X\r\n", 
												p_element->adv_report.peer_addr.addr[0], 
												p_element->adv_report.peer_addr.addr[1], 
												p_element->adv_report.peer_addr.addr[2], 
												p_element->adv_report.peer_addr.addr[3], 
												p_element->adv_report.peer_addr.addr[4], 
												p_element->adv_report.peer_addr.addr[5] );
					NRF_LOG_RAW_INFO("ble_evt_handler() BLE_GAP_EVT_AUTH_STATUS failed to pair with address (LSB): ");
					NRF_LOG_RAW_HEXDUMP_INFO(p_element->adv_report.peer_addr.addr, BLE_GAP_ADDR_LEN);
					ble_state_set(BLE_STATE_TO_PAIR);
				}
			}
		}
            break;
		
        default:
            break;
    }
}

APP_TIMER_DEF(timer_solar_chg_mode);	// Timer for orange LED blink
uint32_t interval_solar_chg_mode = 300;//60;//300;		// Default value is 5 min (300 s)
bool need_to_check_sol_chg_mode = false;
//static bool solar_mode_running = false;		// State flag for Solar mode checking algorithm

void timer_handler_solar_chg_mode(void * p_context)
{
	if (p_context == (void *)timer_solar_chg_mode)
	{
		need_to_check_sol_chg_mode = true;
	}
}

// Restart timer to check solar charging mode
void timer_solar_chg_mode_restart(void)
{
	app_timer_stop(timer_solar_chg_mode);
	app_timer_create(&timer_solar_chg_mode, APP_TIMER_MODE_REPEATED, timer_handler_solar_chg_mode);
	app_timer_start(timer_solar_chg_mode, APP_TIMER_TICKS(interval_solar_chg_mode*1000), (void *)timer_solar_chg_mode);
//	solar_mode_running = true;
}

// Stop timer
void timer_solar_chg_mode_stop(void)
{
	app_timer_stop(timer_solar_chg_mode);
}

// Set timer interval
// Param. value: unit in seconds
void timer_solar_chg_mode_intvl_set(uint32_t value)
{
	interval_solar_chg_mode = value;
}

// Read timer interval
// Return value: unit in seconds
uint32_t timer_solar_chg_mode_intvl_get(void)
{
	return interval_solar_chg_mode;
}

// Periodically check solar power state and set to correct mode
// This function is designed to be periodically called in main loop, default 300 s (5 min)
void solar_chg_mode_check_proc(void)
{
    int32_t temp = 0;
	if (need_to_check_sol_chg_mode != true)
		return;
	need_to_check_sol_chg_mode = false;
	temp = get_temp(1);
    if((temp > CHG_TEMP_LIMIT4) || (temp <  CHG_TEMP_LIMIT1))
    {
        solar_chg_mode_select(FUNC_JUMP_POINT_4);
        NRF_LOG_RAW_INFO("solar_chg_mode_check_proc(), timer triggered.\r");
        NRF_LOG_RAW_INFO("The temp out normal range,choose to off. Temp (%d).\r",temp);
    }
    else
    {
        monet_data.charge_mode_disable = 0;
	    solar_chg_mode_select(FUNC_JUMP_POINT_1);
        mppt_process_nml_deinit();
	    NRF_LOG_RAW_INFO("solar_chg_mode_check_proc(), timer triggered, MPPT deinit\r");
        NRF_LOG_RAW_INFO("The temp in healthy range, normal choose the mode. Temp (%d).\r",temp);
	    NRF_LOG_FLUSH();
    }
	
//	if (only_solar_power() == false)	// When there are other power supply, enable charger and disable checking solar mode
//	{
//		NRF_LOG_RAW_INFO("solar_chg_mode_check_proc(), detected other power supplies\r");
//		if (solar_mode_running == true)
//		{
//			solar_mode_running = false;
////			if (no_external_power() == false)		// There are other power supplies, Main, AUX...
////				&& is_charger_power_on() == false)
//			{
//				NRF_LOG_RAW_INFO("solar_chg_mode_check_proc(), enable Charger, set to Mode 1");
//				solar_chg_mode_set(SOLAR_CHG_MODE1);
////				setChargerOn();
//				timer_solar_chg_mode_stop();
//			}
//		}
//		NRF_LOG_FLUSH();
//		return;
//	}
//	solar_mode_running = true;
//	NRF_LOG_RAW_INFO("solar_chg_mode_check_proc(), solar chg mode (%u)\r", (uint32_t)solar_chg_mode_get());
//    NRF_LOG_FLUSH();
////	if (solar_chg_mode_get() == SOLAR_CHG_MODE1)
////		mode_selection_subproc();
////	else	// SOLAR_CHG_MODE2
//		mode_selection_proc();
}

/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

#define UART_PERI_FOR_DEBUG	0	// UART peri is used for TPMS in App. If debug, set this macro to non-zero value
extern uint32_t count1sec;
uint8_t resetfromSystemOff = 0;
size_t   	  gCount;
config_struct config_data;

uint8_t arr_test[] =
{
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 

	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
};
void ble_speed_test(void)
{
	if (ble_aus_ready_c != true)
		return;
	if (is_ble_send_queue_full() == 0)
    {
		ble_send_data_push(arr_test, sizeof(arr_test), 0, 0);
    }
}

APP_TIMER_DEF(timer_pw_check);			// Timer for power checking
APP_TIMER_DEF(timer_mppt);				// Timer for MPPT
APP_TIMER_DEF(timer_stage2);			// Timer for Stage 2
#define PW_CHK_PERIOD		5000//3000	// Checking period. Unit in ms
#define MPPT_PERIOD			1000		// Checking period. Unit in ms
#define STAGE2_PERIOD		300000		// Checking period. Unit in ms. 300 s = 5 min
static bool pw_check_flag = false;
static uint32_t sys_time_sec = 0;
static bool mppt_flag = false;
static bool stage2_flag = false;

static void timer_handler_low_power(void * p_context)
{
	if (p_context == (void *)timer_pw_check)
	{
		sys_time_sec += (PW_CHK_PERIOD/1000);
		pw_check_flag = true;
	}
	else if (p_context == (void *)timer_mppt)
		mppt_flag = true;
	else if (p_context == (void *)timer_stage2)
		stage2_flag = true;
}

static void timer_pw_start(void)
{
	app_timer_create(&timer_pw_check, APP_TIMER_MODE_REPEATED, timer_handler_low_power);
	app_timer_start(timer_pw_check, APP_TIMER_TICKS(PW_CHK_PERIOD), (void *)timer_pw_check);
}

static void timer_mppt_restart(void)
{
	app_timer_stop(timer_mppt);
	app_timer_create(&timer_mppt, APP_TIMER_MODE_REPEATED, timer_handler_low_power);
	app_timer_start(timer_mppt, APP_TIMER_TICKS(MPPT_PERIOD), (void *)timer_mppt);
}

static void timer_mppt_stop(void)
{
	app_timer_stop(timer_mppt);
	mppt_flag = false;
}

static void timer_stage2_restart(void)
{
	app_timer_stop(timer_stage2);
	app_timer_create(&timer_stage2, APP_TIMER_MODE_REPEATED, timer_handler_low_power);
	app_timer_start(timer_stage2, APP_TIMER_TICKS(STAGE2_PERIOD), (void *)timer_stage2);
}

static void timer_stage2_stop(void)
{
	app_timer_stop(timer_stage2);
	stage2_flag = false;
}

// Solar charging stages
// Solar Stage 1: Mode 1, charger off
// Solar Stage 2: Mode 2, charger off, enable 5 min timer to check solar Power
// Solar Stage 3: Mode 1, charger on, MPPT runs, 1 s timer is enable for MPPT
#define SOLAR_CHG_STAGE_1	1
#define SOLAR_CHG_STAGE_2	2
#define SOLAR_CHG_STAGE_3	3

static uint32_t solar_chg_stage = SOLAR_CHG_STAGE_1;

uint32_t solar_chg_stage_get(void)
{
	return solar_chg_stage;
}

void solar_chg_stage_set(uint32_t stage)
{
	solar_chg_stage = stage;
}

#define PW_CHECK_PROC_JUMP_POINT_0	0
#define PW_CHECK_PROC_JUMP_POINT_1	1

static void solar_chg_recover_to_default(void)
{
	mppt_process_nml_deinit();
	timer_stage2_stop();
	timer_mppt_stop();
	setChargerOff();
	solar_chg_mode_set(SOLAR_CHG_MODE1);
	solar_chg_stage_set(SOLAR_CHG_STAGE_1);
}

void solar_chg_stage_switch_3_to_2(void)
{
	int32_t input_vol_limt_default = 8500;
	
	mppt_process_nml_deinit();
	mp2762a_input_vol_limit_set(input_vol_limt_default);
	setChargerOff();
	solar_chg_mode_set(SOLAR_CHG_MODE2);
	timer_mppt_stop();
	solar_chg_stage_set(SOLAR_CHG_STAGE_2);
	timer_stage2_restart();
	NRF_LOG_RAW_INFO("solar_chg_stage_switch_3_to_2(), set to solar chg Stage 2\r");
}

void pw_check_proc(uint32_t jump_point)
{
	uint32_t bat_vol = 0;
	uint32_t solar_vol = 0;
	uint32_t main_vol = 0;
	uint32_t aux_vol = 0;
	
	uint32_t input_vol = 0;
	double input_cur = 0.0;
	double input_p = 0.0;
	int32_t input_vol_limt_default = 8500;
	
	if (jump_point == PW_CHECK_PROC_JUMP_POINT_1)
		goto pw_check_proc_jump_point_1;
	else
		/* Reserved */;
	
	if (pw_check_flag == true)
	{
		pw_check_flag = false;
		pf_wdt_kick();	// Kick Watchdog
		
		NRF_LOG_RAW_INFO("Time %u s: solar chg mode %u, stage %u, charger %u, MPPT %u\r", 
							sys_time_sec, (uint32_t)solar_chg_mode_get(), solar_chg_stage_get(), is_charger_power_on(), mppt_is_running());
		
		adc_conv_prepare();
		// The first time ADC value uses the value before entering low power mode
		main_vol = adc_to_vol_conv(monet_data.AdcMain, VOL_MAIN_FACTOR);
		aux_vol = adc_to_vol_conv(monet_data.AdcAux, VOL_AUX_FACTOR);
		solar_vol = adc_to_vol_conv(monet_data.AdcSolar, VOL_SOLAR_FACTOR);
		bat_vol = adc_to_vol_conv(monet_data.AdcBackup, VOL_BAT_FACTOR);
		NRF_LOG_RAW_INFO("pw_check_proc() main %u mV, aux %u mV, sol %u mV, bat %u mV\r", 
									main_vol, aux_vol, solar_vol, bat_vol);
		
		if ((bat_vol >= ADC_BAT_OK) || (main_aux_power_existing() == true))
		{
			NRF_LOG_RAW_INFO("pw_check_proc() reset\r");
			NRF_LOG_FLUSH();
			nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_RESET);		// Reset
		}
		else if (only_solar_power() == true)
		{
			if (solar_chg_stage_get() == SOLAR_CHG_STAGE_1)
			{
pw_check_proc_jump_point_1:
				// Mannually ADC conversion
				// Pin BAT_ADC_TMP_EN has already been set high, so it is not needed to do it here
				nrf_delay_ms(20);	// Give some time for voltage of pin VBAT_ADC to reach real level after BAT_ADC_TMP_EN is set high.
				nrf_delay_ms(30);	// Add extra time to measure solar voltage
				atel_adc_converion();
				adc_conv_stop();	// Already get the ADC value, not needed to wait for the ADC timer expires.
				solar_vol = adc_to_vol_conv(monet_data.AdcSolar, VOL_SOLAR_FACTOR);
				NRF_LOG_RAW_INFO("pw_check_proc(), sol V %u mV\r", solar_vol);
				if (solar_vol >= VOL_LIMIT_S_CHG_MODE_SEL)
				{
					setChargerOn();
					mp2762a_input_vol_limit_set(MP2762A_INPUT_VOL_LIMIT_SOL);
					nrf_delay_ms(200);	// Leave time for Charger chip to bring new configuration into effect
					input_vol = mp2762a_input_vol_get();
					input_cur = mp2762a_input_cur_get();
					input_p = input_vol * input_cur;
					NRF_LOG_RAW_INFO("pw_check_proc(), V %u mV, I %u mA, P %u uW\r", input_vol, (uint32_t)input_cur, (uint32_t)input_p);
					if (input_p > SOLAR_POWER_LIMIT)
					{
						mp2762a_input_vol_limit_set(input_vol_limt_default); // Recover Input Voltage Limit setting
						solar_chg_mode_set(SOLAR_CHG_MODE1);
						solar_chg_stage_set(SOLAR_CHG_STAGE_3);
						timer_stage2_stop();
						mppt_process_nml_deinit();
						timer_mppt_restart();
						NRF_LOG_RAW_INFO("pw_check_proc(), set to solar chg Stage 3\r");
					}
					else
					{
						solar_chg_stage_switch_3_to_2();
						NRF_LOG_RAW_INFO("pw_check_proc(), set to solar chg Stage 2\r");
					}
				}
				else
				{
					if (solar_chg_stage_get() != SOLAR_CHG_STAGE_1)
					{
						solar_chg_recover_to_default();
					}
					NRF_LOG_RAW_INFO("pw_check_proc(), set to solar chg Stage 1\r");
				}
			}
		}
		else
		{
			if (solar_chg_stage_get() != SOLAR_CHG_STAGE_1)
			{
				solar_chg_recover_to_default();
				NRF_LOG_RAW_INFO("pw_check_proc(), set to solar chg Stage 1, 2\r");
			}
		}
		NRF_LOG_FLUSH();
	}
}

static void solar_power_chk_proc(void)
{	
	if (mppt_flag == true)
	{
		mppt_flag = false;
		mppt_process_nml();
	}
	
	if (stage2_flag == true)
	{
		stage2_flag = false;
		if (only_solar_power() == true)
		{
			NRF_LOG_RAW_INFO("solar_power_chk_proc(), timer for Stage 2 triggered\r");
			solar_chg_mode_set(SOLAR_CHG_MODE1);
			adc_conv_prepare();
			pw_check_proc(PW_CHECK_PROC_JUMP_POINT_1);
		}
	}
}

void power_checking_loop(void)
{
	nrf_gpio_cfg_output(BAT_ADC_TMP_EN);	// Initialize GPIO pin for battery ADC
	nrf_gpio_pin_set(BAT_ADC_TMP_EN);
	nrf_delay_ms(40);
	atel_adc_converion();
	nrf_gpio_pin_clear(BAT_ADC_TMP_EN);

	timer_pw_start();
	
	pf_i2c_init();
	nrf_gpio_cfg_output(CHRG_SLEEP_EN);
	nrf_gpio_cfg_output(CHRGIN_PWR_EN);
	nrf_gpio_pin_set(CHRG_SLEEP_EN);
	nrf_gpio_pin_set(CHRGIN_PWR_EN);
	setChargerOff();
	
	mppt_power_check_for_low_power_en();
	
	nrf_gpio_cfg_output(SOLAR_CHARGE_SWITCH);
	nrf_gpio_pin_clear(SOLAR_CHARGE_SWITCH);

	while (1)
	{
		idle_state_handle();
		pw_check_proc(PW_CHECK_PROC_JUMP_POINT_0);
		adc_conv_proc();
		solar_power_chk_proc();
	}
}

#define TAMPER_WAKEUP_PIN_VALID_STATE           (0)
#define TAMPER_WAKEUP_PIN_SAMPLE_COUNT          (100)
#define TAMPER_WAKEUP_PIN_SAMPLE_TIME_MS        (2 * 1000)
#define TAMPER_WAKEUP_PIN_SAMPLE_VALID_COUNT    (80) // Tamper really pushed count

bool tamper_valid_debounce(void)
{
    uint16_t count = 0;
    uint16_t valid_count = 0;

    nrf_gpio_cfg_input(BLE_Tamper, NRF_GPIO_PIN_PULLUP);

    while (count < TAMPER_WAKEUP_PIN_SAMPLE_COUNT) // check state need to read 2 second
    {
        count++;

        if (nrf_gpio_pin_read(BLE_Tamper) == TAMPER_WAKEUP_PIN_VALID_STATE)
        {
            valid_count++;

            if (valid_count >= TAMPER_WAKEUP_PIN_SAMPLE_VALID_COUNT)
            {
                NRF_LOG_RAW_INFO("BLE Tamper really was pushed\r");
                NRF_LOG_RAW_INFO("Capture %u sample times,Capture %u times valid(%d)\r", 
                                 count, 
                                 valid_count, 
                                 TAMPER_WAKEUP_PIN_VALID_STATE);
                NRF_LOG_FLUSH();

                nrf_gpio_cfg_default(BLE_Tamper);

                return true;
            }
        }

        pf_delay_ms(TAMPER_WAKEUP_PIN_SAMPLE_TIME_MS / TAMPER_WAKEUP_PIN_SAMPLE_COUNT);
    }

    NRF_LOG_RAW_INFO("Capture %u sample times ,Capture %u times low level\r", 
                     count, valid_count);
    NRF_LOG_FLUSH();

    return false;
}

void tamper_check_after_shipping_mode(void)
{
    //Check if the system woke up from System OFF mode
    if ((NRF_POWER->GPREGRET >> 4) == RESET_MEMORY_TEST_BYTE)
    {
        NRF_POWER->GPREGRET = 0; //need set before SD is enable
        resetfromSystemOff = 1;
        if (tamper_valid_debounce() == false)
        {
            shipping_mode_wkp_src_config(); // Enable wakeup source from shipping mode
            NRF_LOG_RAW_INFO("Enable wakeup source from shipping mode \r");
            NRF_LOG_FLUSH();
            pf_delay_ms(10);
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF); // Set system to System OFF mode. When wakeup source is triggered, system would get reset and run from beginning
        }
    }
    nrf_gpio_cfg_default(BLE_Tamper);
}

/**@brief Application main function.
 */
int main(void)
{
    uint8_t resetfromDFU = 0;
	bool bootup_pw_check = true;
    uint32_t err_code;
	
    // Initialize.
    // uart_init();
    log_init();

    if (*((uint32_t *)(0x2003fffc)) == 0xDFDFDFDF)
    {
        *((uint32_t *)(0x2003fffc)) = 0;
        resetfromDFU = 1;
        reset_info.reset_from_dfu = 1;
        NRF_LOG_RAW_INFO("Reset from DFU\r");
    }
    else
    {
        nrf_gpio_cfg_output(VDD_MDM_EN);
        nrf_gpio_pin_clear(VDD_MDM_EN);
    }

    pf_wdt_init();

    //Check if the system woke up from System OFF mode
    // if ((NRF_POWER->GPREGRET >> 4) == RESET_MEMORY_TEST_BYTE)
    // {
    //     NRF_POWER->GPREGRET = 0;  //need set before SD is enable
    //     resetfromSystemOff = 1;
    // }
    tamper_check_after_shipping_mode();
    
    timer_init();
	HandleReset();	// SIMBAMCU-34
    // buttons_leds_init(&erase_bonds);
	
    power_management_init();
    ble_stack_init();
//    gap_params_init();
    gatt_init();
	peer_manager_init();
	db_discovery_init();
//    services_init();
//    advertising_init();
//    conn_params_init();
	nus_c_init();
    scan_init();

#if (SUPPORT_BLE_CENTRAL_AND_PERIPH == 1)

    #if (SUPPORT_BLE_BEACON == 1)
    conn_params_init();
    gap_params_init();
    services_init();

    beacon_advertising_init(BEACON_MODE);
    //normal_advertising_init(true,beacon_adv_interval);

    //pf_adv_start(0);
    //pf_adv_start(1);
	err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, tx_power_level_default);  // code from simba
	APP_ERROR_CHECK(err_code);
    #endif/*(SUPPORT_BLE_BEACON == 1)*/

#endif /* (SUPPORT_BLE_CENTRAL_AND_PERIPH == 1) */ 

    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT, 0, BLE_POWER_LEVEL_DEFAULT);
    APP_ERROR_CHECK(err_code);

    // Enable DCDC
    // sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    NRF_LOG_RAW_INFO("Logging Nala(%s, %s) RESETREAS(0x%x) DCDCEN(%d).\r", __DATE__, __TIME__, NRF_POWER->RESETREAS, NRF_POWER->DCDCEN);
    NRF_LOG_RAW_INFO("Nala BLE Version %d.%d.%d.%d.\r", MNT_MAJOR, MNT_MINOR, MNT_REVISION, MNT_BUILD);

    sd_ble_gap_addr_get(&central_addr);
    NRF_LOG_RAW_INFO("id_peer(%d) add_type(%d).\r", central_addr.addr_id_peer, central_addr.addr_type);
    NRF_LOG_RAW_INFO("Self Mac:(0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x)\r",
                         central_addr.addr[5],
                         central_addr.addr[4],
                         central_addr.addr[3],
                         central_addr.addr[2],
                         central_addr.addr[1],
                         central_addr.addr[0] );
    NRF_LOG_FLUSH();

	ble_send_recv_init_c();
	
    nrf_delay_ms(1000);

////    ble_send_recv_init();

//    #if BLE_ADVERTISING_ENABLE_BY_DEFAULT
//    ble_data_test(0);
//    #endif /* BLE_ADVERTISING_ENABLE_BY_DEFAULT */
//	ble_aus_advertising_start();	//////////// For test purpose. BLE feature is to be defined
	
    if (!nrfx_gpiote_is_init()) {
        if (nrfx_gpiote_init() != NRF_SUCCESS) {
            NRF_LOG_RAW_INFO("nrfx_gpiote_init fail.\r");
            NRF_LOG_FLUSH();
        }
    }

    InitApp(resetfromDFU);
	{
		gCount = 0;
		// Program the MEMS with default value. The loop exit if the
		// configuration is successful or too many attempts
		while (	(ion_accRegInit2(config_data.ar[1], config_data.at[1], config_data.adur[1], 0, 0) == MEMS_ERROR) &&
				(gCount < 10)) {
			gCount++;
		}
    }
//	scan_start();
	ble_send_timer_start_c();
    memcpy(monet_data.ble_mac_addr, central_addr.addr, BLE_MAC_ADDRESS_LEN);
	
//	// Data output would sometimes influence TPMS device. Comment it now. 20200930
//	printf("BLE started. ID:(0x%02X:0x%02X:0x%02X:0x%02X:0x%02X:0x%02X)\r",
//                         peripheral_addr.addr[5],
//                         peripheral_addr.addr[4],
//                         peripheral_addr.addr[3],
//                         peripheral_addr.addr[2],
//                         peripheral_addr.addr[1],
//                         peripheral_addr.addr[0]);
	// Enter main loop.
    for (;;)
    {
		ret_code_t err_code;
#if !(BLE_BYPASS_TEST_ENABLE)
    #if TIME_UNIT_CHANGE_WHEN_SLEEP
		if (((monet_data.phonePowerOn == 0) ||
		//#if TIME_UNIT_CHANGE_WHEN_SLEEP
		    (monet_data.SleepStateChange == 0)) &&
		//#endif	// #if TIME_UNIT_CHANGE_WHEN_SLEEP
            ((monet_data.uart_idle_100ms >= TIME_UNIT_IDLE_COUNT) &&
            (monet_data.uartMdmBaudRate == 9))
		   )
    #else
        if  (   (monet_data.phonePowerOn == 0) ||
                ((monet_data.uart_idle_100ms >= TIME_UNIT_IDLE_COUNT) &&
                (monet_data.uartMdmBaudRate == 9))
            )
    #endif	// #if TIME_UNIT_CHANGE_WHEN_SLEEP

		{
            monet_data.mcuWillIdle = 1;
			idle_state_handle();
		}
        else
        {
            monet_data.mcuWillIdle = 0;
        }

        if ((monet_data.uart_idle_wake == 1) &&
            (monet_data.phonePowerOn) &&
            (monet_data.uartMdmBaudRate == 9) && (SLEEP_NORMAL != monet_data.SleepState))
        {
            pf_uart_mdm_init(255, 0);
            monet_data.uart_idle_wake = 0;
        }

		err_code = nrf_ble_lesc_request_handler();
		if (err_code)
        {
            NRF_LOG_RAW_INFO("nrf_ble_lesc_request_handler\r");
            NRF_LOG_FLUSH();
            nrf_ble_lesc_request_handler();
        }

		atel_io_queue_process();
#if !UART_PERI_FOR_DEBUG
		uart_peri_rx_process();
#else
		uint8_t tmp;
		if (pf_uart_peri_rx_one(&tmp) == 0)
		{
			if (tmp == 't')
				printf("123\r\n");
		}
#endif
		
//		ble_speed_test();
		
//		mcu_start_capture_process();

		if (mux_com_timer_is_triggered() == true)
		{
			mux_com_timer_clear();
			mux_mcu_proc();
		}
		atel_timerTickHandler(monet_data.sysTickUnit);
		pf_systick_change();
		adc_conv_proc();
		solar_chg_mode_check_proc();
		scan_mode_timeout_proc();
		ble_proc();
		
		// Check if something to send to Modem
		CheckInterrupt();

		ble_send_data_rate_show(0);
		
		if (monet_data.waitOnCSTxEmpty) {
            if (pf_uart_peri_tx_queue_is_empty() == true) { // Check if queue is empty
               monet_data.waitOnCSTxEmpty = 0;
               monet_data.lastdataframecounter = monet_data.dataframecounter; // Save the frame counter for later queries
               BuildFrame('d', &monet_data.waitOnCSTxLen, sizeof(monet_data.waitOnCSTxLen));
            }
//            gIdle = 0;
        }
		
		if ((monet_data.V3PowerOn == 0) && 
            (!monet_data.phonePowerOn &&
			 !monet_gpio.counter[GPIO_MDM_PWR_KEY] &&
			 !monet_gpio.counter[GPIO_CS_3V3_EN])) // If modem power down in progress allow it to complete
        {
			if (monet_data.bEnterSysOff) {
				NRF_LOG_RAW_INFO("Enter Shipping mode\r\n");
				NRF_LOG_FLUSH();
                monet_setGPIOLowPowerMode();
                break; // Note: after "break", code would falls out of the while loop to enter shipping mode. Please see bottom of main()
            }
			
			/* Other cases are to be implemented */
		}
//            if (resetfromSystemOff && count1sec > 5) //count 5 seconds to read ADC correctly
            if (/*resetfromSystemOff && */bootup_pw_check == true && count1sec > 5) //count 5 seconds to read ADC correctly
            {
                resetfromSystemOff = 0;
				bootup_pw_check = false;
				
//                if((PF_ADC_RAW_TO_BATT_MV(monet_data.AdcBackup) < ADC_BAT_OK) && !isOtherPowerSource())
                if((adc_to_vol_conv(monet_data.AdcBackup, VOL_BAT_FACTOR) < ADC_BAT_OK) && (main_aux_power_existing() == false))
                {
//                	NRF_LOG_RAW_INFO("Need to enter into shipping mode again, because the battery voltage is < 7.4v\r\n");
                	NRF_LOG_RAW_INFO("\r\nPeriodically read Main, AUX and Solar power supply\r\n");
                	NRF_LOG_FLUSH();
                	monet_setGPIOLowPowerMode();
					power_checking_loop();	// Periodically read Main and AUX power supply, if there are pluged in, reset chip; otherwise stay in low power consumption state.
//					break;
                }
            }
		
#else
        ble_send_timer_start();
        idle_state_handle();
        pf_wdt_kick();
        NRF_LOG_FLUSH();

        ble_send_data_rate_show();

        #if BLE_BYPASS_TEST_ONEWAY_TX
        static uint32_t onway_tx_count = 0;
        uint8_t buf[128] = {0};
        memset(buf, (onway_tx_count % 26) + 'a', 128);
        buf[126] = '\r';
        buf[127] = '\n';
        memcpy(buf, &onway_tx_count, 4);
        if ((monet_data.ble_info[0].connect_status == BLE_CONNECTION_STATUS_CONNECTED) &&
           (is_ble_send_queue_full() == 0))
        {
            ble_send_data_push(buf, 128, 0);
            onway_tx_count++;
            nrf_delay_ms(10);
        }
        #else
        if (is_ble_recv_queue_empty() == 0)
        {
            uint8_t *tmp_buf;
            uint16_t tmp_len = 0;
            uint16_t tmp_ch = 0;

            if (is_ble_send_queue_full() == 0)
            {
                ble_recv_data_pop(&tmp_buf, &tmp_len, &tmp_ch);
                ble_send_data_push(tmp_buf, tmp_len, tmp_ch);
                ble_recv_data_delete_one();
            }
        }
        #endif /* BLE_BYPASS_TEST_ONEWAY_TX */
#endif /* BLE_BYPASS_TEST_ENABLE */
    }
	shipping_mode_wkp_src_config();	// Enable wakeup source from shipping mode
	nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);	// Set system to System OFF mode. When wakeup source is triggered, system would get reset and run from beginning
}

//ret_code_t ble_aus_data_send_periheral(uint8_t * p_data, uint16_t data_len, uint16_t channel)
//{
//    ret_code_t ret_val;
////    if (monet_data.ble_info[channel].connect_status != BLE_CONNECTION_STATUS_CONNECTED)
////    {
////        return NRF_ERROR_INVALID_STATE;
////    }

////    ret_val = ble_nus_data_send(&m_nus, p_data, &data_len, monet_data.ble_info[channel].handler);
//    return ret_val;
//}

//uint16_t get_ble_aus_max_data_len(void)
//{
//    return m_ble_nus_max_data_len;
//}

//void ble_aus_white_list_set(void)
//{
//    uint32_t err_code;
//    uint16_t i = 0;
//    ble_gap_addr_t const * addr_ptrs[BLE_CHANNEL_NUM_MAX];
//    uint8_t addr_buf[BLE_CHANNEL_NUM_MAX * BLE_MAC_ADDRESS_LEN] = {0};
//    uint8_t *addr = addr_buf;
//    uint16_t channel = 0, len = 0;

//    for (channel = 0; channel < BLE_CHANNEL_NUM_MAX; channel++)
//    {
//        if (((monet_data.ble_info[channel].connect_status <= BLE_CONNECTION_STATUS_MAC_SET) &&
//            (monet_data.ble_info[channel].connect_status != BLE_CONNECTION_STATUS_NOTVALID)))
//        {
//            memcpy(addr + len, monet_data.ble_info[channel].mac_addr, BLE_MAC_ADDRESS_LEN);
//            len += BLE_MAC_ADDRESS_LEN;
//        }
//    }

//    whitelist_count = len / BLE_GAP_ADDR_LEN;

//    for (i = 0; i < whitelist_count; i++)
//    {
//        whitelist_addrs[i].addr_id_peer = 0;
//        whitelist_addrs[i].addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
//        whitelist_addrs[i].addr[0] = addr[0];
//        whitelist_addrs[i].addr[1] = addr[1];
//        whitelist_addrs[i].addr[2] = addr[2];
//        whitelist_addrs[i].addr[3] = addr[3];
//        whitelist_addrs[i].addr[4] = addr[4];
//        whitelist_addrs[i].addr[5] = addr[5];
//        addr += BLE_GAP_ADDR_LEN;
//        addr_ptrs[i] = whitelist_addrs + i;
//        NRF_LOG_RAW_INFO("whitelist Mac:(0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x)\r",
//                         whitelist_addrs[i].addr[0],
//                         whitelist_addrs[i].addr[1],
//                         whitelist_addrs[i].addr[2],
//                         whitelist_addrs[i].addr[3],
//                         whitelist_addrs[i].addr[4],
//                         whitelist_addrs[i].addr[5]
//                         );
//        NRF_LOG_FLUSH();
//    }

//    if (whitelist_count == 0)
//    {
//        NRF_LOG_RAW_INFO("sd_ble_gap_whitelist_set clear.\r");
//        NRF_LOG_FLUSH();
//        err_code = sd_ble_gap_whitelist_set(NULL, 0);
//    }
//    else
//    {
//        err_code = sd_ble_gap_whitelist_set(addr_ptrs, whitelist_count);
//    }
//    NRF_LOG_RAW_INFO("sd_ble_gap_whitelist_set: %d.\r", err_code);
//    NRF_LOG_FLUSH();
//}

//void ble_aus_advertising_stop(void)
//{
//    uint32_t err_code = 0;
//    uint16_t i = 0, handler = 0xffff;

//    for (i = 0; i < BLE_CHANNEL_NUM_MAX; i++)
//    {
//        handler = ble_connected_handler_get_from_channel(i);
//        if (handler != 0xffff)
//        {
//            err_code = sd_ble_gap_disconnect(handler, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//            NRF_LOG_RAW_INFO("ble_aus_advertising_stop CH(%d) Handler(%d) Err(%d).\r", i, handler, err_code);
//            NRF_LOG_FLUSH();
//        }
//    }

//    // TODO: optimize advertising stop
//    // if (m_advertising.adv_mode_current != BLE_ADV_MODE_IDLE)
//    // {
//    //     sd_ble_gap_adv_stop(m_advertising.adv_handle);
//    // }

//    ble_connected_channel_clear();

//    // TODO: disconnect all
//    monet_data.bleDisconnectEvent = 1;
//}

//void advertising_stop(void)
//{
//	sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//    uint32_t err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
//    APP_ERROR_CHECK(err_code);
//}

void ble_disconnect_with_peer(uint8_t inst_id, uint8_t type)
{
    if (m_conn_handle != 0xffff)
    {
        sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    }
}

//uint32_t ble_aus_advertising_start(void)
//{
//    uint32_t err_code = 0;

//    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
////    NRF_LOG_RAW_INFO("ble_aus_advertising_start: %d.\r", err_code);
//    NRF_LOG_RAW_INFO("ble_advertising_start: %d.\r", err_code);
//    NRF_LOG_FLUSH();

//    // APP_ERROR_CHECK(err_code);
//    return err_code;
//}

//uint32_t ble_aus_change_change_conn_params(uint16_t channel, ble_aus_conn_param_t ble_conn_param)
//{
//    ret_code_t err_code = 0;
//    ble_gap_conn_params_t new_params;

//    if (monet_data.ble_info[channel].handler == 0xffff)
//    {
//        NRF_LOG_RAW_INFO("change_conn_params channel err.\r");
//        return 0xfe;
//    }

//    err_code = ble_conn_params_stop();
//    NRF_LOG_RAW_INFO("ble_conn_params_stop Err(0x%x:%d).\r", err_code, err_code);
//    
//    new_params.min_conn_interval = MSEC_TO_UNITS(ble_conn_param.min_ms, UNIT_1_25_MS);
//    new_params.max_conn_interval = MSEC_TO_UNITS(ble_conn_param.max_ms, UNIT_1_25_MS);
//    new_params.slave_latency = ble_conn_param.latency;
//    new_params.conn_sup_timeout = MSEC_TO_UNITS(ble_conn_param.timeout_ms, UNIT_10_MS);

//    err_code = ble_conn_params_change_conn_params(monet_data.ble_info[channel].handler, &new_params);

//    NRF_LOG_RAW_INFO("change_conn_params Err(0x%x:%d).\r", err_code, err_code);
//    NRF_LOG_FLUSH();

//    return err_code;
//}

//uint32_t ble_aus_change_change_conn_params_disconnect(uint16_t channel)
//{
//    ret_code_t err_code = 0;

//    if (channel != 0xffff)
//    {
//        err_code = sd_ble_gap_disconnect(monet_data.ble_info[channel].handler, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);

//        monet_data.ble_info[channel].connect_status = BLE_CONNECTION_STATUS_NOT_CONNECTED;
//        
//        ble_aus_advertising_start();
//    }

//    NRF_LOG_RAW_INFO("conn_params_disconnect CH(%d) Err(%d).\r", channel, err_code);
//    NRF_LOG_FLUSH();

//    return err_code;
//}


uint32_t ble_aus_data_send_central(uint8_t * p_data, uint16_t data_len, uint8_t st, uint8_t id)
{
    ret_code_t ret_val;

    if ((ble_aus_ready_c == false) || (p_data == NULL))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    // ret_val = ble_nus_c_string_send(&m_ble_nus_c, p_data, data_len);
    do 
    {
        ret_val = ble_nus_c_string_send(&m_ble_nus_c, p_data, data_len);
        if ((ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_RESOURCES))
        {
            APP_ERROR_CHECK(ret_val);
        }
    }while (ret_val == NRF_ERROR_RESOURCES);

    return ret_val;
}

uint16_t get_ble_aus_max_data_len(void)
{
    return m_ble_nus_max_data_len;
}

// If connected, return true, else return false
bool ble_aus_ready_state_get_c(void)
{
    return ble_aus_ready_c;
}

void ble_aus_set_scan_filter(uint8_t *p_addr)
{
    ret_code_t err_code = 0;
    static uint8_t scan_filter_set = 0;

    if ((p_addr[0] != monet_data.ble_peer_mac_addr[0]) ||
        (p_addr[1] != monet_data.ble_peer_mac_addr[1]) ||
        (p_addr[2] != monet_data.ble_peer_mac_addr[2]) ||
        (p_addr[3] != monet_data.ble_peer_mac_addr[3]) ||
        (p_addr[4] != monet_data.ble_peer_mac_addr[4]) ||
        (p_addr[5] != monet_data.ble_peer_mac_addr[5]))
    {
        pf_log_raw(atel_log_ctl.error_en, "aus_set_scan_filter Mac not match.\r");
        return;
    }

    if ((p_addr[1] | p_addr[2] | p_addr[3] | p_addr[4] | p_addr[5] | p_addr[6]) == 0)
    {
        pf_log_raw(atel_log_ctl.error_en, "aus_set_scan_filter Mac Err.\r");
        return;
    }

    if (scan_filter_set)
    {
        return;
    }

    m_target_periph_addr.addr[0] = monet_data.ble_peer_mac_addr[0];
    m_target_periph_addr.addr[1] = monet_data.ble_peer_mac_addr[1];
    m_target_periph_addr.addr[2] = monet_data.ble_peer_mac_addr[2];
    m_target_periph_addr.addr[3] = monet_data.ble_peer_mac_addr[3];
    m_target_periph_addr.addr[4] = monet_data.ble_peer_mac_addr[4];
    m_target_periph_addr.addr[5] = monet_data.ble_peer_mac_addr[5];
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_ADDR_FILTER, m_target_periph_addr.addr);
    // APP_ERROR_CHECK(err_code);
    pf_log_raw(atel_log_ctl.error_en, "SCAN_ADDR_FILTER fail(%d).\r", err_code);

    if (err_code == 0)
    {
        scan_filter_set = 1;
    
        err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_ALL_FILTER, true);
        // APP_ERROR_CHECK(err_code);
        pf_log_raw(atel_log_ctl.error_en, "NRF_BLE_SCAN_ALL_FILTER fail(%d).\r", err_code);
    }
}

void ble_aus_set_scan_start(void)
{
    scan_start();
}

void ble_aus_set_scan_mode(uint8_t mode)
{
    ret_code_t          err_code;
    ble_gap_scan_params_t scan_params = {0};

    if (mode == 0)
    {
        scan_params.active        = 1;
        scan_params.interval      = NRF_BLE_SCAN_SCAN_INTERVAL;
        scan_params.window        = NRF_BLE_SCAN_SCAN_WINDOW;
        scan_params.timeout       = NRF_BLE_SCAN_SCAN_DURATION;
        scan_params.filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL;
        scan_params.scan_phys     = BLE_GAP_PHY_1MBPS;
    }
    else if (mode == 1)
    {
        scan_params.active        = 1;
        scan_params.interval      = 64000 / 2;
        scan_params.window        = NRF_BLE_SCAN_SCAN_WINDOW;
        scan_params.timeout       = NRF_BLE_SCAN_SCAN_DURATION;
        scan_params.filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL;
        scan_params.scan_phys     = BLE_GAP_PHY_1MBPS;
    }

    err_code = nrf_ble_scan_params_set(&m_scan, &scan_params);
    // APP_ERROR_CHECK(err_code);
    pf_log_raw(atel_log_ctl.error_en, "nrf_ble_scan_params_set err(%d).\r", err_code);
}

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    pf_log_raw(atel_log_ctl.core_en, "Nala Version: %d.%d.%d.%d.\r\n", MNT_MAJOR, MNT_MINOR, MNT_REVISION, MNT_BUILD);
    if (p_evt->result != NRF_SUCCESS)
    {
        pf_log_raw(atel_log_ctl.core_en, "--> Event received: ERROR while executing an fstorage operation.\r");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            pf_log_raw(atel_log_ctl.core_en, "--> Event received: wrote %d bytes at address 0x%x.\r",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            pf_log_raw(atel_log_ctl.core_en, "--> Event received: erased %d page from address 0x%x.\r",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}

#define FDS_AREA_START_ADDRESS (0xF5000)
#define FDS_AREA_END_ADDRESS (0xF7FFF)

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = FDS_AREA_START_ADDRESS,
    .end_addr   = FDS_AREA_END_ADDRESS,
};

static void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    uint16_t i = 0;
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        if (i < 100)
        {
            i++;
            pf_delay_ms(1);
        }
        else
        {
            break;
        }
    }
}

static void erase_fds_areas(void)
{
    ret_code_t rc;
    nrf_fstorage_api_t * p_fs_api;

    p_fs_api = &nrf_fstorage_sd;

    rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);

    nrf_fstorage_erase(&fstorage, FDS_AREA_START_ADDRESS, 3, NULL);

    wait_for_flash_ready(&fstorage);

    nrf_fstorage_erase(&fstorage, FDS_AREA_START_ADDRESS, 3, NULL);

    wait_for_flash_ready(&fstorage);

    pf_delay_ms(100);

    nrf_fstorage_uninit(&fstorage, NULL);

    if (reset_info.reset_from_dfu == 1)
    {
        DFU_FLAG_REGISTER = DFU_FLAG_VALUE_FROM_BOOT;
    }

    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_DFU);
}

#if 0

static void erase_fds_areas_test(void)
{
    ret_code_t rc;
    nrf_fstorage_api_t * p_fs_api;
    uint32_t read = 0;
    uint32_t write = 0xDEADBEEF;
    uint32_t i = 0;

    p_fs_api = &nrf_fstorage_sd;

    rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);

    nrf_fstorage_read(&fstorage, FDS_AREA_START_ADDRESS, &read, sizeof(read));
    NRF_LOG_RAW_INFO("erase_fds_areas Read(0x%x) DFU_FLAG_REGISTER(0x%x)\r", read, DFU_FLAG_REGISTER);
    NRF_LOG_FLUSH();

    nrf_fstorage_erase(&fstorage, FDS_AREA_START_ADDRESS, 3, NULL);

    wait_for_flash_ready(&fstorage);

    nrf_fstorage_erase(&fstorage, FDS_AREA_START_ADDRESS, 3, NULL);

    wait_for_flash_ready(&fstorage);

    if (((DFU_FLAG_REGISTER) % 2) == 0)
    {
        while (i < (12 * 1024))
        {
            nrf_fstorage_write(&fstorage, FDS_AREA_START_ADDRESS + i, &write, sizeof(write), NULL);
            wait_for_flash_ready(&fstorage);
            i += sizeof(write);
        }

        NRF_LOG_RAW_INFO("erase_fds_areas Write Done\r");
        NRF_LOG_FLUSH();
    }

    pf_delay_ms(1000);

    nrf_fstorage_uninit(&fstorage, NULL);

    if (reset_info.reset_from_dfu == 1)
    {
        DFU_FLAG_REGISTER = DFU_FLAG_VALUE_FROM_BOOT;
    }

    DFU_FLAG_REGISTER++;

    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_DFU);
}

#endif // 0

/**
 * @}
 */
