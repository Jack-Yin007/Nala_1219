#ifndef __ATEL_NALA_HAL_H
#define __ATEL_NALA_HAL_H
#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_saadc.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_wdt.h"
#include "nrf_pwr_mgmt.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "app_uart.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_bootloader_info.h"

#include "nrf_queue.h"
#include "nrf_balloc.h"
#include "user.h"

#define BLE_OSC32_IN				(0)
#define BLE_OSC32_OUT				(1)
#define BLE_GPIO1					(26)
#define BLE_GPIO2					(27)
#define BLE_GPIO3					(4)
#define BLE_Tamper					(5)
#define BLE_RELAY					(6)
#define BLE_TX_DEBUG				(7)
#define BLE_RX_DEBUG				(8)
#define RS232_EN					(NRF_GPIO_PIN_MAP(1, 8))
#define MDM_PWR_KEY					(NRF_GPIO_PIN_MAP(1, 9))
#define BLE_UART1_TX				(11)
#define BLE_UART1_RX				(12)

#define BLE_CAN_PWR_EN				(13)
#define CS_12V_EN					(14)
#define BLE_MUX						PIN_NOT_VALID // (15)
#define Hall_Tamper					(16)
#define VBAT_Heating_Power_EN		PIN_NOT_VALID // (17)
#define BLE_NRST					PIN_NOT_VALID // (18)
#define CAN_INT						PIN_NOT_VALID // (19)
//#define CHRG_PROCHOT				(20)
#define CHRG_SLEEP_EN				(20)
#define CHRG_INT					(21)
#if (HW_VER >= HW_VER_P3)
#define SOLAR_CHARGE_SWITCH			(22)
#else
#define CHRG_ACOK					(22)
#endif
#define ST_MCU_PWR_EN				(23)
#define CS_3V3_EN					(24)
#define VDD_MDM_EN					(25)
#define DC_DC_9V5_EN				(NRF_GPIO_PIN_MAP(1, 0))

#define CS_MERCREBOOT				(NRF_GPIO_PIN_MAP(1, 1))   //PIN_NOT_VALID
#define CS_nRST						(NRF_GPIO_PIN_MAP(1, 2))
#define TMP_INT						PIN_NOT_VALID // (NRF_GPIO_PIN_MAP(1, 3))
#define G_SENSOR_INT				(NRF_GPIO_PIN_MAP(1, 4))
#define LED_ORANGE					(NRF_GPIO_PIN_MAP(1, 5))
#define LED_GREEN					(NRF_GPIO_PIN_MAP(1, 6))
#define LED_RED						(NRF_GPIO_PIN_MAP(1, 7))
#define BLE_I2C2_SCL				(9)
#define BLE_I2C2_SDA				(10)

#define BLE_I2C1_SCL				(NRF_GPIO_PIN_MAP(1, 10))
#define BLE_I2C1_SDA				(NRF_GPIO_PIN_MAP(1, 11))
#define ONE_BUS_SLPZ				(NRF_GPIO_PIN_MAP(1, 12))
#define ST_UART1_TO_UART2_EN		(NRF_GPIO_PIN_MAP(1, 13))
#define ST_UART1_TO_UART3_EN		(NRF_GPIO_PIN_MAP(1, 14))
#define ST_UART2_TO_UART3_EN		(NRF_GPIO_PIN_MAP(1, 15))
#define BAT_ADC_TMP_EN				(3)
//#define NTC_ADC					(2)
#define CHRGIN_PWR_EN				(2)
#define VBAT_ADC					(28)
#define PIN_12V_M_IN_ADC			(29)
#define PIN_12V_A_IN_ADC			(30)
#define PIN_12V_S_IN_ADC			(31)

#define PIN_NOT_VALID              (0xffffffff)

#define ACC_GYRO_INT_VALID (0)
#define ACC_GYRO_INT_INVALID (1)

#ifndef SUPPORT_ACC_STREAMING
#define SUPPORT_ACC_STREAMING (1)
#endif /* SUPPORT_ACC_STREAMING */


#define MDM_PWKEY_TOGGLE_ON_MS		(800)
#define MDM_PWKEY_TOGGLE_OFF_MS		(1000)
#define MDM_POWER_TOGGLE_MS		 	(1000)
#define MDM_WAKE_MCU_LEVEL (0)
#define MDM_OFF_POWER_HOLD_COUNT (33) // Second

#define V3_TURN_ON_DELAY_MS			(50)
#define V3_MDM_PW_DELAY_MS			(5000)

#define ADC_CHANNELS_CNT (4)
#define ADC_REFERENCE_VOLTAGE_FACTOR ((double)(3600.0 / 3300.0))
#define ADC_RESOLUTION_VALUE (1.1375) // 4095/3600, 12 BIT
#define ADC_MAIN_VOL_MULTIPLIER_FACTOR 0.10131// 10/(10+88.7)
#define ADC_MAIN_VOL_MODIFICATION_VALUE 0 //(80 + 180)
#define ADC_BUB_VOL_MULTIPLIER_FACTOR 0.36486 // 27/(47+27)
#define ADC_BUB_VOL_MODIFICATION_VALUE (0)
#define ADC_REPORT_RAW_VALUE (0)

#define BLE_DATA_CHANNEL_SUPPORT (1)
#define BLE_ADVERTISING_ENABLE_BY_DEFAULT (0)
#define BLE_CONNECTION_SUPPORT_HEARTBEAT (1)
#define BLE_BYPASS_TEST_ENABLE (0)
#define BLE_BYPASS_TEST_ONEWAY_TX (0)
#define BLE_SCAN_INTERVAL_CHANGE_EN (1)

#define GLASS_BREAK_EVENT_PIN_VALID_VALUE (1)

#define BLE_FUNCTION_OFF (0)
#define BLE_FUNCTION_ON (1)
#define BLE_FUNCTION_ONOFF BLE_FUNCTION_ON

#define BLE_PROFILE_NONE (0)
#define BLE_PROFILE_CAMERA (1)
#define BLE_PROFILE_SELECTION BLE_PROFILE_CAMERA

#define BLE_DTM_ENABLE (1)

// 9:   115200
// 10:  230400
#define UART_BAUDRATE_SELECT (9)
#define UART_BAUDRATE_SELECT_MAX (10)

#define DFU_FLAG_REGISTER (*((uint32_t *)(0x2003fffc)))
#define DFU_FLAG_VALUE_FROM_BOOT (0xDFDFDFDF)
#define DFU_FLAG_VALUE_FROM_APP (0xFDFDFDFD)
#define DFU_FLAG_VALUE_FROM_DTME (0xDDDDDDDD)

#ifdef SUPPORT_BLE_ADV
#define SUPPORT_BLE_CENTRAL_AND_PERIPH  1
#define SUPPORT_BLE_BEACON              1
#define BEACON_MODE   0
#define NORMAL_MODE   1
#else
#define SUPPORT_BLE_CENTRAL_AND_PERIPH  0
#define SUPPORT_BLE_BEACON              0
#define BEACON_MODE   0
#define NORMAL_MODE   1
#endif /* SUPPORT_BLE_ADV */

#ifdef __cplusplus
}
#endif
#endif	// #ifndef __ATEL_NALA_HAL_H

