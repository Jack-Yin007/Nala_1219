/*
 * user.h
 *
 *  Created on: Dec 12, 2015
 *      Author: J
 */

#ifndef USER_H_
#define USER_H_

/* get compiler defined type definitions (NULL, size_t, etc) */
#include <stddef.h>
#include "system.h"
#include "atel_util.h"
#include "platform_hal_drv.h"
#include "ble_oper.h"
#include "ble_aus_c.h"

/* TODO Application specific user parameters used in user.c may go here */


/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

/* TODO User level functions prototypes (i.e. InitApp) go here */

#define TIME_UNIT                       100//50		//ms
#define TIME_UNIT_CHANGE_WHEN_SLEEP     (1)
#define TIME_UNIT_IN_SLEEP_NORMAL       (TIME_UNIT) //ms
#define TIME_UNIT_IN_SLEEP_HIBERNATION  (1000)      //ms
#define TIME_UNIT_IDLE_COUNT            (2)         // 100ms
#define BB_SLEEP_CONFIG_DELAY_CNT       (3u)

#define DEVICE_BOOT_ENTER_DELAY_MS (1000)

#define IO_CMD_CHAR_BLE_RECV ('1')
#define IO_CMD_CHAR_BLE_SEND ('2')
#define IO_CMD_CHAR_BLE_CTRL ('3')

#define IO_CMD_CHAR_I2C_RECV ('4')
#define IO_CMD_CHAR_I2C_SEND ('5')

#define CMD_CHECKSUM                    1//(0)//(1)
#define CMD_ESCAPE                      (1)
#define USE_CHECKSUM                    (1)
#define MAX_COMMAND                     (256)//(244)

#define MIN_WD_TIMER_DFT    900//(180)    // PUMAMCU-160, 3 minutes.
// @Watchdog_time_change. Note: set to 3600. 180 s would make programing new MM19 board failure, which needs about 600 s, 20201027, QGH.

#define       QUEUE_SIZE (256) 
typedef struct {
    uint8_t            Buffer[QUEUE_SIZE];
    uint8_t            Head;
    uint8_t            Tail;
    uint8_t            Size;
} queue_struct;

#define INT_WAKEUP                      23 // 0x00800000
#define INT_POWERUP                     22 // 0x00400000 Added power up indicator
#define INT_WDFIRED                     21 // 0x00200000 Added WD Fired indicator
#define INT_APP_WAKEUP                  20 // 0x00100000 App Wake MCU
#define INT_VIRTUAL_IGNITION            19 // 0x00080000
#define INT_TILT_TAMPER                 18 // 0x00040000
#define INT_POWER_HIGH                  17 // 0x00020000
#define INT_POWER_LOW                   16 // 0x00010000
#define INT_TEMP_ONEWIRE                15
#define INT_TEMP_HIGH			        14
#define INT_RM_CPU_HOLD				    13
#define INT_ACCELEROMETER_TRIGGER       12 // 0x00001000
#define INT_BAT_ON                      11 // 0x00000800 the device is operating on battery (no main power)
#define INT_BAT_OFF                     10 // 0x00000400 the unit is no longer using battery (is on main power)
#define INT_ACC_SK                      9
#define INT_WAKEUP_TIMER                8  // 0x00000100
#define INT_TAMPER_WAKEUP	 			7  // 0x00000080
#define INT_GPIO_D                      3  // 0x00000008
#define INT_GPIO_C                      2  // 0x00000004
#define INT_GPIO_B                      1  // 0x00000002
#define INT_GPIO_A                      0  // 0x00000001

#define MASK_FOR_BIT(n) (1UL << (n))    // Use  1UL to avoid warning:  #61-D: integer operation result is out of range (for bit 31)

#define INT_MASK_ON  (0xFF | MASK_FOR_BIT(INT_RM_CPU_HOLD) | MASK_FOR_BIT(INT_TEMP_HIGH) | MASK_FOR_BIT(INT_TILT_TAMPER))
//#define INT_MASK_WK    ((1<<INT_WAKEUP_TIMER)| (1<<INT_ACC_SK) | (1<<INT_BAT_OFF)|(1<<INT_BAT_ON) )
// PUMAMCU-107 add one-wire wakeup bit to wakeup mask
#define INT_MASK_WK    (MASK_FOR_BIT(INT_WAKEUP_TIMER)      | \
                        MASK_FOR_BIT(INT_ACC_SK)            | \
                        MASK_FOR_BIT(INT_BAT_OFF)           | \
                        MASK_FOR_BIT(INT_BAT_ON)            | \
                        MASK_FOR_BIT(INT_POWER_LOW)         | \
                        MASK_FOR_BIT(INT_POWER_HIGH)        | \
                        MASK_FOR_BIT(INT_TEMP_ONEWIRE)      | \
                        MASK_FOR_BIT(INT_TILT_TAMPER)       | \
                        MASK_FOR_BIT(INT_VIRTUAL_IGNITION)  | \
                        0xFF)
// Int Source Flags
#define ACC_SRC_XL              (0)
#define ACC_SRC_XH              (1)
#define ACC_SRC_YL              (2)
#define ACC_SRC_YH              (3)
#define ACC_SRC_ZL              (4)
#define ACC_SRC_ZH              (5)
#define ACC_SRC_IA              (6)
#define ACC_SRC_RESERVED        (7)
#define MOTION_INT_MASK() (MASK_FOR_BIT(ACC_SRC_IA) | MASK_FOR_BIT(ACC_SRC_XH) | MASK_FOR_BIT(ACC_SRC_YH) | MASK_FOR_BIT(ACC_SRC_ZH))

#define ACC_INTERRUPT_NUM1 (1)
#define ACC_INTERRUPT_NUM2 (2)

typedef struct {
    uint8_t      status;
    uint8_t      t_on;
    uint8_t      t_off;
    uint8_t      tick;
} LedConfig;

// Readjust voltage trigger thresholds to avoid zombie mode
#define MAIN_ADC_MAIN_OFF               5500        // mV
#define MAIN_ADC_MAIN_VALID             6000        // mV
#define BUB_CRITICAL_THRESHOLD          3700        // mV
#define BUB_CRITICAL_DEBOUNCE           (3)         // second
#define SOLAR_CHARGE_POWER_VALID  (1000*1000)        // uw
#define SOLAR_GET_POWER_COUNT             	(5)         
#define CHARGE_RESTART_CHECK_COUNT         (5)         

#define			MASK_ADC_MAIN		0x01
#define			MASK_ADC_BAT		0x02
#define			MASK_ADC_LIGHT		0x04
#define			MASK_ADC_SOLAR		0x08
#define			MASK_ADC_TEMP		0x10

#define MDM_REQUIRE_ADC_RAW		0//1

#define VOL_MAIN_FACTOR			(98.7/10)
#define VOL_BAT_FACTOR			(74.0/27)
#define VOL_AUX_FACTOR			(98.7/10)
#define VOL_SOLAR_FACTOR		(98.7/10)

#define VOL_LIMIT_S_CHG_MODE_SEL	11000	// Voltage limit for solar charging mode selection. Unit in mV
#define MP2762A_INPUT_VOL_LIMIT_SOL		11000	// Unit in mV. Limit for solar charging

#define APP_COMPANY_ID			0x0059

#define LEDS_BLINK_INTERVAL_FAST	200			// Unit in ms
#define LEDS_BLINK_INTERVAL_SLOW	2000		// Unit in ms

#define LEDS_BLINK_DUR_5000MS		5000		// Unit in ms
#define LEDS_BLINK_DUR_10000MS		10000		// Unit in ms
#define LEDS_BLINK_DUR_UNLIMITED	0xffffffff	// Unit in ms

#define BLE_RESULT_FAILED		0
#define BLE_RESULT_SUCCESS		1

#define DEVICE_PAIRING_TIMEOUT_S (5 * 60ul)  // 300 s timeout 
// Do not rearrarange order without synchronizing with App
typedef enum {
    GPIO_FIRST = -1,

    GPIO_BLE_GPIO1,
    GPIO_BLE_GPIO2,
    GPIO_BLE_GPIO3,
    GPIO_BLE_Tamper,
    GPIO_BLE_RELAY,
	GPIO_RS232_EN,
    GPIO_MDM_PWR_KEY,
    GPIO_BLE_CAN_PWR_EN,
    GPIO_CS_12V_EN,
    GPIO_BLE_MUX,
    GPIO_Hall_Tamper,
    GPIO_VBAT_Heating_Power_EN,
    GPIO_CAN_INT,
//	GPIO_CHRG_PROCHOT,
    GPIO_CHRG_SLEEP_EN,
    GPIO_CHRG_INT,
#if (HW_VER >= HW_VER_P3)
	GPIO_SOLAR_CHARGE_SWITCH,
#else
    GPIO_CHRG_ACOK,
#endif
    GPIO_ST_MCU_PWR_EN,
    GPIO_CS_3V3_EN,
    GPIO_VDD_MDM_EN,
    GPIO_DC_DC_9V5_EN,
    GPIO_CS_MERCREBOOT,
    GPIO_CS_nRST,
    GPIO_TMP_INT,
    GPIO_G_SENSOR_INT,
    GPIO_LED_ORANGE,
    GPIO_LED_GREEN,
    GPIO_LED_RED,
    GPIO_ONE_BUS_SLPZ,
    GPIO_ST_UART1_TO_UART2_EN,
    GPIO_ST_UART1_TO_UART3_EN,
    GPIO_ST_UART2_TO_UART3_EN,
    GPIO_BAT_ADC_TMP_EN,
	GPIO_CHRGIN_PWR_EN,
    GPIO_BLE_WAKE_MDM,
    GPIO_MDM_WAKE_BLE,

    GPIO_LAST
} mntGpio_t;

#define GPIO_BUZZER_INDEX GPIO_BLE_BUZZER

//#define MAX_EXT_GPIOS (6)
#define MAX_EXT_GPIOS (8)	// There is 8 external GPIOs in Simba

#define IS_VALID_GPIO(p)        ((p) > GPIO_FIRST && (p) < GPIO_LAST)
#define GPIO_TO_INDEX(p)        ((p) - GPIO_FIRST - 1)
#define INDEX_TO_GPIO(index)    ((mntGpio_t)((index) + GPIO_FIRST + 1))
#define NUM_OF_GPIO_PINS        INDEX_TO_GPIO(GPIO_LAST)
#define NUM_OF_GPIO             NUM_OF_GPIO_PINS
#define GPIO_NONE               GPIO_LAST

// Define for status
#define LED_RUNNING   	1
#define LED_ON       	2
#define LED_REPEAT   	4

#define NUM_OF_LED		3

#define CHARGER_OFF   	0
#define CHARGER_ON		1

// 10 minutes delay
#define CHARGER_DELAY	(60*10)		

// Macros for hardware version
// Note: Value of new macro should be larger than the old
#define HW_VER_P1	1
#define HW_VER_P2	2
#define HW_VER_P3	3
#define HW_VER	HW_VER_P3

// Component list
#define COMP_NALA_MCU				(4)
#define COMP_NALA_MUX				(5)
#define COMP_ZAZU_BLE				(6)
#define COMP_ZAZU_ESP32				(7)
#define COMP_NALA_CHARGER_CHIP		(8)

// Info type bit position
#define INFO_TYPE_BPOS_VER			(0)		// Version
#define INFO_TYPE_BPOS_MAC			(1)		// MAC Address
#define INFO_TYPE_BPOS_HWID			(2)		// Hardware ID and Revision
#define INFO_TYPE_BPOS_CHG			(3)		// Charger Chip Data
#define INFO_TYPE_BPOS_SNO			(4)		// Serial number
#define INFO_TYPE_BPOS_PW_INFO		(5)		// Power Information

// POWER SOURCE bit position
#define PW_SOURCE_RSV				(0)		// Reserved
#define PW_SOURCE_EXT				(1)		// External (main or auxilary power)
#define PW_SOURCE_BAT				(2)		// Battery
#define PW_SOURCE_SOLAR				(3)		// Solar

#define SOLAR_POWER_LIMIT			1000000	// Power limit for solar charging mode switch. Unit in uW.

#define FUNC_JUMP_POINT_0	0
#define FUNC_JUMP_POINT_1	1
#define FUNC_JUMP_POINT_2	2
#define FUNC_JUMP_POINT_3	3
#define FUNC_JUMP_POINT_4	4
#define FUNC_JUMP_POINT_5	5

// Sensor mask bit position
#define SENSOR_MASK_BPOS_ANY	0	// Any Sensor
#define SENSOR_MASK_BPOS_CMR	1	// Camera
#define SENSOR_MASK_BPOS_TMP	2	// Temperature Sensor
#define SENSOR_MASK_BPOS_DOOR	3	// Door Sensor

// Sensor mask value
#define SENSOR_MASK_ANY_0		0				// Any Sensor
#define SENSOR_MASK_ANY_F		0xffffffff		// Any Sensor

#define ST_POS_IN_SCN_RESP		4	// Sensor Type position in peripheral scan response (starsts from 0)

/* Monet functions and parameters */
typedef enum {
    IO_WAIT_FOR_DOLLAR,
    IO_GET_FRAME_LENGTH,
    IO_GET_COMMAND,
    IO_GET_CARRIAGE_RETURN
} IoCmdState;

typedef enum {
    IO_GET_NORMAL,
    IO_GET_ESCAPE
} IoEspState;

typedef enum {
    MAIN_UNKNOWN,
    MAIN_NORMAL,
    MAIN_GONE
 } MainState_e;

typedef enum {
    SLEEP_OFF,
    SLEEP_NORMAL,
    SLEEP_HIBERNATE
} SleepState_e;

typedef enum {
    MONET_BB_OFF           = 0,  // Baseband power off
    MONET_BB_ON            = 1,  // Baseband power on
    MONET_V3_OFF           = 2,  // Peripheral power off
    MONET_V3_ON            = 3,  // Peripheral power on
    MONET_BUBX_ON          = 4,  // Shipping mode on
    MONET_BUBX_OFF         = 5,  // Shipping mode off
    MONET_WPIN_ON          = 6,  // Wake off pin is on
    MONET_WPIN_OFF         = 7,  // Wake off pin is off
    MONET_BLATCH_ON        = 8,  // Battery latch is on
    MONET_BLATCH_OFF       = 9,  // Battery latch is off  ("suicide" mode for low battery) (Simba=EM4 mode)
    MONET_BUB_CRITICAL     = 10, // Battery critical mode ("EM3" mode for low battery) (Simba=EM3 mode) SIMBA-7 MNT-2240
	MONET_EXT_CS_OFF       = 11, // External Cargo Sensor power off
    MONET_EXT_CS_ON        = 12, // External Cargo Sensor power on
    MONET_POWER_SWMAIN     = 16, // Power State mask (bit 4)
    MONET_POWER_SWBATT     = 32, // Power State mask (bit 5)
    MONET_POWER_LOW_THRES  = 64, // Power State mask (bit 6)
    MONET_POWER_HIGH_THRES = 128, // Power State mask (bit 7)
    MONET_BB_TOGGLEPWRKEY,
    MONET_BB_START,
    MONET_BB_DONE,
    MONET_BB_ISAWAKE,
} Monet_BB_StartUp;

typedef struct {
    IoCmdState      state;
    IoEspState      escape;
    uint8_t         cmd;
    uint8_t         length;
    uint8_t         remaining;
    uint8_t         checksum;
    uint8_t         data[MAX_COMMAND];
}IoRxFrameStruct;

typedef struct {
    uint8_t         Pin;
    uint32_t        Reload;  //in second
} wd_struct;

typedef struct {
    uint8_t         status;
    uint16_t        Reload;
} gpio_struct;

typedef struct{
    uint8_t         IntPin;
    wd_struct       WD;
    gpio_struct     gConf[NUM_OF_GPIO];
} gpio_conf;

typedef struct{
    uint32_t        Intstatus;
    uint32_t        WDtimer;
    uint16_t        counter[NUM_OF_GPIO];
    uint32_t        gpiolevel;
    uint8_t         WDflag;
} gpio_data;

typedef enum {
    MOTION_STATE_FIRST = -1,
    MOTION_STATE_NONE,
    MOTION_STATE_START,
    MOTION_STATE_STOP,
    MOTION_STATE_BOTH,
    NUM_OF_MOTION_STATES
} ACC_MOTION_STATES_t;

#define MOTION_STATE_DEBOUNCE_S (3)
#define IS_MOTION_STATE_VALID(x) ((((int)x) > MOTION_STATE_FIRST) && (((int)x) < NUM_OF_MOTION_STATES))

typedef enum {
    VIGN_STATE_FIRST = -1,
    VIGN_STATE_NONE,
    VIGN_STATE_ON,
    VIGN_STATE_OFF,
    VIGN_STATE_BOTH,
    NUM_OF_IGNITION_STATES
} VIGN_STATES_t;

typedef struct
{
    uint32_t            threshold; // WCMCU-19
    uint32_t            duration;
    uint32_t            durationTime;
    ACC_MOTION_STATES_t motionMode;
    ACC_MOTION_STATES_t motionState;
    uint8_t             motionStateDebounce;
} ACC_DATA_t;

#define BLE_CHANNEL_NUM_MAX (4)
#define BLE_MAC_ADDRESS_LEN (6)
#define BLE_CONNECTION_STATUS_NOTVALID (0)
#define BLE_CONNECTION_STATUS_NOT_CONNECTED (1)
#define BLE_CONNECTION_STATUS_CONNECTED (2)
#define BLE_CONNECTION_STATUS_MAC_SET (3)
#define BLE_CONN_PARAM_UPDATE_RETRY_MAX (3)
#define BLE_CONN_PARM_CHECK_IN_HEARTBEAT_COUNT (2)

typedef struct
{
    uint8_t connect_status;
    uint16_t handler;
    uint8_t mac_addr[BLE_MAC_ADDRESS_LEN];
} BLE_INFORMATION_t;

typedef struct
{
    uint16_t min_100us;
    uint16_t max_100us;
    uint16_t latency;
    uint32_t timeout_100us;
} CAMERA_CONN_PARAM_t;

#define CAMERA_GLASS_BREAK_DEBOUNCE_INVALID (0xffff)
typedef struct
{
    uint16_t debounce;
    uint16_t pic_num;
    uint8_t  level;
    uint8_t  pin_index;
    uint8_t  wb_mode;
    uint8_t  waitValid;
} CAMERAGLASS_BREAK_t;

#pragma pack(push, 1)
typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} cameraTimeSt;

typedef enum {
    GPIO_REPORT_NONE=0,
    GPIO_REPORT_LOW_TO_HIGH,
    GPIO_REPORT_HIGH_TO_LOW,
    GPIO_REPORT_BOTH
} GpioReport;

typedef struct {
    uint8_t        gpio;
    GpioReport     report;
    uint8_t        debounce;
    uint8_t        debouncing;
    uint8_t        debounceCount;
    uint8_t        gpioCurrent;
    uint8_t        gpioPrevious;
} GpioEvent;

typedef struct
{
    uint8_t channel;
    uint16_t cmd_type;
    uint8_t imgNum;
    uint8_t level;
    uint8_t triggerType;
    cameraTimeSt t;
} captureSt;
#pragma pack(pop)

typedef struct {
//	BYTE      	gpio;
	bool      	direct;
	uint8_t  	config;
} GpioConfig;

typedef struct {
    IoRxFrameStruct     iorxframe;
    atel_ring_buff_t    txQueueU1;
	
	LedConfig       	ledConf[NUM_OF_LED];
	GpioEvent           gpioEv[NUM_OF_EXT_GPIO];
	GpioConfig      	gpioConf[NUM_OF_EXT_GPIO];
	
    uint8_t             AccChipAdd;
    uint8_t             AccChipID;
    uint8_t             IntSrc;
    uint8_t             waitingForMT;
	
	uint16_t			ChargerDelay;
    uint8_t				ChargerStatus;
    uint8_t				PrevChargerStatus;
    uint16_t			ChargerRestartValue;
	
    uint32_t            InMotion;
	uint8_t				AccTestMode;
    ACC_DATA_t          AccData;
	uint32_t			bEnterSysOff;		// Similar setting in Simba called "bEnterEM4"
	uint32_t			wakeBBMode;
	uint8_t				BUBX;

    uint16_t            AdcMain;
    uint16_t            AdcAux;
    uint16_t            AdcSolar;
    uint16_t            AdcBackup;
    uint16_t            AdcBackupRaw;
    uint16_t            AdcBackupAccurate;     //when disable charger
    uint16_t            AdcBatC;           // BUB critical threshold
	uint8_t 			ResetBaseBandDelay;
	uint8_t 			ResetBaseBandCounter;
	uint8_t				waitOnCSTxEmpty;
    uint8_t				waitOnCSTxLen;
    uint8_t             powerStateMask;
    uint16_t            powerLowThreshold;
    uint16_t            powerHighThreshold;
    uint16_t            debounceMainPower; // Holds debounce value
    uint16_t            debounceBattPower; // Holds debounce value
    uint16_t            debounceLowPower;  // Holds debounce value
    uint16_t            debounceHighPower; // Holds debounce value
    uint16_t            debounceMain;      // counts debounce
    uint16_t            debounceBatt;      // counts debounce
    uint16_t            debounceLow;       // counts debounce
    uint16_t            debounceHigh;      // counts debounce
    uint8_t             debounceMainFlag;  // Holds debounce flag
    uint8_t             debounceBattFlag;  // Holds debounce flag
    uint8_t             debounceLowFlag;   // Holds debounce flag
    uint8_t             debounceHighFlag;  // Holds debounce flag
	uint8_t				dataframecounter;
	uint8_t				lastdataframecounter;
	
	uint32_t 			rctime;
    uint32_t            bbofftime;
    uint32_t            SleepAlarm;
    uint32_t            sysTickUnit;
    SleepState_e        SleepState;
    uint8_t             SleepStateChange;
    MainState_e         MainState;
    uint8_t             MainStateChanged;

	uint16_t			AccMode;
    uint8_t             phonePowerOn;
    uint32_t            phoneOffCount;
	uint32_t			phoneLive;
    uint8_t             isExitSleep;
    uint8_t             V3PowerOn;
	uint32_t			lastCount;
	uint32_t			bBattMarkedCritical;    // SIMBAMCU-7 MNT-2240
    uint16_t			BubCriticalThreshold;   // SIMBAMCU-7 MNT-2240
    uint16_t			BubCriticalTime;        // SIMBAMCU-7 MNT-2240
    uint8_t             SensorDelayedTest;
    uint8_t             bbSleepNormalDelay;
    uint8_t             bbSleepConfigDelay;
    uint8_t             bbPowerOffDelay;
    uint8_t             bbPowerOffInprocess;
    uint8_t             bbPowerOnDelay;
    uint8_t             bbPowerOnInprocess;
    uint32_t            deviceBootEnterDelay;
    uint8_t             firstPowerUp;
    uint8_t             resetfromDFU;
	
    uint8_t             uartMdmTXDEnabled;
    uint8_t             uartMdmBaudRate;
    uint32_t            uartMdmInitCount;
    uint32_t            uartMdmDeinitCount;
    uint8_t             uartPeriTXDEnabled;
    uint8_t             uartToBeDeinit;
    uint8_t             uartToBeInit;
    uint8_t             uartTickCount;
    uint8_t             appActive;
    uint32_t            uart_idle_100ms;
    uint32_t            uart_idle_wake;
    uint8_t             mcuWillIdle;

    uint8_t             virtualIgnitionTrigger;
    uint16_t            virtualIgnitionVoltage;
    uint16_t            virtualIgnitionHysteresis;
    uint8_t             virtualIgnitiondDebounce;
    uint8_t             virtualIgnitionDebounceTime;
    uint16_t            virtualIgnitionState;

	uint8_t             bleConnectionStatus;
	uint32_t            bleDisconnectTimeMs;
    uint8_t             bleConnectionEvent;
    uint8_t             bleConnectionEventDelay;
    uint8_t             bleDisconnectEvent;
    uint8_t             ble_pairok_notreported;
    uint8_t             ble_recv_st;
    uint8_t             ble_recv_id;
    uint8_t             ble_mac_addr[BLE_MAC_ADDRESS_LEN];
    uint8_t             ble_peer_mac_addr[BLE_MAC_ADDRESS_LEN];
	uint8_t             ble_scan_mode;
	uint32_t            ble_scan_reset_ms;
    BLE_INFORMATION_t   ble_info[BLE_CHANNEL_NUM_MAX];
//    CAMERA_CONN_PARAM_t ble_conn_param[BLE_CHANNEL_NUM_MAX];
    CAMERA_CONN_PARAM_t ble_conn_param;
    uint16_t            ble_conn_param_update_ok[BLE_CHANNEL_NUM_MAX];
    uint16_t            ble_conn_param_update_retry[BLE_CHANNEL_NUM_MAX];
    uint8_t             ble_conn_param_check_in_heartbeat[BLE_CHANNEL_NUM_MAX];
	uint8_t             bleRecvDataEvent;
    uint8_t             ble_list_sync_wait;

    CAMERAGLASS_BREAK_t glass_break;
    uint16_t            glass_break_time_debounce;
    uint8_t             glass_break_pin_index;
    uint8_t             glass_break_detected;
    uint8_t             glass_break_waitMT;
    uint8_t             glass_break_mcu_start_capture;
    uint32_t            glass_break_date_time;

    uint32_t            sysRealTimeSeconds;
	
#ifdef USE_TILT
    double          	TiltThreshold;                      // Value for TLTC/1     PUMAMCU-136
    uint8_t         	TiltEventTimer;                     // Value for TLTC/2     PUMAMCU-136
    uint8_t         	TiltTimer;                          // countdown timer      PUMAMCU-136
#endif
    uint8_t         	isStarted;         // PUMAMCU-152, flag to prevent removing CPU_HOLD if running on battery with voltage < ADC_BAT_OK
//	uint32_t			interrupt;
	uint8_t           	sleepmode;         // SIMBAMCU-29
	uint8_t				AccDataAvailable;
	uint32_t			bNeedReport;
	uint32_t			bActivated;
	uint8_t             motionDebounce;    // Debounce for 5 seconds of quiet time before clearing flag
	
	bool            	bShouldPollAcc;    // SIMBAMCU-30
    bool           		bI2CisEnabled;     // SIMBAMCU-7
	
	uint8_t				chargerBlink;
	uint8_t         	basebandWakeDelay;

                        // NALAMCU-29 Onewire vars
	uint16_t			Temp10;
	uint16_t			TempLow10;
	uint16_t			TempHigh10;
	uint8_t             TempStatus;    
    uint8_t			    OneWireDisable;
	uint16_t          AdcPhase;
	uint32_t			bEnterEM3;

    uint8_t             uartAliveDebounce;
    uint8_t             uartAliveCount;

    uint8_t             charge_status;   //charge status    NalaMcu-194
    uint8_t             charge_mode2_disable; // mode2 solar panel connect to battery directly

    uint32_t            cameraTurnOnTimes;  //NALAMCU-172.
    uint32_t            cameraTurnOffTimes;
    uint16_t            peripheralAdcBackup;
    uint32_t            peripheralRestartTimes;
    uint32_t            peripheralRestartReason;
    uint32_t            camera_commun_fail_times;
    uint32_t            camera_files_read_fail_times;
    uint32_t            peripheralRunTime;
    int16_t             peripheral_temp_ntc;
    int32_t             peripheral_temp_nordic;
   // uint8_t             scanEnable;

    uint8_t             charge_mode_disable;


} monet_struct;

#if (SUPPORT_BLE_CENTRAL_AND_PERIPH == 1)
typedef struct
{
    uint8_t             bleAdvertiseStatus;
    uint8_t             shouldAdvDebounce;
    uint32_t            startAdvErrcode;
}ble_adv_control_t;

#endif /* (SUPPORT_BLE_CENTRAL_AND_PERIPH == 1) */

#define DEVICE_UART_ALIVE_DEBOUNCE  (60)  //unit Seconds
#define DEVICE_UART_ALIVE_COUNT_LIMIT  ((monet_conf.WD.Reload + 120) / DEVICE_UART_ALIVE_DEBOUNCE)

typedef struct {
    uint8_t             button_power_on;
    uint8_t             reset_from_dfu;
    uint32_t            restartReason;
} device_reset_info_t;

typedef struct {
    uint8_t st;                         // Sensor Type
    uint8_t id;                         // Sensor ID
    uint8_t addr[BLE_MAC_ADDRESS_LEN];	// BLE mac address
    uint8_t tn;                         // Total Number
    uint8_t seq;                        // Sequence Number
} paired_ble_info_t;

#define PAIRED_BLE_INFO_ITEM_SIZE (10)

extern paired_ble_info_t paired_ble_info[BLE_CHANNEL_NUM_MAX];

typedef struct
{
    int16_t tx;
    int16_t rx;
} paired_ble_txrx_power_t;

extern paired_ble_txrx_power_t paired_ble_txrx_power[BLE_CHANNEL_NUM_MAX];

#define BLE_LINK_TARGET_DEALY_MAX_MS (8000)
#define BLE_LINK_TARGET_ATTEMPT_COUNT (3)
typedef struct {
    paired_ble_info_t   info;
    uint8_t             action;             // 1: establish;      0: destory
    uint8_t             result;             // 1: success;        0: fail
    uint8_t             noreported;         // 1: not reported    0: reported
    uint8_t             report_delay;       // 1: report delayed  0: report normally
    uint32_t            report_delay_ms;
    uint8_t             attempt;
    uint8_t             ready;
    uint8_t             conn_handle_valid;
    uint16_t            conn_handle;
    uint8_t             disconnected;
    uint8_t             report_intime;
} ble_link_target_t;

extern ble_link_target_t ble_link_target;

typedef enum
{
  HOLD = 0,
  RELEASE = 1
} FunCtrl;

typedef struct {
	uint8_t affinter;		 //maximum free fall interval
	uint8_t affcount;		 //number of free falls
	uint8_t adur[2];		 //duration
	uint8_t ar[2];			 //rate
	uint8_t at[2];			 //threshold
	uint32_t phonetimer;   //turn on phone RTC time
	uint32_t phoneV;       //turn on phone mimum voltage
    uint8_t bShippingMode;	 //Shipping mode when set
    uint8_t bAllowPowerKey; //Allow/Block powerkey emission on MCU only reset (PUMAMCU-65)
    uint8_t bmcuRst;        // mcu reset
    uint8_t resetAfterUpdate;  // SIMBAMCU-28
} config_struct;

typedef enum
{
	COM_METHOD_ZAZU_NONE = 0,	// None
	COM_METHOD_ZAZU_BLE,		// Use BLE
	COM_METHOD_ZAZU_CAN,		// Use CAN bus
} com_method_zazu_t;			// Communication method with Zazu board

typedef enum
{
	SOLAR_CHG_MODE1 = 1,	// Mode 1, Solar power --> Charger chip --> battery
	SOLAR_CHG_MODE2,		// Mode 2, Solar power --> battery. Charger chip is bypassed
} solar_chg_mode_t;			// Solar charging mode.

typedef enum
{
    CHARGE_STATUS_SOLAR_MODE_1_PROCESS = 0, // changer status when in solar mode 1
    CHARGE_STATUS_SOLAR_MODE_1_FULL,        // changer status when battery fully charged in solar mode 1
    CHARGE_STATUS_SOLAR_MODE_2,             // changer status when in solar mode 2
    CHARGE_STATUS_SOLAR_LOWPOWER,           // changer status when in solar low power
    CHARGE_STATUS_EXTERNAL_POWER_PROCESS,   // changer status when in external power
    CHARGE_STATUS_EXTERNAL_POWER_FULL,      // changer status  when battery fully charged in external power
    CHARGE_STATUS_NO_SOURCES,
} charge_status_e;

#if (SUPPORT_BLE_CENTRAL_AND_PERIPH == 1)

#if (SUPPORT_BLE_BEACON == 1)
#define MODULE_SN_LENGTH 8
typedef enum {
    TX_LEVEL_P8,
    TX_LEVEL_P4,
    TX_LEVEL_0,
    TX_LEVEL_N4,
    TX_LEVEL_N8,
    TX_LEVEL_N12,
    TX_LEVEL_N16,
    TX_LEVEL_N20,
    TX_LEVEL_LAST
} TX_Power_Level;

typedef struct {
    uint8_t  txRssi;
}tx_power_struct;

void beacon_advertising_update(uint32_t interval, uint16_t duration, uint8_t update, uint8_t mode);
void ble_aus_advertising_stop(void);
uint32_t pf_tx_power_set(int8_t tx_level);
uint32_t pf_adv_start(uint8_t adv_api);
extern ble_adv_control_t   adv_control;
void ble_advertisement_status_inform(uint8_t state);  //code from simba
#endif/* (SUPPORT_BLE_BEACON == 1) */

#endif /* (SUPPORT_BLE_CENTRAL_AND_PERIPH == 1) */

#define DEVICE_RESET_SOURCE_RESETPIN_DETECTED_POS      (0)
#define DEVICE_RESET_SOURCE_WATCHDOG_DETECTED_POS      (1)
#define DEVICE_RESET_SOURCE_SOFTRESET_DETECTED_POS     (2) 
#define DEVICE_RESET_SOURCE_LOCKUP_DETECTED_POS        (3)
#define DEVICE_RESET_SOURCE_SYSOFF_DETECTED_POS        (16)        
#define DEVICE_RESET_SOURCE_LPCOMP_DETECTED_POS        (17)
#define DEVICE_RESET_SOURCE_DIF_DETECTED_POS           (18)
#define DEVICE_RESET_SOURCE_NFC_DETECTED_POS           (19)
#define DEVICE_RESET_SOURCE_VBUS_DETECTED_POS          (20)

#define DEVICE_RESET_SOURCE_BIT_INVALID                (0)
#define DEVICE_RESET_SOURCE_BIT_VALID                  (1)
#define DEVICE_RESET_SOURCE_BIT_MASK(P)                (0x0001 << P)
#define DEVICE_RESET_SOURCE_BIT_GET(S, P)              ((S >> P) & 0x0001)
#define DEVICE_RESET_SOURCE_BIT_IS_VALID(S, P)         (DEVICE_RESET_SOURCE_BIT_GET(S, P) == DEVICE_RESET_SOURCE_BIT_VALID)
#define DEVICE_RESET_SOURCE_BIT_IS_INVALID(S, P)       (DEVICE_RESET_SOURCE_BIT_GET(S, P) == DEVICE_RESET_SOURCE_BIT_INVALID)

extern monet_struct monet_data;
extern gpio_conf    monet_conf;
extern gpio_data    monet_gpio;
extern config_struct config_data;
extern device_reset_info_t reset_info;

extern uint8_t     gAdcBatCounter;      // SIMBAMCU-30

extern void ars_PollAccStream(void);

void InitApp(uint8_t resetfromDFU);         /* I/O and Peripheral Initialization */
void GetRxCommand(uint8_t rxByte);
void GetRxCommandEsp(uint8_t RXByte);

int8_t atel_io_queue_process(void);
void uart_peri_rx_process(void);
void mcu_start_capture_process(void);
void monet_gpioEventGenerator(void);
void atel_timerTickHandler(uint32_t tickUnit_ms);
void mnt_accHeartbeat(void);
//void accInterruptHandle(void);
void adc_conv_prepare(void);
void atel_adc_converion(void);
void adc_conv_proc(void);
void adc_conv_stop(void);
uint16_t adc_to_vol_conv(uint16_t adc_value, double res_factor);
void atel_timer1s(void);
void monet_timer30s(void);
void flashOrange(uint8_t dur1, uint8_t dur2);
bool no_external_power(void);
bool only_solar_power(void);
bool solar_charge_invalid(void);
void flashChargingStatus(void);
void atel_uart_restore(void);
void isMdmNeedTobeWakeup(uint32_t tick_ms);

void HandleRxCommand(void);
void BuildFrame(uint8_t cmd, uint8_t * pParameters, uint8_t nSize);
void send_to_mdm(uint8_t * p_data, uint8_t len);
uint16_t ble_channel_get_from_mac(uint8_t *p_addr);
uint16_t ble_channel_get_from_handler(uint16_t handler);
uint16_t ble_connected_handler_get_from_channel(uint16_t channel);
uint16_t ble_information_set(uint16_t handler, uint8_t status, uint8_t *p_addr);
void ble_connection_channel_init(void);
void ble_connected_channel_clear(void);
uint16_t ble_connected_channel_num_get(void);
void ble_link_action_result_inform(uint8_t inst_id, uint8_t sensor_typte, uint8_t action, uint8_t result);
void ble_connection_status_inform(uint16_t channel, uint8_t state);
void monet_bleScommand(uint8_t* pParam, uint8_t Length, uint8_t st, uint8_t id);
bool ble_aus_ready_state_get_c(void);

void init_config(void);
void gpio_init_pin(uint8_t pin_index);
void gpio_init(void);
//void setdefaultConfig(uint8_t bat_en);
void setdefaultConfig(void);
void configGPIO(int index, uint8_t status);
void SetGPIOOutput(uint8_t index, bool Active);

void IncreaseCount(uint16_t * value);
void ble_conn_param_updated_check(void);
void device_ble_status_report(uint32_t delta);
void device_bootloader_enter_dealy(uint32_t delta);
void CheckInterrupt(void);
// void clock_hfclk_release(void); //NALAMCU-186//
// void clock_hfclk_request(void); //NALAMCU-186//

void MCU_TurnOn_MDM(void);
void MCU_Wakeup_MDM(void);
void MCU_Sleep_MDM(void);
void MCU_Sleep_APP(void);
void MCU_Wakeup_APP(void);
void ModemPowerStateMachine(void);
void mnt_SendMotionAlert(uint8_t status);
uint8_t isMDMWakingMCU(void);
void MCU_TurnOff_MDM(void);
void disableEventIReadyFlag(void);
bool mp_isEventIReadyFlag(void);
void CheckPowerState(void);
void BatteryPowerHoldEn(FunCtrl Status);

void atel_ImuDataInit(void);
void CheckForMotion(void);
void CheckVirtualIgnition(void);
void pairing_state_check(void);
void CheckGlassBreakEvent(uint8_t debounce);
uint8_t glass_break_pin_valid(void);

void setShouldPollAcc(bool value);

void flashLED(void);
void monet_startLED(uint8_t* pParam, uint8_t status);

uint8_t CheckMainPower(uint16_t volts);

int32_t chip_temp_get(void);

void mux_mcu_update_proc(void);
void mux_mcu_update_proc2(void);

void resetADCcounter(void);
void handleAccInterrupt(void);

com_method_zazu_t com_method_zazu_get(void);
void com_method_zazu_set(com_method_zazu_t);

void cs_prov_test_mode_set(void);
void cs_prov_test_mode_clear(void);
bool is_in_cs_prov_test_mode(void);

solar_chg_mode_t solar_chg_mode_get(void);
void solar_chg_mode_set(solar_chg_mode_t mode);

void mode_selection_proc(void);
void mode_selection_subproc(void);
void timer_solar_chg_mode_restart(void);
void timer_solar_chg_mode_intvl_set(uint32_t value);
uint32_t timer_solar_chg_mode_intvl_get(void);
void timer_solar_chg_mode_stop(void);
void solar_chg_mode_check_proc(void);
void solar_chg_mode_select(uint8_t jump_point);
void solar_chg_mode_proc(void);
bool is_solar_vol_low(void);
void solar_chg_stage_switch_3_to_2(void);

bool is_main_aux_vol_low(void);
void main_aux_power_check(void);

void scan_init_with_param(bool connect_if_match, bool no_pair, const uint8_t *p_ble_addr);

typedef void (*leds_blink_callback_t)(void);
void leds_ctrl_take(void);
bool leds_ctrlled_by_mcu(void);
void leds_ctrl_release(void);
void leds_blink(uint32_t interval, uint32_t duration);
bool are_leds_blinking(void);
void inform_mdm_to_restore_leds_state(void);
void leds_blink_callback_enable(leds_blink_callback_t func);

bool scan_param_set(uint32_t scan_window_ms, uint32_t scan_interval_ms, uint32_t scan_duration_ms);
uint32_t scan_param_dur_get(void);
void pair_resp_to_mdm_send(uint8_t result, uint8_t inst_id, const uint8_t *p_sensor_mask, uint8_t sensor_mask_len, const ble_gap_addr_t *p_addr);

uint32_t scan_start(void);
void scan_stop(void);

const ble_nus_c_t *ble_nus_handle_get(void);

void monet_bleCcommand_QI(void);
void monet_DDcommand_decode(uint8_t* pParam, uint8_t Length);
void monet_DDcommand_decode_byte(uint8_t* pParam, uint8_t Length);
void monet_DDcommand_decode_string(uint8_t* pParam, uint8_t Length);

#endif /* USER_H_ */
