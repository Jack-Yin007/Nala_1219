/*
 * system.c
 *
 *  Created on: Dec 12, 2015
 *      Author: J
 */

#include "user.h"
#include "system.h"
#include "atel_util.h"
//#include "mcu_product.h"
#include "platform_hal_drv.h"
#include "nrf_power.h"
#include "mp2762a_drv.h"
#include "acc_simba_lis2dh12.h"
#include "mux_mcu.h"

extern uint16_t wdfired;

uint32_t gresetCause = 0;

uint32_t egpio_config[NUM_OF_EXT_GPIO] = 
{
	GPIO_BLE_GPIO1,
    GPIO_BLE_GPIO2,
    GPIO_BLE_GPIO3,
	GPIO_BLE_RELAY,
	GPIO_ONE_BUS_SLPZ,
	GPIO_CHRG_INT,
#if (HW_VER >= HW_VER_P3)
	GPIO_CHRG_INT,		// Here used repeated valued.
#else
	GPIO_CHRG_ACOK,
#endif
	GPIO_BLE_Tamper,
};

uint32_t led_config[] =
{
    GPIO_LED_RED,	    //LED 3
	GPIO_LED_GREEN,		//LED 2
	GPIO_LED_ORANGE,	//LED 1
};

extern uint32_t ble_aus_advertising_start(void);
extern uint32_t gBWD;
extern uint8_t resetfromSystemOff;

void pic_turnOnBaseband(void)
{
    if(monet_data.bBattMarkedCritical || (resetfromSystemOff && !isOtherPowerSource()))
    {
        return;
    }
    
    if (monet_data.bbPowerOffInprocess != 0)
    {
        monet_data.bbPowerOnInprocess = 1;
        return;
    }

    monet_data.bbPowerOnInprocess = 0;

	clock_hfclk_request(); //NALAMCU-186//::Need to manually switch to the external 32 MHz crystal

    if (monet_data.SleepState == SLEEP_HIBERNATE)	// Recover from Hibernate mode
    {
        monet_data.SleepState = SLEEP_OFF;
        monet_data.SleepStateChange = 1;
        // WARNING: memset(&monet_gpio, 0, sizeof(monet_gpio));
		init_config();
//		gpio_init();
		pf_cfg_recover_from_hibernation();
		if (monet_data.uartPeriTXDEnabled == 0)
		{
//			pf_uart_peri_init(9, 0);	// Baudrate 115200, stop-bit 1
			pf_uart_peri_init(5, 0);	// Baudrate 9600, stop-bit 1
		}
    }
    pf_uart_mdm_init(UART_BAUDRATE_SELECT, 0);

    pf_mdm_pwr_init();
    // if (!monet_data.phonePowerOn)
    {
        // monet_data.phonePowerOn = 1;
        if (monet_data.resetfromDFU == 0)
        {
            NRF_LOG_RAW_INFO("MCU_TurnOn_MDM resetfromDFU == 0.\r");
            NRF_LOG_FLUSH();
            monet_data.bbPowerOnDelay = 2;
            pf_mdm_pwr_ctrl(true);
            pf_mdm_pwr_key_ctrl(false);
        }
        else
        {
            NRF_LOG_RAW_INFO("MCU_TurnOn_MDM resetfromDFU == 1.\r");
            NRF_LOG_FLUSH();
            monet_data.bbPowerOnDelay = 0;
            MCU_TurnOn_MDM();
            MCU_Wakeup_MDM();
        }
        monet_data.SleepAlarm       = 0; // Disable the sleep timer
        monet_data.bbPowerOffDelay  = 0;
    }
	
	gBWD = MIN_WD_TIMER_DFT;
	monet_gpio.WDtimer = gBWD;

	resetADCcounter();
	
	setShouldPollAcc(true);
	monet_gpio.Intstatus |= MASK_FOR_BIT(INT_WAKEUP);
}

void pic_turnOffBaseband(void)
{
	uint8_t pParamO[4];
	pParamO[0] = 0;
    BuildFrame('z', pParamO, 1);
	
	pic_turnOffV3();    // PUMA-155, Force to turn off V3
	
    if (0 == monet_data.bbPowerOffDelay)
    {
        monet_data.bbPowerOffDelay = 2;
    }
//	disableEventIReadyFlag();
	disableEventIReadyFlag();
	
	monet_data.phoneLive=0;

	resetADCcounter();
	
	setShouldPollAcc(false);
	monet_data.AccDataAvailable = 0;

    // In factory, when external power is connected, do not enter hibernation mode.
    // if (monet_gpio.Intstatus & MASK_FOR_BIT(INT_BAT_OFF))	// External power is plugged in
    // {
    // 	monet_data.SleepAlarm = 1;	// Modify Modem wakeup timer. Wakeup Modem immediately after turning it off
    // }

    monet_gpio.Intstatus= 0;
	// Ensure the Cell and GPS LEDs are set OFF during sleep
	pic_setLedSleepMode();
}

void systemreset(uint8_t flag)
{
    // if(flag && (monet_conf.WD.Pin != GPIO_TO_INDEX(GPIO_NONE))) {
    //     SetGPIOOutput(monet_conf.WD.Pin, 0);
    // }
    // else {
        wdfired = 1;
        while (1);		// Waiting for BLE Watchdog reset, 20200910, QGH
    // }
}

void pic_setLedSleepMode(void) { // SIMBAMCU-22
    pic_turnOffLed(0); // Cell LED
    pic_turnOffLed(1); // GPS LED
}

void pic_turnOnV3(void)
{
	uint8_t pParamO[2];
	
	pParamO[0] = 1;
	pParamO[1] = 0xFF;
	BuildFrame('z', pParamO, 2);
	
	mcu_mux_set(MUX_SETTING_1);		// Set MCU MUX to [Motherboard <==> Internal Cargo sensor]
	nrf_gpio_pin_set(VDD_MDM_EN);
	nrf_gpio_pin_set(CS_3V3_EN);	// Enable internal CS power
	monet_data.V3PowerOn = 1;
//	if (monet_data.ChargerStatus == CHARGER_ON) {
//		monet_data.PrevChargerStatus = monet_data.ChargerStatus;
//	}
//	setChargerOff();
	
//	{
//		pic_setPeriUartTxHigh();
//		GPIO_PinOutSet(gpio_config[DC_2_EN].port, gpio_config[DC_2_EN].pin);
//		GPIO_PinOutSet(gpio_config[V3_EN].port, gpio_config[V3_EN].pin);
//		// Dummy read of the register
//		pParamO[0] = USART1->RXDATA;
//		// Enable the interrupt
//	//		USART_IntEnable(USART1, USART_IEN_RXDATAV);
//		monet_data.V3PowerOn = 1;
//		if (monet_data.ChargerStatus == CHARGER_ON) {
//			monet_data.PrevChargerStatus = monet_data.ChargerStatus;
//		}
//		setChargerOff();
//	}
}

void pic_turnOffV3(void)
{
	uint8_t pParamO[4];
	
	pParamO[0] = 0xf3;
	BuildFrame('z', pParamO, 1);
	
	mcu_mux_set(MUX_SETTING_0);		// Set MCU MUX to default [Motherboard <==> External 12 Pin connector]
	if (!monet_data.phonePowerOn && (monet_gpio.counter[GPIO_VDD_MDM_EN] == 0))		// If Modem is not on, power on it
        monet_gpio.counter[GPIO_VDD_MDM_EN] = (V3_MDM_PW_DELAY_MS + TIME_UNIT - 1) / TIME_UNIT;	//50;	// Value in Simba code here is 50, means 50*100 ms
	nrf_gpio_pin_clear(CS_3V3_EN);	// Disable internal CS power
	monet_data.V3PowerOn = 0;
	
//    if (monet_data.PrevChargerStatus == CHARGER_ON) {
//        monet_data.PrevChargerStatus = CHARGER_OFF; // Clear the state so it does prematurely turn on when not needed
//        setChargerOn();
//    }
}

void pic_turnOnLed(int led)
{
	pf_gpio_write(led_config[led], 0);
}


void pic_turnOffLed(int led)
{
	pf_gpio_write(led_config[led], 1);
}

void pic_toggleLed(int led)
{
	pf_gpio_write(led_config[led], (pf_gpio_read(led_config[led]) > 0) ? 0 : 1);
}

bool pic_IsLedOn(int led)
{
	return (pf_gpio_read(led_config[led]) > 0) ? false : true;
}

void charger_power_off(void)
{
	if (pf_gpio_read(GPIO_CHRG_SLEEP_EN) != 0 && 
		pf_gpio_read(GPIO_CHRGIN_PWR_EN) != 0)		// The charger is disabled
		return;
	if (monet_data.charge_mode_disable)
	{
		goto gpio_write1;
	}
	pf_gpio_write(GPIO_CHRGIN_PWR_EN, 1);
	
gpio_write1:	
	pf_gpio_write(GPIO_CHRG_SLEEP_EN, 1);	// Set high to disable charger chip
}

void charger_power_on(void)
{
	if (pf_gpio_read(GPIO_CHRG_SLEEP_EN) != 0 && 
		pf_gpio_read(GPIO_CHRGIN_PWR_EN) != 0)	// The charger is disabled
	{
		pf_gpio_write(GPIO_CHRG_SLEEP_EN, 0);	// Set low to enable charger chip
		pf_gpio_write(GPIO_CHRGIN_PWR_EN, 0);
		nrf_delay_ms(10);	// Left time for charger chip to initialize
		mp2762a_init();
	}
}

bool is_charger_power_on(void)
{
	if (pf_gpio_read(GPIO_CHRG_SLEEP_EN) != 0 && 
		pf_gpio_read(GPIO_CHRGIN_PWR_EN) != 0)		// If pin is high, charger is off
		return false;
	return true;
}

void setChargerOff(void)
{
//	GPIO_PinOutSet(CHG_EN_PORT, CHG_EN_PIN);
	charger_power_off();
//	mp2762a_charge_ctrl(false);
	monet_data.ChargerStatus = CHARGER_OFF;
	monet_data.ChargerDelay = 0;
}

void setNtcOn(void)
{
	if (pf_gpio_read(GPIO_CHRGIN_PWR_EN) != 0)
	{
		pf_gpio_write(GPIO_CHRGIN_PWR_EN,0);
		nrf_delay_ms(10);	// Left time for charger chip to initialize
	}
}

void setChargerOn(void)
{
//	GPIO_PinOutClear(CHG_EN_PORT, CHG_EN_PIN);
	charger_power_on();
	mp2762a_charge_ctrl(true);
	monet_data.ChargerStatus = CHARGER_ON;
	monet_data.ChargerDelay = 0;
}

//static uint8_t charger_state_bf_tn_off_chg = CHARGER_ON;	// Charger state before turnning off charger
static uint8_t charger_state_record_check_cnt = 1;
static uint8_t charger_state_recover_check_cnt = 0;

void charger_state_record(void)
{
	if (charger_state_record_check_cnt)		// Only run one time
	{
		charger_state_record_check_cnt--;
//		charger_state_bf_tn_off_chg = monet_data.ChargerStatus;
		charger_state_recover_check_cnt = 1;
	}
}

// @External_power_on_when_CS_on
// This is for the case that
// 1 When no external power, charger is turned off, Cargo Sensor is off then turned on.
//    Because the previous state of charger is off, next time turning CS off function would keep the charger off state.
// 2 But when external power is connected while CS is just on, because monet_data.V3PowerOn is 1, charger would not be turned on either.
// Combine condition1 and condition2, they will make the charge always off and never recover.
// To solve this problem, besides the condition (monet_data.V3PowerOn == 0), condition (monet_data.V3PowerOn == 1) should also be checked.
// In condition (monet_data.V3PowerOn == 1), monet_data.PrevChargerStatus should be set to CHARGER_ON.
// When CS is turned off, charger is turned on again.

void charger_state_recover(void)
{
	if (charger_state_recover_check_cnt)	// Only run one time
	{
		charger_state_recover_check_cnt--;
//		if (/*charger_state_bf_tn_off_chg == CHARGER_ON && 
//			*/monet_data.V3PowerOn == 0)
//			setChargerOn();
//		else if (monet_data.V3PowerOn == 1)	// For the case @External_power_on_when_CS_on
//		{
//			if (monet_data.PrevChargerStatus != CHARGER_ON)
//				monet_data.PrevChargerStatus = CHARGER_ON;
//		}
		setChargerOn();
		charger_state_record_check_cnt = 1;
	}
}

#define CHG_STATE_NOT_SPECIFIED		0
#define CHG_STATE_CHG_IN_PROCESS	1
#define CHG_STATE_CHG_COMPLETE		2
#define CHG_STATE_EXCEPTION			3

// @Battery_NTC_resistance_value
// This value is for P2 new board (maybe P3 board) to check the connectivity of battery.
// If the NTC resistance exceeds this value, it means the battery is not connected.
// This method is not applicable for P2 or P1 board, because the charger NTC circuit is changed on P2 new board.
// The resistance value comes from charger, the value is <300 kohm when NTC resistor in -40 degree Celsius,
//   and ~5 megohm when battery is disconnectd.
#define BAT_NTC_OPEN_LIMIT	1000000	// Battery NTC resistor open limit

#define BAT_VOL_LIMIT			8400.0		// Unit in mV
#define CHG_CUR_LIMIT			200.0		// Unit in mA
#define BAT_CHG_DONE_DUR_MAX	86400		// Unit in seconds. 24 hours
#define BAT_CHG_DONE_DUR_LIMIT	3600		// Unit in seconds. 1 hours
static uint32_t bat_chg_done_dur = 0;	// Battery charging done duration. Unit in seconds.

// Get charge state.
// To keep compatibility, charge state definition keeps same with Simba code.
//                                          stat1  stat2  stat
// Not Specified                              0      0      0
// Charge-in-progress                         0      1      1
// Charge complete                            1      0      2
// Charge suspend, timer fault, overvoltage,  1      1      3
//   sleep mode, battery absent
// 
// Other value would be ignored by caller.
uint8_t getChargerState(void)
{
	uint8_t chg_state = 0;
	uint8_t fault_type1 = 0;
	uint8_t fault_type2 = 0;
	uint8_t fault_type3 = 0;
	uint8_t fault_type4 = 0;
	uint8_t fault_type5 = 0;
	double ntc_res = 0.0;
	double bat_vol = 0.0;
	double chg_cur = 0.0;
	
	if (is_charger_power_on() != true)
	{
		NRF_LOG_RAW_INFO("getChargerState(), chg state %u, charger chip is not on\r", CHG_STATE_EXCEPTION);
		return CHG_STATE_EXCEPTION;
	}
	
	if (mp2762a_fault_watchdog_get(&fault_type1) == false)
		return CHG_STATE_NOT_SPECIFIED;
	if (mp2762a_fault_otg_get(&fault_type2) == false)
		return CHG_STATE_NOT_SPECIFIED;
	if (mp2762a_fault_chg_get(&fault_type3) == false)
		return CHG_STATE_NOT_SPECIFIED;
	if (mp2762a_fault_bat_get(&fault_type4) == false)
		return CHG_STATE_NOT_SPECIFIED;
	if (mp2762a_fault_ntc_get(&fault_type5) == false)
		return CHG_STATE_NOT_SPECIFIED;
	if (bat_ntc_res_get(&ntc_res) != true)
		return CHG_STATE_NOT_SPECIFIED;
	
	if (fault_type1 != MP2762A_WD_FAULT_NORMAL ||
		fault_type2 != MP2762A_OTG_FAULT_NORMAL ||
		fault_type3 != MP2762A_CHG_FAULT_NORMAL ||
		fault_type4 != MP2762A_BAT_FAULT_NORMAL ||
		fault_type5 == MP2762A_NTC_FAULT_NTC_COLD || 
		fault_type5 == MP2762A_NTC_FAULT_NTC_COOL)
	{
		NRF_LOG_RAW_INFO("getChargerState(), chg state %u, charger fault %u %u %u %u %u\r", 
						CHG_STATE_EXCEPTION, fault_type1, fault_type2, fault_type3, fault_type4, fault_type5);
		return CHG_STATE_EXCEPTION;
	}
	
	if (ntc_res > BAT_NTC_OPEN_LIMIT)	// Battery is not connected. Refer to @Battery_NTC_resistance_value for more info.
	{
		NRF_LOG_RAW_INFO("getChargerState(), chg state %u, NTC Res %u\r", 
						CHG_STATE_EXCEPTION, (uint32_t)ntc_res);
		return CHG_STATE_EXCEPTION;
	}
	
	bat_vol = mp2762a_bat_vol_get();
	chg_cur = mp2762a_bat_chg_cur_get();
	if (bat_vol >= BAT_VOL_LIMIT && chg_cur < CHG_CUR_LIMIT)
	{
		if (bat_chg_done_dur < BAT_CHG_DONE_DUR_MAX)	// Make sure it will not overflow
			bat_chg_done_dur++;
	}
	else
		bat_chg_done_dur = 0;
	NRF_LOG_RAW_INFO("getChargerState bat_vol %u mV, chg_cur %u mA, bat_chg_done_dur %u\r", (uint32_t)bat_vol, (uint32_t)chg_cur, bat_chg_done_dur);
	
	if (mp2762a_charge_state_get(&chg_state) == false)
		return CHG_STATE_NOT_SPECIFIED;
	if (chg_state == MP2762A_CHG_STAT_NOT_CHG ||
		chg_state == MP2762A_CHG_STAT_CHG_TMN)
	{
		NRF_LOG_RAW_INFO("getChargerState(), chg state ret %u, chg state read %u\r", 
						CHG_STATE_CHG_COMPLETE, chg_state);
		return CHG_STATE_CHG_COMPLETE;
	}
	else if (chg_state == MP2762A_CHG_STAT_PRE_CHG || 
			 chg_state == MP2762A_CHG_STAT_FST_CHG)
	{
		NRF_LOG_RAW_INFO("getChargerState(), chg state ret %u, chg state read %u\r", 
						CHG_STATE_CHG_IN_PROCESS, chg_state);
		if (bat_chg_done_dur > BAT_CHG_DONE_DUR_LIMIT)
			return CHG_STATE_CHG_COMPLETE;
		else
			return CHG_STATE_CHG_IN_PROCESS;
	}
	else // To avoid compiler warning
		return CHG_STATE_NOT_SPECIFIED;
}

void HandleReset(void)
{
	gresetCause = nrf_power_resetreas_get();
	nrf_power_resetreas_clear(gresetCause);
	
//	gresetCause = RMU->RSTCAUSE;
//	RMU_ResetCauseClear();
//    
//    if ((gresetCause & RMU_RSTCAUSE_PORST) || 
//        (gresetCause & RMU_RSTCAUSE_BODREGRST)) 
//    { // SIMBAMCU-34
//        // Reset the one-time powerkey block on power up
//        SetConfiguration(PARAM_ALLOW_POWERKEY, ALLOW_POWER_KEY);
//    }

//	/* Check if reset was caused by a wakeup from EM4 */
//	if (gresetCause & RMU_RSTCAUSE_EM4WURST)
//	{
//	    /* The reset is a wakeup from EM4 */
//	    if ( GPIO->EM4WUCAUSE & GPIO_EM4WUCAUSE_EM4WUCAUSE_A0 )
//	    {
//	      /* The wakeup was triggered by GPIO. Increase counter. */
//	    	IncConfiguration(PARAM_EM4);
//            // If only watchdog reset block the powerkey so we don't kill the module
//            if (!(gresetCause & ~(RMU_RSTCAUSE_EM4WURST))) { // PUMAMCU-95
//                SetConfiguration(PARAM_ALLOW_POWERKEY, BLOCK_POWER_KEY);
//            }
//            monet_data.isStarted = 1;                 // PUMAMCU-157, baseband started 
//            monet_data.interrupt |= MASK_FOR_BIT(INT_TAMPER_WAKEUP); // Allow main loop to start baseband
//        }

//	 }
//	else if(gresetCause & RMU_RSTCAUSE_WDOGRST)
//	{

//		*((uint32_t*)(RESETJUMPTOLOADER_ADDR) + 1) = gresetCause;
////		IncConfiguration(PARAM_WD);
//        // If only watchdog reset block the powerkey so we don't kill the module
//        if (!(gresetCause & ~(RMU_RSTCAUSE_WDOGRST))) { // PUMAMCU-95
//            SetConfiguration(PARAM_ALLOW_POWERKEY, BLOCK_POWER_KEY);
//            monet_data.isStarted = 1;    // PUMAMCU-152, came back from hardware reset
//        }
//  }
}

extern void advertising_stop(void);

void monet_setGPIOLowPowerMode(void)
{
	int i = 0;
	
	pf_adc_deinit();
	pf_systick_stop();
	ion_accRegInit2(LIS_ODR_POWERDOWN, 0, 0, 0, 1);
	if (monet_data.bI2CisEnabled == true)
		pf_i2c_uninit();
	if (monet_data.uartPeriTXDEnabled == 1)
		pf_uart_peri_deinit();
	if (monet_data.uartMdmTXDEnabled == 1)
		pf_uart_mdm_deinit();

	clock_hfclk_release(); //NALAMCU-186//::Need to manually switch to the external 32 MHz crystal

	pf_gpio_write(GPIO_BLE_RELAY, 0);
	pf_gpio_write(GPIO_MDM_PWR_KEY, 0);
	pf_gpio_write(GPIO_RS232_EN, 0);
	pf_gpio_write(GPIO_BLE_CAN_PWR_EN, 0);
	pf_gpio_write(GPIO_CS_12V_EN, 0);
//	pf_gpio_write(GPIO_CHRGIN_PWR_EN, 1);	// Set high to disable external power path to charger chip
//	pf_gpio_write(GPIO_CHRG_SLEEP_EN, 1);	// Set high to disable battery power path to charger chip
	setChargerOff();
#if (HW_VER >= HW_VER_P3)
	pf_gpio_write(SOLAR_CHARGE_SWITCH, 0);
#endif
	pf_gpio_write(GPIO_ST_MCU_PWR_EN, 0);
	pf_gpio_write(GPIO_CS_3V3_EN, 0);
	pf_gpio_write(GPIO_VDD_MDM_EN, 0);
	pf_gpio_write(GPIO_DC_DC_9V5_EN, 0);
	pf_gpio_write(GPIO_LED_ORANGE, 1);
	pf_gpio_write(GPIO_LED_GREEN, 1);
	pf_gpio_write(GPIO_LED_RED, 1);
	pf_gpio_write(GPIO_ONE_BUS_SLPZ, 0);
//	pf_gpio_write(GPIO_ST_UART1_TO_UART2_EN, 0);
//	pf_gpio_write(GPIO_ST_UART1_TO_UART3_EN, 0);
//	pf_gpio_write(GPIO_ST_UART2_TO_UART3_EN, 0);
	pf_gpio_write(GPIO_BAT_ADC_TMP_EN, 0);
	nrf_delay_ms(100);	// Wait for voltage decline
	for (i = 0; i < NUM_OF_GPIO; i++)
	{
		if (i == GPIO_CHRG_SLEEP_EN ||
			i == GPIO_CHRGIN_PWR_EN)	// Skip GPIO_CHRG_SLEEP_EN and GPIO_CHRGIN_PWR_EN to keep it high
			continue;
		gpio_deinit(i);
	}
	if (nrfx_gpiote_is_init() == true)
		nrfx_gpiote_uninit();
//	advertising_stop();
	app_timer_stop_all();
}

void shipping_mode_wkp_src_config(void)
{
	uint32_t err_code;
	err_code = sd_softdevice_disable();
	APP_ERROR_CHECK(err_code);

	NRF_POWER->GPREGRET = ( (RESET_MEMORY_TEST_BYTE << 4) | 0x2);
	
	if (nrfx_gpiote_is_init() != true)
		nrfx_gpiote_init();
	configGPIO(GPIO_BLE_Tamper, PIN_STATUS(1, 1, 1, 0));
	nrf_delay_ms(10);
}

uint8_t isOtherPowerSource()
{
    return adc_to_vol_conv(monet_data.AdcMain, VOL_MAIN_FACTOR) > ADC_MAIN_TH || 
           adc_to_vol_conv(monet_data.AdcSolar, VOL_MAIN_FACTOR) > ADC_MAIN_TH || 
           adc_to_vol_conv(monet_data.AdcAux, VOL_MAIN_FACTOR) > ADC_MAIN_TH;
}

// Check Main/Aux power supply state
// If there are Main OR Aux power, return true; else, return false
bool main_aux_power_existing(void)
{
    return adc_to_vol_conv(monet_data.AdcMain, VOL_MAIN_FACTOR) > ADC_MAIN_TH || 
           adc_to_vol_conv(monet_data.AdcAux, VOL_MAIN_FACTOR) > ADC_MAIN_TH;
}
