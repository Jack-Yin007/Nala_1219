// Maximum Power Point Tracking (MPPT) implementation

#include "solar_mppt.h"
#include "stdbool.h"
#include "stdint.h"
#include "math.h"
#include "mp2762a_drv.h"
#include "user.h"
#include "stdio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define MPPT_DEBUG  0//1
#if MPPT_DEBUG
#define mppt_printf(fmt, ...)	printf(fmt, ##__VA_ARGS__)
#else
#define mppt_printf(fmt, ...)
#endif

#define MPPT_INPUT_CUR_CALI_EN	1//0	// Enable/diable MPPT input current calibration. 0, diable; NOT 0, enable

static bool mppt_state = false;
static int32_t mppt_input_v_cur = 0;	// Current input voltage. Unit in mV
static double mppt_input_i_cur = 0.0;	// Current input current. Unit in mA
static double mppt_power_cur = 0.0;		// Current power. Unit in 10^(-6) W
static double mppt_power_prev = 0.0;	// Previous power. Unit in 10^(-6) W
static int32_t mppt_input_v_setting_cur = 0;	// Current input voltage setting. Unit in mV
static int32_t mppt_input_v_setting_prev = 0;	// Previous input voltage setting. Unit in mV
static const int32_t mppt_input_v_setting_min = 8500;	// Input voltage setting min limt. Unit in mV
static const int32_t mppt_input_v_setting_max = 13500;	// Input voltage setting max limt. Unit in mV
static const int32_t mppt_v_delta = 100;					// Input voltage setting variation. Unit in mV
static const double mppt_power_delta_limit = 36450;	// Power delta limit. Unit in 10^(-6) W

static mppt_state_t mppt_status = MPPT_STATE_SCAN;
static const int32_t mppt_input_v_settinng_limit_df = 8500;	// Input Voltage Setting default limit. Unit in mV
static const int32_t mppt_input_v_settinng_limit_l = 11000;//8500;	// Input Voltage Setting lower limt. Unit in mV
static const int32_t mppt_input_v_settinng_limit_u = 13500;	// Input Voltage Setting upper limt. Unit in mV
static double mppt_max_power = 0.0;
static int32_t mppt_input_v_setting_mark = mppt_input_v_settinng_limit_l;
static bool mppt_state_scan_1st_time = true;
static const int32_t mppt_v_step = 100;		// Input voltage setting variation. Unit in mV
static int32_t mppt_hold_cnt = 0;
static const int32_t mppt_hold_limit = 900;		// 15*60 = 900. 15 min.
static const double mppt_p_ratio_limit = 0.1;	// 10 percent

static int32_t mppt_condition_change_cnt = 0;   //the condition change cnt
static int32_t mppt_condition_change_limited_cnt = 30;   // check once time in 10 second. total: 30*10 = 5 mins.
static int32_t input_v_delta = 200; // unit mV.

// Check conditions to enter MPPT process
// Return value: true, OK; false, conditions are not met
static bool mppt_enter_conditon_check(void)
{
	bool case1 = ((adc_to_vol_conv(monet_data.AdcMain, VOL_MAIN_FACTOR)   < ADC_MAIN_TH) &&
				  (adc_to_vol_conv(monet_data.AdcAux, VOL_AUX_FACTOR)     < ADC_MAIN_TH) &&
				  (adc_to_vol_conv(monet_data.AdcSolar, VOL_SOLAR_FACTOR) > ADC_MAIN_TH));	// There is only Solar power only
	bool case2 = is_charger_power_on();
	bool case3 = (solar_chg_mode_get() == SOLAR_CHG_MODE1);
	bool case4 = (monet_data.charge_mode_disable <= 0);
	
	if (case1 == true && case2 == true && case3 == true && case4 == true)
		return true;
	return false;
}

// Get input voltage value. Unit in mV
static int32_t mppt_input_v_get(void)
{
	return mp2762a_input_vol_get();
}

#if MPPT_INPUT_CUR_CALI_EN
// Calibrate Input Current of charger chip.
// JIRA NALAMCU-65: Input Current read from charger chip is not accurate in MPPT algorithm
// Param. cur_raw: current to be calibrated. Unit in mA
// Return value: calibrated current value. Unit in mA
static double mppt_input_cur_calibrate(double cur_raw)
{
	double cur = 0.0;
	
	if (cur_raw <= 50.0)
		cur = cur_raw;
	else if (cur_raw < 70.0)
		cur = cur_raw - 10;
	else if (cur_raw < 100.0)
		cur = cur_raw - 20;
	else if (cur_raw < 200.0)
		cur = cur_raw - 30;
	else
		cur = cur_raw - 60;
	return cur;
}
#endif

// Get input current value. Unit in mA
static double mppt_input_i_get(void)
{
	double cur = 0.0;
	cur = mp2762a_input_cur_get();
#if MPPT_INPUT_CUR_CALI_EN
	cur = mppt_input_cur_calibrate(cur);
#endif
	return cur;
}

// Get output power value. Unit in uW
double mppt_output_p_get(void)
{
	int32_t input_v = 0;		// Unit in mV
	double input_i = 0.0;	// Unit in mA
	double power = 0.0;		// Unit in uW
	
	if(mppt_status == MPPT_STATE_SCAN)
	{
		input_v = 0;
		input_i = 0.0;
		power = mppt_max_power;
	}
	else
	{
		input_v = mppt_input_v_get();
		input_i = mppt_input_i_get();
		power = 1.0 * input_v * input_i;
	}
	NRF_LOG_RAW_INFO("mppt_output_p_get::mppt_status=%d,input_v=%d,input_i=%d,power=%d.\r\n",mppt_status,(int32_t)input_v,(int32_t)input_i,(int32_t)power);
	NRF_LOG_FLUSH();
	return power;
}

// Set input voltage setting. Unit in mV
static void mppt_input_v_setting_set(int32_t setting)
{
	mp2762a_input_vol_limit_set(setting);
}

// Get input voltage setting. Unit in mV
static int32_t mppt_input_v_setting_get(void)
{
	return mp2762a_input_vol_limit_get();
}

/*static*/ bool mppt_is_running(void)
{
	return mppt_state;
}

// Set MPPT process state
// Param. setting: if true, set MPPT is running; if false, set MPPT is stopped
static void mppt_state_set(bool setting)
{
	mppt_state = setting;
}

static void mppt_init(void)
{
	mppt_input_v_cur = 0;
	mppt_input_i_cur = 0.0;
	mppt_power_cur = 0.0;
	mppt_power_prev = 0.0;
	mppt_input_v_setting_cur = mppt_input_v_setting_min;
	mppt_input_v_setting_prev = mppt_input_v_setting_min;
	mppt_input_v_setting_cur = mppt_input_v_setting_cur + mppt_v_delta;
	mppt_input_v_setting_set(mppt_input_v_setting_cur);
	mppt_state_set(true);
}

// MPPT process Based on Perturbation and Observation (P&O) algrithm
// This function is designed to be called in a periodic function, e.g. one second timer handle.
void mppt_process_po(void)
{
	double power_delta = 0.0;
	
	if (mppt_enter_conditon_check() == true)
	{
		if (mppt_is_running() != true)
		{
			mppt_init();
			mppt_printf("MPPT init\r\n");
			return;
		}
		mppt_input_v_cur = mppt_input_v_get();
		mppt_input_i_cur = mppt_input_i_get();
		mppt_power_cur = 1.0 * mppt_input_v_cur * mppt_input_i_cur;
		power_delta = fabs(mppt_power_cur - mppt_power_prev);
		mppt_printf("Pcur %.3f, Pprev %.3f, Pdelta %.3f, Pdelta_limit %.3f\r\n", mppt_power_cur, mppt_power_prev, power_delta, mppt_power_delta_limit);
		if (power_delta < mppt_power_delta_limit)
		{
			// Keep unchange
			mppt_printf("Set unchange\r\n");
		}
		else if (mppt_power_cur > mppt_power_prev)
		{
			if (mppt_input_v_setting_cur > mppt_input_v_setting_prev)
			{
				mppt_input_v_setting_prev = mppt_input_v_setting_cur;
				mppt_input_v_setting_cur += mppt_v_delta;
				if (mppt_input_v_setting_cur > mppt_input_v_setting_max)
					mppt_input_v_setting_cur = mppt_input_v_setting_max;
				mppt_input_v_setting_set(mppt_input_v_setting_cur);
				mppt_power_prev = mppt_power_cur;
				mppt_printf("Pcur > Pprev, Vcur > Vprev\r\n");
			}
			else
			{
				mppt_input_v_setting_prev = mppt_input_v_setting_cur;
				mppt_input_v_setting_cur -= mppt_v_delta;
				if (mppt_input_v_setting_cur < mppt_input_v_setting_min)
					mppt_input_v_setting_cur = mppt_input_v_setting_min;
				mppt_input_v_setting_set(mppt_input_v_setting_cur);
				mppt_power_prev = mppt_power_cur;
				mppt_printf("Pcur > Pprev, Vcur <= Vprev\r\n");
			}
		}
		else
		{
			if (mppt_input_v_setting_cur > mppt_input_v_setting_prev)
			{
				mppt_input_v_setting_prev = mppt_input_v_setting_cur;
				mppt_input_v_setting_cur -= mppt_v_delta;
				if (mppt_input_v_setting_cur < mppt_input_v_setting_min)
					mppt_input_v_setting_cur = mppt_input_v_setting_min;
				mppt_input_v_setting_set(mppt_input_v_setting_cur);
				mppt_power_prev = mppt_power_cur;
				mppt_printf("Pcur <= Pprev, Vcur > Vprev\r\n");
			}
			else
			{
				mppt_input_v_setting_prev = mppt_input_v_setting_cur;
				mppt_input_v_setting_cur += mppt_v_delta;
				if (mppt_input_v_setting_cur > mppt_input_v_setting_max)
					mppt_input_v_setting_cur = mppt_input_v_setting_max;
				mppt_input_v_setting_set(mppt_input_v_setting_cur);
				mppt_power_prev = mppt_power_cur;
				mppt_printf("Pcur <= Pprev, Vcur <= Vprev\r\n");
			}
		}
	}
	else
	{
		if (mppt_is_running() == true)
		{
			mppt_state_set(false);
			mppt_printf("MPPT deinit\r\n");
		}
	}
	mppt_printf("state %u, v %u mV, i %.2f mA, Pcur %.3f uW, Pprev %.3f uW, v_cfg_cur %u mV, v_cfg_prev %u mV\r\n", 
					mppt_is_running(), mppt_input_v_cur, mppt_input_i_cur, 
					mppt_power_cur, mppt_power_prev, 
					mppt_input_v_setting_cur, mppt_input_v_setting_prev);
}

static bool solar_pw_low_power_flag = false;

// Set backdoor function for MPPT algorithm. Called in low power mode (solar power checking)
void mppt_power_check_for_low_power_en(void)
{
	solar_pw_low_power_flag = true;
}

static bool mppt_power_check_for_low_power_state(void)
{
	return solar_pw_low_power_flag;
}

static void solar_power_check(void)
{
	int32_t input_v = 0;	// Unit in mV
	double input_i = 0.0;	// Unit in mA
	double power = 0.0;		// Unit in uW
	
	input_v = mppt_input_v_get();
	input_i = mppt_input_i_get();
	power = 1.0 * input_v * input_i;
	int32_t temp_nordic = 0;
	
	NRF_LOG_RAW_INFO("solar_power_check() %d mV, %u mA, %u uW\r", input_v + input_v_delta, (uint32_t)input_i, (uint32_t)power);
	if (power < SOLAR_POWER_LIMIT)
	{
		if (mppt_power_check_for_low_power_state() == false)
		{
			NRF_LOG_RAW_INFO("solar_power_check() Power < 1 W\r");
            // if ((input_v < VOL_LIMIT_S_CHG_MODE_SEL) ||
            //      (monet_data.charge_mode2_disable == 0))
            // {
            //     solar_chg_mode_select(FUNC_JUMP_POINT_0);
            // }

			if ((input_v + input_v_delta) < VOL_LIMIT_S_CHG_MODE_SEL)  // if input_V < 11V,  
			{
				mppt_condition_change_cnt = 0;
				// if (monet_data.charge_mode2_disable != 0)
				// {
				// 	//mppt_process_nml_deinit(); 
				// 	solar_chg_mode_select(FUNC_JUMP_POINT_4);
				// 	NRF_LOG_RAW_INFO(">>Input_v < 11V && Vbat > 8.2V choose to FUNC_JUMP_POINT_4  \r");
				// }
				// else
				{
					solar_chg_mode_select(FUNC_JUMP_POINT_4);//for <1W && bat<8.2v && input_v < 11v, choose mode 2 
					NRF_LOG_RAW_INFO(">>Mode1 change to off status \r");
				}                   
			}
			else
			{
				if (monet_data.charge_mode2_disable != 0)
				{
					mppt_condition_change_cnt++;
					// solar_chg_mode_select(FUNC_JUMP_POINT_1);
					NRF_LOG_RAW_INFO(">>Bat high 8.2V, still in mode 1  condition_cnt(%d)\r",mppt_condition_change_cnt);
					if (mppt_condition_change_cnt == mppt_condition_change_limited_cnt)
					{
						mppt_process_nml_deinit();
						mppt_condition_change_cnt = 0;
					//    mppt_hold_cnt = 0; 				// Avoid scan at same time error.
					} 
				}
				else
				{
					temp_nordic = chip_temp_get();
					if(temp_nordic > 60 || temp_nordic < -30)
					{
						NRF_LOG_RAW_INFO(">>Bat high 8.2V, still in mode 1 \r");	
					}
					else
					{
						mppt_condition_change_cnt = 0;
						solar_chg_mode_select(FUNC_JUMP_POINT_3); //for <1W && bat<8.2v, choose mode 2 
						NRF_LOG_RAW_INFO(">>Mode1 change to mode2 \r");
					}

				}
			}
			// if (monet_data.charge_mode2_disable != 0)
			// {
			// 	//solar_chg_mode_select(FUNC_JUMP_POINT_0); //for < 1w, choose mode 1
			// 	NRF_LOG_RAW_INFO(">>Bat high 8.2V, still in mode 1 \r");
			// }
			// else
			// {
			// 	if (input_v < VOL_LIMIT_S_CHG_MODE_SEL)  // if input_V < 11V,  
			// 	{
			// 		solar_chg_mode_select(FUNC_JUMP_POINT_4);
			// 	 	NRF_LOG_RAW_INFO(">>Input_v < 11V, choose to FUNC_JUMP_POINT_4  \r");
			// 	}
			// 	else
			// 	{
			// 		solar_chg_mode_select(FUNC_JUMP_POINT_3); //for <1W && bat<8.2v, choose mode 2 
			// 	    NRF_LOG_RAW_INFO(">>Mode1 change to mode2 \r");
			// 	}
			// }
		}
		else
			solar_chg_stage_switch_3_to_2();
	}
	else
	{
		mppt_condition_change_cnt = 0;
		NRF_LOG_RAW_INFO("solar_power_check() Power > 1 W\r");
	}
	NRF_LOG_FLUSH();
}

// MPPT process. Normal algrithm.
// Scan input voltage setting in Vstart ~ Vend, and select the V with max power.
// Hold 15 minutes, then check the power again. If it gets 10% change, re-scan.
// This function is designed to be called in a 1 second period function
void mppt_process_nml(void)
{
	int32_t input_v = 0;	// Unit in mV
	double input_i = 0.0;	// Unit in mA
	double power = 0.0;		// Unit in uW
	int32_t input_v_setting = 0;	// Unit in mV
	double power_delta = 0.0;
	
	if (mppt_enter_conditon_check() == true)
	{
        monet_data.charge_status = CHARGE_STATUS_SOLAR_MODE_1_PROCESS;

        if (mppt_is_running() != true)
		{
			mppt_printf("MPPT init\r\n");
			mppt_state_set(true);
			mppt_status = MPPT_STATE_SCAN;
			mppt_state_scan_1st_time = true;
		}
		
		if (mppt_status == MPPT_STATE_SCAN)
		{
            NRF_LOG_RAW_INFO("mppt_process_nml mppt_status Scan.\r");
            NRF_LOG_FLUSH();
            if (mppt_state_scan_1st_time == true)
			{
				mppt_printf("MPPT status SCAN\r\n");
				mppt_state_scan_1st_time = false;
				mppt_max_power = 0.0;
				mppt_input_v_setting_mark = mppt_input_v_settinng_limit_l;
				mppt_input_v_setting_set(mppt_input_v_settinng_limit_l);	// Scan starts at lower limit
				return;
			}
			input_v = mppt_input_v_get();
			input_i = mppt_input_i_get();
			power = 1.0 * input_v * input_i;
			input_v_setting = mppt_input_v_setting_get();
			mppt_printf("MPPT, V: %d mV, I: %.2f mA, P: %.2f uW, Vsetting: %d mV\r\n", input_v, input_i, power, input_v_setting);
			if (power > mppt_max_power)
			{
				mppt_max_power = power;
				mppt_input_v_setting_mark = input_v_setting;
			}
			input_v_setting += mppt_v_step;
			if (input_v_setting > mppt_input_v_settinng_limit_u)	// Status SCAN ends
			{
				mppt_input_v_setting_set(mppt_input_v_setting_mark);
				mppt_status = MPPT_STATE_HOLD;
				mppt_state_scan_1st_time = true;
				mppt_printf("MPPT, Pmax: %.2f uW, Vsetting: %d mV\r\n", mppt_max_power, mppt_input_v_setting_mark);
				mppt_printf("MPPT status is set to HOLD\r\n");
			}
			else
				mppt_input_v_setting_set(input_v_setting);
		}
		else if (mppt_status == MPPT_STATE_HOLD)
		{
            NRF_LOG_RAW_INFO("mppt_process_nml mppt_status HOLD.\r");
            NRF_LOG_FLUSH();
			mppt_hold_cnt++;
			if (mppt_hold_cnt % 10 == 0)	// Check the solar power every 10 seconds
				solar_power_check();
			if (mppt_hold_cnt >= mppt_hold_limit)
			{
				mppt_hold_cnt = 0;
				input_v = mppt_input_v_get();
				input_i = mppt_input_i_get();
				power = 1.0 * input_v * input_i;
				input_v_setting = mppt_input_v_setting_get();
				mppt_printf("MPPT, V: %d mV, I: %.2f mA, P: %.2f uW, Vsetting: %d mV\r\n", input_v, input_i, power, input_v_setting);
				power_delta = fabs(mppt_max_power - power);
				if ((mppt_max_power == 0.0 && (power > mppt_max_power)) ||	// When divisor is zero
					((mppt_max_power != 0.0) && ((power_delta / mppt_max_power) > mppt_p_ratio_limit)))	// Status HOLD ends
				{
					mppt_status = MPPT_STATE_SCAN;	// Switch to Scan status
					mppt_printf("P %.2f, power_delta %.2f, Pmax %.2f\r\n", power, power_delta, mppt_max_power);
					mppt_printf("MPPT status is set to SCAN\r\n");
				}
			} // if (mppt_hold_cnt > mppt_hold_limit)
		} // else if (mppt_status == MPPT_STATE_HOLD)
	} // if (mppt_enter_conditon_check() == true)
	else
	{
		if (mppt_is_running() == true)
		{
			mppt_printf("MPPT deinit\r\n");
			mppt_state_set(false);
			mppt_max_power = 0.0;	// Clear this value to avoid misuse in other place
			mppt_input_v_setting_set(mppt_input_v_settinng_limit_df);	// Recover input voltage to default value. Otherwise there is risk that Main/AUX power cannot charge battery.
            mppt_hold_cnt = 0;
		}
	}
}

// De-init MPPT nml algorithm.
// Called when mppt_process_nml() cannot deinit itself.
void mppt_process_nml_deinit(void)
{
	if (mppt_is_running() == true)
	{
		mppt_printf("MPPT deinit\r\n");
		mppt_state_set(false);
		mppt_max_power = 0.0;	// Clear this value to avoid misuse in other place
		mppt_input_v_setting_set(mppt_input_v_settinng_limit_df);	// Recover input voltage to default value. Otherwise there is risk that Main/AUX power cannot charge battery.
        mppt_hold_cnt = 0;
	}
}

// Scan input voltage setting from 8.5 V to 13.5 V and get respective info.
void mppt_test(void)
{
	int32_t input_v = 0;	// Unit in mV
	double input_i = 0.0;	// Unit in mA
	double power = 0.0;		// Unit in uW
	int32_t input_v_setting = 0;	// Unit in mV
	
	input_v = mppt_input_v_get();
	input_i = mppt_input_i_get();
	power = 1.0 * input_v * input_i;
	power = power;	// To avoid compiler warning
	input_v_setting = mppt_input_v_setting_get();
	mppt_printf("Input V: %d mV, input I: %.2f mA, power: %.2f uW, input V setting: %d mV\r\n", input_v, input_i, power, input_v_setting);
	input_v_setting += 100;	// 100 mV
	if (input_v_setting > 13500)
		input_v_setting = 8500;
	mppt_input_v_setting_set(input_v_setting);
}

