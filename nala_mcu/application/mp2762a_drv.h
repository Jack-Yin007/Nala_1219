#ifndef __MP2762A_DRV_H__
#define __MP2762A_DRV_H__

#include "stdbool.h"
#include "stdint.h"

#define MP2762A_REG00H	0x00
#define MP2762A_REG01H	0x01
#define MP2762A_REG02H	0x02
#define MP2762A_REG03H	0x03
#define MP2762A_REG04H	0x04
#define MP2762A_REG05H	0x05
#define MP2762A_REG06H	0x06
#define MP2762A_REG07H	0x07

#define MP2762A_REG08H	0x08
#define MP2762A_REG09H	0x09
#define MP2762A_REG0AH	0x0A
#define MP2762A_REG0BH	0x0B
#define MP2762A_REG0CH	0x0C

#define MP2762A_REG0DH	0x0D
#define MP2762A_REG0EH	0x0E
#define MP2762A_REG0FH	0x0F
#define MP2762A_REG10H	0x10
#define MP2762A_REG11H	0x11
#define MP2762A_REG12H	0x12
#define MP2762A_REG13H	0x13
#define MP2762A_REG14H	0x14

#define MP2762A_REG16H	0x16
#define MP2762A_REG17H	0x17

#define MP2762A_REG18H	0x18
#define MP2762A_REG19H	0x19

#define MP2762A_REG1AH	0x1A
#define MP2762A_REG1BH	0x1B

#define MP2762A_REG1CH	0x1C
#define MP2762A_REG1DH	0x1D

#define MP2762A_REG1EH	0x1E
#define MP2762A_REG1FH	0x1F

#define MP2762A_REG20H	0x20
#define MP2762A_REG21H	0x21

#define MP2762A_REG22H	0x22
#define MP2762A_REG23H	0x23

#define MP2762A_REG24H	0x24
#define MP2762A_REG25H	0x25

#define MP2762A_REG26H	0x26
#define MP2762A_REG27H	0x27

#define MP2762A_REG28H	0x28
#define MP2762A_REG29H	0x29

#define MP2762A_REG2BH	0x2B
#define MP2762A_REG2DH	0x2D
#define MP2762A_REG31H	0x31
#define MP2762A_REG33H	0x33
#define MP2762A_REG36H	0x36
#define MP2762A_REG40H	0x40	// Reserved register for NTC voltage. Vol(NTC) = 1.6V*(REG40H41H[11:0])/4096
#define MP2762A_REG41H	0x41	// Reserved register for NTC voltage
#define MP2762A_REG48H	0x48


//  MP2762A_REG00H
#define MP2762A_INPUT_CUR_LIMIT1_BIT_POS			0
#define MP2762A_INPUT_CUR_LIMIT1_BIT_LEN			7
#define MP2762A_INPUT_CUR_LIMIT1_MA_TO_NUM(CUR_mA)	(((uint32_t)(CUR_mA))/50)	// Convert current mA to register number
#define MP2762A_INPUT_CUR_LIMIT1_NUM_TO_MA(REGN)	(((uint32_t)(REGN))*50)		// Convert register number to current mA


// MP2762A_REG01H
#define MP2762A_INPUT_VOL_LIMIT_BIT_POS				0
#define MP2762A_INPUT_VOL_LIMIT_BIT_LEN				8
#define MP2762A_INPUT_VOL_LIMIT_MV_TO_REGN(VOL_mV)	((VOL_mV)/100)	// Voltage (mV) to Reg number
#define MP2762A_INPUT_VOL_LIMIT_REGN_TO_MV(REGN)	((REGN)*100)	// Reg number to voltage (mV)


// MP2762A_REG02H
#define MP2762A_CHG_CUR_BIT_POS		0
#define MP2762A_CHG_CUR_BIT_LEN		7
#define MP2762A_CHG_CUR_MA_TO_NUM(CUR_MA)		((CUR_MA)/50)
#define MP2762A_CHG_CUR_NUM_TO_MA(NUM)			((NUM)*50)


// MP2762A_REG03H
#define MP2762A_PRE_CHG_CUR_BIT_POS		4	// Pre-charge Current Limit
#define MP2762A_PRE_CHG_CUR_BIT_LEN		4
#define MP2762A_PRE_CHG_CUR_MA_TO_NUM(CUR_MA)		(((CUR_MA)<240)? 0: (((CUR_MA)-180)/60+4))	// CUR_MA: 180 ~ 840mA, default 180 mA. Result, number, 0 ~ 15.
#define MP2762A_PRE_CHG_CUR_NUM_TO_MA(NUM)			(((NUM)<5)? 180: (180+((NUM)-4)*60) )		// NUM: 0 ~ 15. Result, current, unit in mA.


// MP2762A_REG04H
#define MP2762A_CHG_FULL_VOL_BIT_POS	1	// Charge Full Voltage
#define MP2762A_CHG_FULL_VOL_BIT_LEN	7
#define MP2762A_CHG_FULL_VOL_VOL_TO_NUM(VOL)	(((VOL) <= 7425)? 0: ((VOL) - 7425)/25)	// VOL's unit is mV
#define MP2762A_CHG_FULL_VOL_NUM_TO_VOL(NUM)	((NUM)*25 + 7425)	// Unit in mV


// MP2762A_REG08H
#define MP2762A_CHG_EN_BIT_POS		4	// CHG_EN
#define MP2762A_CHG_EN_BIT_LEN		1
#define MP2762A_CHG_EN_DISABLE		0
#define MP2762A_CHG_EN_ENABLE		1

#define MP2762A_BFET_BIT_POS		1	// BFET_EN
#define MP2762A_BFET_LEN			1
#define MP2762A_BFET_DISABLE		0	// Disable charging and Discharge
#define MP2762A_BFET_ENABLE			1	// Enable charging or discharge


// MP2762A_REG09H
#define MP2762A_F_CHG_TM_BIT_POS	1	// Fast Charge Timer Setting
#define MP2762A_F_CHG_TM_BIT_LEN	2
#define MP2762A_F_CHG_TM_5HRS		0
#define MP2762A_F_CHG_TM_8HRS		1
#define MP2762A_F_CHG_TM_12HRS		2	// Default
#define MP2762A_F_CHG_TM_20HRS		3

#define MP2762A_EN_TMR_BIT_POS		3	// Charging Safety Timer Enable (Both Pre-charge Timer and Complete charge cycle timer)
#define MP2762A_EN_TMR_BIT_LEN		1
#define MP2762A_EN_TMR_DISABLE		0
#define MP2762A_EN_TMR_ENABLE		1	// Default


// MP2762A_REG0AH
#define MP2762A_JEITA_ISET_BIT_POS	7	// JEITA_ISET, JEITA Low Temperature Current Setting
#define MP2762A_JEITA_ISET_BIT_LEN	1
#define MP2762A_JEITA_ISET_50_PER	0	// 50 percent
#define MP2762A_JEITA_ISET_20_PER	1	// 20 percent, default value

#define MP2762A_JEITA_VSET_BIT_POS	6	// JEITA_VSET, JEITA High Temperature Voltage Setting
#define MP2762A_JEITA_VSET_BIT_LEN	1
#define MP2762A_JEITA_VSET_150_MV	0	// Set Charge Voltage to VB_FULL-150mV, default value
#define MP2762A_JEITA_VSET_300_MV	1	// Set Charge Voltage to VB_FULL-300mV

#define MP2762A_NTC_CTRL_BIT_POS	4	// NTC Protection Type Setting
#define MP2762A_NTC_CTRL_BIT_LEN	2
#define MP2762A_NTC_CTRL_JEITA		0	// 0/1/2: JEITA
#define MP2762A_NTC_CTRL_JEITA1		1	// Reserved option
#define MP2762A_NTC_CTRL_JEITA2		2	// Reserved option
#define MP2762A_NTC_CTRL_DISABLE	3	// Default: Disable

#define MP2762A_NTC_WARM_BIT_POS	2	// NTC_WARM
#define MP2762A_NTC_WARM_BIT_LEN	2
#define MP2762A_NTC_WARM_58_3P_40D	0	// 58.3% (40 degrees Celsius)
#define MP2762A_NTC_WARM_56_1P_45D	1	// 56.1% (45 degrees Celsius)
#define MP2762A_NTC_WARM_53_7P_50D	2	// 53.7% (50 degrees Celsius)
#define MP2762A_NTC_WARM_51_3P_55D	3	// 51.3% (55 degrees Celsius)


// MP2762A_REG0BH
#define MP2762A_SW_FREQ_BIT_POS		3	// Switching Frequency Setting
#define MP2762A_SW_FREQ_BIT_LEN		2
#define MP2762A_SW_FREQ_600K		0	// Default: 600k
#define MP2762A_SW_FREQ_800K		1
#define MP2762A_SW_FREQ_1000K		2
#define MP2762A_SW_FREQ_INVALID		3


// MP2762A_REG0FH
#define MP2762A_INPUT_CUR_LIMIT2_BIT_POS			0
#define MP2762A_INPUT_CUR_LIMIT2_BIT_LEN			7
#define MP2762A_INPUT_CUR_LIMIT2_MA_TO_NUM(CUR_mA)	(((uint32_t)(CUR_mA))/50)	// Convert current mA to register number
#define MP2762A_INPUT_CUR_LIMIT2_NUM_TO_MA(REGN)	(((uint32_t)(REGN))*50)		// Convert register number to current mA


// MP2762A_REG13H
#define MP2762A_CHG_STAT_BIT_POS	2
#define MP2762A_CHG_STAT_BIT_LEN	2
#define MP2762A_CHG_STAT_NOT_CHG	0	// Not Charging
#define MP2762A_CHG_STAT_PRE_CHG	1	// Pre-charge
#define MP2762A_CHG_STAT_FST_CHG	2	// Fast Charging or Trickle Charge
#define MP2762A_CHG_STAT_CHG_TMN	3	// Charge Termination


// MP2762A_REG14H
#define MP2762A_WD_FAULT_BIT_POS	7
#define MP2762A_WD_FAULT_BIT_LEN	1
#define MP2762A_WD_FAULT_NORMAL		0	// Normal
#define MP2762A_WD_FAULT_WD_EXP		1	// Watchdog timer expiration

#define MP2762A_OTG_FAULT_BIT_POS	6
#define MP2762A_OTG_FAULT_BIT_LEN	1
#define MP2762A_OTG_FAULT_NORMAL	0	// Normal
#define MP2762A_OTG_FAULT_VBUS_ERR	1	// VBUS overloaded, or VBUS OVP

#define MP2762A_CHG_FAULT_BIT_POS		4
#define MP2762A_CHG_FAULT_BIT_LEN		2
#define MP2762A_CHG_FAULT_NORMAL		0	// Normal
#define MP2762A_CHG_FAULT_INPUT_OVP		1	// Input OVP
#define MP2762A_CHG_FAULT_THM_SHD		2	// Thermal Shutdown
#define MP2762A_CHG_FAULT_SFT_TM_EXP	3	// Safety timer expiration

#define MP2762A_BAT_FAULT_BIT_POS		3
#define MP2762A_BAT_FAULT_BIT_LEN		1
#define MP2762A_BAT_FAULT_NORMAL		0	// Normal
#define MP2762A_BAT_FAULT_BAT_OVP		1	// Battery OVP

#define MP2762A_NTC_FAULT_BIT_POS		0
#define MP2762A_NTC_FAULT_BIT_LEN		3
#define MP2762A_NTC_FAULT_NORMAL		0	// Normal
#define MP2762A_NTC_FAULT_NTC_COLD		1	// NTC Cold
#define MP2762A_NTC_FAULT_NTC_COOL		2	// NTC Cool
#define MP2762A_NTC_FAULT_NTC_WARM		3	// NTC Warm
#define MP2762A_NTC_FAULT_NTC_HOT		4	// NTC Hot


// MP2762A_REG16H
#define MP2762A_BAT_VOL_L_BIT_POS		6
#define MP2762A_BAT_VOL_L_BIT_LEN		2
#define MP2762A_BAT_VOL_RESO			12.5	// Resolution of data. Unit in mV


// MP2762A_REG17H
#define MP2762A_BAT_VOL_H_BIT_POS		0
#define MP2762A_BAT_VOL_H_BIT_LEN		8


// MP2762A_REG1AH
#define MP2762A_BAT_CHG_CUR_L_BIT_POS	6
#define MP2762A_BAT_CHG_CUR_L_BIT_LEN	2
#define MP2762A_BAT_CHG_CUR_RESO		12.5	// Resolution of data. Unit in mA


// MP2762A_REG1BH
#define MP2762A_BAT_CHG_CUR_H_BIT_POS	0
#define MP2762A_BAT_CHG_CUR_H_BIT_LEN	8


// MP2762A_REG1CH
#define MP2762A_INPUT_VOL_L_BIT_POS		6
#define MP2762A_INPUT_VOL_L_BIT_LEN		2
#define MP2762A_INPUT_VOL_RESO			25		// Resolution of data. Unit in mV


// MP2762A_REG1DH
#define MP2762A_INPUT_VOL_H_BIT_POS		0
#define MP2762A_INPUT_VOL_H_BIT_LEN		8


// MP2762A_REG1EH
#define MP2762A_INPUT_CUR_L_BIT_POS		6
#define MP2762A_INPUT_CUR_L_BIT_LEN		2
#define MP2762A_INPUT_CUR_RESO			6.25	// Resolution of data. Unit in mA


// MP2762A_REG1FH
#define MP2762A_INPUT_CUR_H_BIT_POS		0
#define MP2762A_INPUT_CUR_H_BIT_LEN		8


// remove from .c
#define NTC_SAMPLE_INTVL	5//5 	// NTC sample interval. Unit in seconds

#define CHG_TEMP_LIMIT1		(-30) 	// Charging temperature limit. Unit in Celsius
#define CHG_TEMP_LIMIT2		0 		//
#define CHG_TEMP_LIMIT3		45 		//
#define CHG_TEMP_LIMIT4		60 		//

#define CHG_CUR_LIMIT0		0 		// Charging current limit. Unit in mA
#define CHG_CUR_LIMIT400	400
#define CHG_CUR_LIMIT1500	1500

#define TEMP_DIFF_LIMIT		10		// Difference limit between Nordic and charger NTC temperature. Unit in degree Celsius
#define CHGR_TMP_DELTA		3		// Temperature delta for charger current. Unit in degree Celsius. 


bool mp2762a_charge_ctrl(bool setting);
bool mp2762a_charge_en_get(uint8_t *p_value);
bool mp2762a_bfet_ctrl(bool setting);
bool mp2762a_charge_state_get(uint8_t *p_chg_state);

bool mp2762a_fault_watchdog_get(uint8_t *p_fault);
bool mp2762a_fault_otg_get(uint8_t *p_fault);
bool mp2762a_fault_chg_get(uint8_t *p_fault);
bool mp2762a_fault_bat_get(uint8_t *p_fault);
bool mp2762a_fault_ntc_get(uint8_t *p_fault);

double mp2762a_bat_vol_get(void);
double mp2762a_bat_chg_cur_get(void);
uint32_t mp2762a_input_vol_get(void);
double mp2762a_input_cur_get(void);
bool mp2762a_input_cur_limit1_set(uint32_t cur_setting);
uint32_t mp2762a_input_cur_limit1_get(void);
bool mp2762a_input_cur_limit2_set(uint32_t cur_setting);
uint32_t mp2762a_input_cur_limit2_get(void);
bool mp2762a_input_vol_limit_set(uint32_t vol_setting);
int32_t mp2762a_input_vol_limit_get(void);
bool mp2762a_ntc_vol_get(double *p_vol);
int8_t mp2762a_read_reg_value(uint8_t reg, uint8_t* p_val);

void mp2762a_init(void);
bool bat_ntc_res_get(double *p_ntc_res);
int16_t bat_temp_get(double res);
void mp2762a_proc(void);
int32_t get_temp(uint8_t mode);  // mode 0 is ntc Temp. mode1 is nrf temp.

#endif	// #ifndef __MP2762A_DRV_H__
