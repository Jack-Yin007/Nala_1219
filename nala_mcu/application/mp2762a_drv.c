#include "mp2762a_drv.h"
#include "platform_hal_drv.h"
#include "solar_mppt.h"

#define MP2762A_I2C_ADDR			0x5C	// 7-bit I2C address. I2C 1st byte = 7-bit addr + 1-bit R/W

#define MP2762A_NTC_REF_VOL		1.6		// NTC referenc voltage. Unit in V
#define MP2762A_NTC_VAL_MAX		4096
#define MP2762A_NTC_BIT_LEN		12		// Lenght of bits

static const uint8_t mask_table[8] = {0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f, 0x7f, 0xff};
static const uint32_t mask_table_32bit[32] = {0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f, 0x7f, 0xff,
										0x01ff, 0x03ff, 0x07ff, 0x0fff, 0x1fff, 0x3fff, 0x7fff, 0xffff,
										0x01ffff, 0x03ffff, 0x07ffff, 0x0fffff, 0x1fffff, 0x3fffff, 0x7fffff, 0xffffff,
										0x01ffffff, 0x03ffffff, 0x07ffffff, 0x0fffffff, 0x1fffffff, 0x3fffffff, 0x7fffffff, 0xffffffff};

static void mp2762a_ntc_sample_proc(bool init_call);
										
// Generate mask with specific count of '1'
// Example:
// bit_len		Return value
// 	1			0x01 (00000001)b
// 	2			0x03 (00000011)b
// ... 
// 	8			0xff (11111111)b
static uint8_t mask_generate(uint8_t bit_len)
{
	if (bit_len > 8) return 0;
	return mask_table[bit_len-1];	// Use mask table to improve efficiency
}

// Generate mask with specific count of '1'
// Example:
// bit_len		Return value
// 	1			0x01 (00000001)b
// 	2			0x03 (00000011)b
// ... 
// 	8			0xff (11111111)b
// ... 
// 	32			0xffffffff (11111111 11111111 11111111 11111111)b
static uint32_t mask_generate_32bit(uint8_t bit_len)
{
	if (bit_len > 32) return 0;
	return mask_table_32bit[bit_len-1];	// Use mask table to improve efficiency
}

static bool mp2762a_i2c_write_byte(uint8_t reg, uint8_t val)
{
	ret_code_t ret;
	uint8_t w_buf[2] = {0};
	w_buf[0] = reg;
	w_buf[1] = val;
	
	ret = nrf_drv_twi_tx(&m_twi2, MP2762A_I2C_ADDR, w_buf, sizeof(w_buf), false);	// Nordic I2C needs 7-bit LSB address. E.g. If I2C first byte is [0011001][x], 0011001b(0x19) should be filled here
	if(ret != NRF_SUCCESS)
		return false;
	return true;
}

static bool mp2762a_i2c_read_byte(uint8_t reg, uint8_t *p_data)
{
	ret_code_t ret;
	ret = nrf_drv_twi_tx(&m_twi2, MP2762A_I2C_ADDR, &reg, sizeof(reg), true);		// Nordic I2C needs 7-bit LSB address
	if(ret != NRF_SUCCESS)
		return false;
	ret = nrf_drv_twi_rx(&m_twi2, MP2762A_I2C_ADDR, p_data, sizeof(uint8_t));
	if(ret != NRF_SUCCESS)
		return false;
	return true;
}

static bool mp2762a_field_update(uint8_t reg, uint8_t bit_pos, uint8_t bit_len, uint8_t value)
{
	uint8_t reg_data = 0;
	uint8_t mask = 0;
	
	if (mp2762a_i2c_read_byte(reg, &reg_data) != true)
		return false;
	mask = mask_generate(bit_len);
	reg_data &= ~(mask << bit_pos);
	reg_data |= (value << bit_pos);
	return mp2762a_i2c_write_byte(reg, reg_data);
}

static bool mp2762a_field_read(uint8_t reg, uint8_t bit_pos, uint8_t bit_len, uint8_t *p_value)
{
	uint8_t reg_data = 0;
	uint8_t mask = 0;
	
	if (mp2762a_i2c_read_byte(reg, &reg_data) != true)
		return false;
	mask = mask_generate(bit_len);
	reg_data &= (mask << bit_pos);
	*p_value = (reg_data >> bit_pos);
	return true;
}

// Charge control
// Param. setting: true, enable charging; false, disable charging
bool mp2762a_charge_ctrl(bool setting)
{
	bool ret = false;
	if (setting == false)  // Disable charging
		ret = mp2762a_field_update(MP2762A_REG08H, MP2762A_CHG_EN_BIT_POS, MP2762A_CHG_EN_BIT_LEN, MP2762A_CHG_EN_DISABLE);
	else  // Enable charging
		ret = mp2762a_field_update(MP2762A_REG08H, MP2762A_CHG_EN_BIT_POS, MP2762A_CHG_EN_BIT_LEN, MP2762A_CHG_EN_ENABLE);
	return ret;
}

// Get Charge state
// Param. p_value: result value, MP2762A_CHG_EN_DISABLE or MP2762A_CHG_EN_ENABLE
// Return value: if true, OK; if false, failed to read data
bool mp2762a_charge_en_get(uint8_t *p_value)
{
	bool ret = false;
	ret = mp2762a_field_read(MP2762A_REG08H, MP2762A_CHG_EN_BIT_POS, MP2762A_CHG_EN_BIT_LEN, p_value);
	return ret;
}

// BFET control
// Param. setting: true, Enable charging or discharge; false, Disable charging and Discharge
bool mp2762a_bfet_ctrl(bool setting)
{
	bool ret = false;
	if (setting == false)  // Disable charging
		ret = mp2762a_field_update(MP2762A_REG08H, MP2762A_BFET_BIT_POS, MP2762A_BFET_LEN, MP2762A_BFET_DISABLE);
	else  // Enable charging
		ret = mp2762a_field_update(MP2762A_REG08H, MP2762A_BFET_BIT_POS, MP2762A_BFET_LEN, MP2762A_BFET_ENABLE);
	return ret;
}

// Get MP2762A charge state.
// Param. p_chg_state: charge state, value in 0/1/2/3
//   See macro MP2762A_CHG_STAT_NOT_CHG ~ MP2762A_CHG_STAT_CHG_TMN for more information.
// Return value: true, OK; false, failed.
bool mp2762a_charge_state_get(uint8_t *p_chg_state)
{	
	if (mp2762a_field_read(MP2762A_REG13H, MP2762A_CHG_STAT_BIT_POS, MP2762A_CHG_STAT_BIT_LEN, p_chg_state) != true)
		return false;
	return true;
}

// Get watchdog fault.
// Param. p_fault: fault type, value in 0/1
//   See macro MP2762A_WD_FAULT_NORMAL ~ MP2762A_WD_FAULT_WD_EXP for more information.
// Return value: true, OK; false, failed.
bool mp2762a_fault_watchdog_get(uint8_t *p_fault)
{	
	if (mp2762a_field_read(MP2762A_REG14H, MP2762A_WD_FAULT_BIT_POS, MP2762A_WD_FAULT_BIT_LEN, p_fault) != true)
		return false;
	return true;
}

// Get OTG mode fault.
// Param. p_fault: fault type, value in 0/1
//   See macro MP2762A_WD_FAULT_NORMAL ~ MP2762A_WD_FAULT_WD_EXP for more information.
// Return value: true, OK; false, failed.
bool mp2762a_fault_otg_get(uint8_t *p_fault)
{	
	if (mp2762a_field_read(MP2762A_REG14H, MP2762A_OTG_FAULT_BIT_POS, MP2762A_OTG_FAULT_BIT_LEN, p_fault) != true)
		return false;
	return true;
}

// Get charge fault.
// Param. p_fault: fault type, value in 0/1/2/3
//   See macro MP2762A_CHG_FAULT_NORMAL ~ MP2762A_CHG_FAULT_SFT_TM_EXP for more information.
// Return value: true, OK; false, failed.
bool mp2762a_fault_chg_get(uint8_t *p_fault)
{	
	if (mp2762a_field_read(MP2762A_REG14H, MP2762A_CHG_FAULT_BIT_POS, MP2762A_CHG_FAULT_BIT_LEN, p_fault) != true)
		return false;
	return true;
}

// Get battery fault.
// Param. p_fault: fault type, value in 0/1
//   See macro MP2762A_BAT_FAULT_NORMAL ~ MP2762A_BAT_FAULT_BAT_OVP for more information.
// Return value: true, OK; false, failed.
bool mp2762a_fault_bat_get(uint8_t *p_fault)
{	
	if (mp2762a_field_read(MP2762A_REG14H, MP2762A_BAT_FAULT_BIT_POS, MP2762A_BAT_FAULT_BIT_LEN, p_fault) != true)
		return false;
	return true;
}

// Get NTC fault.
// Param. p_fault: fault type, value in 0~4
//   See macro MP2762A_NTC_FAULT_NORMAL ~ MP2762A_NTC_FAULT_NTC_HOT for more information.
// Return value: true, OK; false, failed.
bool mp2762a_fault_ntc_get(uint8_t *p_fault)
{	
	if (mp2762a_field_read(MP2762A_REG14H, MP2762A_NTC_FAULT_BIT_POS, MP2762A_NTC_FAULT_BIT_LEN, p_fault) != true)
		return false;
	return true;
}

// Set charge current
// Param. cur_setting: current setting, unit in mA. Ranges 0 to 6000 mA. Default 1000 mA.
// Return value: true, OK; false, failed.
bool mp2762a_chg_cur_set(uint16_t cur_setting)
{
	bool ret = false;
	ret = mp2762a_field_update(MP2762A_REG02H, MP2762A_CHG_CUR_BIT_POS, MP2762A_CHG_CUR_BIT_LEN, MP2762A_CHG_CUR_MA_TO_NUM(cur_setting));
	return ret;
}

// Read back charge current setting
// Param. cur_setting: current setting, unit in mA. Ranges 0 to 6000 mA. Default 1000 mA.
// Return value: true, OK; false, failed.
bool mp2762a_chg_cur_get(uint16_t *p_cur_setting)
{
	bool ret = false;
	uint8_t cur_num = 0;
	ret = mp2762a_field_read(MP2762A_REG02H, MP2762A_CHG_CUR_BIT_POS, MP2762A_CHG_CUR_BIT_LEN, &cur_num);
	if (ret == true)
		*p_cur_setting = MP2762A_CHG_CUR_NUM_TO_MA(cur_num);
	return ret;
}

// Get Battery Voltage. Unit in mV
double mp2762a_bat_vol_get(void)
{
	uint8_t bat_vol_l = 0, bat_vol_h = 0;
	mp2762a_i2c_read_byte(MP2762A_REG16H, &bat_vol_l);
	mp2762a_i2c_read_byte(MP2762A_REG17H, &bat_vol_h);
	return ((bat_vol_l | (bat_vol_h<<8)) >> MP2762A_BAT_VOL_L_BIT_POS) * MP2762A_BAT_VOL_RESO;
}

// Get Battery Charge Current. Unit in mA
double mp2762a_bat_chg_cur_get(void)
{
	uint8_t bat_chg_cur_l = 0, bat_chg_cur_h = 0;
	mp2762a_i2c_read_byte(MP2762A_REG1AH, &bat_chg_cur_l);
	mp2762a_i2c_read_byte(MP2762A_REG1BH, &bat_chg_cur_h);
	return ((bat_chg_cur_l | (bat_chg_cur_h<<8)) >> MP2762A_BAT_CHG_CUR_L_BIT_POS) * MP2762A_BAT_CHG_CUR_RESO;
}

// Get Input Voltage. Unit in mV
uint32_t mp2762a_input_vol_get(void)
{
	uint8_t input_vol_l = 0, input_vol_h = 0;
	mp2762a_i2c_read_byte(MP2762A_REG1CH, &input_vol_l);
	mp2762a_i2c_read_byte(MP2762A_REG1DH, &input_vol_h);
	return ((input_vol_l | (input_vol_h<<8)) >> MP2762A_INPUT_VOL_L_BIT_POS) * MP2762A_INPUT_VOL_RESO;
}

// Get Input Current. Unit in mA
double mp2762a_input_cur_get(void)
{
	uint8_t input_cur_l = 0, input_cur_h = 0;
	mp2762a_i2c_read_byte(MP2762A_REG1EH, &input_cur_l);
	mp2762a_i2c_read_byte(MP2762A_REG1FH, &input_cur_h);
	return ((input_cur_l | (input_cur_h<<8)) >> MP2762A_INPUT_CUR_L_BIT_POS) * MP2762A_INPUT_CUR_RESO;
}

// Set Input Current Limit1. Unit in mA
bool mp2762a_input_cur_limit1_set(uint32_t cur_setting)
{
	return mp2762a_field_update(MP2762A_REG00H,
								MP2762A_INPUT_CUR_LIMIT1_BIT_POS, MP2762A_INPUT_CUR_LIMIT1_BIT_LEN, 
								MP2762A_INPUT_CUR_LIMIT1_MA_TO_NUM(cur_setting));
}

// Get Input Current Limit1. Unit in mA
uint32_t mp2762a_input_cur_limit1_get(void)
{
	uint8_t data = 0;
	mp2762a_field_read(MP2762A_REG00H, MP2762A_INPUT_CUR_LIMIT1_BIT_POS, MP2762A_INPUT_CUR_LIMIT1_BIT_LEN, &data);
	return MP2762A_INPUT_CUR_LIMIT1_NUM_TO_MA(data);
}

// Set Input Current Limit1. Unit in mA
bool mp2762a_input_cur_limit2_set(uint32_t cur_setting)
{
	return mp2762a_field_update(MP2762A_REG0FH,
								MP2762A_INPUT_CUR_LIMIT2_BIT_POS, MP2762A_INPUT_CUR_LIMIT2_BIT_LEN, 
								MP2762A_INPUT_CUR_LIMIT2_MA_TO_NUM(cur_setting));
}

// Get Input Current Limit1. Unit in mA
uint32_t mp2762a_input_cur_limit2_get(void)
{
	uint8_t data = 0;
	mp2762a_field_read(MP2762A_REG0FH, MP2762A_INPUT_CUR_LIMIT2_BIT_POS, MP2762A_INPUT_CUR_LIMIT2_BIT_LEN, &data);
	return MP2762A_INPUT_CUR_LIMIT2_NUM_TO_MA(data);
}

// Set Input Voltage Limit Threshold. Unit in mV
bool mp2762a_input_vol_limit_set(uint32_t vol_setting)
{
	return mp2762a_field_update(MP2762A_REG01H,
								MP2762A_INPUT_VOL_LIMIT_BIT_POS, MP2762A_INPUT_VOL_LIMIT_BIT_LEN, 
								MP2762A_INPUT_VOL_LIMIT_MV_TO_REGN(vol_setting));
}

// Get Input Voltage Limit Threshold. Unit in mV
int32_t mp2762a_input_vol_limit_get(void)
{
	uint8_t data = 0;
	mp2762a_field_read(MP2762A_REG01H, MP2762A_INPUT_VOL_LIMIT_BIT_POS, MP2762A_INPUT_VOL_LIMIT_BIT_LEN, &data);
	return MP2762A_INPUT_VOL_LIMIT_REGN_TO_MV(data);
}

// Get NTC voltage.
// Para. p_vol: NTC voltage. Unit in V
// Return value: if true, OK; if false, failed to get voltage data
bool mp2762a_ntc_vol_get(double *p_vol)
{
	uint8_t ntc_l = 0;
	uint8_t ntc_h = 0;
	uint32_t ntc_whole = 0;
	uint32_t ntc_data_mask = mask_generate_32bit(MP2762A_NTC_BIT_LEN);
	
	if (mp2762a_i2c_read_byte(MP2762A_REG40H, &ntc_l) != true)
		return false;
	if (mp2762a_i2c_read_byte(MP2762A_REG41H, &ntc_h) != true)
		return false;
	ntc_whole = ((ntc_l | (ntc_h<<8)) & ntc_data_mask);
	*p_vol = MP2762A_NTC_REF_VOL * ntc_whole / MP2762A_NTC_VAL_MAX;
	return true;
}

int8_t mp2762a_read_reg_value(uint8_t reg, uint8_t* p_val)
{
    uint8_t val = 0;
    if (mp2762a_i2c_read_byte(reg, &val))
    {
        if (p_val != NULL)
        {
            *p_val = val;
            return 0;
        }
    }

    return -1;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

//void i2c_test(void)
//{
//	int32_t i = 0;
//	uint8_t data;
//	
//	for (i = 0; i <= 0x7f; i++)
//	{
//		if (nrf_drv_twi_rx(&m_twi2, i, &data, sizeof(uint8_t)) == NRF_SUCCESS)
//			printf("I2C address 0x%02x\r\n", i);
//		nrf_delay_ms(1);
//	}
//	printf("\r\n");
//	printf("I2C address test done\r\n");
//}

//void mp2762a_test_read_all(void)
//{
//	int i = 0;
//	uint8_t data = 0;
//	
//	for (i = 0; i <= 0x48; i++)
//	{
//		mp2762a_i2c_read_byte(i, &data);
//		printf("0x%02x: 0x%02x\r\n", i, data);
//		nrf_delay_ms(1);
//	}
//}

//void mp2762a_test_read_reg(uint8_t reg)
//{
//	uint8_t data = 0;

//	mp2762a_i2c_read_byte(reg, &data);
////	printf("0x%02x: 0x%02x\r\n", reg, data);
//	NRF_LOG_RAW_INFO("0x%02x: 0x%02x\r\n", reg, data);
//}

//void mp2762a_test(void)
//{
//	uint8_t data1 = 0;
//	uint8_t data2 = 0;
//	uint8_t data3 = 0;
//	uint8_t data4 = 0;
//	
//	mp2762a_i2c_read_byte(MP2762A_REG13H, &data1);
//	mp2762a_i2c_read_byte(MP2762A_REG14H, &data2);
//	mp2762a_i2c_read_byte(MP2762A_REG1AH, &data3);
//	mp2762a_i2c_read_byte(MP2762A_REG1BH, &data4);
//	NRF_LOG_RAW_INFO("MP2762A_REG13H: 0x%x, MP2762A_REG14H: 0x%x, MP2762A_REG1A-BH: 0x%x\r\n", data1, data2, (data3|(data4<<8)));
//}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

#include "ntc_tmp_conv.h"

#define MP2762A_INPUT_VOL_LIMIT		8500	// Unit in mV
#define MP2762A_CHG_CUR_LIMIT		1500	// Unit in mA
#define MP2762A_PRE_CHG_CUR_LIMIT	480		// Unit in mA
#define MP2762A_INPUT_CUR_LIMIT1	2000	// Unit in mA
#define MP2762A_INPUT_CUR_LIMIT2	2000	// Unit in mA
#define MP2762A_CHG_FULL_VOL_LIMIT	8450	// Unit in mV

#define NTC_REF_RES	47000	// Reference resistor value. Unit in ohm

static bool mp2762a_s_tmr_en = true;	// If true, safety timer is enabled; if false, disabled.

void mp2762a_init(void)
{
	mp2762a_input_vol_limit_set(MP2762A_INPUT_VOL_LIMIT);
	mp2762a_charge_ctrl(true);
	mp2762a_bfet_ctrl(true);
//	mp2762a_chg_cur_set(MP2762A_CHG_CUR_LIMIT);
	mp2762a_ntc_sample_proc(true);	// Set charge current according to temperature when initializing the charger chip
	mp2762a_field_update(MP2762A_REG0AH, MP2762A_JEITA_ISET_BIT_POS, MP2762A_JEITA_ISET_BIT_LEN, MP2762A_JEITA_ISET_20_PER);
	mp2762a_field_update(MP2762A_REG0AH, MP2762A_JEITA_VSET_BIT_POS, MP2762A_JEITA_VSET_BIT_LEN, MP2762A_JEITA_VSET_150_MV);
//	mp2762a_field_update(MP2762A_REG0AH, MP2762A_NTC_CTRL_BIT_POS, MP2762A_NTC_CTRL_BIT_LEN, MP2762A_NTC_CTRL_JEITA);	// JEITA is disabled by default
	mp2762a_field_update(MP2762A_REG0AH, MP2762A_NTC_CTRL_BIT_POS, MP2762A_NTC_CTRL_BIT_LEN, MP2762A_NTC_CTRL_DISABLE);
	mp2762a_field_update(MP2762A_REG0BH, MP2762A_SW_FREQ_BIT_POS, MP2762A_SW_FREQ_BIT_LEN, MP2762A_SW_FREQ_800K);	// Change SW freq. to enhance RF signal
	mp2762a_field_update(MP2762A_REG03H, MP2762A_PRE_CHG_CUR_BIT_POS, MP2762A_PRE_CHG_CUR_BIT_LEN, MP2762A_PRE_CHG_CUR_MA_TO_NUM(MP2762A_PRE_CHG_CUR_LIMIT));
	mp2762a_field_update(MP2762A_REG09H, MP2762A_F_CHG_TM_BIT_POS, MP2762A_F_CHG_TM_BIT_LEN, MP2762A_F_CHG_TM_20HRS);
	mp2762a_field_update(MP2762A_REG0AH, MP2762A_NTC_WARM_BIT_POS, MP2762A_NTC_WARM_BIT_LEN, MP2762A_NTC_WARM_51_3P_55D);
	mp2762a_field_update(MP2762A_REG09H, MP2762A_EN_TMR_BIT_POS, MP2762A_EN_TMR_BIT_LEN, MP2762A_EN_TMR_ENABLE);
	mp2762a_input_cur_limit1_set(MP2762A_INPUT_CUR_LIMIT1);
	mp2762a_input_cur_limit2_set(MP2762A_INPUT_CUR_LIMIT2);
	mp2762a_field_update(MP2762A_REG04H, MP2762A_CHG_FULL_VOL_BIT_POS, MP2762A_CHG_FULL_VOL_BIT_LEN, MP2762A_CHG_FULL_VOL_VOL_TO_NUM(MP2762A_CHG_FULL_VOL_LIMIT));
}

// Charger safety timer process
// If there is only solar power supply, disable charger safety timer; else enable it.
// MP2762A safety timer is enabled by default.
// This function is designed to be called in a period function, e.g., 1 s period function
static void mp2762a_s_tm_proc(void)
{
	if (no_external_power() == true || is_charger_power_on() != true)
	{
		mp2762a_s_tmr_en = true;	// For the case solar panel is disconnect and then connected
		return;
	}
	if (only_solar_power() == true)
	{
		if (mp2762a_s_tmr_en == true)
		{
			mp2762a_field_update(MP2762A_REG09H, MP2762A_EN_TMR_BIT_POS, MP2762A_EN_TMR_BIT_LEN, MP2762A_EN_TMR_DISABLE);
			mp2762a_s_tmr_en = false;
		}
		NRF_LOG_FLUSH();
	}
	else
	{
		if (mp2762a_s_tmr_en == false)
		{
			mp2762a_field_update(MP2762A_REG09H, MP2762A_EN_TMR_BIT_POS, MP2762A_EN_TMR_BIT_LEN, MP2762A_EN_TMR_ENABLE);
			mp2762a_s_tmr_en = true;
		}
	}
}

// Get resistance of battery NTC resistor
// Para. p_ntc_res: NTC resistance result. Unit in ohm
// Return value: if true, OK; if false, failed to get data.
// New P2 board NTC circuit. 20201031
// ------------|
//  Charger IC |
//             |
//        VNTC |------|
//             |      |
//             | [R243,47kohm]
//             |      |
//         NTC |------+------[R244,0ohm]----[BAT_NTC,10kohm,25dC]
//             |  [R262,DNP]                         |
//-------------|      |                              |
//                   GND                            GND
bool bat_ntc_res_get(double *p_ntc_res)
{
	double ntc_vol;
	double ntc_res;	// NTC resistance. Unit in ohm
	uint32_t ntc_res_max = 0xffffffff;
	
	if (mp2762a_ntc_vol_get(&ntc_vol) != true)
		return false;
	if (MP2762A_NTC_REF_VOL <= ntc_vol)
		ntc_res = ntc_res_max;
	else
		ntc_res = ntc_vol * NTC_REF_RES / (MP2762A_NTC_REF_VOL - ntc_vol);
	*p_ntc_res = ntc_res;
	return true;
}

// Get battery temperature via NTC resistance
// Para. res: resistance value, unit in ohm
// Return value: temperature, unit in degree Celsius
int16_t bat_temp_get(double res)
{
	int16_t temp = 0;
	CalTemp(res, &temp);
	return temp;
}


static bool mp2762a_deflt_chg_cur_needed = true;	// Default charge current is needed to be configured to charger. If true, yes, else, not.

// Calculate absolute value of int32_t type
static int32_t fabs_int32(int32_t value)
{
	if (value < 0)
		return -value;
	return value;
}

// Charger current calibration for temp left end. Left side of the temperature zone is open.
//   0 mA | 400 mA |   1500 mA   | 400 mA | 0 mA
// -----------------------------------------------> T (DegC)
//       -30       0             45       60
// i_limit: i limit of the temp zone, unit in mA
// i_limit_r: i limit of the right temp zone, unit in mA
// i_setting: i setting in charger, unit in mA
// temp_cur: current temperature, unit in DegC
// temp_limit_r: right (upper) temp limt of current temp zone
// Return value: calibrated current value, unit in mA
// Example: chg_cur_cali_l(0, 400, 400, -31, -30), returns 400 mA
static uint16_t chg_cur_cali_l(uint16_t i_limit, uint16_t i_limit_r, uint16_t i_setting, int16_t temp_cur, int16_t temp_limit_r)
{
	uint16_t chg_cur = i_limit;
	if (i_setting == i_limit_r)
	{
		if (fabs_int32(temp_cur - temp_limit_r) <= CHGR_TMP_DELTA)
			chg_cur = i_limit_r;
	}
	return chg_cur;
}

// Charger current calibration. There are left and right temperature zones.
//   0 mA | 400 mA |   1500 mA   | 400 mA | 0 mA
// -----------------------------------------------> T (DegC)
//       -30       0             45       60
// i_limit_l: i limit of the left temp zone, unit in mA
// i_limit: i limit of the temp zone, unit in mA
// i_limit_r: i limit of the right temp zone, unit in mA
// i_setting: i setting in charger, unit in mA
// temp_cur: current temperature, unit in DegC
// temp_limit_l: left (lower) temp limt of current temp zone
// temp_limit_r: right (upper) temp limt of current temp zone
// Return value: calibrated current value, unit in mA
// Example: chg_cur_cali_lr(0, 400, 1500, 1500, -2, -30, 0), returns 1500 mA
static uint16_t chg_cur_cali_lr(uint16_t i_limit_l, uint16_t i_limit, uint16_t i_limit_r, uint16_t i_setting, 
							int16_t temp_cur, int16_t temp_limit_l, int16_t temp_limit_r)
{
	uint16_t chg_cur = i_limit;
	if (i_setting == i_limit_l)
	{
		if (fabs_int32(temp_cur - temp_limit_l) <= CHGR_TMP_DELTA)
			chg_cur = i_limit_l;
	}
	if (i_setting == i_limit_r)	// Note: i_limit_l and i_limit_r might be same value, so use "if" here while not "else if"
	{
		if (fabs_int32(temp_cur - temp_limit_r) <= CHGR_TMP_DELTA)
			chg_cur = i_limit_r;
	}
	return chg_cur;
}

// Charger current calibration for right end. Right side of the temperature zone is open.
//   0 mA | 400 mA |   1500 mA   | 400 mA | 0 mA
// -----------------------------------------------> T (DegC)
//       -30       0             45       60
// i_limit_l: i limit of the left temp zone, unit in mA
// i_limit: i limit of the temp zone, unit in mA
// i_setting: i setting in charger, unit in mA
// temp_cur: current temperature, unit in DegC
// temp_limit_l: left (lower) temp limt of current temp zone
// Return value: calibrated current value, unit in mA
// Example: chg_cur_cali_r(400, 0, 400, 61, 60), returns 400 mA
static uint16_t chg_cur_cali_r(uint16_t i_limit_l, uint16_t i_limit, uint16_t i_setting, int16_t temp_cur, int16_t temp_limit_l)
{
	uint16_t chg_cur = i_limit;
	if (i_setting == i_limit_l)
	{
		if (fabs_int32(temp_cur - temp_limit_l) <= CHGR_TMP_DELTA)
			chg_cur = i_limit_l;
	}
	return chg_cur;
}

// Sample battery NTC process
// Read NTC resistance and calculate battery temperature.
// According to the temperature, set different charge current.
// This function is designed to be called in a period function, e.g., 1 s period function
// Param. init_call: true, this function is called in initialization; false, not.
static void mp2762a_ntc_sample_proc(bool init_call)
{
	double ntc_res = 0.0;
	static uint32_t interval = 0;	// Unit in seconds
	int16_t temp = 0;
	int16_t temp_ntc = 0;
	int32_t temp_nordic = 0;
	uint16_t chg_cur_settting = 0;	// Charge current setting
	uint16_t chg_cur_target = 0;	// Charge current target
	double chg_cur_prac = 0.0;		// Charge current in practical
	uint8_t charge_en = MP2762A_CHG_EN_DISABLE;

	static uint8_t charge1_en = 0;
	static uint8_t temp_disable = 0;
	static uint8_t temp_disable_cnt_29 = 0;   //have time to improve write ways
	static uint8_t temp_disable_cnt_31 = 0;
	static uint8_t temp_disable_cnt_59 = 0;
	static uint8_t temp_disable_cnt_61 = 0;
	static uint8_t temp_disable_cnt_45 = 0;
	
	//if (no_external_power() == true || is_charger_power_on() != true) //warning the condition has changed
	if (no_external_power() == true)
	{
		interval = 0;
		mp2762a_deflt_chg_cur_needed = true;
		return;
	}
	
	if (init_call != true)
	{
		interval++;
		if (interval < NTC_SAMPLE_INTVL)	// Sample every NTC_SAMPLE_INTVL seconds
			return;
		interval = 0;
	}
	else
	{
		interval = 0;
		mp2762a_deflt_chg_cur_needed = true;
	}
	
	if (bat_ntc_res_get(&ntc_res) != true)
	{
		NRF_LOG_RAW_INFO("ERROR: failed to read NTC resistance\r");
		return;
	}
	
	temp_ntc = bat_temp_get(ntc_res);
	temp_nordic = chip_temp_get();	// Read Nordic buildin temperature to make current software compatible with old board
	if (fabs_int32(temp_nordic - temp_ntc) > TEMP_DIFF_LIMIT)
		temp = temp_nordic;
	else
		temp = temp_ntc;

	mp2762a_chg_cur_get(&chg_cur_settting);	// Read back charge current setting in charger
	
	if (temp <= CHG_TEMP_LIMIT1)
	{
		chg_cur_target = (mp2762a_deflt_chg_cur_needed == true) ? CHG_CUR_LIMIT0 :
						chg_cur_cali_l(CHG_CUR_LIMIT0, CHG_CUR_LIMIT400, chg_cur_settting, temp, CHG_TEMP_LIMIT1);
		temp_disable_cnt_31++;
		if (temp_disable_cnt_31 > 3)
		{
			temp_disable = 1;
		}
		temp_disable_cnt_29 = 0;
	 	//temp_disable_cnt_31 = 0;
	 	temp_disable_cnt_59 = 0;
	 	temp_disable_cnt_61 = 0;
		temp_disable_cnt_45 = 0;				
	}
	else if (temp <= CHG_TEMP_LIMIT2)
	{
		chg_cur_target = (mp2762a_deflt_chg_cur_needed == true) ? CHG_CUR_LIMIT400 :
						chg_cur_cali_lr(CHG_CUR_LIMIT0, CHG_CUR_LIMIT400, CHG_CUR_LIMIT1500, chg_cur_settting, temp, CHG_TEMP_LIMIT1, CHG_TEMP_LIMIT2);
		temp_disable_cnt_29++;
		if (temp_disable_cnt_29 > 3)
		{
			temp_disable = 0;
		}
		//temp_disable_cnt_29 = 0;
	 	temp_disable_cnt_31 = 0;
	 	temp_disable_cnt_59 = 0;
	 	temp_disable_cnt_61 = 0;
		temp_disable_cnt_45 = 0;
	}
	else if (temp < CHG_TEMP_LIMIT3)
	{
		chg_cur_target = (mp2762a_deflt_chg_cur_needed == true) ? CHG_CUR_LIMIT1500 :
						chg_cur_cali_lr(CHG_CUR_LIMIT400, CHG_CUR_LIMIT1500, CHG_CUR_LIMIT400, chg_cur_settting, temp, CHG_TEMP_LIMIT2, CHG_TEMP_LIMIT3);
		temp_disable_cnt_45++;
		if (temp_disable_cnt_45 > 3)
		{
			temp_disable = 0;
		}
		temp_disable_cnt_31 = 0;
	 	temp_disable_cnt_59 = 0;
	 	temp_disable_cnt_61 = 0;
		temp_disable_cnt_29 = 0;
		

	}
	else if (temp < CHG_TEMP_LIMIT4)
	{
		chg_cur_target = (mp2762a_deflt_chg_cur_needed == true) ? CHG_CUR_LIMIT400 :
						chg_cur_cali_lr(CHG_CUR_LIMIT1500, CHG_CUR_LIMIT400, CHG_CUR_LIMIT0, chg_cur_settting, temp, CHG_TEMP_LIMIT3, CHG_TEMP_LIMIT4);
		temp_disable_cnt_59++;
		if (temp_disable_cnt_59 > 3)
		{
			temp_disable = 0;
		}
		temp_disable_cnt_29 = 0;
	 	temp_disable_cnt_31 = 0;
	 	//temp_disable_cnt_59 = 0;
	 	temp_disable_cnt_61 = 0;
		temp_disable_cnt_45 = 0;
	}
	else
	{
		chg_cur_target = (mp2762a_deflt_chg_cur_needed == true) ? CHG_CUR_LIMIT0 :
							chg_cur_cali_r(CHG_CUR_LIMIT400, CHG_CUR_LIMIT0, chg_cur_settting, temp, CHG_TEMP_LIMIT4);
		temp_disable_cnt_61++;
		if (temp_disable_cnt_61 > 3)
		{
			temp_disable = 1;
		}
	    temp_disable_cnt_29 = 0;
	 	temp_disable_cnt_31 = 0;
	 	temp_disable_cnt_59 = 0;
		temp_disable_cnt_45 = 0;
	}
	mp2762a_deflt_chg_cur_needed = false;
	
	if (chg_cur_settting != chg_cur_target)	// Only update the current setting when the old is different with new
		mp2762a_chg_cur_set(chg_cur_target);
	
	mp2762a_charge_en_get(&charge_en);
	if (chg_cur_target == CHG_CUR_LIMIT0 && charge_en == MP2762A_CHG_EN_ENABLE)
	{
		// mp2762a_charge_ctrl(false);
		// charge_en = MP2762A_CHG_EN_DISABLE;
	}
	else if (chg_cur_target != CHG_CUR_LIMIT0 && charge_en == MP2762A_CHG_EN_DISABLE)
	{
		// mp2762a_charge_ctrl(true);
		// charge_en = MP2762A_CHG_EN_ENABLE;
	}
	
	if (temp_disable && charge_en == MP2762A_CHG_EN_ENABLE)
	{
		monet_data.charge_mode_disable = 1;
		mp2762a_charge_ctrl(false);
		charge_en = MP2762A_CHG_EN_DISABLE;
		charge1_en = 1;
		NRF_LOG_RAW_INFO("mp2762a_ntc_sample_proc. Outside Temp range suspend charging.charge ON \r");
		mppt_process_nml_deinit();
		// setChargerOff();
        setNtcOn();
		solar_chg_mode_select(FUNC_JUMP_POINT_0);
	}

	if (temp_disable && charge_en == MP2762A_CHG_EN_DISABLE)  //dut to condition change add
	{
		solar_chg_mode_set(SOLAR_CHG_MODE1);
		monet_data.charge_mode_disable = 1;
		mp2762a_charge_ctrl(false);
		charge_en = MP2762A_CHG_EN_DISABLE;
		charge1_en = 1;
		NRF_LOG_RAW_INFO("mp2762a_ntc_sample_proc. Outside Temp range suspend charging.charge OFF \r");
		mppt_process_nml_deinit();
		//solar_chg_mode_select(FUNC_JUMP_POINT_0);
	}

	if ((temp_disable) == 0 && charge1_en == 1)
	{
		charge1_en = 0;
		monet_data.charge_mode_disable = 0;
		mp2762a_charge_ctrl(true);
		charge_en = MP2762A_CHG_EN_ENABLE;
		NRF_LOG_RAW_INFO("mp2762a_ntc_sample_proc. Inside Temp range resume charging  \r");
		mppt_process_nml_deinit();
		solar_chg_mode_select(FUNC_JUMP_POINT_0);
	}

	chg_cur_prac = mp2762a_bat_chg_cur_get();	// Battery charge current practical value
	NRF_LOG_RAW_INFO("Bat ResNTC %u ohm, TempNTC %d dC, TempNordic %d dC, TempUsed %d dC\r", 
					(uint32_t)ntc_res, temp_ntc, temp_nordic, temp);
	NRF_LOG_RAW_INFO("ChgCurCfg %u mA, ChgCurPrac %u mA, ChgEN %u\r", 
					chg_cur_target, (uint32_t)chg_cur_prac, charge_en);
	NRF_LOG_RAW_INFO("temp_disable (%d), charge_en (%d), charge1_en (%d)\r", 
					temp_disable, charge_en, charge1_en);
}


int32_t get_temp(uint8_t mode)  // mode 0 is ntc Temp. mode1 is nrf temp.
{
	int32_t temp_ntc = 0;
	int32_t temp_nordic = 0;
	double ntc_res = 0.0;
	if (bat_ntc_res_get(&ntc_res) != true)
	{
		NRF_LOG_RAW_INFO("ERROR: failed to read NTC resistance\r");
		temp_ntc = 0;
	}
	else
	{
		temp_ntc = (int32_t) bat_temp_get(ntc_res);
	}
	temp_nordic = chip_temp_get();	// Read Nordic buildin temperature to make current software compatible with old board
	if (mode == 0)
	{
		return temp_ntc;
	}
	else
	{
		return temp_nordic;
	}
}
// Charger process
// This function is designed to be called in a period function, e.g., 1 s period function
void mp2762a_proc(void)
{
	mp2762a_s_tm_proc();
	mp2762a_ntc_sample_proc(false);
}
