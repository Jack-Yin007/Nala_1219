#include "mux_mcu.h"
#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "platform_hal_drv.h"
#include "version.h"

#define MUX_MCU_PADDING_BYTE	0
#include "mux_mcu.h"

typedef struct
{
	const uint8_t *opt;
	uint32_t len;
} mux_mcu_opt_t;

static bool mux_mcu_w = false;
//static bool mux_mcu_r = false;
static mux_mcu_opt_t mux_mcu_opt[] = 
{
	{(uint8_t *)"\x24\x03\x4d\x4d\x00\x65\x0d\x0a", sizeof("\x24\x03\x4d\x4d\x00\x65\x0d\x0a")-1},	// MUX_SETTING_0
	{(uint8_t *)"\x24\x03\x4d\x4d\x01\x64\x0d\x0a", sizeof("\x24\x03\x4d\x4d\x01\x64\x0d\x0a")-1},	// MUX_SETTING_1
	{(uint8_t *)"\x24\x03\x4d\x4d\x02\x63\x0d\x0a", sizeof("\x24\x03\x4d\x4d\x02\x63\x0d\x0a")-1},	// MUX_SETTING_2
	{(uint8_t *)"\x24\x03\x4d\x4d\x03\x62\x0d\x0a", sizeof("\x24\x03\x4d\x4d\x03\x62\x0d\x0a")-1},	// MUX_SETTING_3
	{(uint8_t *)"\x24\x03\x4d\x4d\x04\x61\x0d\x0a", sizeof("\x24\x03\x4d\x4d\x04\x61\x0d\x0a")-1},	// MUX_SETTING_4
	{(uint8_t *)"\x24\x03\x4d\x4d\x05\x60\x9f\x0a", sizeof("\x24\x03\x4d\x4d\x05\x60\x9f\x0a")-1},	// MUX_SETTING_5
	{(uint8_t *)"\x24\x03\x4d\x4d\x06\x5f\x0d\x0a", sizeof("\x24\x03\x4d\x4d\x06\x5f\x0d\x0a")-1},	// MUX_SETTING_6
	{(uint8_t *)"\x24\x03\x4d\x4d\x07\x5e\x0d\x0a", sizeof("\x24\x03\x4d\x4d\x07\x5e\x0d\x0a")-1},	// MUX_SETTING_7
};
static uint8_t mux_mcu_w_buf[64] = {0};
static uint32_t mux_mcu_w_len;
static bool mux_update = false;
static bool mux_version_read_flag = false;
static uint8_t mux_version_info_type = 0;

bool mux_mcu_write(const uint8_t *p_data, uint32_t len)
{
	int i = 0;
	uint8_t w_len = 0;
	uint8_t w_buf[MUX_MCU_PACK_SIZE];
	uint32_t len_left = len;
	ret_code_t ret;
	
	for (i = 0; i < len; /* */)
	{
		memset(w_buf, 0, sizeof(w_buf));
		w_len = (len_left > (MUX_MCU_PACK_SIZE - 1))? (MUX_MCU_PACK_SIZE - 1): len_left;
		w_buf[0] = w_len;
		memcpy(&w_buf[1], &p_data[i], w_len);
		if ((w_len+1) < MUX_MCU_PACK_SIZE)
			memset(&w_buf[w_len+1], MUX_MCU_PADDING_BYTE, (MUX_MCU_PACK_SIZE - (w_len+1)));
		ret = nrf_drv_twi_tx(&m_twi2, MUX_MCU_I2C_ADDR, w_buf, sizeof(w_buf), false);
		if (ret != NRFX_SUCCESS)
			return false;
		len_left -= w_len;
		i += w_len;
	}
	return true;
}

// MUX MCU needs a read after write
// Para. p_buf: space to store data
// Para. len: length of data to read
// Return value: true, OK; false, failed to read
bool mux_mcu_read(uint8_t *p_buf, uint32_t len)
{
	ret_code_t ret;
	ret = nrf_drv_twi_rx(&m_twi2, MUX_MCU_I2C_ADDR, p_buf, len);
	return (ret == NRFX_SUCCESS)? true: false;
}

void mux_update_mode_set(void)
{
	mux_update = true;
}

void mux_update_mode_clear(void)
{
	mux_update = false;
}

bool mux_in_update_mode(void)
{
	return mux_update;
}

static bool find_key_pos(const uint8_t *p_buf, uint32_t len, uint8_t key, uint32_t *p_pos)
{
	int i = 0;
	for (i = 0; i < len; i++)
	{
		if (p_buf[i] == key)
		{
			*p_pos = i;
			return true;
		}
	}
	return false;
}

static void mux_version_report(uint8_t major_mux, uint8_t minor_mux, uint8_t revision_mux, uint8_t build_mux)
{
	uint8_t param[8] = {0};

	param[0] = 0x05;	// Component nunmber for Nala MUX MCU
	param[1] = mux_version_info_type;	// Information Type Version
	param[2] = 0x00;	// Bit number inside bitmask for a version
	param[3] = 0x04;	// 4 bytes of version
	param[4] = major_mux;
	param[5] = minor_mux;
	param[6] = revision_mux;
	param[7] = build_mux;
	BuildFrame(0xAA, param, 8);
}

static void mux_version_data_proc(const uint8_t *p_buf, uint32_t len)
{
	uint8_t data_len = p_buf[0];
	uint32_t pos = 0;
	uint8_t major_mux = 255, minor_mux = 255, revision_mux = 255, build_mux = 255;
	
	if (data_len > 0 && data_len < 255 && find_key_pos(&p_buf[1], data_len, 0x76, &pos) == true)	// 0x76 = 'v'
	{
		major_mux = p_buf[1+pos+1];
		minor_mux = p_buf[1+pos+2];
		revision_mux = p_buf[1+pos+3];
		build_mux = p_buf[1+pos+4];
	}
	mux_version_report(major_mux, minor_mux, revision_mux, build_mux);
}

extern void i2c_data_send_with_queue(void);
extern void i2c_received_data_proc(uint8_t * p_data, uint16_t data_len);
extern bool is_tx_queue_empty(void);
//static uint32_t i2c_wr_needed = 0;	// Write flag. MUX MCU needs a I2C write after multiple I2C reads
static uint32_t i2c_wr_0_cnt = 0;

// MUX MCU mode process
// This function is designed for polling use.
void mux_mcu_proc(void)
{
	if (mux_in_update_mode() == true)
		return;
	
	if (pf_gpio_read(GPIO_ST_UART1_TO_UART2_EN) && pf_gpio_read(GPIO_ST_UART2_TO_UART3_EN))
	{
		uint8_t buf[MUX_MCU_PACK_SIZE];
		bool ret = false;
		
//		i2c_wr_needed = 10;
		ret = mux_mcu_read(buf, MUX_MCU_PACK_SIZE);	// Read length is fixed value MUX_MCU_PACK_SIZE. MUX I2C code is to be updated.

		if (mux_version_read_flag == true)
		{
			mux_version_read_flag = false;
			NRF_LOG_RAW_INFO("mux_mcu_proc(), mux_version_read_flag\r");
			NRF_LOG_RAW_HEXDUMP_INFO(buf, MUX_MCU_PACK_SIZE);
			NRF_LOG_FLUSH();
			if (ret == true)
				mux_version_data_proc(buf, MUX_MCU_PACK_SIZE);
			else
			{
				NRF_LOG_RAW_INFO("Error: failed to read MUX I2C\r");
				NRF_LOG_FLUSH();
				mux_version_report(255, 255, 255, 255);	// Meet error when read MUX I2C
			}
		}
		else
		{
			if (ret == true)
			{
				if (com_method_zazu_get() == COM_METHOD_ZAZU_CAN)
				{
					if (buf[0] > 0 && buf[0] < BLE_DATA_RECV_BUFFER_CELL_LEN)
						i2c_received_data_proc(&buf[1], buf[0]);	// buf[0] is the length of data
					else
					{
						NRF_LOG_RAW_INFO("mux_mcu_proc() Rx len(%u) err\r\n", buf[0]);
						NRF_LOG_FLUSH();
					}
				}
			}
			else
			{
				NRF_LOG_RAW_INFO("mux_mcu_proc() Rx read err\r\n");
				NRF_LOG_FLUSH();
			}
		}
	}
	
//	if (mux_mcu_w != true)
//		return;
	
	if (pf_gpio_read(GPIO_ST_UART1_TO_UART2_EN) && !pf_gpio_read(GPIO_ST_UART2_TO_UART3_EN))
	{
		if (mux_mcu_w == true)
		{
			mux_mcu_w = false;
			NRF_LOG_RAW_INFO("mux_mcu_proc(), mux_mcu_w, len%u\r", mux_mcu_w_len);
			NRF_LOG_RAW_HEXDUMP_INFO(mux_mcu_w_buf, mux_mcu_w_len);
			NRF_LOG_FLUSH();
			if (mux_mcu_write(mux_mcu_w_buf, mux_mcu_w_len) != true)
			{
				NRF_LOG_RAW_INFO("Error: failed to write MUX I2C\r");
				NRF_LOG_FLUSH();
			}
		}
		else
		{
			if (com_method_zazu_get() == COM_METHOD_ZAZU_CAN)
			{
				if (is_tx_queue_empty() == true)
				{
//					if (i2c_wr_needed > 0)
//					{
						uint8_t arr[] = {0x00, 0x00, 0x00};	// Array size should not be 0
						mux_mcu_write(arr, sizeof(arr));	// Send all-0-pkg to MUX
						i2c_wr_0_cnt++;
						if (i2c_wr_0_cnt > (1000/BLE_DATA_SEND_TIMER_PERIOD_MS))
						{
							NRF_LOG_RAW_INFO("mux_mcu_proc(), write all-0-pkg to MUX, %u times\r\n", i2c_wr_0_cnt);
							NRF_LOG_FLUSH();
							i2c_wr_0_cnt = 0;
						}
//					}
				}
				else
					i2c_data_send_with_queue();
//				i2c_wr_needed--;
			}
		}
	}
}

// Set STM32 MCU MUX value
// Para. mux_sel, see @MCU_MUX_selections
// Return value, if true, OK; else, failed
bool mcu_mux_set(uint8_t mux_sel)
{
	bool mux_sel_error = false;
	
	switch (mux_sel)
	{
		case MUX_SETTING_0:
		case MUX_SETTING_1:
		case MUX_SETTING_2:
		case MUX_SETTING_4:
			break;
		default:
			mux_sel_error = true;
			break;
	}
	if (mux_sel_error == true)
	  goto MUX_SEL_ERROR_PROC;
//	pf_gpio_write(GPIO_ST_UART1_TO_UART3_EN, (BIT_IS_SET(mux_sel, 2) > 0)? 1: 0);
//	pf_gpio_write(GPIO_ST_UART2_TO_UART3_EN, (BIT_IS_SET(mux_sel, 1) > 0)? 1: 0);
//	pf_gpio_write(GPIO_ST_UART1_TO_UART2_EN, (BIT_IS_SET(mux_sel, 0) > 0)? 1: 0);
	mux_mcu_w = true;
	memcpy(mux_mcu_w_buf, mux_mcu_opt[mux_sel].opt, mux_mcu_opt[mux_sel].len);
	mux_mcu_w_len = mux_mcu_opt[mux_sel].len;
	return true;
	
MUX_SEL_ERROR_PROC:
	return false;
}

void mux_version_read(uint8_t info_type)
{
	uint8_t data_w[] = {0x24, 0x01, 0x56, 0xA9};	// 'V' command
	mux_mcu_w = true;
	memcpy(mux_mcu_w_buf, data_w, sizeof(data_w));
	mux_mcu_w_len = sizeof(data_w);
	mux_version_read_flag = true;
	mux_version_info_type = info_type;
	mux_version_info_type = mux_version_info_type;	// To avoid compiler warning
}

static volatile bool mux_com_timer = false;

// Set MUX communication timer
void mux_com_timer_set(void)
{
	mux_com_timer = true;
}

// Check MUX communication timer state
// If true, triggered; else, not
bool mux_com_timer_is_triggered(void)
{
	return mux_com_timer;
}

// Clear MUX communication timer
void mux_com_timer_clear(void)
{
	mux_com_timer = false;
}
