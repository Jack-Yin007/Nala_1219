#ifndef __PLATFORM_HAL_DRV_H
#define __PLATFORM_HAL_DRV_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "nala_hal.h"
#include "ble_data_c.h"

#define pf_log_raw(onoff, ...) do {/*if (onoff)*/ {NRF_LOG_RAW_INFO(__VA_ARGS__); NRF_LOG_FLUSH();}} while (0);
#define pf_log_raw_force(...) do {NRF_LOG_RAW_INFO(__VA_ARGS__); NRF_LOG_FLUSH();} while (0);

extern const nrf_drv_twi_t m_twi;	// TWI0, for pins BLE_I2C1_SCL and BLE_I2C1_SDA
extern const nrf_drv_twi_t m_twi2;	// TWI1, for pins BLE_I2C2_SCL and BLE_I2C2_SDA

typedef enum
{
    PF_ACC_MODE_OFF,
    PF_ACC_MODE_DRIVING_BEHAVIOR,
    PF_ACC_MODE_COLLISION_REPORT,
    PF_ACC_MODE_WAKE_ON_MOTION,
    PF_ACC_MODE_MOTION_DETECTION,
    PF_ACC_MODE_MOTION_DETECTION_LOW_POWER,
    PF_ACC_MODE_LAST
} pf_AccWorkMode_t;

#define PF_IS_VALID_ACC_WORKMODE(a) (((a) >= PF_ACC_MODE_OFF) && ((a) < PF_ACC_MODE_LAST))

extern volatile uint8_t adc_convert_over;
extern volatile uint32_t gTimer;
extern volatile uint32_t accInterruptFlag;
extern volatile uint8_t  keep_sync;

void pf_adc_init(void);
void pf_adc_start(void);
void pf_adc_poll_finish(void);
void pf_adc_deinit(void);

void gpio_deinit(uint32_t index);
int8_t pf_gpio_cfg(uint32_t index, atel_gpio_cfg_t cfg);
int8_t pf_gpio_write(uint32_t index, uint32_t value);
int8_t pf_gpio_toggle(uint32_t index);
int32_t pf_gpio_read(uint32_t index);
int32_t pf_gpio_dir_get(uint32_t index);

void pf_i2c_init(void);
void pf_i2c_uninit(void);
int32_t platform_imu_i2c_write(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);
int32_t platform_imu_i2c_write2(void *handle, uint8_t Reg, uint8_t val);
int32_t platform_imu_i2c_2_write(uint8_t addr, uint8_t reg, uint8_t *pbuf, uint16_t len);
int32_t platform_imu_i2c_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);
int32_t platform_imu_i2c_2_read(uint8_t addr, uint8_t reg, uint8_t *pbuf, uint16_t len);

void pf_systick_start(uint32_t period_ms);
void pf_systick_stop(void);
uint32_t pf_systick_get(void);
void pf_systick_change(void);

//void pf_pwm_start(void);
//void pf_pwm_stop(void);

void pf_print_mdm_uart_rx(uint8_t cmd, uint8_t *p_data, uint8_t len);
void pf_print_mdm_uart_tx_init(void);
void pf_print_mdm_uart_tx(uint8_t data);
void pf_print_mdm_uart_tx_flush(void);

void pf_uart_mdm_init(uint8_t baud, uint8_t mode);
void pf_uart_mdm_deinit(void);
uint32_t pf_uart_mdm_tx_one(uint8_t byte);
uint32_t pf_uart_mdm_rx_one(uint8_t *p_data);
bool pf_uart_mdm_tx_queue_is_empty(void);

void pf_uart_peri_init(uint8_t baud, uint8_t mode);
void pf_uart_peri_deinit(void);
uint32_t pf_uart_peri_tx_one(uint8_t byte);
uint32_t pf_uart_peri_rx_one(uint8_t *p_data);
bool pf_uart_peri_tx_queue_is_empty(void);

void ble_dg_info_to_tpms_enable(void);
void ble_dg_info_to_tpms_disable(void);
bool is_ble_dg_info_to_tpms(void);
int ble_dg_printf(const char* fmt, ...);

void pf_wdt_init(void);
void pf_wdt_kick(void);
void delay_10ms_wds(uint32_t count);

void pf_bootloader_pre_enter(void);
uint32_t pf_bootloader_enter(void);

void pf_BuzzerOff(void);
void pf_BuzzerOn(void);

void pf_cfg_before_hibernation(void);
void pf_cfg_recover_from_hibernation(void);
void pf_cfg_before_sleep(void);
void pf_cfg_recovery_from_sleep(void);

void pf_delay_ms(uint32_t ms);

void pf_dtm_enter(void);
void pf_dtm_init(void);
void pf_dtm_process(void);
void pf_dtm_exit(void);
void pf_dtm_cmd(uint8_t cmd, uint8_t freq, uint8_t len, uint8_t payload);
void pf_dtm_enter_nala(void);
	
void pf_imu_init(void);
void pf_imu_sensitivity_set(uint32_t val);
//void pf_imu_reset(void);
//uint8_t pf_imu_int_src_get(uint8_t int_num);
void pf_imu_value_stream(void);
//void pf_imu_workmode_set(pf_AccWorkMode_t workmode);

void pf_mdm_pwr_init(void);
void pf_mdm_pwr_deinit(void);
void pf_mdm_pwr_ctrl(bool state);
void pf_mdm_pwr_key_ctrl(bool state);

void ble_disconnect_with_peer(uint8_t inst_id, uint8_t type);

#define PF_ADC_CONVERT_REFERENCE_VOLTAGE(x) ((double)(x) * ADC_REFERENCE_VOLTAGE_FACTOR)
#define PF_ADC_RAW_TO_BATT_MV(v) (((((double)v) / (double)ADC_RESOLUTION_VALUE) / (double)ADC_BUB_VOL_MULTIPLIER_FACTOR) + ADC_BUB_VOL_MODIFICATION_VALUE)
#define PF_ADC_RAW_TO_MAIN_MV(v) (((((double)v) / (double)ADC_RESOLUTION_VALUE) / (double)ADC_MAIN_VOL_MULTIPLIER_FACTOR) + ADC_MAIN_VOL_MODIFICATION_VALUE)

#define PF_SYSTICK_TIMER_USE_RTC (1)

#define PF_IMU_LSM6DSL_I2C_ADDR (0x6b)
#define PF_IMU_LIS3DH_I2C_ADDR  (0x19)
#define PF_IMU_LIS2DH_I2C_ADDR  (0x19)

#define PF_PROTOCOL_TXRX_FILTER_EN (1)
// #define PF_PROTOCOL_RXFILTER_FORMAT(x) (((x) == 'G') || ((x) == 'E'))
// #define PF_PROTOCOL_TXFILTER_FORMAT(x) (((x) == 'g') || ((x) == 'e'))
#define PF_PROTOCOL_RXFILTER_FORMAT(x) (((x) != 'A') && ((x) != 'Q') && ((x) != 'K') && ((x) != '2'))
#define PF_PROTOCOL_TXFILTER_FORMAT(x) (((x) != 'a') && ((x) != 'q') && ((x) != 'k') && ((x) != '1'))

#define PF_USE_JUMP_TO_ENTER_BOOT (0)

void printf_hex_and_char(uint8_t *p_data, uint16_t len);


void pf_sd_hfclk_request(void);
void pf_sd_hfclk_release(void);


#ifdef __cplusplus
}
#endif
#endif /* __PLATFORM_HAL_DRV_H */
