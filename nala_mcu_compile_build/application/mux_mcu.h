#ifndef __MUX_MCU_H__
#define __MUX_MCU_H__

#include "stdbool.h"
#include "stdint.h"

#define MUX_MCU_I2C_ADDR	0
#define MUX_MCU_PACK_SIZE	255

// @MCU_MUX_selections
#define MUX_SETTING_0	0	// (000)b, Motherboard <==> External 12 Pin connector, default setting
#define MUX_SETTING_1	1	// (001)b, Motherboard <==> Internal Cargo sensor
#define MUX_SETTING_2	2	// (010)b, Internal Cargo Sensor <==> External 12 Pin Connector
#define MUX_SETTING_3	3	// (011)b, Motherboard <==> ESP32 WiFi Chip (Future)
#define MUX_SETTING_4	4	// (100)b, Motherboard <==> STM32G070KBT6
#define MUX_SETTING_5	5	// (101)b, Motherboard <==> CAN Bus Interface (SPI) (Future)
#define MUX_SETTING_6	6	// (110)b, Not used currently
#define MUX_SETTING_7	7	// (111)b, Not used currently

bool mux_mcu_write(const uint8_t *p_data, uint32_t len);
bool mux_mcu_read(uint8_t *p_buf, uint32_t len);
bool mcu_mux_set(uint8_t mux_sel);
void mux_mcu_proc(void);

void mux_update_mode_set(void);
void mux_update_mode_clear(void);
bool mux_in_update_mode(void);

void mux_version_read(uint8_t info_type);

void mux_com_timer_set(void);
void mux_com_timer_clear(void);
bool mux_com_timer_is_triggered(void);

#endif	// #ifndef __MUX_MCU_H__
