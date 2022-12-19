/*
 * system.h
 *
 *  Created on: Dec 12, 2015
 *      Author: J
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "stdint.h"
#include "stdbool.h"

#define EEPROM_MODEL     0
#define EEPROM_REV       2
#define EEPROM_PU        4
#define EEPROM_RST       6
#define EEPROM_CONF      8
#define CONF_VALID_FLAG  0x020024B9

typedef enum
{
    ATEL_GPIO_FUNC_IN = 0,
    ATEL_GPIO_FUNC_INT,
    ATEL_GPIO_FUNC_OUT,
    ATEL_GPIO_FUNC_OD,

    ATEL_GPIO_FUNC_MAX,
} atel_gpio_func_t;

typedef enum
{
    ATEL_GPIO_SENSE_LOWTOHI = 0,
    ATEL_GPIO_SENSE_HITOLOW,
    ATEL_GPIO_SENSE_TOGGLE,

    ATEL_GPIO_SENSE_MAX,
} atel_gpio_sense_t;

typedef enum
{
    ATEL_GPIO_NOPULL = 0,
    ATEL_GPIO_PULLUP,
    ATEL_GPIO_PULLDOWN,

    ATEL_GPIO_PULL_MAX,
} atel_gpio_pull_t;

typedef struct
{
    atel_gpio_func_t        func;
    atel_gpio_sense_t       sense;
    atel_gpio_pull_t        pull;
} atel_gpio_cfg_t;

#define GPIO_DIRECTION      0x80 //input/output
#define GPIO_MODE           0x40 //input: 0: float, 1: pull-up, for output: 0: open 1:push-pull
#define GPIO_OUTPUT_HIGH    0x20 //output: 1: H active 0: Low Active
#define GPIO_SET_HIGH       0x10 //GPIO: 0: low, 1: high default value

#define DIRECTION_IN        GPIO_DIRECTION
#define DIRECTION_OUT       0

#define GPIO_INTO_MASK      0x8
#define GPIO_WD_MASK        0x4

#define GPIO_IT_MODE        0x3 //input interrupt mode mask
#define GPIO_IT_NONE        0x0
#define GPIO_IT_LOW_TO_HIGH 0x1
#define GPIO_IT_HIGH_TO_LOW 0x2
#define GPIO_IT_BOTH        0x3

#define PIN_STATUS(x,y,z,s) ((x>0?(GPIO_DIRECTION|GPIO_IT_BOTH):0)| (y>0?GPIO_MODE:0)| (z>0? GPIO_OUTPUT_HIGH:0)|(s>0?GPIO_SET_HIGH:0))

#define BIT_IS_SET(val, pos)	((val) & (1 << (pos)))

#define		USE_TILT	        1 // PUMAMCU-136
#define		ACC_TEST_NONE		0
#define		ACC_TEST_SHAKE		1
#define		ACC_TEST_MOTION		2
#define		ACC_TEST_DEFAULT	ACC_TEST_NONE
//#define		ACC_TEST_DEFAULT	ACC_TEST_SHAKE | ACC_TEST_MOTION

/* Accelerometer */
#define ACC_ADDRESS 0x19

#define NUM_OF_EXT_GPIO 8	// Number of external GPIO. Modify this number if needed. Here keep compatibility with Simba code

enum {
	RESOLUTION_8B,
	RESOLUTION_10B,
	RESOLUTION_12B
};

#define ADC_BAT_TH                          6400 // SIMBAMCU-36 SIMBAMCU-26 MVtoADC (6400UL, BUB_ADC_INPUT_IMPEDANCE_RATIO, 270UL) - diode drop 270K 150K resitor divider
#define ADC_BAT_OK                          7400 // SIMBAMCU-36 SIMBAMCU-26 MVtoADC (7400UL, BUB_ADC_INPUT_IMPEDANCE_RATIO, 270UL) - diode drop 270K 150K resitor divider
#define ADC_MAIN_TH                         6000 // SIMBAMCU-36 SIMBAMCU-26 MVtoADC (6000UL, MAIN_ADC_INPUT_IMPEDANCE_RATIO, 887UL) - diode drop 88.7K 10K resitor divider
#define ADC_MAIN_AUX_TH                     7500 // Main and Aux power supply limit. Unit in mV.

#define NUM_OF_ADC		5

#define RESET_MEMORY_TEST_BYTE  0x04//(0x0DUL), make sure the value is not used in bootloader     /**< Known sequence written to a special register to check if this wake up is from System OFF. */

/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/
void pic_turnOnBaseband (void);
void pic_turnOffBaseband (void);

void systemreset(uint8_t flag);

bool mcu_mux_set(uint8_t mux_sel);

void pic_turnOnV3(void);
void pic_turnOffV3(void);

void pic_turnOnLed(int led);
void pic_turnOffLed(int led);
void pic_toggleLed(int led);
bool pic_IsLedOn(int led);

void charger_power_off(void);
void charger_power_on(void);
bool is_charger_power_on(void);
void setChargerOff(void);
void setNtcOn(void);
void setChargerOn(void);
void charger_state_record(void);
void charger_state_recover(void);
uint8_t getChargerState(void);

void HandleReset(void);

void monet_setGPIOLowPowerMode(void);
void shipping_mode_wkp_src_config(void);

void pic_setLedSleepMode(void); // SIMBAMCU-22
uint8_t isOtherPowerSource(void);
bool main_aux_power_existing(void);

#endif /* SYSTEM_H_ */
