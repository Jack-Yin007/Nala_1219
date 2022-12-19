/*
 *
 * File:    md_lis.h
 * Purpose: Exported functions from LIS2DH accelerometer device module (Arsenal)
 *
 * Author:  J. Bar On
 */

#ifndef _MD_LIS_H_
#define _MD_LIS_H_

#include <stdint.h>

#define BIT(n)	        (n)

#define LIS_I2C_ADDRESS             0x19
#define LIS_I2C_AUTOINCREMENT       0x80
#define LIS_FIFO_SIZE               32

 //Register Definition
#define LIS_OUT_ADC3_L          0x0C
#define LIS_OUT_ADC3_H          0x0D

#define LIS_WHO_AM_I			0x0F    // Device identification register
#define	LIS_ID_VALUE			0x33    // Expected ID value

//  TEMPERATURE CONFIGURATION 
#define LIS_TEMP_CFG_REG	    0x1F
#define LIS_TEMP_ADC_PD			BIT(7)
#define LIS_TEMP_EN				BIT(6)

// CONTROL REGISTER 1
#define LIS_CTRL_REG1			0x20
#define LIS_ODR_BIT				BIT(4)
#define LIS_LPEN				BIT(3)
#define LIS_ZEN					BIT(2)
#define LIS_YEN					BIT(1)
#define LIS_XEN					BIT(0)

//CONTROL REGISTER 2
#define LIS_CTRL_REG2			0x21
#define LIS_HPM     			BIT(6)
#define LIS_HPCF				BIT(4)
#define LIS_FDS					BIT(3)
#define LIS_HPCLICK				BIT(2)
#define LIS_HPIS2				BIT(1)
#define LIS_HPIS1				BIT(0)

//CONTROL REGISTER 3
#define LIS_CTRL_REG3			0x22
#define LIS_I1_CLICK			BIT(7)
#define LIS_I1_AOI1				BIT(6)
#define LIS_I1_AOI2		        BIT(5)
#define LIS_I1_DRDY1			BIT(4)
#define LIS_I1_DRDY2			BIT(3)
#define LIS_I1_WTM				BIT(2)
#define LIS_I1_ORUN				BIT(1)

//CONTROL REGISTER 4
#define LIS_CTRL_REG4			0x23
#define LIS_BDU					BIT(7)
#define LIS_BLE					BIT(6)
#define LIS_FS					BIT(4)
#define LIS_HR					BIT(3)
#define LIS_ST       			BIT(1)
#define LIS_SIM					BIT(0)

//CONTROL REGISTER 5
#define LIS_CTRL_REG5			0x24
#define LIS_BOOT                BIT(7)
#define LIS_FIFO_EN             BIT(6)
#define LIS_LIR_INT1            BIT(3)
#define LIS_D4D_INT1            BIT(2)
#define LIS_LIR_INT2            BIT(1)
#define LIS_D4D_INT2            BIT(0)

//CONTROL REGISTER 6
#define LIS_CTRL_REG6			0x25
#define LIS_I2_CLICK            BIT(7)
#define LIS_I2_AOI1             BIT(6)
#define LIS_I2_AOI2             BIT(5)
#define LIS_I2_BOOT             BIT(4)
#define LIS_I2_ACT              BIT(3)
#define LIS_INT_POLARITY        BIT(1)

//REFERENCE/DATACAPTURE
#define LIS_REFERENCE			0x26

//STATUS_REG_AXIES
#define LIS_STATUS_REG			0x27
#define LIS_ZYXOR               0x80
#define LIS_ZOR                 0x40
#define LIS_YOR                 0x20
#define LIS_XOR                 0x10
#define LIS_ZYXDA               0x08
#define LIS_ZDA                 0x04
#define LIS_YDA                 0x02
#define LIS_XDA                 0x01

//FIFO CONTROL REGISTER
#define LIS_FIFO_CTRL_REG       0x2E
#define LIS_FM                  BIT(6)
#define LIS_TR                  BIT(5)
#define LIS_FTH                 BIT(0)

//OUTPUT REGISTER
#define LIS_OUT_X_L				0x28
#define LIS_OUT_X_H				0x29
#define LIS_OUT_Y_L				0x2A
#define LIS_OUT_Y_H				0x2B
#define LIS_OUT_Z_L				0x2C
#define LIS_OUT_Z_H				0x2D

// FIFO SOURCE REGISTER 
#define LIS_FIFO_SRC_REG		0x2F

//INTERRUPT 1 CONFIGURATION
#define LIS_INT1_CFG			0x30
//INTERRUPT 2 CONFIGURATION
#define LIS_INT2_CFG			0x34
// Int Config Flags
#define LIS_AOI                 BIT(7)
#define LIS_6D                  BIT(6)
#define LIS_ZHIE                BIT(5)
#define LIS_ZLIE                BIT(4)
#define LIS_YHIE                BIT(3)
#define LIS_YLIE                BIT(2)
#define LIS_XHIE                BIT(1)
#define LIS_XLIE                BIT(0)

//INT1 SOURCE
#define LIS_INT1_SOURCE			0x31
//INT2 SOURCE
#define LIS_INT2_SOURCE			0x35
// Int Source Flags
// #define LIS_SRC_XL              BIT(0)
// #define LIS_SRC_XH              BIT(1)
// #define LIS_SRC_YL              BIT(2)
// #define LIS_SRC_YH              BIT(3)
// #define LIS_SRC_ZL              BIT(4)
// #define LIS_SRC_ZH              BIT(5)
// #define LIS_SRC_IA              BIT(6)
// #define LIS_SRC_RESERVED        BIT(7)
// #define MOTION_INT_MASK (MASK_FOR_BIT(LIS_SRC_IA) | MASK_FOR_BIT(LIS_SRC_XH) | MASK_FOR_BIT(LIS_SRC_YH) | MASK_FOR_BIT(LIS_SRC_ZH))

//INT1 REGISTERS
#define LIS_INT1_THS			0x32
#define LIS_INT1_DURATION		0x33

//INT2 REGISTERS
#define LIS_INT2_THS			0x36
#define LIS_INT2_DURATION		0x37

// CLICK_CFG
#define LIS_CLICK_CFG           0x38

//CONTROL REG3 bit mask
#define LIS_CLICK_ON_PIN_INT1_ENABLE                0x80
#define LIS_CLICK_ON_PIN_INT1_DISABLE               0x00
#define LIS_I1_INT1_ON_PIN_INT1_ENABLE              0x40
#define LIS_I1_INT1_ON_PIN_INT1_DISABLE             0x00
#define LIS_I1_INT2_ON_PIN_INT1_ENABLE              0x20
#define LIS_I1_INT2_ON_PIN_INT1_DISABLE             0x00
#define LIS_I1_DRDY1_ON_INT1_ENABLE                 0x10
#define LIS_I1_DRDY1_ON_INT1_DISABLE                0x00
#define LIS_I1_DRDY2_ON_INT1_ENABLE                 0x08
#define LIS_I1_DRDY2_ON_INT1_DISABLE                0x00
#define LIS_WTM_ON_INT1_ENABLE                      0x04
#define LIS_WTM_ON_INT1_DISABLE                     0x00
#define LIS_INT1_OVERRUN_ENABLE                     0x02
#define LIS_INT1_OVERRUN_DISABLE                    0x00

//INT1_2_CFG bit mask
#define LIS_INT_AND                                0x80
#define LIS_INT_OR                                 0x00
#define LIS_INT_6D_MOVEMENT                        0x40
#define LIS_INT_6D_POSITION                        0x00
#define LIS_INT_ZHIE_ENABLE                        0x20
#define LIS_INT_ZHIE_DISABLE                       0x00
#define LIS_INT_ZLIE_ENABLE                        0x10
#define LIS_INT_ZLIE_DISABLE                       0x00
#define LIS_INT_YHIE_ENABLE                        0x08
#define LIS_INT_YHIE_DISABLE                       0x00
#define LIS_INT_YLIE_ENABLE                        0x04
#define LIS_INT_YLIE_DISABLE                       0x00
#define LIS_INT_XHIE_ENABLE                        0x02
#define LIS_INT_XHIE_DISABLE                       0x00
#define LIS_INT_XLIE_ENABLE                        0x01
#define LIS_INT_XLIE_DISABLE                       0x00

typedef enum {
    LIS_FIFO_BYPASS_MODE = 0x00,
    LIS_FIFO_MODE = 0x01,
    LIS_FIFO_STREAM_MODE = 0x02,
    LIS_FIFO_TRIGGER_MODE = 0x03,
    LIS_FIFO_DISABLE = 0x04
} LIS_FifoMode_t;

typedef enum {
    LIS_X_ENABLE = 0x01,
    LIS_X_DISABLE = 0x00,
    LIS_Y_ENABLE = 0x02,
    LIS_Y_DISABLE = 0x00,
    LIS_Z_ENABLE = 0x04,
    LIS_Z_DISABLE = 0x00
} LIS_AXISenable_t;

#define LIS_INT_POLARITY		BIT(1)

typedef enum {
    LIS_ODR_0Hz = 0x00,
    LIS_ODR_1Hz = 0x01,
    LIS_ODR_10Hz = 0x02,
    LIS_ODR_25Hz = 0x03,
    LIS_ODR_50Hz = 0x04,
    LIS_ODR_100Hz = 0x05,
    LIS_ODR_200Hz = 0x06,
    LIS_ODR_400Hz = 0x07,
    LIS_ODR_1620Hz_LP = 0x08,
    LIS_ODR_1344Hz_NP_5367HZ_LP = 0x09
} LIS_ODR_t;

typedef enum {
    LIS_POWER_DOWN = 0x00,
    LIS_LOW_POWER = 0x01,
    LIS_NORMAL = 0x02
} LIS_Mode_t;

typedef enum {
    LIS_HPM_NORMAL_MODE_RES = 0x00,
    LIS_HPM_REF_SIGNAL = 0x01,
    LIS_HPM_NORMAL_MODE = 0x02,
    LIS_HPM_AUTORESET_INT = 0x03
} LIS_HPFMode_t;

typedef enum {
    LIS_FULLSCALE_2             =   0x00,
    LIS_FULLSCALE_4             =   0x01,
    LIS_FULLSCALE_8             =   0x02,
    LIS_FULLSCALE_16            =   0x03
} LIS_Fullscale_t;

typedef enum {
    LIS_LOW_RES                 =   0x00,
    LIS_NORMAL_RES              =   0x01,
    LIS_HIGH_RES                =   0x02,
} LIS_Resolution_t;

/* Calculation of the minimum and maximum values of the threshold in mm/sec^2
    1 g = 9.80665 m/sec^2
   or
    1  mg = 9.80665 mm/sec^2
   Depending of the full scale value used (based on the mode) we get the following value for each 
   threshold unit programmed into THS[6:0] register (7 bits only):
    +/- 2G  ->  16 mg
    +/- 4G  ->  32 mg
    +/- 8G  ->  62 mg
    +/- 16G ->  186 mg
    Taking into the consideration the lowest 16 mg, for minimum possible register value of 1:
        16*1*9.80665 = 156.9064 
    rounding it up gives minimum 160 mm/sec^2.
    The maximum possible register value is 127, so we get:
        16*127*9.80665 = 19927.1128
    rounding it up gives maximum 20000 mm/sec^2.
    Default: 1600 mm/sec^2
    */

#define LIS_THRESHOLD_MIN_MM_BY_SEC2        160
#define LIS_THRESHOLD_MAX_MM_BY_SEC2        20000
#define LIS_THRESHOLD_DEF_MM_BY_SEC2        700
#define LIS_THRESHOLD_COL_DEF_MM_BY_SEC2    2000
#define LIS_THRESHOLD_MM_BY_SEC2_TO_REG(threshold, unit)    ((int32_t)(((threshold) * 1000) / (unit) / 9807))
#define LIS_2GTHRESHOLD_MM_BY_SEC2_TO_REG(threshold)        LIS_THRESHOLD_MM_BY_SEC2_TO_REG(threshold, 16)  // For +/- 2G
// MNT-1226
#define LIS_8GTHRESHOLD_MM_BY_SEC2_TO_REG(threshold)        LIS_THRESHOLD_MM_BY_SEC2_TO_REG(threshold, 62)  // For +/- 8G
#define LIS_4GTHRESHOLD_MM_BY_SEC2_TO_REG(threshold)        LIS_THRESHOLD_MM_BY_SEC2_TO_REG(threshold, 32)  // For +/- 4G AT102PMCU-11
#define LIS_16GTHRESHOLD_MM_BY_SEC2_TO_REG(threshold)       LIS_THRESHOLD_MM_BY_SEC2_TO_REG(threshold, 186) // For +/- 16G AT102PMCU-11
// Convert filter value to mm/Sec2
#define LIS_THRESHOLD_REG_TO_MM_BY_SEC2(filter, unit)       ((uint32_t)(((uint32_t)(filter) * (unit) * 9807) / 1000))
#define LIS_2GTHRESHOLD_REG_TO_MM_BY_SEC2(filter)           LIS_THRESHOLD_REG_TO_MM_BY_SEC2(((uint32_t)(filter)), 16)     // For +/- 2G
#define LIS_4GTHRESHOLD_REG_TO_MM_BY_SEC2(filter)           LIS_THRESHOLD_REG_TO_MM_BY_SEC2(((uint32_t)(filter)), 32)     // For +/- 4G
#define LIS_8GTHRESHOLD_REG_TO_MM_BY_SEC2(filter)           LIS_THRESHOLD_REG_TO_MM_BY_SEC2(((uint32_t)(filter)), 62)     // For +/- 8G
#define LIS_16GTHRESHOLD_REG_TO_MM_BY_SEC2(filter)          LIS_THRESHOLD_REG_TO_MM_BY_SEC2(((uint32_t)(filter)), 186)    // For +/- 16G
/*
    Calculation of the minimum and maximum values of the duration in 100 msec units.
    Usual sampling rates: 
    - 10 Hz for driving behavior, motion detection, regular wake-up and logging
    - 100 Hz for crash detection
   
    Duration D[6:0] provides 7 bits to program duration in 1/ODR units, giving us:
    10Hz: 1/ODR is 100 msec
            Min:   1 * 100 msec =   100 msec =  0.1 sec or   1 configuration unit
            Max: 127 * 100 msec = 12700 msec = 12.7 sec or 127 configuration units
    100Hz: 1/ODR is 10 msec
            Min:   1 *  10 msec =    10 msec = 0.01 sec or   1 configuration unit
            Max: 127 *  10 msec =  1270 msec = 1.27 sec or 127 configuration units
*/
#define LIS_DURATION_MIN_100_MSEC_UNITS     1   //  0.1 seconds
#define LIS_DURATION_MAX_100_MSEC_UNITS   127   // 12.7 seconds
#define LIS_DURATION_DEF_100_MSEC_UNITS     5   //  0.5 seconds
#define LIS_DURATION_100MSEC_TO_REG(duration, odr)              ((int32_t)(((duration) * 10) / (odr)))
#define LIS_DURATION_100MSEC_TO_REG_NORMAL(duration)            (duration)  // ((int32_t)(((duration) * 10) / 10)) For 10 Hz

uint8_t             md_LisReadReg               (uint16_t regAddr, uint8_t * pValue, uint8_t count);
uint8_t             md_LisWriteReg              (uint16_t regAddr, uint8_t value);
uint8_t             md_LisReadModifyWriteReg    (uint16_t regAddr, uint8_t value, uint8_t mask);
uint8_t             md_LisFifoRead              (uint8_t * pSamples, int * pCount);

//void                md_LisDataInit              (void);
//void                md_LisInit                  (void);
//uint8_t             md_LisIsPresent             (void);
//uint8_t             md_LisStart                 (void);
//uint8_t             md_LisStop                  (void);

void                md_LisHeartbeat             (void);

void                md_LisUpdateWakeupParameters    (void);
void                md_LisUpdateMotionParameters    (void);

#endif /* _MD_LIS_H_ */
