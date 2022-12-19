/*
 * acc.h
 *
 *  Created on: Jan 27, 2016
 *      Author: J
 */

#ifndef __ACC_SIMBA_LIS2DH12_H__
#define __ACC_SIMBA_LIS2DH12_H__

#include "GenericTypeDefs.h"
#include "system.h"
#include "util.h"
//#include "acc.h"
//#include "acc_types.h"

extern int32_t platform_imu_i2c_write(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);
extern int32_t platform_imu_i2c_write2(void *handle, uint8_t Reg, uint8_t val);
extern int32_t platform_imu_i2c_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);

typedef uint8_t LIS_IntPinConf_t;
typedef uint8_t LIS_Axis_t;
typedef uint8_t LIS_Int1Conf_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} accValues_t;

typedef struct
{
    int32_t     x;
    int32_t     y;
    int32_t     z;
} accValuesCalc_t;

#ifdef USE_TILT // PUMAMCU-136
#define ARS_ACC_Q_SIZE      16   // Size should be enough to hold about 1 second data at 10 Hz rate
#define ARS_ACC_SLO_Q_SIZE  1    // MNT-1494 Size Slow IIR filter
#define ARS_ACC_VSLO_Q_SIZE 256  // MNT-1494 Size Very Slow IIR filter

ARS_QUEUE_DEFINE_TYPE(ars_accQ_t, accValues_t, ARS_ACC_Q_SIZE);
extern ARS_QUEUE_DEFINE(ars_accQ_t, ars_accQ);

#endif

#define MEMS_SET        0x01
#define MEMS_RESET      0x00

#define FIFO_WATERMARK_LEVEL	20
#define BIT(x)	(x)

typedef enum {
	ACC_STOPPED					=	0x00,
	ACC_NORMAL					=	0x01
} accMode_t;

typedef enum {
    ACC_TEST_DUMP_NONE,
	ACC_TEST_DUMP_RAW,
	ACC_TEST_DUMP_CAL,
    ACC_TEST_DUMP_RAW_CAL
} accLoggingMode_t;

typedef enum {
    ACC_DATA_RAW,
    ACC_DATA_PROCESSED,
    ACC_DATA_TRIGGER
} accData_t;

typedef enum {
  MEMS_SUCCESS					=	0x01,
  MEMS_ERROR					=	0x00
} status_t;

typedef enum {
    LIS_ODR_POWERDOWN           =   0x00, // SIMBAMCU-7
    LIS_ODR_1Hz                 =   0x01,
    LIS_ODR_10Hz                =	0x02,
	LIS_ODR_25Hz		        =	0x03,
	LIS_ODR_50Hz		        =	0x04,
	LIS_ODR_100Hz		        =	0x05,
	LIS_ODR_200Hz		        =	0x06,
	LIS_ODR_400Hz		        =	0x07,
	LIS_ODR_1620Hz_LP		    =	0x08,
	LIS_ODR_1344Hz_NP_5367HZ_LP =	0x09
} LIS_ODR_t;

typedef enum {
  LIS_POWER_DOWN				=	0x00,
  LIS_LOW_POWER 				=	0x01,
  LIS_NORMAL					=	0x02
} LIS_Mode_t;

typedef enum {
  LIS_HPM_NORMAL_MODE_RES       =   0x00,
  LIS_HPM_REF_SIGNAL            =   0x01,
  LIS_HPM_NORMAL_MODE           =   0x02,
  LIS_HPM_AUTORESET_INT         =   0x03
} LIS_HPFMode_t;

typedef enum {
  LIS_FULLSCALE_2               =       0x00,
  LIS_FULLSCALE_4               =       0x01,
  LIS_FULLSCALE_8               =       0x02,
  LIS_FULLSCALE_16              =       0x03
} LIS_Fullscale_t;
#define ACC_FULL_SCALE_2_MAXG(scale)    (2 << (scale))	// 2, 4, 8, 16

typedef enum {
  LIS_FIFO_BYPASS_MODE          =       0x00,
  LIS_FIFO_MODE                 =       0x01,
  LIS_FIFO_STREAM_MODE          =       0x02,
  LIS_FIFO_TRIGGER_MODE         =       0x03,
  LIS_FIFO_DISABLE              =       0x04
} LIS_FifoMode_t;

typedef enum {
  LIS_X_ENABLE					=		0x01,
  LIS_X_DISABLE                 =       0x00,
  LIS_Y_ENABLE					=       0x02,
  LIS_Y_DISABLE                 =       0x00,
  LIS_Z_ENABLE                  =       0x04,
  LIS_Z_DISABLE                 =       0x00
} LIS_AXISenable_t;

//Register Definition
#define LIS_WHO_AM_I			0x0F  // device identification register
#define	ID_VALUE				0x33

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

//CONTROL REGISTER 6
#define LIS_CTRL_REG6			0x25
#define LIS_INT_POLARITY		BIT(1)

//REFERENCE/DATACAPTURE
#define LIS_REFERENCE			0x26

//STATUS_REG_AXIES
#define LIS_STATUS_REG			0x27

//FIFO CONTROL REGISTER
#define LIS_FIFO_CTRL_REG       0x2E
#define LIS_FM                  BIT(6)
#define LIS_TR                  BIT(5)
#define LIS_FTH                 BIT(0)

//OUTPUT REGISTER
#define LIS_OUT_X_L					0x28
#define LIS_OUT_X_H					0x29
#define LIS_OUT_Y_L					0x2A
#define LIS_OUT_Y_H					0x2B
#define LIS_OUT_Z_L					0x2C
#define LIS_OUT_Z_H					0x2D

// FIFO SOURCE REGISTER
#define LIS_FIFO_SRC_REG		0x2F

//INTERRUPT 1 CONFIGURATION
#define LIS_INT1_CFG			0x30
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

//INT1 REGISTERS
#define LIS_INT1_THS			0x32
#define LIS_INT1_DURATION		0x33

#define LIS_INT2_CFG			0x34
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

//INT1_CFG bit mask
#define LIS_INT1_AND                                0x80
#define LIS_INT1_OR                                 0x00
#define LIS_INT1_ZHIE_ENABLE                        0x20
#define LIS_INT1_ZHIE_DISABLE                       0x00
#define LIS_INT1_ZLIE_ENABLE                        0x10
#define LIS_INT1_ZLIE_DISABLE                       0x00
#define LIS_INT1_YHIE_ENABLE                        0x08
#define LIS_INT1_YHIE_DISABLE                       0x00
#define LIS_INT1_YLIE_ENABLE                        0x04
#define LIS_INT1_YLIE_DISABLE                       0x00
#define LIS_INT1_XHIE_ENABLE                        0x02
#define LIS_INT1_XHIE_DISABLE                       0x00
#define LIS_INT1_XLIE_ENABLE                        0x01
#define LIS_INT1_XLIE_DISABLE                       0x00

typedef enum {
    TILT_STATE_NONE,
    TILT_STATE_CHECKING,
    TILT_STATE_BREECHED,
}TILT_STATES_te;

// Functions
UINT32 ion_accGetPowerMode( void);

int ion_accInit(void);
void ion_accTestMode(UINT32 nMode);
int ion_accStart(uint8_t maxG, uint8_t rate);
void ion_accFifoRead(uint8_t *pSamples, int *pCount);

void ion_accIrq(void);

status_t ion_accRegInit2(uint8_t rate, uint8_t thresh, uint8_t dur, uint8_t bFF, uint8_t lpenable);
//uint8 LIS_ReadReg(uint8 Reg, uint8* pData, UINT32 nLength);
//uint8 LIS_WriteReg(uint8 Reg, uint8 Data);

//#define LIS_ReadReg(reg, pval, length) pic_i2c_read(ACC_ADDRESS, reg, pval, length)
//#define LIS_WriteReg(reg, val)         pic_i2c_write(ACC_ADDRESS, reg, val)
#define LIS_ReadReg(reg, pval, length) platform_imu_i2c_read(NULL, reg, pval, length)
#define LIS_WriteReg(reg, val)         platform_imu_i2c_write2(NULL, reg, val)
uint8_t md_LisReadReg(uint16_t regAddr, uint8_t *pdata, uint8_t len);
uint8_t md_LisWriteReg(uint16_t regAddr, uint8_t value);

typedef struct { uint8_t addr; uint8_t value; } LisRegValuePair_t;

uint8_t ion_accRead(uint8_t reg, uint8_t *pValue, UINT32 nLength);
uint8_t ion_accWrite(uint8_t reg, uint8_t value);

status_t ion_accRegInit(uint8_t maxG, uint8_t rate);

status_t pic_accEM4Mode(void);

void ion_accHeartbeat(void);
void md_LisDumpRegisters(void);
//uint8_t md_LisIsPresent(void);
#ifdef USE_TILT // PUMAMCU-136
void ars_clearTiltState(void);
void ars_PollAcc(void);
void ars_addToSlowIIR(accValues_t datapoint);
void ars_addToVerySlowIIR(accValues_t datapoint);
accValues_t ars_getAvgSlowIIRData(void);
accValues_t ars_getAvgVerySlowIIRData(void);
double ars_calcCosTiltVector(accValues_t datapoint1, accValues_t datapoint2);
void ars_checkTiltState(double differentialTilt);
int16_t ars_calcMotionVector(accValues_t datapoint1, accValues_t datapoint2);
TILT_STATES_te ars_getTiltState(void);
double ars_shiftCosValue(double cosvalue); // MNT-1494
void ion_accStop(void);
void accInterruptHandle(uint8_t src);
#endif
//uint8_t lis2dh_intstatus_get(uint8_t intnum);

#endif /* __ACC_SIMBA_LIS2DH12_H__ */
