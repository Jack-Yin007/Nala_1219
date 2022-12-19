/*
 * acc.c
 *
 *  Created on: Jan 27, 2016
 *      Author: J
 */
#include "platform_hal_drv.h"
#include "acc_simba_lis2dh12.h"
#include "mnt_math.h"
#include "core_types.h"
//#include "em_i2c.h"
#include <stdlib.h>
//#include "acc_types.h"

#if 1

#define		ION_SAMPLING_RATE		LIS_ODR_10Hz
#define		ION_FULL_SCALE			LIS_FULLSCALE_2
#define		I2C_AUTO_INCREMENT		0x80

#define		HIT_THRESHOLD			3
#define		LOCKOUT_TIMER			30
#define		MIN_GPS_SPEED			6
#define		ACC_START_DELAY			10
#define     ACC_FIFO_SIZE           32
#ifdef USE_TILT // PUMAMCU-136
#define     ACC_FIFO_WATERMARK      16
#define     ACC_THRESHOLD_WEIGTHING 16

#else
// We don't want to get accelerometer FIFO interrupts, so set above max. supported 25 Hz rate
#define     ACC_FIFO_WATERMARK      30
#endif

// Forward definition
status_t ion_accRegInit(uint8_t maxG, uint8_t rate);
status_t ion_LIS_FIFOModeEnable(LIS_FifoMode_t fm);

void ion_accReadData(void);
void ion_accFifoRead(uint8_t *pSamples, int *pCount);

#ifdef USE_TILT
static accValues_t RawSamples[ACC_FIFO_SIZE] = { 0 };

//static int16	Samples[3*ACC_FIFO_SIZE]    = { 0 };

ARS_QUEUE_DEFINE(ars_accQ_t, ars_accQ);

static accValuesCalc_t accSigmaSlo = { 0 };
static accValuesCalc_t accSigmaVSlo = { 0 };

static TILT_STATES_te tiltState = TILT_STATE_NONE;
static uint8_t tiltcountdownTimer = 0;
static uint8_t md_LisConfigureRegs(const LisRegValuePair_t * regs, uint8_t nRegs, const char * const func);
#endif

static uint8_t md_LisIsPresent(void);

typedef enum
{
  ACC_NONE,
  ACC_ACCELERATION,
  ACC_BRAKING,
  ACC_CORNERING
} ACC_TYPE_t;

#define myabs(x)  ((x)>0? (x):(-x))

#define LIS_REG_WRITE       0
#define LIS_REG_READ        MASK_FOR_BIT(7)
static tbool LisIsPresent = TBOOL_NOT_ASSIGNED; // FIXIT


/*
 *
 * ion_accIrq
 *
 * Purpose: Accelerometer ISR
 *
 */

///*
// *
// * ion_accFifoRead
// *
// * Purpose:Read the Accelerometer FIFO
// *
// */
//void ion_accIrq(void)
//{
//	int	Count = 0;
//	ion_accFifoRead((uint8_t *)RawSamples, &Count);
//}


/*__attribute__((optimize("-O0")))*/ void ion_accFifoRead(uint8_t *pSamples, int *pCount)
{
    uint8_t Value=0;
	LIS_ReadReg(LIS_FIFO_SRC_REG, &Value, 1);
	Value &= 0x1F;
	if(Value)
	{
		LIS_ReadReg((I2C_AUTO_INCREMENT | LIS_OUT_X_L), pSamples, 6*Value);
	}
	*pCount = Value;
}

uint8_t md_LisReadReg(uint16_t regAddr, uint8_t *pdata, uint8_t len) {
    return (LIS_ReadReg(regAddr, pdata, len) == 0) ? MEMS_SUCCESS : MEMS_ERROR;
}

uint8_t md_LisWriteReg(uint16_t regAddr, uint8_t value)
{
    return (LIS_WriteReg(regAddr, (uint8_t )value) == 0) ? MEMS_SUCCESS : MEMS_ERROR;
}

static uint8_t md_LisIsPresent(void)
{
    // Check the existance once only
    if (TBOOL_NOT_ASSIGNED == LisIsPresent) {
        uint8_t id = 0;
        uint8_t success = md_LisReadReg(LIS_WHO_AM_I, &id, 1);

        LisIsPresent = (success && (ID_VALUE == id) ? TBOOL_TRUE : TBOOL_FALSE);
    }

    // It is guaranteed now that the value will be either TBOOL_TRUE or TBOOL_FALSE,
    // so we can use casting to uint8_t instead of
    // return TBOOL_TRUE == LisIsPresent ? true : false;
    return (uint8_t)LisIsPresent;
}

int ion_accStart(uint8_t maxG, uint8_t rate)
{
	ion_accRegInit(maxG, rate);
	return 0;
}

// Set ACC to default settings
void ion_accStop(void)
{
	ion_accRegInit(2, 0);
}
/*
 *
 * ion_accStartHandler
 *
 * Purpose: start the interrupt
 *
 */
void ion_accStartFIFOinterrupt()
{
    uint8_t Value     = 0;
    uint8_t watermark = ACC_FIFO_WATERMARK;

//	Value = (LIS_FIFO_BYPASS_MODE<<LIS_FM) | watermark;
//	LIS_WriteReg(LIS_FIFO_CTRL_REG, Value);
	Value = (LIS_FIFO_STREAM_MODE<<LIS_FM) | watermark;
	LIS_WriteReg(LIS_FIFO_CTRL_REG, Value);

}

/*
 *
 * ion_accGps
 *
 * Purpose:
 *
 */

uint8_t  ion_accRead(uint8_t  reg, uint8_t  *pValue, UINT32 nLength)
{
	uint8_t  rc;
	if (nLength > 1) {
		reg |= I2C_AUTO_INCREMENT;
	}
	rc = LIS_ReadReg(reg, pValue, nLength);
	return rc;
}

uint8_t  ion_accWrite(uint8_t  reg, uint8_t  value)
{
	uint8_t  rc;
	rc = LIS_WriteReg(reg, (uint8_t )value);
	return rc;
}

status_t pic_accEM4Mode(void) {
    //LIS_WriteReg(LIS_CTRL_REG1, (LIS_ODR_1Hz << LIS_ODR_BIT) | (1 << LIS_LPEN));
    //LIS_WriteReg(LIS_CTRL_REG2, 0);
    //LIS_WriteReg(LIS_CTRL_REG3, 0);
    //LIS_WriteReg(LIS_CTRL_REG4, 0);
    //LIS_WriteReg(LIS_CTRL_REG5, 0 | (1 << LIS_BDU));
    //LIS_WriteReg(LIS_FIFO_CTRL_REG, 0);
    //LIS_WriteReg(LIS_INT1_CFG, 0);
    //LIS_WriteReg(LIS_INT1_THS, 0);
    //LIS_WriteReg(LIS_INT1_DURATION, 0);
    //LIS_WriteReg(LIS_INT2_CFG, 0);
    //LIS_WriteReg(LIS_INT2_THS, 0);
    //LIS_WriteReg(LIS_INT2_DURATION, 0);
    //LIS_WriteReg(LIS_CLICK_CFG, 0);
    LIS_WriteReg(LIS_CTRL_REG6, 0 | (1 << LIS_INT_POLARITY)); // Reverse Int polarity to reduce current drain
    LIS_WriteReg(LIS_CTRL_REG1, (LIS_ODR_1Hz << LIS_ODR_BIT) | (1 << LIS_LPEN));

    return MEMS_SUCCESS;
}

status_t ion_accRegInit(uint8_t  maxG, uint8_t  rate) //maxG: 0:2, 1:4, 2:8, 3:16, rate: 1:1, 2:10, 3:25, 4:50, per second
{
	uint8_t  Value     = 0;
	uint8_t  watermark = ACC_FIFO_WATERMARK;

#ifdef USE_TILT // PUMAMCU-136
    uint8_t success   = md_LisIsPresent();
    UNUSED_BUT_SET_VARIABLE(Value);

    if (success) {
        LisRegValuePair_t regs[] = {
            // Enable wake-up threshold INT1
            { LIS_REG_WRITE | LIS_INT1_THS,         0 },
            // Set INT1 duration to 0
            { LIS_REG_WRITE | LIS_INT1_DURATION,    0 },
            // Enable wake-up threshold INT2
            // Enable High Resolution, set 2G full scale, LSB at low address and block sample updates until read
            { LIS_REG_WRITE | LIS_CTRL_REG4,        (1 << LIS_BDU) | (0 << LIS_BLE) | (LIS_FULLSCALE_2 << LIS_FS) | (1 << LIS_HR) },
            // Enable FIFO and latch interrupt until INT_SRC register is read and direct int pin to APP
            { LIS_REG_WRITE | LIS_CTRL_REG5,        (1 << LIS_FIFO_EN) | (1 << LIS_LIR_INT1) | (0 << LIS_D4D_INT1) },
            // Set FIFO stream mode, interrupt trigger selection to INT1 and threshold to 0
            { LIS_REG_WRITE | LIS_FIFO_CTRL_REG,    (LIS_FIFO_STREAM_MODE << LIS_FM) | (0 << LIS_TR) | (watermark << LIS_FTH) },
            // Read a reference
            { LIS_REG_READ | LIS_REFERENCE,        0 },
            // Enable High-Pass Filter (HPF)
            { LIS_REG_WRITE | LIS_CTRL_REG2,        (1 << LIS_HPIS1) | (1 << LIS_HPIS2) | (0 << LIS_HPCF) | (LIS_HPM_NORMAL_MODE << LIS_HPM) },
            // Enable movement/rotation related interrupts
            { LIS_REG_WRITE | LIS_INT1_CFG,         (0 << LIS_AOI) | (0 << LIS_6D) | (1 << LIS_ZHIE) | (0 << LIS_ZLIE) | (1 << LIS_YHIE) | (0 << LIS_YLIE) | (1 << LIS_XHIE) | (0 << LIS_XLIE) },
            // Enable movement/rotation related interrupts
            // Enable Data Ready interrupt, disable all the rest
            { LIS_REG_WRITE | LIS_CTRL_REG3,        (1 << LIS_I1_AOI1) | (0 << LIS_I1_DRDY1) | LIS_WTM_ON_INT1_ENABLE | LIS_INT1_OVERRUN_ENABLE},
            // Enable all 3 axes, set 10 Hz sampling rate and high power mode
            { LIS_REG_WRITE | LIS_CTRL_REG1,        (LIS_ODR_10Hz << LIS_ODR_BIT) | (0 << LIS_LPEN) | LIS_X_ENABLE | LIS_Y_ENABLE | LIS_Z_ENABLE },
            // Clear LIS_INT1_SOURCE reg
            { LIS_REG_READ | LIS_INT1_SOURCE,      0 },
            // Clear LIS_INT2_SOURCE reg
        };

        success = md_LisConfigureRegs(regs, ARRAY_SIZE(regs), __func__);
    }
    return success ? MEMS_SUCCESS : MEMS_ERROR;
#else
    // Set CTRL_REG1
	Value = (rate<<LIS_ODR_BIT) | (LIS_X_ENABLE | LIS_Y_ENABLE | LIS_Z_ENABLE);
    if (LIS_WriteReg(LIS_CTRL_REG1, Value)) {
        return MEMS_ERROR;
    }
	// Set CTRL_REG2
	Value = (1 << LIS_HPIS1) | (1 << LIS_HPIS2) | (0 << LIS_HPCF) | (LIS_HPM_NORMAL_MODE << LIS_HPM);
    if (LIS_WriteReg(LIS_CTRL_REG2, Value)) {
        return MEMS_ERROR;
    }

	// Set CTRL_REG4
	Value = (1 << LIS_BDU) | (0 << LIS_BLE) | (LIS_FULLSCALE_2 << LIS_FS) | (1 << LIS_HR);
    if (LIS_WriteReg(LIS_CTRL_REG4, Value)) {
        return MEMS_ERROR;
    }

	// Set CTRL_REG5
	Value = (1 << LIS_FIFO_EN) | (1 << LIS_LIR_INT1) | (0 << LIS_D4D_INT1);
    if (LIS_WriteReg(LIS_CTRL_REG5, Value)) {
        return MEMS_ERROR;
    }

    // Set LIS_FIFO_CTRL_REG
    Value = (LIS_FIFO_STREAM_MODE << LIS_FM) | watermark;
    if (LIS_WriteReg(LIS_FIFO_CTRL_REG, Value)) {
        return MEMS_ERROR;
    }

    // Set LIS_INT1_CFG
    Value = (0 << LIS_AOI) | (1 << LIS_6D) | (1 << LIS_ZHIE) | (0 << LIS_ZLIE) | (1 << LIS_YHIE) | (0 << LIS_YLIE) | (1 << LIS_XHIE) | (0 << LIS_XLIE);
    if (LIS_WriteReg(LIS_INT1_CFG, Value)) {
        return MEMS_ERROR;
    }
	// Enable interrupts on INT1
	Value = LIS_WTM_ON_INT1_ENABLE | LIS_INT1_OVERRUN_ENABLE;
    if (LIS_WriteReg(LIS_CTRL_REG3, Value)) {
        return MEMS_ERROR;
    }

	// Set LIS_FIFO_CTRL_REG
	Value = (LIS_FIFO_STREAM_MODE << LIS_FM) | watermark;
    if (LIS_WriteReg(LIS_FIFO_CTRL_REG, Value)) {
        return MEMS_ERROR;
    }
    return MEMS_SUCCESS;
#endif
}


status_t ion_accRegInit2(uint8_t  rate, uint8_t  thresh, uint8_t  dur, uint8_t  bDisableHPF, uint8_t lpenable) //rate: 1:1, 2:10, 3:25, 4:50, per second
{
    uint8_t  low = 1;
    uint8_t  i;
    uint8_t  regList[] = {
        LIS_CTRL_REG1,
        LIS_CTRL_REG2,
        LIS_CTRL_REG3,
        LIS_CTRL_REG4,
        LIS_CTRL_REG5,
        LIS_CTRL_REG6,
        LIS_FIFO_CTRL_REG,
        LIS_INT1_CFG,
        0xFF
    };

    low = 0;

    /* Zero all registers */
    i = 0;
    while (regList[i] != 0xFF) {
        if (LIS_WriteReg(regList[i], 0)) {
            return MEMS_ERROR;
        }
        i++;
    }

#ifdef USE_TILT // PUMAMCU-136
    uint8_t success = md_LisIsPresent();
    UNUSED_BUT_SET_VARIABLE(low);

    if (success) {
        LisRegValuePair_t regs[] = {
            // Enable wake-up threshold INT1
            { LIS_REG_WRITE | LIS_INT1_THS,         0 },
            // Set INT1 duration to 0
            { LIS_REG_WRITE | LIS_INT1_DURATION,    0 },
            // Enable wake-up threshold INT2
            // Enable High Resolution, set 2G full scale, LSB at low address and block sample updates until read
            { LIS_REG_WRITE | LIS_CTRL_REG4,        (1 << LIS_BDU) | (0 << LIS_BLE) | (LIS_FULLSCALE_2 << LIS_FS) | ((1-lpenable) << LIS_HR) },
            // Enable FIFO and latch interrupt until INT_SRC register is read and direct int pin to APP
            { LIS_REG_WRITE | LIS_CTRL_REG5,        (1 << LIS_FIFO_EN) | (1 << LIS_LIR_INT1) | (0 << LIS_D4D_INT1) },
            // Set FIFO stream mode, interrupt trigger selection to INT1 and threshold to 0
            { LIS_REG_WRITE | LIS_FIFO_CTRL_REG,    (LIS_FIFO_STREAM_MODE << LIS_FM) | (0 << LIS_TR) | (ACC_FIFO_WATERMARK << LIS_FTH) },
            // Read a reference
            { LIS_REG_READ | LIS_REFERENCE,        0 },
            // Enable High-Pass Filter (HPF)
            { LIS_REG_WRITE | LIS_CTRL_REG2,        (1 << LIS_HPIS1) | (1 << LIS_HPIS2) | (0 << LIS_HPCF) | (LIS_HPM_NORMAL_MODE << LIS_HPM) },
            // Enable movement/rotation related interrupts
            { LIS_REG_WRITE | LIS_INT1_CFG,         (0 << LIS_AOI) | (0 << LIS_6D) | (1 << LIS_ZHIE) | (0 << LIS_ZLIE) | (1 << LIS_YHIE) | (0 << LIS_YLIE) | (1 << LIS_XHIE) | (0 << LIS_XLIE) },
            // Enable movement/rotation related interrupts
            // Enable AOI, WTM, Overrun interrupt, disable all the rest
            { LIS_REG_WRITE | LIS_CTRL_REG3,        (1 << LIS_I1_AOI1) | (0 << LIS_I1_DRDY1) | LIS_WTM_ON_INT1_ENABLE | LIS_INT1_OVERRUN_ENABLE},
            // Enable all 3 axes, set 10 Hz sampling rate and high power mode
            { LIS_REG_WRITE | LIS_CTRL_REG1,        (rate << LIS_ODR_BIT) | (lpenable << LIS_LPEN) | LIS_X_ENABLE | LIS_Y_ENABLE | LIS_Z_ENABLE },
            // Clear LIS_INT1_SOURCE reg
            { LIS_REG_READ | LIS_INT1_SOURCE,      0 },
            // Clear LIS_INT2_SOURCE reg
        };

        regs[0].value = (uint8_t)(thresh == 0) ? 127 : thresh; // MNT-1226
        regs[0].value = MAX(1, regs[0].value);  // Make sure it is not 0
        regs[0].value = MIN(127, regs[0].value);  // Make sure it fits the register
        success = md_LisConfigureRegs(regs, ARRAY_SIZE(regs), __func__);
    }
    return success ? MEMS_SUCCESS : MEMS_ERROR;
#else
	uint8_t  Value = 0;
	uint8_t  maxG = LIS_FULLSCALE_2;

	// Set CTRL_REG1
	Value = (rate<<LIS_ODR_BIT) | (LIS_X_ENABLE | LIS_Y_ENABLE | LIS_Z_ENABLE);
	if(low) Value |= 1<<3;
	if( LIS_WriteReg(LIS_CTRL_REG1, Value) )
		return MEMS_ERROR;
	

	// Set CTRL_REG2
	if (!bDisableHPF) {
		// With HPF
//		Value = 0x09;		//	Set up high pass filter
//		Value |= 0xC0;		//	Set auto reset on interrupt
        Value = (1 << LIS_HPIS1) | (1 << LIS_HPIS2) | (0 << LIS_HPCF) | (LIS_HPM_NORMAL_MODE << LIS_HPM);
        if( LIS_WriteReg(LIS_CTRL_REG2, Value) )
			return MEMS_ERROR;
	}

	// Set LIS_CTRL_REG3
	Value = LIS_I1_INT1_ON_PIN_INT1_ENABLE;
	if (LIS_WriteReg(LIS_CTRL_REG3, Value))
		return MEMS_ERROR;

	// Set CTRL_REG4
	Value = (1 << LIS_BDU) | (0 << LIS_BLE) | (maxG<<LIS_FS) | (1 << LIS_HR);
	if( LIS_WriteReg(LIS_CTRL_REG4, Value) )
		return MEMS_ERROR;

	// Set CTRL_REG5 ==> don't Latch the value
	Value = (1 << LIS_FIFO_EN) | (1 << LIS_LIR_INT1) | (0 << LIS_D4D_INT1);
	if (LIS_WriteReg(LIS_CTRL_REG5, Value))
		return MEMS_ERROR;
	
	// Set CTRL_REG6 ==> Interrupt is active low
	Value = 0x02;	
	if (LIS_WriteReg(LIS_CTRL_REG6, Value))
		return MEMS_ERROR;

	if(thresh>127) thresh = 127; ////16mg per Lsb for 2G
	if( LIS_WriteReg(LIS_INT1_THS, thresh) )
		return MEMS_ERROR;

	Value = dur;
	if( LIS_WriteReg(LIS_INT1_DURATION, Value) )
		return MEMS_ERROR;

	//Reset reference
	if(LIS_ReadReg(LIS_REFERENCE, &Value, 1))
		return MEMS_ERROR;

	if (bDisableHPF) {
		// No HPF
		Value = 0x15;
	} else {
		// With HPF
		Value = (0 << LIS_AOI) | (1 << LIS_6D) | (1 << LIS_ZHIE) | (0 << LIS_ZLIE) | (1 << LIS_YHIE) | (0 << LIS_YLIE) | (1 << LIS_XHIE) | (0 << LIS_XLIE);
	}
	if( LIS_WriteReg(LIS_INT1_CFG, Value) )
		return MEMS_ERROR;
	return MEMS_SUCCESS;
#endif

}

#if 0
//Free fall detection, thresh=22 (350mg)
status_t ion_accRegInit3(uint8_t  rate, uint8_t  thresh, uint8_t  duration, uint8_t  low) //rate: 1:1, 2:10, 3:25, 4:50, per second
{
	uint8_t  Value     = 0;
	uint8_t  maxG = 0;

	LIS_WriteReg(LIS_CTRL_REG2, 0);
	LIS_WriteReg(LIS_CTRL_REG3, 0);
	LIS_WriteReg(LIS_CTRL_REG4, 0);
	LIS_WriteReg(LIS_CTRL_REG5, 0);
	LIS_WriteReg(LIS_CTRL_REG6, 2);
	LIS_WriteReg(LIS_FIFO_CTRL_REG, 0);

	// Set CTRL_REG1
	Value = (rate<<LIS_ODR_BIT) | (LIS_X_ENABLE | LIS_Y_ENABLE | LIS_Z_ENABLE);
	if(low) Value |= 1<<3;
	if( LIS_WriteReg(LIS_CTRL_REG1, Value) )
		return MEMS_ERROR;


	// Set CTRL_REG4
	Value = /*(1<<LIS_BDU) |*/ (maxG<<LIS_FS);
	if( LIS_WriteReg(LIS_CTRL_REG4, Value) )
		return MEMS_ERROR;

	//80h to REG5 to latch interrupt

	if(thresh>127) thresh = 127; ////16mg per Lsb for 2G
	if( LIS_WriteReg(LIS_INT1_THS, thresh) )
		return MEMS_ERROR;

	Value = duration;
	if( LIS_WriteReg(LIS_INT1_DURATION, Value) )
		return MEMS_ERROR;

	Value = 0x95;
	if( LIS_WriteReg(LIS_INT1_CFG, Value) )
		return MEMS_ERROR;

	// Set LIS_FIFO_CTRL_REG
	Value = LIS_I1_INT1_ON_PIN_INT1_ENABLE;
	if( LIS_WriteReg(LIS_CTRL_REG3, Value) )
		return MEMS_ERROR;

	return MEMS_SUCCESS;
}
#endif
/*
 *
 * ion_accHeartbeat
 *
 * Purpose: one second check of the accelerometer
 *
 */
void ion_accHeartbeat( void )
{
}

#ifdef USE_TILT // PUMAMCU-136
static uint8_t md_LisConfigureRegs(const LisRegValuePair_t * regs, uint8_t nRegs, const char * const func)
{
    uint8_t success = md_LisIsPresent();
	uint8_t temp = 0;

    UNUSED_PARAMETER(func);
    if (success && NULL != regs && nRegs > 0) {
        int i = 0;

        for (; success && i < nRegs; i++) {
            uint8_t address = regs[i].addr;
            if (LIS_REG_READ == (address & LIS_REG_READ)) {
                address &= ~LIS_REG_READ;   // Clear the action bit
                success = md_LisReadReg(address, &temp, 1);
            }
            else {
                success = md_LisWriteReg(address, regs[i].value);
            }
        }
    }

    return success;
}

void ars_checkTiltState(double differentialTilt) {
    switch (tiltState) {
    case TILT_STATE_NONE: {
        if (differentialTilt > monet_data.TiltThreshold) {
            if (tiltcountdownTimer > 0) {
                tiltState = TILT_STATE_CHECKING;
            }
        }
        else {
            tiltcountdownTimer = monet_data.TiltEventTimer * 10; // Number of 1/10 second intervals
        }
    }
    break;
    case TILT_STATE_CHECKING: {
        if (differentialTilt < monet_data.TiltThreshold) { // MNT-1494 improved detection
            tiltState = TILT_STATE_NONE;
        } else {
            if (tiltcountdownTimer > 0) {
                tiltcountdownTimer--;
                if (tiltcountdownTimer == 0) {
                    monet_gpio.Intstatus |= MASK_FOR_BIT(INT_TILT_TAMPER);
                    monet_data.sleepmode = 0; // SIMBAMCU-39
                    tiltState = TILT_STATE_BREECHED;
                }
            }
        }
    }
    break;
    default:
        break;
    }
}

void ars_clearTiltState(void) {
    if (tiltState == TILT_STATE_BREECHED) {
        tiltState = TILT_STATE_NONE;
    }
}

TILT_STATES_te ars_getTiltState(void) {
    return tiltState;
}
#endif

/*******************************************************************************
* Function Name  : ion_LIS_FIFOModeEnable
* Description    : Sets Fifo Modality
* Input          : LIS_FIFO_DISABLE, LIS_FIFO_BYPASS_MODE, LIS_FIFO_MODE,
				   LIS_FIFO_STREAM_MODE, LIS_FIFO_TRIGGER_MODE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t ion_LIS_FIFOModeEnable(LIS_FifoMode_t fm)
{
	uint8_t  value;

	if (fm == LIS_FIFO_DISABLE) {
		if( LIS_ReadReg(LIS_FIFO_CTRL_REG, &value, 1) )
			return MEMS_ERROR;

		value &= 0x1F;
		value |= (LIS_FIFO_BYPASS_MODE<<LIS_FM);

		if ( LIS_WriteReg(LIS_FIFO_CTRL_REG, value) )           //fifo mode bypass
			return MEMS_ERROR;
		if ( LIS_ReadReg(LIS_CTRL_REG5, &value, 1) )
			return MEMS_ERROR;

		value &= 0xBF;

		if	( LIS_WriteReg(LIS_CTRL_REG5, value) )               //fifo disable
			return MEMS_ERROR;
	}

	if (fm == LIS_FIFO_BYPASS_MODE)   {
		if ( LIS_ReadReg(LIS_CTRL_REG5, &value, 1) )
			return MEMS_ERROR;

		value &= 0xBF;
		value |= MEMS_SET<<LIS_FIFO_EN;

		if ( LIS_WriteReg(LIS_CTRL_REG5, value) )               //fifo enable
			return MEMS_ERROR;
		if( LIS_ReadReg(LIS_FIFO_CTRL_REG, &value, 1) )
			return MEMS_ERROR;

		value &= 0x1f;
		value |= (fm<<LIS_FM);									 //fifo mode configuration

		if ( LIS_WriteReg(LIS_FIFO_CTRL_REG, value) )
			return MEMS_ERROR;
	}

	if (fm == LIS_FIFO_MODE) {
		if ( LIS_ReadReg(LIS_CTRL_REG5, &value, 1) )
			return MEMS_ERROR;

		value &= 0xBF;
		value |= MEMS_SET<<LIS_FIFO_EN;

		if ( LIS_WriteReg(LIS_CTRL_REG5, value) )               //fifo enable
			return MEMS_ERROR;
		if ( !LIS_ReadReg(LIS_FIFO_CTRL_REG, &value, 1) )
			return MEMS_ERROR;

		value &= 0x1f;
		value |= (fm<<LIS_FM);                      //fifo mode configuration

		if ( LIS_WriteReg(LIS_FIFO_CTRL_REG, value) )
			return MEMS_ERROR;
	}

	if (fm == LIS_FIFO_STREAM_MODE)   {
		if( LIS_ReadReg(LIS_CTRL_REG5, &value, 1) )
			return MEMS_ERROR;

		value &= 0xBF;
		value |= MEMS_SET<<LIS_FIFO_EN;

		if( LIS_WriteReg(LIS_CTRL_REG5, value) )               //fifo enable
			return MEMS_ERROR;
		if( !LIS_ReadReg(LIS_FIFO_CTRL_REG, &value, 1) )
			return MEMS_ERROR;

		value &= 0x1f;
		value |= (fm<<LIS_FM);                      //fifo mode configuration

		if( LIS_WriteReg(LIS_FIFO_CTRL_REG, value) )
			return MEMS_ERROR;
	}

	if (fm == LIS_FIFO_TRIGGER_MODE)   {
		if( LIS_ReadReg(LIS_CTRL_REG5, &value, 1) )
			return MEMS_ERROR;

		value &= 0xBF;
		value |= MEMS_SET<<LIS_FIFO_EN;

		if ( LIS_WriteReg(LIS_CTRL_REG5, value) )               //fifo enable
			return MEMS_ERROR;
		if ( LIS_ReadReg(LIS_FIFO_CTRL_REG, &value, 1) )
			return MEMS_ERROR;

		value &= 0x1f;
		value |= (fm<<LIS_FM);                      //fifo mode configuration

		if ( LIS_WriteReg(LIS_FIFO_CTRL_REG, value) )
			return MEMS_ERROR;
	}

	return MEMS_SUCCESS;
}

#ifdef USE_TILT // PUMAMCU-136
void ars_addToSlowIIR(accValues_t datapoint) {
    accValuesCalc_t accDataPoint = { 0 };

    if ((accSigmaSlo.x + accSigmaSlo.y + accSigmaSlo.z) == 0) { // Never set yet so initialize
        accSigmaSlo.x = datapoint.x * ARS_ACC_SLO_Q_SIZE;
        accSigmaSlo.y = datapoint.y * ARS_ACC_SLO_Q_SIZE;
        accSigmaSlo.z = datapoint.z * ARS_ACC_SLO_Q_SIZE;
    }

    accDataPoint.x = ((accSigmaSlo.x - (accSigmaSlo.x / ARS_ACC_SLO_Q_SIZE)) + datapoint.x);
    accDataPoint.y = ((accSigmaSlo.y - (accSigmaSlo.y / ARS_ACC_SLO_Q_SIZE)) + datapoint.y);
    accDataPoint.z = ((accSigmaSlo.z - (accSigmaSlo.z / ARS_ACC_SLO_Q_SIZE)) + datapoint.z);
    accSigmaSlo.x = accDataPoint.x;
    accSigmaSlo.y = accDataPoint.y;
    accSigmaSlo.z = accDataPoint.z;
}

void ars_addToVerySlowIIR(accValues_t datapoint) {
    accValuesCalc_t accDataPoint = { 0 };

    if ((accSigmaVSlo.x + accSigmaVSlo.y + accSigmaVSlo.z) == 0) { // Never set yet so initialize
        accSigmaVSlo.x = datapoint.x * ARS_ACC_VSLO_Q_SIZE;
        accSigmaVSlo.y = datapoint.y * ARS_ACC_VSLO_Q_SIZE;
        accSigmaVSlo.z = datapoint.z * ARS_ACC_VSLO_Q_SIZE;
    }

    accDataPoint.x = ((accSigmaVSlo.x - (accSigmaVSlo.x / ARS_ACC_VSLO_Q_SIZE)) + datapoint.x);
    accDataPoint.y = ((accSigmaVSlo.y - (accSigmaVSlo.y / ARS_ACC_VSLO_Q_SIZE)) + datapoint.y);
    accDataPoint.z = ((accSigmaVSlo.z - (accSigmaVSlo.z / ARS_ACC_VSLO_Q_SIZE)) + datapoint.z);
    accSigmaVSlo.x = accDataPoint.x;
    accSigmaVSlo.y = accDataPoint.y;
    accSigmaVSlo.z = accDataPoint.z;
}

accValues_t ars_getAvgSlowIIRData(void) {
    accValues_t avgAccValues = { 0 };
    avgAccValues.x = (accSigmaSlo.x / ARS_ACC_SLO_Q_SIZE);
    avgAccValues.y = (accSigmaSlo.y / ARS_ACC_SLO_Q_SIZE);
    avgAccValues.z = (accSigmaSlo.z / ARS_ACC_SLO_Q_SIZE);
    return avgAccValues;
}

accValues_t ars_getAvgVerySlowIIRData(void) {
    accValues_t avgAccValues = { 0 };
    avgAccValues.x = (accSigmaVSlo.x / ARS_ACC_VSLO_Q_SIZE);
    avgAccValues.y = (accSigmaVSlo.y / ARS_ACC_VSLO_Q_SIZE);
    avgAccValues.z = (accSigmaVSlo.z / ARS_ACC_VSLO_Q_SIZE);
    return avgAccValues;
}

//double ars_shiftCosValue(double cosvalue) {
//    // Shift cos value to 0 to 2 instead of 1 (cos(0)) to -1 (cos(180))
//    return 2-(cosvalue+1);
//}

double ars_calcCosTiltVector(accValues_t datapoint1, accValues_t datapoint2) {
    volatile double sumofsq1rt, sumofsq2rt, tiltVector = 0;
    volatile long dotProduct = 0;
    volatile long sumofsq1 = (((double)datapoint1.x*(double)datapoint1.x) + ((double)datapoint1.y*(double)datapoint1.y) + ((double)datapoint1.z*(double)datapoint1.z));
    volatile long sumofsq2 = (((double)datapoint2.x*(double)datapoint2.x) + ((double)datapoint2.y*(double)datapoint2.y) + ((double)datapoint2.z*(double)datapoint2.z));

    dotProduct = (datapoint1.x * datapoint2.x) + (datapoint1.y * datapoint2.y) + (datapoint1.z * datapoint2.z);

    sumofsq1rt = mnt_sqrt(sumofsq1);
    sumofsq2rt = mnt_sqrt(sumofsq2);

    tiltVector = ars_shiftCosValue((dotProduct / (sumofsq1rt * sumofsq2rt)));
    return tiltVector;
}

int16_t ars_calcMotionVector(accValues_t datapoint1, accValues_t datapoint2) {
    int16_t differenceX, differenceY, differenceZ, motionVector;

    differenceX = abs(datapoint1.x - datapoint2.x);
    differenceY = abs(datapoint1.y - datapoint2.y);
    differenceZ = abs(datapoint1.z - datapoint2.z);

    motionVector = MAX(MAX(differenceX, differenceY), differenceZ) / ACC_THRESHOLD_WEIGTHING; // Scale to 16mg per weighted motion
    return motionVector;
}

//void md_LisDumpRegisters(void)
//{
//    uint8_t reg = LIS_CTRL_REG1;
//    uint8_t dummy = 0;
//    volatile uint8_t regs[LIS_CLICK_CFG + 1] = { 0 };
//    UNUSED_BUT_SET_VARIABLE(regs);
//
//    for (; reg <= LIS_CLICK_CFG; reg++) {
//        //  Don't read reference and samples
//        if (LIS_REFERENCE != reg && !IS_IN_RANGE(reg, LIS_OUT_X_L, LIS_OUT_Z_H))
//        {
//            LIS_ReadReg(reg, &dummy, 1);
//            regs[reg] = dummy;
//        }
//    }
//}

void ars_PollAcc(void) {
    int	index;
    //    uint8_t i;
    int             Count = 0;
    static bool     readyFlag = TRUE;
    accValues_t     axisVector = { 0 };

    if (readyFlag) {
        readyFlag = FALSE;
        ion_accFifoRead((uint8_t  *)RawSamples, &Count);
        if (Count) {
            for (index = 0; index < Count; index++) {
                //                i = (samplecount%ACC_MAX_NUM_OF_SAMPLES);
                axisVector.x = (RawSamples[index].x / 16); // Normalize to mg
                axisVector.y = (RawSamples[index].y / 16); // Normalize to mg
                axisVector.z = (RawSamples[index].z / 16); // Normalize to mg
                (void)ARS_QUEUE_PUSH_CYCLIC(ars_accQ, axisVector);
            }
        }
        readyFlag = TRUE;
        monet_data.AccDataAvailable = 0;
        //md_LisDumpRegisters();
    }
}

#endif  // USE_TILT

void accInterruptHandle(uint8_t src)
{
    monet_data.bNeedReport = 1;
    if (monet_data.bActivated) {
        if (src & 0x6A) { // Check all relevant bits
            if (monet_data.InMotion < 0xff) { // This counter is cleared each second and serves only as an movement indicator
                monet_data.InMotion++;
                monet_data.motionDebounce = 5; // Allow a 5 secs quiet period before clearing the InMotion var
            }
        }
    }
    else {
        if (src & 0x6A) { // Check all relevant bits
            if (monet_data.InMotion < 0xff) { // This counter is cleared each second and serves only as an movement indicator
                monet_data.InMotion++;
                monet_data.motionDebounce = 5; // Allow a 5 secs quiet period before clearing the InMotion var
            }
            if (monet_data.AccDataAvailable == 0) {//first detection
                monet_data.AccDataAvailable++;
            }
//            else if (RTC->CNT - monet_data.lastCount < config_data.affinter) {
            else if ((pf_systick_get() - monet_data.lastCount) < config_data.affinter) {
                monet_data.AccDataAvailable++;
            }
            else { //over timer interval reset to zero
                monet_data.AccDataAvailable = 1;
            }
//            monet_data.lastCount = RTC->CNT;
            monet_data.lastCount = pf_systick_get();
        }
    }
}

double ars_shiftCosValue(double cosvalue) {
    // Shift cos value to 0 to 2 instead of 1 (cos(0)) to -1 (cos(180))
    return 2-(cosvalue+1);
}
#endif

//uint8_t lis2dh_intstatus_get(uint8_t intnum)
//{
//    uint8_t status = 0;

//    if (intnum == 1)
//    {
//        md_LisReadReg(LIS_INT1_SOURCE, &status, 1);
//        // if ((status & MOTION_INT_MASK())) {
//        //     // It is motion interrupt
//        //     // Set a new reference
//        //     md_LisReadReg(LIS_REFERENCE, (uint8_t *)NULL, 0);
//        // }
//    }
//    else if (intnum == 2)
//    {
//        md_LisReadReg(LIS_INT2_SOURCE, &status, 1);
//    }

//    return status;
//}
