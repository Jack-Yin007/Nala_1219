

#include "acc.h"
#include "user.h"

extern int32_t platform_imu_i2c_write(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);
extern int32_t platform_imu_i2c_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);

static uint8_t md_LisConfigureRegs(const LisRegValuePair_t * regs, uint8_t nRegs, const char * const func);

#if !defined(ARRAY_SIZE)
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

#if !defined(MIN)
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#if !defined(MAX)
#define MAX(a, b) ((a) < (b) ? (b) : (a))
#endif

#define		I2C_AUTO_INCREMENT		0x80

#define		HIT_THRESHOLD			3
#define		LOCKOUT_TIMER			30
#define		MIN_GPS_SPEED			6
#define		ACC_START_DELAY			10
#define     ACC_FIFO_SIZE           32
#define     ACC_FIFO_WATERMARK      16
#define     ACC_THRESHOLD_WEIGTHING 16

#if (SUPPORT_ACC_REVERSE_POLARITY == 1)   //SLP01MCU-130
#define LIS_INT_POLARITY_MODE  (1) 
#else
#define LIS_INT_POLARITY_MODE  (0) 
#endif

static volatile accValues_t RawSamples[ACC_FIFO_SIZE] = { 0 };

ARS_QUEUE_DEFINE(ars_accStreamQ_t, ars_accStreamQ);

static LIS_Fullscale_t  currentAccScaling = LIS_FULLSCALE_2;
static LIS_Resolution_t currentAccResolution = LIS_HIGH_RES;
static LIS_ODR_t        currentODR = LIS_ODR_100Hz;

/*
    Low Power Mode:     { 2g = 16mg/digit, 4g = 32mg/digit, 8g = 64mg/digit, 16g = 192mg/digit }
    Normal Power Mode:  { 2g = 4mg/digit,  4g = 8mg/digit,  8g = 16mg/digit, 16g = 148mg/digit }
    High Power Mode:    { 2g = 1mg/digit,  4g = 2mg/digit,  8g = 4mg/digit,  16g = 12mg/digit }
 */
// static uint8_t acc_scalefactor[3][4] = {
//     {16, 32, 64, 192},
//     {4,  8,  16, 148},
//     {1,  2,  4,  12}
// };

static uint8_t acc_thresholdfactor[4] = {
    16,  32,  62,  186
};

static uint8_t md_LisConfigureRegs(const LisRegValuePair_t * regs, uint8_t nRegs, const char * const func);

/* The only one accelerometer shared database */
accDatabase_t   ars_accdata = { 0 };

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
static tbool LisIsPresent = TBOOL_NOT_ASSIGNED;

LIS_ODR_t md_getCurrentODR(void) { // AT102PMCU-11
    return currentODR;
}

uint8_t md_LisReadAccReg(uint16_t regAddr, uint8_t * data, uint8_t count) {
    return (platform_imu_i2c_read(NULL, regAddr, data, count) == 0) ? MEMS_SUCCESS : MEMS_ERROR;
}

uint8_t md_LisWriteAccReg(uint16_t regAddr, uint8_t value)
{
    return (platform_imu_i2c_write(NULL, regAddr, &value, 1) == 0) ? MEMS_SUCCESS : MEMS_ERROR;
}

uint8_t md_LisIsPresent(void)
{
    // Check the existance once only
    if (TBOOL_NOT_ASSIGNED == LisIsPresent) {
        uint8_t id = 0;
        uint8_t success = md_LisReadAccReg(LIS_WHO_AM_I, &id, 1);

        LisIsPresent = (success && (LIS_ID_VALUE == id) ? TBOOL_TRUE : TBOOL_FALSE);
    }

    // It is guaranteed now that the value will be either TBOOL_TRUE or TBOOL_FALSE,
    // so we can use casting to uint8_t instead of
    // return TBOOL_TRUE == LisIsPresent ? true : false;
    return (uint8_t)LisIsPresent;
}

void md_LisInit(void)
{
    // It is assumed that mp_I2cInit() was already called during HW initialization
    md_LisStop();
}

uint8_t md_LisStart(void)
{
    uint8_t success = md_LisIsPresent();

    if (success) {
        success = md_LisSetWorkMode(ACC_MODE_MOTION_DETECTION);  // FIXIT
    }
    return success;
}

uint8_t md_LisStop(void)
{
    uint8_t success = md_LisIsPresent();

    if (success) {
        const LisRegValuePair_t regs[] = {
            // Disable all axis and select power down mode by indicating 0 in the ODR field */
            { LIS_REG_WRITE | LIS_CTRL_REG1,        (LIS_ODR_0Hz << LIS_ODR_BIT) | (LIS_Z_DISABLE | LIS_Y_DISABLE | LIS_X_DISABLE) },
            { LIS_REG_WRITE | LIS_CTRL_REG2,        0 },    // Bypass HPF
            { LIS_REG_WRITE | LIS_CTRL_REG3,        0 },    // Disable all interrupts on INT1
            { LIS_REG_WRITE | LIS_CTRL_REG4,        0 },    // Restore default value
            { LIS_REG_WRITE | LIS_CTRL_REG5,        0 },    // Disable FIFO and restore other defaults
            { LIS_REG_WRITE | LIS_CTRL_REG6,        (LIS_INT_POLARITY_MODE << LIS_INT_POLARITY)},    // Restore default value
            { LIS_REG_WRITE | LIS_FIFO_CTRL_REG,    0 },    // Set FIFO to bypass mode
            { LIS_REG_WRITE | LIS_INT1_CFG,         0 },    // Disable all INT1 triggers
            { LIS_REG_WRITE | LIS_INT1_THS,         0 },    // Restore default for threshold
            { LIS_REG_WRITE | LIS_INT1_DURATION,    0 },    // Restore default for duration
            { LIS_REG_WRITE | LIS_INT2_CFG,         0 },    // Disable all INT2 triggers
            { LIS_REG_WRITE | LIS_INT2_THS,         0 },    // Restore default for threshold
            { LIS_REG_WRITE | LIS_INT2_DURATION,    0 },    // Restore default for duration
            { LIS_REG_WRITE | LIS_CLICK_CFG,        0 },    // Disable all click interrupts

            { LIS_REG_READ | LIS_REFERENCE,        0 },    // Set a reference point by reading it
            { LIS_REG_READ | LIS_STATUS_REG,       0 },    // Clear status register by reading it
            { LIS_REG_READ | LIS_FIFO_SRC_REG,     0 },    // Read FIFO source register
            // No need to read LIS_OUT_... registers to get samples
            { LIS_REG_READ | LIS_INT1_SOURCE,     0 },    // Clear INT1 source register by reading it
            { LIS_REG_READ | LIS_INT2_SOURCE,     0 },    // Clear INT2 source register by reading it
        };

        success = md_LisConfigureRegs(regs, ARRAY_SIZE(regs), NULL);
        //    } else {
        //#if (ACC_MODE == PIC) && defined(_LIONESS) 
        //        mp_debug_printf("%s: PIC Mode\r\n", __func__, success);
        //        md_McuSetAccWorkmode(ACC_MODE_OFF, gMonetData.GsensorMotionThreshold, gMonetData.MotionStartTimeAcc);
        //#endif
    }
    // mp_debug_printf("%s: %d\r\n", __func__, success);

    return success;
}


static uint8_t md_LisConfigureRegs(const LisRegValuePair_t * regs, uint8_t nRegs, const char * const func)
{
    uint8_t success = md_LisIsPresent();
    uint8_t temp = 0;

    // UNUSED_PARAMETER(func);
    if (success && NULL != regs && nRegs > 0) {
        int i = 0;

        for (; success && i < nRegs; i++) {
            uint8_t address = regs[i].addr;
            if (LIS_REG_READ == (address & LIS_REG_READ)) {
                address &= ~LIS_REG_READ;   // Clear the action bit
                success = md_LisReadAccReg(address, &temp, 1);
            }
            else {
                success = md_LisWriteAccReg(address, regs[i].value);
            }
        }
    }

    return success;
}

void ion_accStreamFifoRead(uint8_t *pSamples, int *pCount)
{
    uint8_t Value=0;
    md_LisReadAccReg(LIS_FIFO_SRC_REG, &Value, 1);
	Value &= 0x1F;
	if(Value)
	{
        md_LisReadAccReg((I2C_AUTO_INCREMENT | LIS_OUT_X_L), pSamples, 6*Value);
	}
	*pCount = Value;
}

void ars_PollAccStream(void) {
    volatile int            index;
    int                     Count           = 0;
    static bool             readyFlag       = 1;
    uint8_t                 scalingShift    = 1; // AT102PMCU-11
    volatile accValues_t    axisVector      = { 0 };
    
    switch(currentAccScaling) { //AT102PMCU-11
        case LIS_FULLSCALE_4: {
            scalingShift = 2;
            break;
        }
        case LIS_FULLSCALE_8: {
            scalingShift = 4;
            break;
        }
        case LIS_FULLSCALE_16: {
            scalingShift = 12;
            break;
        }
        default: {
            scalingShift = 1;
            break;
        }
    }

    if (readyFlag) {
        readyFlag = 0;
        ion_accStreamFifoRead((uint8_t  *)RawSamples, &Count);
        if (Count) {
            for (index = 0; index < Count; index++) {
                axisVector.x = (RawSamples[index].x * scalingShift) >> 4; // AT102PMCU-11
                axisVector.y = (RawSamples[index].y * scalingShift) >> 4; // AT102PMCU-11
                axisVector.z = (RawSamples[index].z * scalingShift) >> 4; // AT102PMCU-11
                (void)ARS_QUEUE_PUSH_CYCLIC(ars_accStreamQ, axisVector);
            }
        }
        readyFlag = 1;
    }
}

static void md_getAccMode(ars_AccWorkMode_t mode, LIS_Fullscale_t *fullscale, LIS_ODR_t *odr) {
    switch(mode) {
        case ACC_MODE_MOTION_DETECTION_LOW_POWER:
            break;
        case ACC_MODE_DRIVING_BEHAVIOR:
        case ACC_MODE_WAKE_ON_MOTION:
        case ACC_MODE_MOTION_DETECTION:
        case ACC_MODE_MOTION_210:
            *fullscale = LIS_FULLSCALE_2;
            *odr = LIS_ODR_10Hz;
            break;
        case ACC_MODE_MOTION_225:
            *fullscale = LIS_FULLSCALE_2;
            *odr = LIS_ODR_25Hz;
            break;
        case ACC_MODE_MOTION_250:
            *fullscale = LIS_FULLSCALE_2;
            *odr = LIS_ODR_50Hz;
            break;
        case ACC_MODE_MOTION_2100:
            *fullscale = LIS_FULLSCALE_2;
            *odr = LIS_ODR_100Hz;
            break;
        case ACC_MODE_MOTION_410:
            *fullscale = LIS_FULLSCALE_4;
            *odr = LIS_ODR_10Hz;
            break;
        case ACC_MODE_MOTION_425:
            *fullscale = LIS_FULLSCALE_4;
            *odr = LIS_ODR_25Hz;
            break;
        case ACC_MODE_MOTION_450:
            *fullscale = LIS_FULLSCALE_4;
            *odr = LIS_ODR_50Hz;
            break;
        case ACC_MODE_MOTION_4100:
            *fullscale = LIS_FULLSCALE_4;
            *odr = LIS_ODR_100Hz;
            break;
        case ACC_MODE_MOTION_810:
            *fullscale = LIS_FULLSCALE_8;
            *odr = LIS_ODR_10Hz;
            break;
        case ACC_MODE_MOTION_825:
            *fullscale = LIS_FULLSCALE_8;
            *odr = LIS_ODR_25Hz;
            break;
        case ACC_MODE_MOTION_850:
            *fullscale = LIS_FULLSCALE_8;
            *odr = LIS_ODR_50Hz;
            break;
        case ACC_MODE_COLLISION_REPORT:
        case ACC_MODE_MOTION_8100:
            *fullscale = LIS_FULLSCALE_8;
            *odr = LIS_ODR_100Hz;
            break;
        case ACC_MODE_MOTION_1610:
            *fullscale = LIS_FULLSCALE_16;
            *odr = LIS_ODR_10Hz;
            break;
        case ACC_MODE_MOTION_1625:
            *fullscale = LIS_FULLSCALE_16;
            *odr = LIS_ODR_25Hz;
            break;
        case ACC_MODE_MOTION_1650:
            *fullscale = LIS_FULLSCALE_16;
            *odr = LIS_ODR_50Hz;
            break;
        case ACC_MODE_MOTION_16100:
            *fullscale = LIS_FULLSCALE_16;
            *odr = LIS_ODR_100Hz;
            break;
        default:
            break;
    }
}

uint8_t md_LisSetWorkMode(ars_AccWorkMode_t  mode)
{
    uint8_t success = md_LisIsPresent();
    LIS_Fullscale_t fullscale   = LIS_FULLSCALE_2; // AT102PMCU-11 //
    LIS_ODR_t       odr         = LIS_ODR_0Hz; // AT102PMCU-11
    
    md_getAccMode(mode, &fullscale, &odr);

    if (success) {
        switch (mode) {
        case ACC_MODE_OFF:
            success = md_LisStop();
            break;
        case ACC_MODE_COLLISION_REPORT: // WCMCU-143 8G/100Hz
        case ACC_MODE_MOTION_410:    // AT102PMCU-11 4G/10Hz
        case ACC_MODE_MOTION_425:    // AT102PMCU-11 4G/10Hz
        case ACC_MODE_MOTION_450:    // AT102PMCU-11 4G/10Hz
        case ACC_MODE_MOTION_4100:    // AT102PMCU-11 4G/10Hz
        case ACC_MODE_MOTION_810:    // AT102PMCU-11 8G/10Hz
        case ACC_MODE_MOTION_825:    // AT102PMCU-11 8G/10Hz
        case ACC_MODE_MOTION_850:    // AT102PMCU-11 8G/10Hz
        case ACC_MODE_MOTION_8100:    // AT102PMCU-11 8G/10Hz
        case ACC_MODE_MOTION_1610:   // AT102PMCU-11 16G/10Hz
        case ACC_MODE_MOTION_1625:   // AT102PMCU-11 16G/10Hz
        case ACC_MODE_MOTION_1650:   // AT102PMCU-11 16G/10Hz
        case ACC_MODE_MOTION_16100:   // AT102PMCU-11 16G/10Hz
            // MNT-2409
            /*  - Minimum 100 Hz sampling rate is required for reliable collision detection
             *  - High resolution is required
             *  - Minimum of 6G scale is required, so use 8G
             *  - If driver behavior should be also detected then FIFO could not be used:
             *    TODO: Explain why
             */
        {
            LisRegValuePair_t regs[] = {
                // Enable wake-up threshold INT1
                { LIS_REG_WRITE | LIS_INT1_THS,         0 },
                // Set INT1 duration to 0
                { LIS_REG_WRITE | LIS_INT1_DURATION,    0 },
                // Enable wake-up threshold INT2
                { LIS_REG_WRITE | LIS_INT2_THS,         0 },
                // Set INT2 duration to 0
                { LIS_REG_WRITE | LIS_INT2_DURATION,    0 },
                // Enable High Resolution, set 2G full scale, LSB at low address and block sample updates until read
                { LIS_REG_WRITE | LIS_CTRL_REG4,        (1 << LIS_BDU) | (0 << LIS_BLE) | (fullscale << LIS_FS) | (1 << LIS_HR) },
                // Enable FIFO and latch interrupt until INT_SRC register is read and direct int pin to APP
                 // WCMCU-185 configure interrupt latch for INT2
                { LIS_REG_WRITE | LIS_CTRL_REG5,        (1 << LIS_FIFO_EN) | (0 << LIS_LIR_INT1) | (1 << LIS_LIR_INT2) | (1 << LIS_D4D_INT1) | (1 << LIS_D4D_INT2) },
                // Set FIFO stream mode, interrupt trigger selection to INT1 and threshold to 0
                { LIS_REG_WRITE | LIS_FIFO_CTRL_REG,    (LIS_FIFO_STREAM_MODE << LIS_FM) | (0 << LIS_TR) | (LIS_DEFAULT_FIFO_WATERMARK << LIS_FTH) },
                // Read a reference
                { LIS_REG_READ | LIS_REFERENCE,        0 },
                // Enable High-Pass Filter (HPF)
                { LIS_REG_WRITE | LIS_CTRL_REG2,        (1 << LIS_HPIS1) | (1 << LIS_HPIS2) | (0 << LIS_HPCF) | (LIS_HPM_NORMAL_MODE << LIS_HPM) },
                // Enable movement/rotation related interrupts
                { LIS_REG_WRITE | LIS_INT1_CFG,         (0 << LIS_AOI) | (1 << LIS_6D) | (1 << LIS_ZHIE) | (0 << LIS_ZLIE) | (1 << LIS_YHIE) | (0 << LIS_YLIE) | (1 << LIS_XHIE) | (0 << LIS_XLIE) },
                // Enable movement/rotation related interrupts
                { LIS_REG_WRITE | LIS_INT2_CFG,         (0 << LIS_AOI) | (1 << LIS_6D) | (1 << LIS_ZHIE) | (0 << LIS_ZLIE) | (1 << LIS_YHIE) | (0 << LIS_YLIE) | (1 << LIS_XHIE) | (0 << LIS_XLIE) },
                // WCMCU-185 Enable INT2 interrupts
                { LIS_REG_WRITE | LIS_CTRL_REG6,        (1 << LIS_I2_AOI2) | (0 << LIS_I2_ACT) | (0 << LIS_I2_CLICK) | (LIS_INT_POLARITY_MODE << LIS_INT_POLARITY) },
                // Enable Data Ready interrupt, disable all the rest
                { LIS_REG_WRITE | LIS_CTRL_REG3,        (1 << LIS_I1_AOI1) | (0 << LIS_I1_DRDY1) },
                // Enable all 3 axes, set 10 Hz sampling rate and high power mode
                { LIS_REG_WRITE | LIS_CTRL_REG1,        (odr << LIS_ODR_BIT) | (0 << LIS_LPEN) | LIS_X_ENABLE | LIS_Y_ENABLE | LIS_Z_ENABLE },
                // Clear LIS_INT1_SOURCE reg
                { LIS_REG_READ | LIS_INT1_SOURCE,      0 },
                // Clear LIS_INT2_SOURCE reg
                { LIS_REG_READ | LIS_INT2_SOURCE,      0 },
            };
            currentAccScaling = fullscale; // AT102PMCU-11
            currentAccResolution = LIS_HIGH_RES;
            currentODR = odr; // AT102PMCU-11

            {
                uint16_t    collisionthreshold = ars_accdata.AccCollisionThreshold; // WCMCU-185
                uint16_t    wakeupthreshold = ars_accdata.AccWakeupThreshold;
                /* Using variables during initialization of local variables is not permitted in C90,
                so need to perform post init actions. */
                // LIS_INT1_THS
                if (LIS_FULLSCALE_4 == fullscale) { // AT102PMCU-11
                    regs[0].value = (uint8_t)LIS_4GTHRESHOLD_MM_BY_SEC2_TO_REG(wakeupthreshold); // AT102PMCU-11
                } else if (LIS_FULLSCALE_8 == fullscale) {
                    regs[0].value = (uint8_t)LIS_8GTHRESHOLD_MM_BY_SEC2_TO_REG(wakeupthreshold); // AT102PMCU-11
                } else if (LIS_FULLSCALE_16 == fullscale) {
                    regs[0].value = (uint8_t)LIS_16GTHRESHOLD_MM_BY_SEC2_TO_REG(wakeupthreshold); // AT102PMCU-11
                }
                regs[0].value = MAX(1, regs[0].value);  // Make sure it is not 0
                regs[0].value = MIN(127, regs[0].value);  // Make sure it fits the register
                // LIS_INT1_DURATION
                // Duration debounce is done by application, use 0 even for just a wake up
                // LIS_INT2_THS
                if (LIS_FULLSCALE_4 == fullscale) {
                    regs[2].value = (uint8_t)LIS_4GTHRESHOLD_MM_BY_SEC2_TO_REG(collisionthreshold); // AT102PMCU-11
                } else if (LIS_FULLSCALE_8 == fullscale) {
                    regs[2].value = (uint8_t)LIS_8GTHRESHOLD_MM_BY_SEC2_TO_REG(collisionthreshold); // AT102PMCU-11
                } else if (LIS_FULLSCALE_16 == fullscale) {
                    regs[2].value = (uint8_t)LIS_16GTHRESHOLD_MM_BY_SEC2_TO_REG(collisionthreshold); // AT102PMCU-11
                }
                regs[2].value = MAX(1, regs[2].value);  // Make sure it is not 0
                regs[2].value = MIN(127, regs[2].value);  // Make sure it fits the register
                // LIS_INT2_DURATION
                // Duration debounce is done by application, use 0 even for just a wake up
            }

            // MP_I2C_LOCK();
            success = md_LisStop();
            if (success) {
                success = md_LisConfigureRegs(regs, ARRAY_SIZE(regs), NULL);
            }
            // MP_I2C_UNLOCK();
        }
        break;
        case ACC_MODE_DRIVING_BEHAVIOR:
        case ACC_MODE_WAKE_ON_MOTION:
        case ACC_MODE_MOTION_DETECTION:
        case ACC_MODE_MOTION_210:
        case ACC_MODE_MOTION_225:
        case ACC_MODE_MOTION_250:
        case ACC_MODE_MOTION_2100:
            // WCMCU-143
            /*  For motion detection accelerometer settings should be
             *  as low as possible to save as much power as possible:
             *  - Minimum 10 Hz sampling rate is recommended for reliable motion detection
             *  - Low power mode accuracy is enough
             *  - 2G scale is enough
             */
        {
            LisRegValuePair_t regs[] = {
                // Enable wake-up threshold INT1
                { LIS_REG_WRITE | LIS_INT1_THS,         0 },
                // Set INT1 duration to 0
                { LIS_REG_WRITE | LIS_INT1_DURATION,    0 },
                // Enable wake-up threshold INT2
                { LIS_REG_WRITE | LIS_INT2_THS,         0 },
                // Set INT2 duration to 0
                { LIS_REG_WRITE | LIS_INT2_DURATION,    0 },
                // Enable High Resolution, set 2G full scale, LSB at low address and block sample updates until read
                { LIS_REG_WRITE | LIS_CTRL_REG4,        (1 << LIS_BDU) | (0 << LIS_BLE) | (fullscale << LIS_FS) | (1 << LIS_HR) },
                // Enable FIFO and latch interrupt until INT_SRC register is read and direct int pin to APP
                // WCMCU-22 remove interrupt latch
                { LIS_REG_WRITE | LIS_CTRL_REG5,        (1 << LIS_FIFO_EN) | (0 << LIS_LIR_INT1) | (0 << LIS_LIR_INT2) | (1 << LIS_D4D_INT1) | (1 << LIS_D4D_INT2) },
                // Set FIFO stream mode, interrupt trigger selection to INT1 and threshold to 0
                { LIS_REG_WRITE | LIS_FIFO_CTRL_REG,    (LIS_FIFO_STREAM_MODE << LIS_FM) | (0 << LIS_TR) | (LIS_DEFAULT_FIFO_WATERMARK << LIS_FTH) },
                // Read a reference
                { LIS_REG_READ | LIS_REFERENCE,        0 },
                // Enable High-Pass Filter (HPF)
                { LIS_REG_WRITE | LIS_CTRL_REG2,        (1 << LIS_HPIS1) | (1 << LIS_HPIS2) | (0 << LIS_HPCF) | (LIS_HPM_NORMAL_MODE << LIS_HPM) },
                // Enable movement/rotation related interrupts
                { LIS_REG_WRITE | LIS_INT1_CFG,         (0 << LIS_AOI) | (1 << LIS_6D) | (1 << LIS_ZHIE) | (0 << LIS_ZLIE) | (1 << LIS_YHIE) | (0 << LIS_YLIE) | (1 << LIS_XHIE) | (0 << LIS_XLIE) },
                // Enable movement/rotation related interrupts
                { LIS_REG_WRITE | LIS_INT2_CFG,         (0 << LIS_AOI) | (1 << LIS_6D) | (1 << LIS_ZHIE) | (0 << LIS_ZLIE) | (1 << LIS_YHIE) | (0 << LIS_YLIE) | (1 << LIS_XHIE) | (0 << LIS_XLIE) },
                // Enable INT2 interrupts
                { LIS_REG_WRITE | LIS_CTRL_REG6,        (0 << LIS_I2_AOI2) | (0 << LIS_I2_ACT) | (0 << LIS_I2_CLICK) | (LIS_INT_POLARITY_MODE << LIS_INT_POLARITY) },
                // Enable Data Ready interrupt, disable all the rest
                { LIS_REG_WRITE | LIS_CTRL_REG3,        (1 << LIS_I1_AOI1) | (0 << LIS_I1_DRDY1) },
                // Enable all 3 axes, set 10 Hz sampling rate and high power mode
                { LIS_REG_WRITE | LIS_CTRL_REG1,        (odr << LIS_ODR_BIT) | (0 << LIS_LPEN) | LIS_X_ENABLE | LIS_Y_ENABLE | LIS_Z_ENABLE },
                // Clear LIS_INT1_SOURCE reg
                { LIS_REG_READ | LIS_INT1_SOURCE,      0 },
                // Clear LIS_INT2_SOURCE reg
                { LIS_REG_READ | LIS_INT2_SOURCE,      0 },
            };
            currentAccScaling = fullscale;
            currentAccResolution = LIS_HIGH_RES;
            currentODR = odr;

            {
                // uint16_t    threshold = ars_accdata.AccWakeupThreshold;
                /* Using variables during initialization of local variables is not permitted in C90,
                so need to perform post init actions. */
                // LIS_INT1_THS
                regs[0].value = (uint8_t)LIS_2GTHRESHOLD_MM_BY_SEC2_TO_REG(monet_data.AccData.threshold); // WCMCU-19
                regs[0].value = MAX(1, regs[0].value);  // Make sure it is not 0
                regs[0].value = MIN(127, regs[0].value);  // Make sure it fits the register
                // LIS_INT1_DURATION
                // Duration debounce is done by application, use 0 even for just a wake up
            }

            // MP_I2C_LOCK();
            success = md_LisStop();
            if (success) {
                success = md_LisConfigureRegs(regs, ARRAY_SIZE(regs), NULL);
            }
            // MP_I2C_UNLOCK();
        }
        break;
        // Configuration for low power mode
        case ACC_MODE_MOTION_DETECTION_LOW_POWER:
            /*  For motion detection accelerometer settings should be
             *  as low as possible to save as much power as possible:
             *  - Minimum 1 Hz sampling rate is recommended for reliable motion detection
             *  - Low power mode accuracy is enough
             *  - 2G scale is enough
             */
        {
            LisRegValuePair_t regs[] = {
                // Enable wake-up threshold INT1
                { LIS_REG_WRITE | LIS_INT1_THS,         0 },
                // Set INT1 duration to 0
                { LIS_REG_WRITE | LIS_INT1_DURATION,    0 },
                // Enable wake-up threshold INT2
                { LIS_REG_WRITE | LIS_INT2_THS,         0 },
                // Set INT2 duration to 0
                { LIS_REG_WRITE | LIS_INT2_DURATION,    0 },
                // Enable High Resolution, set 2G full scale, LSB at low address and block sample updates until read
                { LIS_REG_WRITE | LIS_CTRL_REG4,        (1 << LIS_BDU) | (0 << LIS_BLE) | (LIS_FULLSCALE_2 << LIS_FS) | (0 << LIS_HR) },
                // Enable FIFO and latch interrupt until INT_SRC register is read and direct int pin to APP
                { LIS_REG_WRITE | LIS_CTRL_REG5,        (1 << LIS_FIFO_EN) | (1 << LIS_LIR_INT1) | (0 << LIS_LIR_INT2) | (1 << LIS_D4D_INT1) | (1 << LIS_D4D_INT2) },
                // Set FIFO stream mode, interrupt trigger selection to INT1 and threshold to 0
                { LIS_REG_WRITE | LIS_FIFO_CTRL_REG,    (LIS_FIFO_STREAM_MODE << LIS_FM) | (0 << LIS_TR) | (LIS_DEFAULT_FIFO_WATERMARK << LIS_FTH) },
                // Read a reference
                { LIS_REG_READ | LIS_REFERENCE,        0 },
                // Enable High-Pass Filter (HPF)
                { LIS_REG_WRITE | LIS_CTRL_REG2,        (1 << LIS_HPIS1) | (1 << LIS_HPIS2) | (0 << LIS_HPCF) | (LIS_HPM_NORMAL_MODE << LIS_HPM) },
                // Enable movement/rotation related interrupts
                { LIS_REG_WRITE | LIS_INT1_CFG,         (0 << LIS_AOI) | (1 << LIS_6D) | (1 << LIS_ZHIE) | (0 << LIS_ZLIE) | (1 << LIS_YHIE) | (0 << LIS_YLIE) | (1 << LIS_XHIE) | (0 << LIS_XLIE) },
                // Enable movement/rotation related interrupts
                { LIS_REG_WRITE | LIS_INT2_CFG,         (0 << LIS_AOI) | (1 << LIS_6D) | (1 << LIS_ZHIE) | (0 << LIS_ZLIE) | (1 << LIS_YHIE) | (0 << LIS_YLIE) | (1 << LIS_XHIE) | (0 << LIS_XLIE) },
                // Enable INT2 interrupts
                { LIS_REG_WRITE | LIS_CTRL_REG6,        (0 << LIS_I2_AOI2) | (0 << LIS_I2_ACT) | (0 << LIS_I2_CLICK) | (LIS_INT_POLARITY_MODE << LIS_INT_POLARITY) },
                // Enable Data Ready interrupt, disable all the rest
                { LIS_REG_WRITE | LIS_CTRL_REG3,        (1 << LIS_I1_AOI1) | (0 << LIS_I1_DRDY1) },
                // Enable all 3 axes, set 10 Hz sampling rate and high power mode
                { LIS_REG_WRITE | LIS_CTRL_REG1,        (LIS_ODR_1Hz << LIS_ODR_BIT) | (1 << LIS_LPEN) | LIS_X_ENABLE | LIS_Y_ENABLE | LIS_Z_ENABLE },
                // Clear LIS_INT1_SOURCE reg
                { LIS_REG_READ | LIS_INT1_SOURCE,      0 },
                // Clear LIS_INT2_SOURCE reg
                { LIS_REG_READ | LIS_INT2_SOURCE,      0 },
            };
            currentAccScaling = LIS_FULLSCALE_2;
            currentAccResolution = LIS_HIGH_RES;
            if (currentAccResolution)
            {
                //Avoid warning
            }
            currentODR = LIS_ODR_1Hz;

            {
                // uint16_t    threshold = ars_accdata.AccWakeupThreshold;
                /* Using variables during initialization of local variables is not permitted in C90,
                so need to perform post init actions. */
                // LIS_INT1_THS
                regs[0].value = (uint8_t)LIS_2GTHRESHOLD_MM_BY_SEC2_TO_REG(monet_data.AccData.threshold);
                regs[0].value = MAX(1, regs[0].value);  // Make sure it is not 0
                regs[0].value = MIN(127, regs[0].value);  // Make sure it fits the register
                // LIS_INT1_DURATION
                // Duration debounce is done by application, use 0 even for just a wake up
            }

            // MP_I2C_LOCK();
            success = md_LisStop();
            if (success) {
                success = md_LisConfigureRegs(regs, ARRAY_SIZE(regs), NULL);
            }
            // MP_I2C_UNLOCK();
        }
        break;
        default:
            success = false;
        }

        // if (success) {
        //     sgLisInfo.workMode = mode;
        // }
    }

    return success;
}

extern uint8_t pf_imu_int_src_get(uint8_t int_num);
void md_LisIrq2Handler(void)
{
    //pf_imu_int_src_get(ACC_INTERRUPT_NUM2);
}

uint8_t md_getThsScaleFactor(LIS_Fullscale_t scale) {
    return acc_thresholdfactor[scale];
}

uint8_t lis3dh_intstatus_get(uint8_t intnum)
{
    uint8_t status = 0;

    if (intnum == 1)
    {
        md_LisReadAccReg(LIS_INT1_SOURCE, &status, 1);
        // if ((status & MOTION_INT_MASK())) {
        //     // It is motion interrupt
        //     // Set a new reference
        //     md_LisReadAccReg(LIS_REFERENCE, (uint8_t *)NULL, 0);
        // }
    }
    else if (intnum == 2)
    {
        md_LisReadAccReg(LIS_INT2_SOURCE, &status, 1);
    }

    return status;
}

uint8_t md_LisEnableTemperature(uint8_t enable) // AT102PMCU-25,SLP01MCU-145
{
    // MNT-1357, enable/disable temperature sensor
    return md_LisWriteAccReg(LIS_TEMP_CFG_REG, enable ? ((1 << LIS_TEMP_EN) | (1 << LIS_TEMP_ADC_PD)) : (1 << LIS_TEMP_ADC_PD));
}

uint8_t md_LisReadTemperature(int8_t * pTemp)
{ // AT102PMCU-25,SLP01MCU-145
    uint8_t success = false;

    if (NULL != pTemp) {
        int8_t   temperature = 0;
        uint8_t  out_ADC3[2]; // AT102PMCU-25,SLP01MCU-145
        if (MEMS_SUCCESS == md_LisReadAccReg(LIS_OUT_ADC3_L, &out_ADC3[0], sizeof(uint8_t))) {
            if (MEMS_SUCCESS == md_LisReadAccReg(LIS_OUT_ADC3_H, &out_ADC3[1], sizeof(uint8_t))) {
                temperature = (int8_t)out_ADC3[1]; // Holds the delta value of the temperature deviation 
                *pTemp = temperature + 25; // Add the internal reference temperature baseline 25C
                success = true;
            }
        }
    }

    return success;
}

