#ifndef __ACC_H__
#define __ACC_H__

#include "core_types.h"
#include "ars_queue.h"
#include "md_lis.h"
#include "acc_types.h"

typedef uint8_t LIS_IntPinConf_t;
typedef uint8_t LIS_Axis_t;
typedef uint8_t LIS_Int1Conf_t;

#define ARS_ACC_STREAM_Q_SIZE      128  // Size should be enough to hold more than 1 second data at 100 Hz rate
#define ARS_ACC_SLO_Q_SIZE  1    // MNT-1494 Size Slow IIR filter
#define ARS_ACC_VSLO_Q_SIZE 256  // MNT-1494 Size Very Slow IIR filter

/* --------------------------*/
/* Accelerometer definitions */
/* --------------------------*/
//typedef struct
//{
//    int16_t     x;
//    int16_t     y;
//    int16_t     z;
//} accValues_t;

//typedef struct
//{
//    int32_t     x;
//    int32_t     y;
//    int32_t     z;
//} accValuesCalc_t;

#include "stdio.h"	// Add this line to avoid compiler error: "error:  #20: identifier "size_t" is undefined"
ARS_QUEUE_DEFINE_TYPE(ars_accStreamQ_t, accValues_t, ARS_ACC_STREAM_Q_SIZE);
extern ARS_QUEUE_DEFINE(ars_accStreamQ_t, ars_accStreamQ);

#define MEMS_SET        0x01
#define MEMS_RESET      0x00

#define LIS_DEFAULT_FIFO_WATERMARK  (LIS_FIFO_SIZE - 2)
#define FIFO_WATERMARK_LEVEL	    20

//#define BIT(x)	(x)

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

#define ACC_FULL_SCALE_2_MAXG(scale)    (2 << (scale))	// 2, 4, 8, 16

typedef enum {
    TILT_STATE_NONE,
    TILT_STATE_CHECKING,
    TILT_STATE_BREECHED,
}TILT_STATES_te;

typedef enum
{
    ACC_MODE_OFF,
    ACC_MODE_DRIVING_BEHAVIOR,
    ACC_MODE_COLLISION_REPORT,
    ACC_MODE_WAKE_ON_MOTION,
    ACC_MODE_MOTION_DETECTION,
    ACC_MODE_MOTION_DETECTION_LOW_POWER,
    ACC_MODE_MOTION_210,
    ACC_MODE_MOTION_225,
    ACC_MODE_MOTION_250,
    ACC_MODE_MOTION_2100,
    ACC_MODE_MOTION_410,
    ACC_MODE_MOTION_425,
    ACC_MODE_MOTION_450,
    ACC_MODE_MOTION_4100,
    ACC_MODE_MOTION_810,
    ACC_MODE_MOTION_825,
    ACC_MODE_MOTION_850,
    ACC_MODE_MOTION_8100,
    ACC_MODE_MOTION_1610,
    ACC_MODE_MOTION_1625,
    ACC_MODE_MOTION_1650,
    ACC_MODE_MOTION_16100,
    ACC_MODE_LAST
} ars_AccWorkMode_t; // AT102PMCU-11

#define IS_VALID_ACC_WORKMODE(a) (((a) >= ACC_MODE_OFF) && ((a) < ACC_MODE_LAST))

typedef struct {
    /* All below comes from the default device configuration */
    uint32_t    AccMinAngle;			    /* Minimum heading change (in degrees) for calibration */
    uint32_t    AccMinSpeed;			    /* Minimum speed (meters/h) to collect calibration data */
    uint16_t    AccAcceleration;		    /* Acceleration threshold */
    uint16_t    AccBraking;				    /* Braking threshold */
    uint16_t    AccCornering;			    /* Cornering threshold */
    uint16_t    AccWakeupDebounce;          /* Wakeup Debounce time in seconds */
    uint16_t    AccMotionStartDebounce;     /* Motion Start Debounce time in seconds */
    uint16_t    AccMotionStopDebounce;      /* Motion Stop Debounce time in seconds */
    uint16_t    AccWakeupThreshold;         /* Wakeup threshold */
    uint16_t    AccMotionStartThreshold;    /* Motion start threshold */
    uint16_t    AccMotionStopThreshold;     /* Motion stop threshold */
    uint8_t     AccMotionIntWeight;         /* Motion Interrupt Weight in seconds */
    uint8_t     AccReportEnabled;           /* Accelerometer report is enabled */
    uint8_t     AccPowerMode;               /* 0 - Disable accelerometer based PM, 1 - enable */
    /* All below are maintenance variables */
    uint8_t     bAccIsPresent;		        /* Accelerometer is present */
    uint8_t     bAccCalibrated;             /* If accelerometer is calibrated or not */
    uint8_t		AccRunningMode;			    /* Accelerometer mode */
    uint8_t     AccTestMode;                /* Accelerometer test mode (logging type) */
    uint16_t    AccCollisionThreshold;      /* Collision threshold */
    ars_AccWorkMode_t AccWorkingMode;       /* WCMCU-143 Acc Working Mode */
    uint8_t     AccStreamingEnabled;        /* Accelerometer Streaming mode WCMCU-187 */
} accDatabase_t;

extern accDatabase_t ars_accdata;

#define MASK_FOR_BIT(n) (1UL << (n))    /* Use  1UL to avoid warning:  #61-D: integer operation result is out of range (for bit 31) */

// MNT-1094
typedef enum {
    MNT_NO_COLLISON = 0,
    MNT_LIGHT_COLLISION = MASK_FOR_BIT(0),
    MNT_HEAVY_COLLISION = MASK_FOR_BIT(1)
} mntCollision_e;

typedef struct { uint8_t addr; uint8_t value; } LisRegValuePair_t;

uint8_t md_LisStart(void);
uint8_t md_LisStop(void);
void ars_PollAccStream(void);
uint8_t md_LisSetWorkMode(ars_AccWorkMode_t  mode);

void                md_LisIrq1Handler(void);
void                md_LisIrq2Handler(void);

uint8_t md_getThsScaleFactor(LIS_Fullscale_t scale);
uint8_t lis3dh_intstatus_get(uint8_t intnum);

LIS_ODR_t md_getCurrentODR(void);   // AT102PMCU-11

uint8_t             md_LisEnableTemperature(uint8_t enable);// AT102PMCU-25, AT102PMCU-25,SLP01MCU-145
uint8_t             md_LisReadTemperature(int8_t * pTemp); // AT102PMCU-25, AT102PMCU-25,SLP01MCU-145

#endif // _ACC_H_
