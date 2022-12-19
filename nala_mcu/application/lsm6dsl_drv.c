#include <stddef.h>

#include "lsm6dsl_reg.h"

extern int32_t platform_imu_i2c_write(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);
extern int32_t platform_imu_i2c_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);

/*
 * Initialize mems driver interface
 */
static lsm6dsl_ctx_t dev_ctx = {
    .write_reg = platform_imu_i2c_write,
    .read_reg = platform_imu_i2c_read,
    .handle = NULL
};

/* Calculation of the minimum and maximum values of the threshold in mm/sec^2
    1 g = 9.80665 m/sec^2
   or
    1  mg = 9.80665 mm/sec^2

   For lsm6dsl:
   Depending of the full scale value used (based on the mode) we get the following value for each
   threshold unit programmed into THS[5:0] register (6 bits only):
    mg/LSB
    +/- 2G  ->  31.25 mg
    +/- 4G  ->  62.50 mg
    +/- 8G  ->  125.00 mg
    +/- 16G ->  250.00 mg
    Taking into the consideration the lowest 31.25 mg, for minimum possible register value of 1:
        31.25*1*9.80665 = 306.457812  mm/sec^2
    rounding it up gives minimum 306 mm/sec^2.
    The maximum possible register value is 63, so we get:
        31.25*63*9.80665 = 19306.8422 mm/sec^2
    rounding it up gives maximum 20000 mm/sec^2.
*/
#define LSM6DSL_2G_THRESHOLD_MM_BY_SEC2_TO_REG(t) (((t) + (306/2)) / (306))

static uint8_t lsm6dsl_wkup_senstivity_old = 2;

void lsm6dsl_drv_init(uint8_t *id)
{
    lsm6dsl_int1_route_t int_1_reg;

    uint8_t whoamI = 0;
    
    /*
     *  Check device ID
     */
    whoamI = 0;
    lsm6dsl_device_id_get(&dev_ctx, &whoamI);
    if ( whoamI != LSM6DSL_ID )
    {
        lsm6dsl_device_id_get(&dev_ctx, &whoamI);
    }
    *id = whoamI;
    if ( whoamI != LSM6DSL_ID )
    {
        *id = 0;
        return;
    }

    //  Code below is commented because it would generate interrupts on LSM6DSL pin INT1. QGH, 20191014.
    //  /*
    //   *  Restore default configuration
    //   */
    //  lsm6dsl_reset_set(&dev_ctx, PROPERTY_ENABLE);
    //  do {
    //    lsm6dsl_reset_get(&dev_ctx, &rst);
    //  } while (rst);

    /*
     *  Enable Block Data Update
     */
    lsm6dsl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    lsm6dsl_xl_power_mode_set(&dev_ctx, LSM6DSL_XL_NORMAL);
    lsm6dsl_pin_polarity_set(&dev_ctx, LSM6DSL_ACTIVE_LOW);
    /*
     * Set Output Data Rate
     */
    lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_52Hz);
    lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_OFF);
    /*
     * Set full scale
     */
    lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_2g);
    // lsm6dsl_gy_full_scale_set(&dev_ctx, LSM6DSL_2000dps);

    /*
     * Configure filtering chain(No aux interface)
     */
    /* Accelerometer - analog filter */
    lsm6dsl_xl_filter_analog_set(&dev_ctx, LSM6DSL_XL_ANA_BW_400Hz);

    /* Accelerometer - LPF1 path ( LPF2 not used )*/
    //lsm6dsl_xl_lp1_bandwidth_set(&dev_ctx, LSM6DSL_XL_LP1_ODR_DIV_4);

    /* Accelerometer - LPF1 + LPF2 path */
    lsm6dsl_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100);

    /* Accelerometer - High Pass / Slope path */
    //lsm6dsl_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
    //lsm6dsl_xl_hp_bandwidth_set(&dev_ctx, LSM6DSL_XL_HP_ODR_DIV_100);

    /* Gyroscope - filtering chain */
    // lsm6dsl_gy_band_pass_set(&dev_ctx, LSM6DSL_HP_260mHz_LP1_STRONG);

    /*
     * Apply high-pass digital filter on Wake-Up function
     */
    lsm6dsl_xl_hp_path_internal_set(&dev_ctx, LSM6DSL_USE_HPF);

    /*
     * Apply high-pass digital filter on Wake-Up function
     * Duration time is set to zero so Wake-Up interrupt signal
     * is generated for each X,Y,Z filtered data exceeding the
     * configured threshold
     */
    lsm6dsl_wkup_dur_set(&dev_ctx, 0);

    /*
     * Set Wake-Up threshold: 1 LSb corresponds to FS_XL/2^6
     */
    lsm6dsl_wkup_threshold_set(&dev_ctx, lsm6dsl_wkup_senstivity_old);

    /*
     * Enable interrupt generation on Wake-Up INT1 pin
     */
    lsm6dsl_pin_int1_route_get(&dev_ctx, &int_1_reg);
    int_1_reg.int1_wu = PROPERTY_ENABLE;
    lsm6dsl_pin_int1_route_set(&dev_ctx, int_1_reg);

    /*
     * Uncomment if interrupt generation on Wake-Up INT2 pin
     */
    //lsm6dsl_pin_int2_route_get(&dev_ctx, &int_2_reg);
    //int_2_reg.int2_wu = PROPERTY_ENABLE;
    //lsm6dsl_pin_int2_route_set(&dev_ctx, int_2_reg);
}

uint32_t lsm6dsl_wkup_senstivity_set(uint32_t val) /* val: mm/sec^2 */
{
    uint8_t thresh = (uint8_t)(LSM6DSL_2G_THRESHOLD_MM_BY_SEC2_TO_REG(val));

    if (lsm6dsl_wkup_senstivity_old != thresh)
    {
        lsm6dsl_wkup_senstivity_old = thresh;
    }
    else
    {
        return lsm6dsl_wkup_senstivity_old;
    }
    lsm6dsl_wkup_threshold_set(&dev_ctx, thresh);

    return (uint32_t)thresh;
}

void lsm6dsl_drv_reset(void)
{
    lsm6dsl_reset_set(&dev_ctx, 1);
    lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_OFF);
    lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_OFF);
}

uint8_t lsm6dsl_intstatus_get(uint8_t int_num)
{
    lsm6dsl_all_sources_t val = {0};
    int32_t ret = lsm6dsl_all_sources_get(&dev_ctx, &val);
    uint8_t wake_up_src = *((uint8_t *)(&val));
    
    if (ret == 0)
    {
        return wake_up_src;
    }
    return 0;
}

