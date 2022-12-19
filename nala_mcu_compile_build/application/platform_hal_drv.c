
/* Includes ------------------------------------------------------------------*/
#include "platform_hal_drv.h"
//#include "acc.h"
#include "uart_drv_ext.h"
#include "acc_simba_lis2dh12.h"
#include "ble_dtm.h"
#include <stdarg.h>

/* Private define ------------------------------------------------------------*/
typedef struct {
    uint32_t pin;
} pin_struct;

#define UART_TX_BUF_SIZE (1024 * 8)  /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE (1024 * 8) /**< UART RX buffer size. */

/* External Variables --------------------------------------------------------*/

/* External Functions --------------------------------------------------------*/
extern void lsm6dsl_drv_init(uint8_t *id);
extern uint32_t lsm6dsl_wkup_senstivity_set(uint32_t val);
extern void lsm6dsl_drv_reset(void);
extern uint8_t lsm6dsl_intstatus_get(uint8_t int_num);

/* Variables -----------------------------------------------------------------*/
static nrf_saadc_value_t saadc_buffer[2][ADC_CHANNELS_CNT];
volatile uint8_t adc_convert_over = 0;
uint8_t pf_adc_initialized = 0;

volatile uint32_t accInterruptFlag = 0;

static const pin_struct gpin[GPIO_LAST] = {
    [GPIO_BLE_GPIO1].pin = BLE_GPIO1,
    [GPIO_BLE_GPIO2].pin = BLE_GPIO2,
    [GPIO_BLE_GPIO3].pin = BLE_GPIO3,
    [GPIO_BLE_Tamper].pin = BLE_Tamper,
    [GPIO_BLE_RELAY].pin = BLE_RELAY,
    [GPIO_RS232_EN].pin = RS232_EN,
    [GPIO_MDM_PWR_KEY].pin =  MDM_PWR_KEY,
    [GPIO_BLE_CAN_PWR_EN].pin = BLE_CAN_PWR_EN,
    [GPIO_CS_12V_EN].pin = CS_12V_EN,
    [GPIO_BLE_MUX].pin = BLE_MUX,
    [GPIO_Hall_Tamper].pin = Hall_Tamper,
    [GPIO_VBAT_Heating_Power_EN].pin = VBAT_Heating_Power_EN,
    [GPIO_CAN_INT].pin = CAN_INT,
//	[GPIO_CHRG_PROCHOT].pin = CHRG_PROCHOT,
    [GPIO_CHRG_SLEEP_EN].pin = CHRG_SLEEP_EN,
    [GPIO_CHRG_INT].pin = CHRG_INT,
#if (HW_VER >= HW_VER_P3)
	[GPIO_SOLAR_CHARGE_SWITCH].pin = SOLAR_CHARGE_SWITCH,
#else
    [GPIO_CHRG_ACOK].pin = CHRG_ACOK,
#endif
    [GPIO_ST_MCU_PWR_EN].pin = ST_MCU_PWR_EN,
    [GPIO_CS_3V3_EN].pin = CS_3V3_EN,
    [GPIO_VDD_MDM_EN].pin = VDD_MDM_EN,
    [GPIO_DC_DC_9V5_EN].pin = DC_DC_9V5_EN,
    [GPIO_CS_MERCREBOOT].pin = CS_MERCREBOOT,
    [GPIO_CS_nRST].pin = CS_nRST,
    [GPIO_TMP_INT].pin = TMP_INT,
    [GPIO_G_SENSOR_INT].pin = G_SENSOR_INT,
    [GPIO_LED_ORANGE].pin = LED_ORANGE,
    [GPIO_LED_GREEN].pin = LED_GREEN,
    [GPIO_LED_RED].pin = LED_RED,
    [GPIO_ONE_BUS_SLPZ].pin = ONE_BUS_SLPZ,
    [GPIO_ST_UART1_TO_UART2_EN].pin = ST_UART1_TO_UART2_EN,
    [GPIO_ST_UART1_TO_UART3_EN].pin = ST_UART1_TO_UART3_EN,
    [GPIO_ST_UART2_TO_UART3_EN].pin = ST_UART2_TO_UART3_EN,
    [GPIO_BAT_ADC_TMP_EN].pin = BAT_ADC_TMP_EN,
    [GPIO_CHRGIN_PWR_EN].pin = CHRGIN_PWR_EN,
    [GPIO_BLE_WAKE_MDM].pin = BLE_UART1_TX,   // pin11 ble tx
    [GPIO_MDM_WAKE_BLE].pin = BLE_UART1_RX,   // pin12 ble rx  
};

volatile uint32_t gTimer = 0;
volatile uint32_t gTick = 0;
#if (!PF_SYSTICK_TIMER_USE_RTC)
const nrf_drv_timer_t pf_systick_timer = NRF_DRV_TIMER_INSTANCE(2);
#else
APP_TIMER_DEF(pf_systick_timer);
#endif /* PF_SYSTICK_TIMER_USE_RTC */

#define TWI_INSTANCE_ID   0
#define TWI_INSTANCE_ID2  1
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);	// TWI0, for pins BLE_I2C1_SCL and BLE_I2C1_SDA
const nrf_drv_twi_t m_twi2 = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID2);// TWI1, for pins BLE_I2C2_SCL and BLE_I2C2_SDA

//static nrf_drv_pwm_t pf_m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

nrf_drv_wdt_channel_id m_pf_wdt_channel_id;

//static bool buzzer_Is_On = false;

/* On Chip Peripheral's Drivers ----------------------------------------------*/

static void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        //ret_code_t err_code;
        int16_t bat_val, main_val, aux_val, solar_val;

        bat_val = p_event->data.done.p_buffer[0];
        main_val = p_event->data.done.p_buffer[1];
        aux_val = p_event->data.done.p_buffer[2];
        solar_val = p_event->data.done.p_buffer[3];

        nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, ADC_CHANNELS_CNT);
        // APP_ERROR_CHECK(err_code);
		
        monet_data.AdcBackupRaw = (bat_val > 0)? bat_val: 0;
        monet_data.AdcMain = (main_val > 0)? main_val: 0;
        monet_data.AdcAux = (aux_val > 0)? aux_val: 0;
        monet_data.AdcSolar = (solar_val > 0)? solar_val: 0;
		
        adc_convert_over = 1;
    }
    // else if (p_event->type == NRF_DRV_SAADC_EVT_CALIBRATEDONE)
    // {
    //     NRF_LOG_RAW_INFO("NRF_DRV_SAADC_EVT_CALIBRATEDONE\r");
    //     NRF_LOG_FLUSH();
    // }
}

// nrf_drv_saadc_calibrate_offset should be used after nrf_drv_saadc_init
static void pf_adc_calibration(void)
{
    uint16_t count = 0;
    ret_code_t err_code = 0;

    err_code = nrf_drv_saadc_calibrate_offset();

    if (err_code)
    {
        NRF_LOG_RAW_INFO("pf_adc_calibration err_code(%d).\r", err_code);
        NRF_LOG_FLUSH();
    }

    if (nrf_drv_saadc_is_busy())
    {
        pf_delay_ms(1);
        count++;
        if (count > 5)
        {
            NRF_LOG_RAW_INFO("pf_adc_calibration fail.\r");
            NRF_LOG_FLUSH();
            return;
        }
    }
}

void pf_adc_init(void)
{
    ret_code_t err_code;

    if (pf_adc_initialized) return;
	
    nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    channel_config.acq_time = NRF_SAADC_ACQTIME_40US;

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    // APP_ERROR_CHECK(err_code);

    pf_adc_calibration();

	channel_config.pin_p = NRF_SAADC_INPUT_AIN4;
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    // APP_ERROR_CHECK(err_code);

	channel_config.pin_p = NRF_SAADC_INPUT_AIN5;
    err_code = nrf_drv_saadc_channel_init(1, &channel_config);
    // APP_ERROR_CHECK(err_code);

	channel_config.pin_p = NRF_SAADC_INPUT_AIN6;
    err_code = nrf_drv_saadc_channel_init(2, &channel_config);
    // APP_ERROR_CHECK(err_code);

	channel_config.pin_p = NRF_SAADC_INPUT_AIN7;
    err_code = nrf_drv_saadc_channel_init(3, &channel_config);
    // APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(saadc_buffer[0], ADC_CHANNELS_CNT);
    // APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(saadc_buffer[1], ADC_CHANNELS_CNT);
    // APP_ERROR_CHECK(err_code);
    if (err_code)
    {
        
    }
    pf_adc_initialized = 1;
}

void pf_adc_start(void)
{
    adc_convert_over = 0;
    nrf_drv_saadc_sample();
}

#define BAT_ADC_RAW_ARRAY_SIZE (60)
#define BAT_ADC_RAW_ARRAY_FRONT (10)
#define BAT_ADC_RAW_ARRAY_BACK (10)
static uint16_t bat_adc_array[BAT_ADC_RAW_ARRAY_SIZE] = {0};
static uint16_t bat_adc_in = 0;

static void swap(uint16_t array[], uint16_t i, uint16_t j)
{
    uint16_t temp = array[i];

    array[i] = array[j];

    array[j] = temp;
}

static void bubble_sort(uint16_t array[], uint16_t len)
{
    uint16_t i = 0;
    uint16_t j = 0;
    uint16_t exchange = 1;

    for (i = 0; (i < len) && exchange; i++)
    {
        exchange = 0;

        for (j = len - 1; j > i; j--)
        {
            if (array[j] < array[j - 1])
            {
                swap(array, j, j - 1);

                exchange = 1;
            }
        }
    }
}

void pf_adc_poll_finish(void)
{
    uint16_t count = 0;
    uint16_t i = 0;
    uint16_t len = 0;
    uint16_t front = 0;
    uint16_t back = 0;
    uint32_t tmp = 0;
    uint16_t bat_sort_array[BAT_ADC_RAW_ARRAY_SIZE] = {0};

    while (adc_convert_over == 0)
    {
        pf_delay_ms(1);
        count++;
        if (count > 5)
        {
            NRF_LOG_RAW_INFO("pf_adc_poll_finish fail In(%d) Raw(0x%x: %u Mv).\r",
                              bat_adc_in,
                              monet_data.AdcBackupRaw,
                              adc_to_vol_conv(monet_data.AdcBackupRaw, VOL_BAT_FACTOR));
            NRF_LOG_FLUSH();
            return;
        }
    }

    bat_adc_array[bat_adc_in % BAT_ADC_RAW_ARRAY_SIZE] = monet_data.AdcBackupRaw;
    bat_adc_in++;
    if (bat_adc_in >= (2 * BAT_ADC_RAW_ARRAY_SIZE))
    {
        bat_adc_in = BAT_ADC_RAW_ARRAY_SIZE;
    }

    if (bat_adc_in < BAT_ADC_RAW_ARRAY_SIZE)
    {
        len = bat_adc_in;
        front = bat_adc_in / 6;
        back = bat_adc_in / 6;
    }
    else
    {
        len = BAT_ADC_RAW_ARRAY_SIZE;
        front = BAT_ADC_RAW_ARRAY_FRONT;
        back = BAT_ADC_RAW_ARRAY_BACK;
    }

    memcpy(bat_sort_array, bat_adc_array, len * sizeof(bat_adc_array[0]));

    bubble_sort(bat_sort_array, len);

    for (i = front;
         i < (len - back);
         i++)
    {
        tmp += bat_sort_array[i];
    }
    monet_data.AdcBackup = tmp / (len - front - back);

    //Warnig: NalaMcu-170 Filter Vbat value above 8.5v.
    if (monet_data.AdcBackup >= 3529)
    {
        NRF_LOG_RAW_INFO("<<High AdcBackup(0x%x: %u Mv).\r",
        monet_data.AdcBackup,(uint16_t)adc_to_vol_conv(monet_data.AdcBackup, VOL_BAT_FACTOR));
        monet_data.AdcBackup = 3525; // after transfer,the vat will be about 8.49v
    }
    #if 0
    NRF_LOG_RAW_INFO("len %u front %u back %u", len, front, back);
    NRF_LOG_FLUSH();
    for (i = 0; i < len; i++)
    {
        if (i % 15 == 0)
        {
            NRF_LOG_RAW_INFO("\r");
        }
        NRF_LOG_RAW_INFO("%d ", adc_to_vol_conv(bat_sort_array[i], VOL_BAT_FACTOR));
        NRF_LOG_FLUSH();
    }
    NRF_LOG_RAW_INFO("\r");
    NRF_LOG_FLUSH();
    #endif /* #if 0 */
}

void pf_adc_deinit(void)
{
    if (0 == pf_adc_initialized) return;
	nrf_drv_saadc_channel_uninit(3);
	nrf_drv_saadc_channel_uninit(2);
	nrf_drv_saadc_channel_uninit(1);
	nrf_drv_saadc_channel_uninit(0);
    nrf_drv_saadc_uninit();
    pf_adc_initialized = 0;
}

static void gpiote_event_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    switch (pin)
    {
        // TODO: Modem wake up ble pin not verified
        case BLE_GPIO1:
//            NRF_LOG_RAW_INFO("BLE_GPIO1.\r");
//            NRF_LOG_FLUSH();
            break;

        case BLE_GPIO2:
//            NRF_LOG_RAW_INFO("BLE_GPIO3.\r");
//            NRF_LOG_FLUSH();
            break;

        case BLE_GPIO3:
//            NRF_LOG_RAW_INFO("BLE_GPIO3.\r");
//            NRF_LOG_FLUSH();
            break;

        case BLE_Tamper:
//            NRF_LOG_RAW_INFO("BLE_Tamper.\r");
//            NRF_LOG_FLUSH();
            break;
		
        case Hall_Tamper:
//            NRF_LOG_RAW_INFO("Hall_Tamper.\r");
//            NRF_LOG_FLUSH();
            break;

//        case CHRG_PROCHOT:
////            NRF_LOG_RAW_INFO("CHRG_PROCHOT.\r");
////            NRF_LOG_FLUSH();
//            break;

        case CHRG_INT:
//            NRF_LOG_RAW_INFO("CHRG_INT.\r");
//            NRF_LOG_FLUSH();
            break;

#if (HW_VER >= HW_VER_P3)
		// Do nothing. This pin in P3 board is output pin.
#else
        case CHRG_ACOK:
//            NRF_LOG_RAW_INFO("CHRG_ACOK.\r");
//            NRF_LOG_FLUSH();
            break;
#endif
		
        case G_SENSOR_INT:
//			monet_data.InMotion++;
			accInterruptFlag = /*(GPIO_IntGet() & */(1 << 11);	// Set same value as in Samba code.
            break;
        
//        case BLE_P005_IN1_ANALOG:
//            if (monet_data.glass_break_pin_index == GPIO_BLE_P005_IN1_ANALOG)
//            {
//                monet_data.glass_break_detected++;
//            }
//            break;

//        case BLE_P018_OUT2_Digital:
//            if (monet_data.glass_break_pin_index == GPIO_BLE_P018_OUT2_Digital)
//            {
//                monet_data.glass_break_detected++;
//            }
//            break;
        case BLE_UART1_RX:
        // if (nrf_gpio_pin_read(BLE_UART1_RX) == 1)
        // {
            monet_data.uart_idle_wake = 1;
            // NRF_LOG_RAW_INFO("monet_data.uart_idle_wake(1)\r");
            // NRF_LOG_FLUSH();
        // }
        break;

        default:
            break;
    }
}

void gpio_deinit(uint32_t index)
{
	if (gpin[index].pin != PIN_NOT_VALID)
	{
		nrfx_gpiote_in_uninit(gpin[index].pin);
		nrf_gpio_cfg_default(gpin[index].pin);
	}
}

int8_t pf_gpio_cfg(uint32_t index, atel_gpio_cfg_t cfg)
{
    nrfx_err_t err_code = NRF_SUCCESS;
    nrf_drv_gpiote_in_config_t config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
    nrf_gpio_pin_dir_t dir = NRF_GPIO_PIN_DIR_INPUT;
    nrf_gpio_pin_input_t input = NRF_GPIO_PIN_INPUT_CONNECT;
    nrf_gpio_pin_drive_t drive = NRF_GPIO_PIN_S0S1;
    nrf_gpio_pin_sense_t sense = NRF_GPIO_PIN_NOSENSE;
    nrf_gpio_pin_pull_t pull = NRF_GPIO_PIN_NOPULL;

    if ((gpin[index].pin == PIN_NOT_VALID) /*|| (gpin[index].pin == P016_BAT_EN)*/)
    {
//        NRF_LOG_INFO("pf_gpio_cfg(index%d:%x) no need to config.", index, gpin[index].pin);
//        NRF_LOG_FLUSH();
        return 1;
    }

    if (cfg.pull == ATEL_GPIO_PULLUP) {
        pull = NRF_GPIO_PIN_PULLUP;
    }
    else if (cfg.pull == ATEL_GPIO_PULLDOWN) {
        pull = NRF_GPIO_PIN_PULLDOWN;
    }

    if (cfg.func == ATEL_GPIO_FUNC_INT) {
        config.pull = pull;

        if (!nrfx_gpiote_is_init()) {
            if (nrfx_gpiote_init() != NRF_SUCCESS) {
                NRF_LOG_RAW_INFO("pf_gpio_cfg gpiote_init fail.\r");
                NRF_LOG_FLUSH();
            }
            return -1;
        }

        if (cfg.sense == ATEL_GPIO_SENSE_TOGGLE) {
            config.sense = NRF_GPIOTE_POLARITY_TOGGLE;
        }
        else if (cfg.sense == ATEL_GPIO_SENSE_HITOLOW)
        {
            config.sense = NRF_GPIOTE_POLARITY_HITOLO;
        }

//        NRF_LOG_RAW_INFO("pf_gpio_cfg index(%d:%d) pull(%d) sense(%d).\r", index, gpin[index].pin, pull, config.sense);
//        NRF_LOG_FLUSH();

        err_code = nrfx_gpiote_in_init(gpin[index].pin, &config, gpiote_event_handler);
        if (err_code != NRF_SUCCESS) {
            return err_code;
        }
        nrfx_gpiote_in_event_enable(gpin[index].pin, true);
    }
    else
    {
//        if (GPIO_BLE_LDO_EN == index)
//        {
//            nrf_gpio_pin_set(gpin[index].pin);
//        }
        nrf_gpio_cfg_default(gpin[index].pin);

        if ((cfg.func == ATEL_GPIO_FUNC_OUT) ||
            (cfg.func == ATEL_GPIO_FUNC_OD)) {
            dir = NRF_GPIO_PIN_DIR_OUTPUT;
            if (cfg.func == ATEL_GPIO_FUNC_OD) {
                drive = NRF_GPIO_PIN_S0D1;
            }

            input = NRF_GPIO_PIN_INPUT_DISCONNECT;

//            // WARNING: not using the setting passed down
//            if ((index == GPIO_BLE_P005_IN1_ANALOG) || (index == GPIO_BLE_P018_OUT2_Digital))
//            {
//                pull = NRF_GPIO_PIN_NOPULL;
//                drive = NRF_GPIO_PIN_S0H1;
//            }

            nrf_gpio_pin_clear(gpin[index].pin);
        }

//        NRF_LOG_RAW_INFO("pf_gpio_cfg index(%d:%d) out(%d) pull(%d) drive(%d).\r", index, gpin[index].pin, dir, pull, drive);
//        NRF_LOG_FLUSH();

        nrf_gpio_cfg(gpin[index].pin, dir, input, pull, drive, sense);
        // if (dir == NRF_GPIO_PIN_DIR_OUTPUT)
        // {
        //     nrf_gpio_cfg_output(gpin[index].pin);
        // }
        // else
        // {
        //     nrf_gpio_cfg_input(gpin[index].pin, NRF_GPIO_PIN_NOPULL);
        // }
    }
	
    return 0;
}

int8_t pf_gpio_write(uint32_t index, uint32_t value)
{
/*    // TODO: for now 1 is for buzzer
    // If PIN == 1 means buzzer.
    if (index == GPIO_BUZZER_INDEX) {
        if (value == 0)
        {
            pf_BuzzerOff();
        }
        else
        {
            pf_BuzzerOn();
        }
    }
    else*/ if (gpin[index].pin == PIN_NOT_VALID)
    {
        return -1;
    }
    else if (nrf_gpio_pin_dir_get(gpin[index].pin) == NRF_GPIO_PIN_DIR_OUTPUT) {

        // NRF_LOG_INFO("pf_gpio_write index(%d:%d) value(%d).", index, gpin[index].pin, value);
        // NRF_LOG_FLUSH();
        if (value == 0)
        {
            nrf_gpio_pin_clear(gpin[index].pin);
        }
        else
        {
           nrf_gpio_pin_set(gpin[index].pin);
        }
    }
    else {
        return -1;
    }

    return 0;
}

int8_t pf_gpio_toggle(uint32_t index)
{
/*    if (index == GPIO_BUZZER_INDEX)
    {
        if (buzzer_Is_On)
        {
            pf_BuzzerOff();
        }
        else
        {
            pf_BuzzerOn();
        }
    }
    else*/ if (gpin[index].pin == PIN_NOT_VALID)
    {
        return -1;
    }
    else if (nrf_gpio_pin_dir_get(gpin[index].pin) == NRF_GPIO_PIN_DIR_OUTPUT) {
        nrf_gpio_pin_toggle(gpin[index].pin);
    }
    else {
        return -1;
    }

    return 0;
}

int32_t pf_gpio_read(uint32_t index)
{
/*    if (index == GPIO_BUZZER_INDEX)
    {
        return (buzzer_Is_On ? 1 : 0);
    }
    else*/ if (gpin[index].pin == PIN_NOT_VALID)
    {
        return -1;
    }
    else if (nrf_gpio_pin_dir_get(gpin[index].pin) == NRF_GPIO_PIN_DIR_OUTPUT) {
        return nrf_gpio_pin_out_read(gpin[index].pin);
    }
    else {
        return nrf_gpio_pin_read(gpin[index].pin);
    }
}

int32_t pf_gpio_dir_get(uint32_t index)
{
    if (gpin[index].pin == PIN_NOT_VALID)
    {
        return -1;
    }

    if (nrf_gpio_pin_dir_get(gpin[index].pin) == NRF_GPIO_PIN_DIR_OUTPUT)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

// Nordic TWI is I2C compatible two-wire interface
void pf_i2c_init(void)
{
    ret_code_t err_code;
//	uint8_t address;
//	uint8_t sample_data;
//	bool detected_device = false;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = BLE_I2C1_SCL,
       .sda                = BLE_I2C1_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };
    const nrf_drv_twi_config_t twi_config2 = {
       .scl                = BLE_I2C2_SCL,
       .sda                = BLE_I2C2_SDA,
//       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_twi_init(&m_twi2, &twi_config2, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
    nrf_drv_twi_enable(&m_twi2);

	monet_data.AccChipAdd = PF_IMU_LIS2DH_I2C_ADDR;
	
//    for (address = 1; address <= 127; address++)
//    {
//        err_code = nrf_drv_twi_rx(&m_twi, address, &sample_data, sizeof(sample_data));
//        if (err_code == NRF_SUCCESS)
//        {
//            detected_device = true;
//            NRF_LOG_RAW_INFO("TWI device detected at address 0x%x.\r", address);

////            if ((address == PF_IMU_LSM6DSL_I2C_ADDR) || (address == PF_IMU_LIS3DH_I2C_ADDR))
//            if (address == PF_IMU_LIS2DH_I2C_ADDR)
//            {
//                monet_data.AccChipAdd = address;
//            }
//        }
//        NRF_LOG_FLUSH();
//    }

//    if (!detected_device)
//    {
//        NRF_LOG_RAW_INFO("No device was found.\r");
//        NRF_LOG_FLUSH();
//    }
	monet_data.bI2CisEnabled = true;
}

void pf_i2c_uninit(void)
{
    nrf_drv_twi_disable(&m_twi);
    nrf_drv_twi_disable(&m_twi2);
    nrf_drv_twi_uninit(&m_twi);
    nrf_drv_twi_uninit(&m_twi2);
	monet_data.bI2CisEnabled = false;
}

#if (!PF_SYSTICK_TIMER_USE_RTC)
/**
 * @brief Handler for timer events.
 */
void timer_systick_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            gTimer++;
            break;

        default:
            //Do nothing.
            break;
    }
}
#else
static void timer_systick_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t ret = 0;

    gTimer++;
	gTick++;
	
    #if TIME_UNIT_CHANGE_WHEN_SLEEP
    if (1 == monet_data.SleepStateChange)
    {
        monet_data.SleepStateChange = 2;
    }

    ret = app_timer_start(pf_systick_timer, APP_TIMER_TICKS(monet_data.sysTickUnit), NULL);
    APP_ERROR_CHECK(ret);

    // if (SLEEP_OFF == monet_data.SleepState)
    // {
    //     ret = app_timer_start(pf_systick_timer, APP_TIMER_TICKS(TIME_UNIT_IN_SLEEP_NORMAL), NULL);
    //     APP_ERROR_CHECK(ret);
    // }
    // else
    // {
    //     ret = app_timer_start(pf_systick_timer, APP_TIMER_TICKS(TIME_UNIT_IN_SLEEP_HIBERNATION), NULL);
    //     APP_ERROR_CHECK(ret);
    // }
    #endif /* TIME_UNIT_CHANGE_WHEN_SLEEP */
}
#endif /* PF_SYSTICK_TIMER_USE_RTC */

void pf_systick_start(uint32_t period_ms)
{
    #if (!PF_SYSTICK_TIMER_USE_RTC)
    uint32_t time_ms = period_ms; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;

    //Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&pf_systick_timer, &timer_cfg, timer_systick_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&pf_systick_timer, time_ms);

    nrf_drv_timer_extended_compare( &pf_systick_timer, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&pf_systick_timer);
    #else
    ret_code_t ret;

    #if TIME_UNIT_CHANGE_WHEN_SLEEP
    ret = app_timer_create(&pf_systick_timer, APP_TIMER_MODE_SINGLE_SHOT, timer_systick_handler);
    APP_ERROR_CHECK(ret);
    #else
    ret = app_timer_create(&pf_systick_timer, APP_TIMER_MODE_REPEATED, timer_systick_handler);
    APP_ERROR_CHECK(ret);
    #endif /* TIME_UNIT_CHANGE_WHEN_SLEEP */

    ret = app_timer_start(pf_systick_timer, APP_TIMER_TICKS(period_ms), NULL);
    APP_ERROR_CHECK(ret);
    #endif /* PF_SYSTICK_TIMER_USE_RTC */
}

void pf_systick_stop(void)
{
    #if (!PF_SYSTICK_TIMER_USE_RTC)
    nrf_drv_timer_disable(&pf_systick_timer);
    nrf_drv_timer_uninit(&pf_systick_timer);
    #else
    app_timer_stop(pf_systick_timer);
    #endif /* PF_SYSTICK_TIMER_USE_RTC */
}

uint32_t pf_systick_get(void)
{
	return gTick;
}

void pf_systick_change(void)
{
#if TIME_UNIT_CHANGE_WHEN_SLEEP
    if (2 == monet_data.SleepStateChange)
    {
        if (gTimer)
        {
            atel_timerTickHandler(monet_data.sysTickUnit);
        }

        if (SLEEP_OFF == monet_data.SleepState)
        {
            if (monet_data.sysTickUnit != TIME_UNIT)
            {
                monet_data.sysTickUnit = TIME_UNIT;
            }
        }
        else
        {
            if (monet_data.sysTickUnit != TIME_UNIT_IN_SLEEP_HIBERNATION)
            {
                monet_data.sysTickUnit = TIME_UNIT_IN_SLEEP_HIBERNATION;
            }
        }

        monet_data.SleepStateChange = 0;

        NRF_LOG_RAW_INFO(">>>pf_systick_change SC(%d)\r", monet_data.SleepStateChange);
        NRF_LOG_FLUSH();
    }
#endif /* TIME_UNIT_CHANGE_WHEN_SLEEP */
}

//void pf_pwm_start(void)
//{
//    // NRF_LOG_INFO("pf_pwm_start Demo3.");
//    // NRF_LOG_FLUSH();

//    nrf_drv_pwm_config_t const config0 =
//    {
//        .output_pins =
//        {
//            PWM_CH0_PIN | NRF_DRV_PWM_PIN_INVERTED, // channel 0
//            NRF_DRV_PWM_PIN_NOT_USED,             // channel 1
//            NRF_DRV_PWM_PIN_NOT_USED,             // channel 2
//            NRF_DRV_PWM_PIN_NOT_USED,             // channel 3
//        },
//        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
//        .base_clock   = NRF_PWM_CLK_1MHz,
//        .count_mode   = NRF_PWM_MODE_UP,
//        .top_value    = 500,
//        .load_mode    = NRF_PWM_LOAD_COMMON,
//        .step_mode    = NRF_PWM_STEP_AUTO
//    };
//    APP_ERROR_CHECK(nrf_drv_pwm_init(&pf_m_pwm0, &config0, NULL));

//    // This array cannot be allocated on stack (hence "static") and it must
//    // be in RAM (hence no "const", though its content is not changed).
//    static uint16_t /*const*/ seq_values[] =
//    {
//        0x8000,
//             0,
//        0x8000,
//             0,
//        0x8000,
//             0
//    };
//    nrf_pwm_sequence_t const seq =
//    {
//        .values.p_common = seq_values,
//        .length          = NRF_PWM_VALUES_LENGTH(seq_values),
//        .repeats         = 0,
//        .end_delay       = 0
//    };

//    (void)nrf_drv_pwm_simple_playback(&pf_m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
//}

//void pf_pwm_stop(void)
//{
//    nrf_drv_pwm_stop(&pf_m_pwm0, true);
//    nrf_drv_pwm_uninit(&pf_m_pwm0);
//    nrf_gpio_pin_clear(PWM_CH0_PIN);
//    nrf_gpio_cfg_output(PWM_CH0_PIN);
//}

void pf_print_mdm_uart_rx(uint8_t cmd, uint8_t *p_data, uint8_t len)
{
#if PF_PROTOCOL_TXRX_FILTER_EN
    if (PF_PROTOCOL_RXFILTER_FORMAT(cmd))
    {
        NRF_LOG_RAW_INFO("IO_CMD(%c, %x) Value:\r", cmd, cmd);
        NRF_LOG_FLUSH();
        printf_hex_and_char(p_data, len);
    }
#else
    NRF_LOG_RAW_INFO("IO_CMD(%c) len(%u) Value:\r", cmd, len);
    NRF_LOG_FLUSH();
    printf_hex_and_char(p_data, len);
#endif /* PF_PROTOCOL_TXRX_FILTER_EN */
}

#define PRINT_UART_TX_BUF_SIZE (256)
static uint8_t print_uart_tx_buf[PRINT_UART_TX_BUF_SIZE] = {0};
static uint16_t print_uart_tx_in = 0;

void pf_print_mdm_uart_tx_init(void)
{
    print_uart_tx_in = 0;
    memset(print_uart_tx_buf, 0, PRINT_UART_TX_BUF_SIZE);
}

void pf_print_mdm_uart_tx(uint8_t data)
{
    print_uart_tx_buf[print_uart_tx_in] = data;
    print_uart_tx_in++;
}

//    if (data == 0x0d)
void pf_print_mdm_uart_tx_flush(void)
{
#if PF_PROTOCOL_TXRX_FILTER_EN
	if (print_uart_tx_buf[0] == '$')
	{
		if (PF_PROTOCOL_TXFILTER_FORMAT(print_uart_tx_buf[2]))
		{
			printf_hex_and_char(print_uart_tx_buf, print_uart_tx_in);
		}
	}
#else
    printf_hex_and_char(print_uart_tx_buf, print_uart_rx_in);
#endif /* PF_PROTOCOL_TXRX_FILTER_EN */

	print_uart_tx_in = 0;
	memset(print_uart_tx_buf, 0, PRINT_UART_TX_BUF_SIZE);
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
//    switch (p_event->evt_type)
//    {
//        case APP_UART_DATA_READY:
//            // NRF_LOG_RAW_INFO("uart_event_handle(0x%x)\r", tmp);
//            // NRF_LOG_FLUSH();
//            break;

//        case APP_UART_COMMUNICATION_ERROR:
//            // APP_ERROR_HANDLER(p_event->data.error_communication);
//            NRF_LOG_WARNING("APP_UART_COMMUNICATION_ERROR.");
//            NRF_LOG_FLUSH();
//            break;

//        case APP_UART_FIFO_ERROR:
//            // APP_ERROR_HANDLER(p_event->data.error_code);
//            NRF_LOG_WARNING("APP_UART_FIFO_ERROR.");
//            NRF_LOG_FLUSH();
//            break;

//        default:
//            break;
//    }
}
/**@snippet [Handling the data received over UART] */

/* When UART is used for communication with the host do not use flow control.*/
#define UART_HWFC				APP_UART_FLOW_CONTROL_DISABLED
#ifdef  UART1_RX_BUFFER_SIZE
#undef  UART1_RX_BUFFER_SIZE
#define UART1_RX_BUFFER_SIZE	(1024 * 32)//512
#endif
#ifdef  UART1_TX_BUFFER_SIZE
#undef  UART1_TX_BUFFER_SIZE
#define UART1_TX_BUFFER_SIZE	(1024 * 8)//512
#endif
#define UART0_RX_BUFFER_SIZE	(1024 * 8)//512
#define UART0_TX_BUFFER_SIZE	(1024 * 8)//512

static const uint32_t BAUD_RATE_TABLE[] =
{
    0,
    0,
    NRF_UART_BAUDRATE_1200,
    NRF_UART_BAUDRATE_2400,
    NRF_UART_BAUDRATE_4800,
    NRF_UART_BAUDRATE_9600,
    NRF_UART_BAUDRATE_19200,
    NRF_UART_BAUDRATE_38400,
    NRF_UART_BAUDRATE_57600,
    NRF_UART_BAUDRATE_115200,
    NRF_UART_BAUDRATE_230400
}; // Baud rate 300 and 600 in Simba are not surported in Nordic nRF52840 chip

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
// Para. baud: baud rate selection. Index of table BAUD_RATE_TABLE[]. If out of range of the table, 9 would be filled
// Para. mode: stop bit length. Not used.  Just set it to 0
void pf_uart_mdm_init(uint8_t baud, uint8_t mode)
{
	uint32_t err_code;
	uint32_t baud_rate;
    static uint8_t default_baud = 9;

    if (monet_data.uartMdmTXDEnabled == 1)
    {
        return;
    }

    nrfx_gpiote_in_uninit(BLE_UART1_RX);
    nrf_gpio_cfg_default(BLE_UART1_RX);
    nrfx_gpiote_in_uninit(BLE_UART1_TX);
    nrf_gpio_cfg_default(BLE_UART1_TX);

	if (baud <= 1 || baud > UART_BAUDRATE_SELECT_MAX)	// Baud rate 300 and 600 in Simba are not surported in Nordic nRF52840 chip
	{
        baud = default_baud;
    }
    baud_rate = BAUD_RATE_TABLE[baud];
    default_baud = baud;
    monet_data.uartMdmBaudRate = default_baud;
	
	UNUSED_VARIABLE(mode);		// Changing stop bit length is not surported
	
	// UART0 init
	const app_uart_comm_params_t comm_params =
	{
		BLE_UART1_RX,
		BLE_UART1_TX,
		
		UART_PIN_DISCONNECTED,
		UART_PIN_DISCONNECTED,
		UART_HWFC,
		false,
#if defined (UART_PRESENT)
		baud_rate // NRF_UART_BAUDRATE_115200
#else
		baud_rate // NRF_UARTE_BAUDRATE_115200
#endif
	};
	UART_DRV_EX_INIT(UART1_DRV_EX, 
						&comm_params,
						UART1_RX_BUFFER_SIZE,		// FIFO Needed
						UART1_TX_BUFFER_SIZE,		// FIFO Needed
						uart_event_handle,			// Needed. NULL makes error
						APP_IRQ_PRIORITY_HIGHEST,
						err_code);
	APP_ERROR_CHECK(err_code);

    // NRF_LOG_RAW_INFO(">>>pf_uart_mdm_init(%d)\r", baud);
    // NRF_LOG_FLUSH();

    monet_data.uartMdmInitCount++;
    monet_data.uartMdmTXDEnabled = 1;
}
/**@snippet [UART Initialization] */

void pf_uart_mdm_deinit(void)
{
    if (monet_data.uartMdmTXDEnabled == 0)
    {
        return;
    }
    // NRF_LOG_RAW_INFO(">>>pf_uart_mdm_deinit\r");
    // NRF_LOG_FLUSH();
	uart_drv_ext_close(UART1_DRV_EX);

    {
        nrfx_err_t err_code = NRF_SUCCESS;
        nrf_drv_gpiote_in_config_t config = NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_HITOLO(false);
        config.pull = NRF_GPIO_PIN_PULLDOWN;

        if (!nrfx_gpiote_is_init())
        {
            if (nrfx_gpiote_init() != NRF_SUCCESS)
            {
                NRF_LOG_RAW_INFO("pf_uart_mdm_deinit gpiote_init fail.\r");
                NRF_LOG_FLUSH();
            }
        }

        err_code = nrfx_gpiote_in_init(BLE_UART1_RX, &config, gpiote_event_handler);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_RAW_INFO("pf_uart_mdm_deinit BLE_UART1_RX fail.\r");
            NRF_LOG_FLUSH();
        }
        else
        {
            nrfx_gpiote_in_event_enable(BLE_UART1_RX, true);
        }

        nrfx_gpiote_in_uninit(BLE_UART1_TX);
        nrf_gpio_cfg_default(BLE_UART1_TX);
    }

    monet_data.uartMdmDeinitCount++;
    monet_data.uartMdmTXDEnabled = 0;
}

uint32_t pf_uart_mdm_tx_one(uint8_t byte)
{
    return uart_drv_ext_put(UART1_DRV_EX, byte);
}

uint32_t pf_uart_mdm_rx_one(uint8_t *p_data)
{
    return uart_drv_ext_get(UART1_DRV_EX, p_data);
}

bool pf_uart_mdm_tx_queue_is_empty(void)
{
	return uart_drv_ext_tx_fifo_is_empty(UART1_DRV_EX);
}

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
// Para. baud: baud rate selection. Index of table BAUD_RATE_TABLE[]. If out of range of the table, 9 would be filled
// Para. mode: stop bit length. Not used. Just set it to 0
void pf_uart_peri_init(uint8_t baud, uint8_t mode)
{
	uint32_t err_code;
	uint32_t baud_rate;
	
	if (baud <= 1 || baud > UART_BAUDRATE_SELECT_MAX)	// Baud rate 300 and 600 in Simba are not surported in Nordic nRF52840 chip
	{
        baud = 9;
    }
    baud_rate = BAUD_RATE_TABLE[baud];
	
	UNUSED_VARIABLE(mode);		// Changing stop bit length is not surported
	
	// UART0 init
	const app_uart_comm_params_t comm_params =
	{
		BLE_RX_DEBUG,
		BLE_TX_DEBUG,
		
		UART_PIN_DISCONNECTED,
		UART_PIN_DISCONNECTED,
		UART_HWFC,
		false,
#if defined (UART_PRESENT)
		baud_rate // NRF_UART_BAUDRATE_115200
#else
		baud_rate // NRF_UARTE_BAUDRATE_115200
#endif
	};
	UART_DRV_EX_INIT(UART0_DRV_EX, 
						&comm_params,
						UART0_RX_BUFFER_SIZE,		// FIFO Needed
						UART0_TX_BUFFER_SIZE,		// FIFO Needed
						uart_event_handle,			// Needed. NULL makes error
						APP_IRQ_PRIORITY_LOWEST,
						err_code);
	APP_ERROR_CHECK(err_code);

	monet_data.uartPeriTXDEnabled = 1;
}
/**@snippet [UART Initialization] */

void pf_uart_peri_deinit(void)
{
	uart_drv_ext_close(UART0_DRV_EX);
	monet_data.uartPeriTXDEnabled = 0;
}

uint32_t pf_uart_peri_tx_one(uint8_t byte)
{
    return uart_drv_ext_put(UART0_DRV_EX, byte);
}

uint32_t pf_uart_peri_rx_one(uint8_t *p_data)
{
    return uart_drv_ext_get(UART0_DRV_EX, p_data);
}

bool pf_uart_peri_tx_queue_is_empty(void)
{
	return uart_drv_ext_tx_fifo_is_empty(UART0_DRV_EX);
}

static bool ble_dg_info_to_tpms = false;

// Enable BLE diagnostic info output to TPMS UART port
void ble_dg_info_to_tpms_enable(void)
{
	ble_dg_info_to_tpms = true;
}

// Disable BLE diagnostic info output to TPMS UART port
void ble_dg_info_to_tpms_disable(void)
{
	ble_dg_info_to_tpms = false;
}

// Check BLE diagnostic info output is to TPMS UART port or not
bool is_ble_dg_info_to_tpms(void)
{
	return ble_dg_info_to_tpms;
}

// Print BLE diagnostic info to TPMS UART port
// Note: ble_dg_printf() print info to peri UART
int ble_dg_printf(const char* fmt, ...)
{
	int retval=0;
	va_list ap;

	if (is_ble_dg_info_to_tpms() != true)
		return 0;
	va_start(ap, fmt);			/* Initialize the va_list */
	retval = vprintf(fmt, ap);	/* Call vprintf */
	va_end(ap);					/* Cleanup the va_list */
	return retval;
}

/**
 * @brief WDT events handler.
 */
static void wdt_event_handler(void)
{
    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

void pf_wdt_init(void)
{
    uint32_t err_code = NRF_SUCCESS;

    //Configure WDT.
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_pf_wdt_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
}

void pf_wdt_kick(void)
{
    nrf_drv_wdt_channel_feed(m_pf_wdt_channel_id);
}

// Delay (count * 10) ms. Watchdog safe
// Note: watchdog should have been initialized before calling this function
void delay_10ms_wds(uint32_t count)
{
	while (count > 0)
	{
		pf_wdt_kick();
		nrf_delay_ms(10);
		count--;
	}
}

#if defined ( __CC_ARM )
__ASM __STATIC_INLINE void __jump_to_addr(uint32_t new_msp, uint32_t new_lr, uint32_t addr)
{
    MSR MSP, R0;
    MOV LR,  R1;
    BX       R2;
}
#else
__STATIC_INLINE void __jump_to_addr(uint32_t new_msp, uint32_t new_lr, uint32_t addr)
{
    __ASM volatile ("MSR MSP, %[arg]" : : [arg] "r" (new_msp));
    __ASM volatile ("MOV LR,  %[arg]" : : [arg] "r" (new_lr) : "lr");
    __ASM volatile ("BX       %[arg]" : : [arg] "r" (addr));
}
#endif

__STATIC_INLINE void __app_start(uint32_t vector_table_addr)
{
    const uint32_t current_isr_num = (__get_IPSR() & IPSR_ISR_Msk);
    const uint32_t new_msp         = *((uint32_t *)(vector_table_addr));                    // The app's Stack Pointer is found as the first word of the vector table.
    const uint32_t reset_handler   = *((uint32_t *)(vector_table_addr + sizeof(uint32_t))); // The app's Reset Handler is found as the second word of the vector table.
    const uint32_t new_lr          = 0xFFFFFFFF;

    __set_CONTROL(0x00000000);   // Set CONTROL to its reset value 0.
    __set_PRIMASK(0x00000000);   // Set PRIMASK to its reset value 0.
    __set_BASEPRI(0x00000000);   // Set BASEPRI to its reset value 0.
    __set_FAULTMASK(0x00000000); // Set FAULTMASK to its reset value 0.

    ASSERT(current_isr_num == 0); // If this is triggered, the CPU is currently in an interrupt.

    // The CPU is in Thread mode (main context).
    __jump_to_addr(new_msp, new_lr, reset_handler); // Jump directly to the App's Reset Handler.
}

void pf_bootloader_start(void)
{
    uint32_t start_addr = BOOTLOADER_ADDRESS;
    NRF_LOG_RAW_INFO("Running nrf_bootloader_app_start with address: 0x%08x\r", start_addr);

    // Disable and clear interrupts
    // Notice that this disables only 'external' interrupts (positive IRQn).
    NRF_LOG_RAW_INFO("Disabling interrupts. NVIC->ICER[0]: 0x%x\r", NVIC->ICER[0]);

    NVIC->ICER[0]=0xFFFFFFFF;
    NVIC->ICPR[0]=0xFFFFFFFF;
#if defined(__NRF_NVIC_ISER_COUNT) && __NRF_NVIC_ISER_COUNT == 2
    NRF_LOG_RAW_INFO("Disabling interrupts. NVIC->ICER[1]: 0x%x\r", NVIC->ICER[1]);
    NVIC->ICER[1]=0xFFFFFFFF;
    NVIC->ICPR[1]=0xFFFFFFFF;
#endif

    NRF_LOG_FLUSH();
    __app_start(start_addr);
}

extern void advertising_stop(void);

void pf_bootloader_pre_enter(void)
{
//    NRF_LOG_RAW_INFO("In pf_bootloader_pre_enter\r");
//    NRF_LOG_FLUSH();

//    nrfx_gpiote_in_uninit(gpin[GPIO_BLE_WAKE_MDM].pin);
//    nrfx_gpiote_in_uninit(gpin[GPIO_ACC_INT1_PIN].pin);
//    nrfx_gpiote_uninit();
//    pf_adc_poll_finish();
//    pf_adc_deinit();
//    pf_imu_workmode_set(PF_ACC_MODE_OFF);
//    pf_imu_reset();
//    pf_i2c_uninit();
//    pf_systick_stop();
//    app_timer_stop_all();
//    pf_uart_mdm_deinit();
//    pf_wdt_kick();
}

uint32_t pf_bootloader_enter(void)
{
    uint32_t err_code;

    NRF_LOG_RAW_INFO("In pf_bootloader_enter\r");
    NRF_LOG_FLUSH();

    pf_bootloader_pre_enter();

    err_code = sd_power_gpregret_clr(0, 0xffffffff);
    APP_ERROR_CHECK(err_code);

    err_code = sd_power_gpregret_set(0, BOOTLOADER_DFU_START);
    APP_ERROR_CHECK(err_code);

    err_code = sd_softdevice_disable();
    APP_ERROR_CHECK(err_code);

    *((uint32_t *)(0x2003fffc)) = 0xFDFDFDFD;

    // Signal that DFU mode is to be enter to the power management module
    #if PF_USE_JUMP_TO_ENTER_BOOT
    pf_bootloader_start();
    #else
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_DFU);
    #endif /* PF_USE_JUMP_TO_ENTER_BOOT */

    return NRF_SUCCESS;
}

//void pf_BuzzerOff(void)
//{
//    // Turn the buzzer OFF
//    if (buzzer_Is_On)
//    {
//        buzzer_Is_On = false;
//        pf_pwm_stop();
//    }
//}

//void pf_BuzzerOn(void)
//{
//    // Turn the buzzer ON
//    if (buzzer_Is_On == false)
//    {
//        buzzer_Is_On = true;
//        pf_pwm_start();
//    }
//}

/* External Peripheral's Drivers ---------------------------------------------*/
void pf_cfg_before_hibernation(void)
{
//	int i = 0;
	
//	pf_adc_deinit();
//	pf_systick_stop();
//	ion_accRegInit2(LIS_ODR_POWERDOWN, 0, 0, 0, 1);
//	if (monet_data.bI2CisEnabled == true)
//		pf_i2c_uninit();
	if (monet_data.uartPeriTXDEnabled == 1)
		pf_uart_peri_deinit();
	if (monet_data.uartMdmTXDEnabled == 1)
		pf_uart_mdm_deinit();
	
    clock_hfclk_release();  //NALAMCU-186//
	ble_send_timer_stop_c();
	
//	extern bool mp2762a_bfet_ctrl(bool setting);///////
//	mp2762a_bfet_ctrl(false);	///////
	
//	pf_gpio_write(GPIO_BLE_RELAY, 0);
	pf_gpio_write(GPIO_RS232_EN, 0);
	// pf_gpio_write(GPIO_MDM_PWR_KEY, 0);
	pf_gpio_write(GPIO_BLE_CAN_PWR_EN, 0);
	pf_gpio_write(GPIO_CS_12V_EN, 0);
	gpio_deinit(GPIO_Hall_Tamper);
//	pf_gpio_write(GPIO_CHRG_SLEEP_EN, 1);	// Set high to disable battery power path to charger chip
	pf_gpio_write(GPIO_ST_MCU_PWR_EN, 0);
	pf_gpio_write(GPIO_CS_3V3_EN, 0);
	// pf_gpio_write(GPIO_VDD_MDM_EN, 0);
	pf_gpio_write(GPIO_DC_DC_9V5_EN, 0);
	pf_gpio_write(GPIO_CS_nRST, 0);
//	pf_gpio_write(GPIO_LED_ORANGE, 1);
//	pf_gpio_write(GPIO_LED_GREEN, 1);
//	pf_gpio_write(GPIO_LED_RED, 1);
//	pf_gpio_write(GPIO_ONE_BUS_SLPZ, 0);	//
//	pf_gpio_write(GPIO_ST_UART1_TO_UART2_EN, 0);
//	pf_gpio_write(GPIO_ST_UART1_TO_UART3_EN, 0);
//	pf_gpio_write(GPIO_ST_UART2_TO_UART3_EN, 0);
//	pf_gpio_write(GPIO_BAT_ADC_TMP_EN, 0);
	nrf_delay_ms(10);	// Wait for voltage decline
	
	//	pf_gpio_write(GPIO_BLE_RELAY, 0);
	gpio_deinit(GPIO_RS232_EN);
	// gpio_deinit(GPIO_MDM_PWR_KEY);
	gpio_deinit(GPIO_BLE_CAN_PWR_EN);
	gpio_deinit(GPIO_CS_12V_EN);
	gpio_deinit(GPIO_ST_MCU_PWR_EN);
	gpio_deinit(GPIO_CS_3V3_EN);
	// gpio_deinit(GPIO_VDD_MDM_EN);
	gpio_deinit(GPIO_DC_DC_9V5_EN);
	gpio_deinit(GPIO_CS_nRST);
//	pf_gpio_write(GPIO_LED_ORANGE, 1);
//	pf_gpio_write(GPIO_LED_GREEN, 1);
//	pf_gpio_write(GPIO_LED_RED, 1);
//	gpio_deinit(GPIO_ONE_BUS_SLPZ);		//
	gpio_deinit(GPIO_ST_UART1_TO_UART2_EN);
	gpio_deinit(GPIO_ST_UART1_TO_UART3_EN);
	gpio_deinit(GPIO_ST_UART2_TO_UART3_EN);
//	pf_gpio_write(GPIO_BAT_ADC_TMP_EN, 0);
	
//	int i = 0;
//	for (i = 0; i < NUM_OF_GPIO; i++)
//		gpio_deinit(i);
//gpio_deinit(GPIO_BLE_GPIO1);//////
//gpio_deinit(GPIO_BLE_GPIO2);//////
//gpio_deinit(GPIO_BLE_GPIO3);//////
//gpio_deinit(GPIO_BLE_RELAY);//////  //
//gpio_deinit(GPIO_CHRG_INT);//////
//gpio_deinit(GPIO_CHRG_ACOK);//////
//gpio_deinit(GPIO_BLE_Tamper);//////
//gpio_deinit(GPIO_G_SENSOR_INT);//////
//gpio_deinit(GPIO_CHRG_PROCHOT);//////
//gpio_deinit(GPIO_LED_ORANGE);//////
//gpio_deinit(GPIO_LED_GREEN);//////
//gpio_deinit(GPIO_LED_RED);//////
//gpio_deinit(GPIO_BAT_ADC_TMP_EN);//////  //
//	if (nrfx_gpiote_is_init() == true)
//		nrfx_gpiote_uninit();
	
//	advertising_stop();
//	app_timer_stop_all();
}

extern uint32_t ble_aus_advertising_start(void);

void pf_cfg_recover_from_hibernation(void)
{
//	ble_aus_advertising_start();
//	gpio_init_pin(GPIO_ONE_BUS_SLPZ);	//
	gpio_init_pin(GPIO_CS_nRST);
	gpio_init_pin(GPIO_DC_DC_9V5_EN);
	gpio_init_pin(GPIO_VDD_MDM_EN);
	gpio_init_pin(GPIO_CS_3V3_EN);
//	pf_gpio_write(GPIO_CHRG_SLEEP_EN, 0);	// Set low to enable battery power path to charger chip
	gpio_init_pin(GPIO_ST_MCU_PWR_EN);
	gpio_init_pin(GPIO_ST_UART2_TO_UART3_EN);	// Should called after GPIO_ST_MCU_PWR_EN init
	gpio_init_pin(GPIO_ST_UART1_TO_UART3_EN);
	gpio_init_pin(GPIO_ST_UART1_TO_UART2_EN);
	gpio_init_pin(GPIO_Hall_Tamper);
	gpio_init_pin(GPIO_CS_12V_EN);
	gpio_init_pin(GPIO_BLE_CAN_PWR_EN);
	gpio_init_pin(GPIO_MDM_PWR_KEY);
	gpio_init_pin(GPIO_RS232_EN);
	
//	extern bool mp2762a_bfet_ctrl(bool setting);///////
//	mp2762a_bfet_ctrl(true);	///////
	
	ble_send_timer_start_c();
}

void pf_cfg_before_sleep(void)
{
// WARNING: TODO.do not change

//    // nrf_gpio_cfg_default(V_MODULE_EN);
//    // nrf_gpio_cfg_default(P014_DC_DC_EN);
//    // nrf_gpio_cfg_default(BLE_WAKE_MDM);
//    // nrf_gpio_cfg_default(BLE_UART_TXD);
//    nrf_gpio_cfg_default(MOD_PWRKEY);
//    nrf_gpio_cfg_default(BLE_RESET_MDM);
//    nrf_gpio_cfg_default(BLE_UART_RXD);
//    // nrf_gpio_cfg_default(MDM_WAKE_BLE);

//    // nrf_gpio_cfg_default(I2C_SDA);
//    // nrf_gpio_cfg_default(I2C_SCL);
//    // nrf_gpio_cfg_default(ACC_INT1_PIN);
//    // nrf_gpio_cfg_default(P016_BAT_EN);
//    // nrf_gpio_cfg_default(BLE_P017_OUT1_RELAY);
//    // nrf_gpio_cfg_default(VBAT_ADC_BLE);
//    // nrf_gpio_cfg_default(VIN12V_ADC_BLE);
//    // nrf_gpio_cfg_default(BLE_P005_IN1_ANALOG);
//    // nrf_gpio_pin_clear(BLE_P017_OUT1_RELAY);
//    // nrf_gpio_cfg_default(BLE_P018_OUT2_Digital);
//    // nrf_gpio_cfg_default(BLE_P019_IGNI);
//    // nrf_gpio_cfg_default(PWM_CH1_PIN);
//    // nrf_gpio_cfg_default(PWM_CH0_PIN);
//    nrf_gpio_cfg_default(BLE_LDO_EN);
//    // nrf_gpio_cfg_default(P027_CHARGE_EN);
}

void pf_delay_ms(uint32_t ms)
{
    nrf_delay_ms(ms);
}

#if BLE_DTM_ENABLE
void pf_dtm_enter(void)
{
    DFU_FLAG_REGISTER = 0xDDDDDDDD;

    NRF_LOG_RAW_INFO("pf_dtm_enter\r\n");
    NRF_LOG_FLUSH();

    pf_delay_ms(10);

    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_RESET);

    while (1);
}

extern void scan_stop(void);

void pf_dtm_enter_nala(void)
{
	uint32_t err_code;
	
	NRF_LOG_RAW_INFO("pf_dtm_enter\r\n");
    NRF_LOG_FLUSH();
	ble_send_timer_stop_c();
	scan_stop();
	err_code = sd_softdevice_disable();
	if (err_code != NRF_SUCCESS)
	{
		NRF_LOG_RAW_INFO("pf_dtm_enter_nala() SD deinit err %u\r\n", err_code);
		NRF_LOG_FLUSH();
		return;
	}
	pf_delay_ms(10);
	err_code = dtm_init();	// DTM init
	if (err_code != DTM_SUCCESS)
	{
		NRF_LOG_RAW_INFO("pf_dtm_enter_nala() DTM init err %u\r\n", err_code);
		NRF_LOG_FLUSH();
		return;
	}
}

void pf_dtm_init(void)
{
    uint32_t dtm_error_code;

    dtm_error_code = dtm_init();

    if (dtm_error_code != DTM_SUCCESS)
    {
        // DTM cannot be correctly initialized.
    }

    NRF_LOG_RAW_INFO("pf_dtm_init(%d)\r\n", dtm_error_code);
    NRF_LOG_FLUSH();

    monet_data.phonePowerOn = 1;
    monet_data.appActive = 1;

//    pf_uart_init();

    pf_wdt_init();
}

void pf_dtm_process(void)
{
    uint8_t buf[] = "dE";
    BuildFrame('3', buf, 2);
    while (1)
    {
        pf_wdt_kick();
        atel_io_queue_process();
    }
}

void pf_dtm_exit(void)
{
    DFU_FLAG_REGISTER = 0;

    NRF_LOG_RAW_INFO("pf_dtm_exit\r\n");
    NRF_LOG_FLUSH();

    pf_delay_ms(10);

    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_RESET);

    while (1);
}

void pf_dtm_cmd(uint8_t cmd, uint8_t freq, uint8_t len, uint8_t payload)
{
    dtm_cmd_t      command_code = cmd & 0x03;
    dtm_freq_t     command_freq = freq & 0x3F;
    uint32_t       length       = len & 0x3F;
    dtm_pkt_type_t command_payload = payload & 0x03;
    uint32_t       err_code;
    dtm_event_t    result; // Result of a DTM operation.

    err_code = dtm_cmd(command_code, command_freq, length, command_payload);

    NRF_LOG_RAW_INFO("pf_dtm_cmd C(%d) F(%d) L(%d) P(%d) E(%d)\r\n"
                     , cmd
                     , freq
                     , len
                     , payload
                     , err_code);
    NRF_LOG_FLUSH();

    if (dtm_event_get(&result))
    {
        NRF_LOG_RAW_INFO("dtm_event_get(0x%04x)\r\n", result);
        NRF_LOG_FLUSH();
    }
}
#endif /* BLE_DTM_ENABLE */

// Return value: 0, OK; others, error
int32_t platform_imu_i2c_write(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
{
    ret_code_t err_code;
    uint8_t buf[32] = {0};

    if(Bufp == NULL || len > 31)
        return 1;
    buf[0] = Reg;
    memcpy(buf + 1, Bufp, len);
    err_code = nrf_drv_twi_tx(&m_twi, monet_data.AccChipAdd, buf, len + 1, false);
    if (err_code != NRF_SUCCESS)
    {
        return 1;
    }
    return 0;
}

int32_t platform_imu_i2c_2_write(uint8_t addr, uint8_t reg, uint8_t *pbuf, uint16_t len)
{
//    ret_code_t err_code;
//    uint8_t buf[128] = {0};

//    if(pbuf == NULL || len > 127)
//        return -1;
//    err_code = nrf_drv_twi_tx(&m_twi2, addr, &reg, 1, true);
//    if (err_code != NRF_SUCCESS)
//        return -1;
//	
//    memcpy(buf, pbuf, len);
//    err_code = nrf_drv_twi_tx(&m_twi2, addr, buf, len, false);
//    if (err_code != NRF_SUCCESS)
//        return -1;
//    return 0;

	
    ret_code_t err_code;
    uint8_t buf[128] = {0};

    if(pbuf == NULL || len > 127)
        return 1;
    buf[0] = reg;
    memcpy(buf + 1, pbuf, len);
    err_code = nrf_drv_twi_tx(&m_twi2, addr, buf, len + 1, false);
    if (err_code != NRF_SUCCESS)
    {
        return 1;
    }
    return 0;
}

int32_t platform_imu_i2c_write2(void *handle, uint8_t Reg, uint8_t val)
{
	return platform_imu_i2c_write(handle, Reg, &val, 1);
}

int32_t platform_imu_i2c_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
{
    ret_code_t err_code;

    err_code = nrf_drv_twi_tx(&m_twi, monet_data.AccChipAdd, &Reg, 1, true);
    if (err_code != NRF_SUCCESS)
    {
        return 1;
    }

    err_code = nrf_drv_twi_rx(&m_twi, monet_data.AccChipAdd, Bufp, len);
    if (err_code != NRF_SUCCESS)
    {
        return 1;
    }
    return 0;
}

int32_t platform_imu_i2c_2_read(uint8_t addr, uint8_t reg, uint8_t *pbuf, uint16_t len)
{
    ret_code_t err_code;

    err_code = nrf_drv_twi_tx(&m_twi2, addr, &reg, 1, true);
    if (err_code != NRF_SUCCESS)
    {
        return 1;
    }

    err_code = nrf_drv_twi_rx(&m_twi2, addr, pbuf, len);
    if (err_code != NRF_SUCCESS)
    {
        return 1;
    }
    return 0;
}

void pf_imu_init(void)
{
    monet_data.AccChipID = 0;

    if (monet_data.AccChipAdd == PF_IMU_LSM6DSL_I2C_ADDR)
    {
        NRF_LOG_RAW_INFO("Imu lsm6dsl.\r\n");
        NRF_LOG_FLUSH();
        lsm6dsl_drv_init(&(monet_data.AccChipID));
    }
//    else if (monet_data.AccChipAdd == PF_IMU_LIS3DH_I2C_ADDR)
//    {
//        NRF_LOG_RAW_INFO("Imu lis3dh.\r");
//        NRF_LOG_FLUSH();
//        monet_data.AccChipID = 0x33; // LIS_ID_VALUE lis3dh
//        md_LisStart();
//    }
    else if (monet_data.AccChipAdd == PF_IMU_LIS2DH_I2C_ADDR)	// LIS2DH and LIS3DH have same I2C address
    {
//        NRF_LOG_RAW_INFO("Imu lis3dh.\r");
        NRF_LOG_RAW_INFO("Imu lis2dh.\r");
        NRF_LOG_FLUSH();
        monet_data.AccChipID = 0x33; // LIS_ID_VALUE lis2dh
		ion_accRegInit2(config_data.ar[1], config_data.at[1], config_data.adur[1], 0, 0);
    }
    else
    {
        NRF_LOG_RAW_INFO("Imu not found.\r");
        NRF_LOG_FLUSH();
    }
}

void pf_imu_sensitivity_set(uint32_t val)
{
    uint32_t ret_val = 0;

    if (monet_data.AccChipAdd == PF_IMU_LSM6DSL_I2C_ADDR)
    {
        ret_val = lsm6dsl_wkup_senstivity_set(val);
        NRF_LOG_RAW_INFO("Imu lsm6dsl thresh(%d).\r", ret_val);
        NRF_LOG_FLUSH();
    }
    else if (monet_data.AccChipAdd == PF_IMU_LIS3DH_I2C_ADDR)
    {
        NRF_LOG_RAW_INFO("Imu lis3dh thresh(%d).\r", ret_val);
        NRF_LOG_FLUSH();
    }
    else
    {
        NRF_LOG_RAW_INFO("Imu thresh err.\r");
        NRF_LOG_FLUSH();
    }
}

//void pf_imu_reset(void)
//{
//    monet_data.AccChipID = 0;
//    
//    if (monet_data.AccChipAdd == PF_IMU_LSM6DSL_I2C_ADDR)
//    {
//        NRF_LOG_RAW_INFO("Imu lsm6dsl reset.\r");
//        NRF_LOG_FLUSH();
//        lsm6dsl_drv_reset();
//    }
////    else if (monet_data.AccChipAdd == PF_IMU_LIS3DH_I2C_ADDR)
////    {
////        NRF_LOG_RAW_INFO("Imu lis3dh reset.\r");
////        NRF_LOG_FLUSH();
////        // md_LisStop();
////    }
//    else if (monet_data.AccChipAdd == PF_IMU_LIS2DH_I2C_ADDR)	// LIS2DH and LIS3DH have same I2C address
//    {
//        NRF_LOG_RAW_INFO("Imu lis2dh reset.\r");
//        NRF_LOG_FLUSH();
//        ion_accStop();
//    }
//    else
//    {
//        NRF_LOG_RAW_INFO("Imu not found.\r");
//        NRF_LOG_FLUSH();
//    }
//}

//uint8_t pf_imu_int_src_get(uint8_t int_num)
//{
//    uint8_t int_src = 0;

//    if ((int_num == ACC_INTERRUPT_NUM1) && (pf_gpio_read(GPIO_G_SENSOR_INT) == ACC_GYRO_INT_VALID))
//    {
//        if (monet_data.AccChipAdd == PF_IMU_LSM6DSL_I2C_ADDR)
//        {
//            uint8_t int_src_tmp = 0;
//            int_src_tmp = lsm6dsl_intstatus_get(int_num);
//            // NRF_LOG_RAW_INFO("Imu int_src_get int_src_tmp(0x%x).\r", int_src_tmp);
//            // NRF_LOG_FLUSH();
//            if (int_src_tmp & (0x01U << 0)) // bit0: z_wu
//            {
//                int_src |= (0x01U << 5);
//            }
//            if (int_src_tmp & (0x01U << 1)) // bit1: y_wu
//            {
//                int_src |= (0x01U << 3);
//            }
//            if (int_src_tmp & (0x01U << 2)) // bit2: x_wu
//            {
//                int_src |= (0x01U << 1);
//            }
//            if (int_src_tmp & (0x01U << 3)) // bit3: wu_ia
//            {
//                int_src |= (0x01U << 6);
//            }
//        }
////        else if (monet_data.AccChipAdd == PF_IMU_LIS3DH_I2C_ADDR)
////        {
////            int_src = lis3dh_intstatus_get(int_num);
////            // NRF_LOG_RAW_INFO("Imu int_src_get int_src(0x%x).\r", int_src);
////            // NRF_LOG_FLUSH();
////        }
//        else if (monet_data.AccChipAdd == PF_IMU_LIS2DH_I2C_ADDR)
//        {
//            int_src = lis3dh_intstatus_get(int_num);
//            // NRF_LOG_RAW_INFO("Imu int_src_get int_src(0x%x).\r", int_src);
//            // NRF_LOG_FLUSH();
//        }
//        else
//        {
//            NRF_LOG_RAW_INFO("Imu int_src_get err.\r");
//            NRF_LOG_FLUSH();
//        }
//    }

//    return int_src;
//}

#if defined(SUPPORT_ACC_STREAMING)
void pf_imu_value_stream(void)
{
    if (monet_data.AccChipAdd == PF_IMU_LIS3DH_I2C_ADDR)
    {
        mnt_accHeartbeat();
    }
}
#endif

//void pf_imu_workmode_set(pf_AccWorkMode_t workmode)
//{
//    if (monet_data.AccChipAdd == PF_IMU_LSM6DSL_I2C_ADDR)
//    {
//        NRF_LOG_RAW_INFO("Imu lsm6dsl workmode(%d).\r", workmode);
//        NRF_LOG_FLUSH();
//        pf_imu_sensitivity_set(monet_data.AccData.threshold);
//    }
//    else if (monet_data.AccChipAdd == PF_IMU_LIS3DH_I2C_ADDR)
//    {
//        NRF_LOG_RAW_INFO("Imu lis3dh workmode(%d).\r", workmode);
//        NRF_LOG_FLUSH();
//        md_LisSetWorkMode((ars_AccWorkMode_t)workmode);
//    }
//    else
//    {
//        NRF_LOG_RAW_INFO("Imu not found workmode.\r");
//        NRF_LOG_FLUSH();
//    }
//}

void pf_mdm_pwr_init(void)
{
    nrf_gpio_cfg_output(VDD_MDM_EN);
    nrf_gpio_cfg_output(MDM_PWR_KEY);
	nrf_gpio_pin_clear(VDD_MDM_EN);
	nrf_gpio_pin_clear(MDM_PWR_KEY);
}

void pf_mdm_pwr_deinit(void)
{
    // nrf_gpio_cfg_default(VDD_MDM_EN);
    nrf_gpio_cfg_default(MDM_PWR_KEY);
}

void pf_mdm_pwr_ctrl(bool state)
{
    if (state == true)
    {
        nrf_gpio_pin_set(MDM_PWR_KEY);
        nrf_gpio_pin_set(VDD_MDM_EN);
        // Use monet_gpio.counter to set MOD_PWRKEY
        // nrf_delay_ms(600);
        // nrf_gpio_pin_set(MOD_PWRKEY);
    }
    else
    {
        nrf_gpio_pin_clear(MDM_PWR_KEY);
        nrf_gpio_pin_clear(VDD_MDM_EN);
    }
}

void pf_mdm_pwr_key_ctrl(bool state)
{
    if (state == true)
    {
        nrf_gpio_pin_set(MDM_PWR_KEY);
    }
    else
    {
        nrf_gpio_pin_clear(MDM_PWR_KEY);
    }
}

void uint8_to_ascii(uint8_t *p_buf, uint8_t num)
{
    uint8_t temp = num;
    uint16_t i = 0, j = 0;

    for (i = 0; i < 2; i++)
    {
        j = (temp >> ((1 - i) * 4)) & 0x0ful;
        if (j > 9)
        {
            p_buf[i] = j - 10 + 'a';
        }
        else
        {
            p_buf[i] = j + '0';
        }
    }
}

void printf_hex_and_char(uint8_t *p_data, uint16_t len)
{
    uint16_t i = 0, j = 0, k = 0;//, n = 0;
    uint8_t buf[64] = {0};

    for (i = 0; i < len; i++) // 0x20-0x7e
    {
        // sprintf((char *)(buf + k), "%02x ", p_data[i]);
        uint8_to_ascii(buf + k, p_data[i]);
        *(buf + k + 2) = ' ';
        k += 3;

        if ((((i + 1) % 16) == 0) || ((i + 1) == len))
        {
            NRF_LOG_RAW_INFO("%s\r", buf);
            NRF_LOG_FLUSH();
            // n = ((((i + 1) % 16) == 0) ? 16 : (i % 16 + 1));

            // for (k = 0; k < (16 - n); k++)
            // {
            //     sprintf((char *)(buf + k), "   ");
            //     k += 3;
            // }
            // sprintf((char *)(buf + k), "|  ");
            // k += 3;

            memset(buf, 0, sizeof(buf));
            k = 0;

            for (j = ((((i + 1) % 16) == 0) ? (i - 15) : (i - (i % 16))); j <= i; j++)
            {
                if ((p_data[j] >= 0x20) && (p_data[j] <= 0x7e))
                {
                    // sprintf((char *)(buf + k), "%c  ", p_data[j]);
                    *(buf + k) = p_data[j];
                }
                else
                {
                    // sprintf((char *)(buf + k), ".  ");
                    *(buf + k) = '.';
                }
                *(buf + k + 1) = ' ';
                *(buf + k + 2) = ' ';
                k += 3;
            }
            // sprintf((char *)(buf + k), "\r\n");
            // k += 2;

            NRF_LOG_RAW_INFO("%s\r", buf);
            NRF_LOG_FLUSH();

            memset(buf, 0, sizeof(buf));
            k = 0;
        }
    }
}

