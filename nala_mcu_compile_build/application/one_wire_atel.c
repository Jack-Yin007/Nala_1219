/*******************************************************************************
* File Name          : onebus.c
* Author               : Jevon
* Description        : This file provides all the obd functions.

* History:
*  02/23/2017 : onebus V1.00
*******************************************************************************/
#include "one_wire_atel.h"
#include "platform_hal_drv.h"
#include <stdint.h>

/* Private variables ---------------------------------------------------------*/
/*    Symbol 	Description
	S 		START Condition
	AD, 0 	Select DS2482 for Write Access
	AD, 1 	Select DS2482 for Read Access
	Sr 		Repeated START Condition
	P 		STOP Condition
	A 		Acknowledged
	A\ 		Not acknowledged
	(Idle) 	Bus not busy
			Transfer of one byte
	DRST 	Command 'Device Reset', F0h
	WCFG 	Command 'Write Configuration', D2h
	CHSL 	Command 'Channel Select', C3h (DS2482-800 only)
	SRP 		Command 'Set Read Pointer', E1h
	1WRS 	Command '1-Wire Reset', B4h
	1WWB 	Command '1-Wire Write Byte', A5h
	1WRB 	Command '1-Wire Read Byte', 96h
	1WSB 	Command '1-Wire Single Bit', 87h
	1WT 		Command '1-Wire Triplet', 78h*/

#define EXTIIC_DELAY   	10

#define DS2482 0x18

//DS2482-100 Command
#define CMD_DRST			0xF0
#define CMD_WCFG			0xD2
#define CMD_CHSL			0xC3
#define CMD_1WRS			0xB4
#define CMD_1WSB			0xB7
#define CMD_1WWB			0xA5
#define CMD_1WRB			0x96
#define CMD_SRP				0xE1
#define CMD_1WT				0x78

#define STATUS_1WB			0x01
#define STATUS_PPD			0x02
#define STATUS_SD			0x04
#define STATUS_LL			0x08
#define STATUS_RST			0x10
#define STATUS_SBR			0x20
#define STATUS_TSB			0x40
#define STATUS_DIR			0x80

#define POLL_LIMIT			0x20

#define CONFIG_APU			0x01
#define CONFIG_1WS			0x08
#define CONFIG_SPU			0x04


#define MODE_STANDARD		0x00
#define MODE_OVERDRIVE		0x01
#define MODE_STRONG			0x02

//DS18S20 Command
//#define Read_Rom 				0x33   
//#define Match_Rom 			0x55   
//#define Skip_Rom 				0xCC   
//#define Search_Rom 			0xF0   
//#define Alarm_Search 			0xEC   
//#define Temp_Change 			0x44   
//#define Read_Temp 			0xBE   

// DS2482  global state
u8 I2C_address = DS2482;
u8 c1WS, cSPU, cPPM, cAPU;
u8 short_detected;
// DS2482 Search state
u8 LastDiscrepancy;
u8 LastFamilyDiscrepancy;
u8 LastDeviceFlag;

bool is1WireInitialized = false;

static uint8_t doCRC8(uint8_t inData, uint8_t seed);

const u8 crc_array[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
    0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
    0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
    0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
    0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
    0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
    0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
    0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
    0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
    0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
    0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

static u8 calc_crc8(u8 *p, char counter)
{
   u8 crc8 = 0;

   for( ; counter > 0; counter--)
   {
       crc8 = crc_array[crc8^*p];
       p++;
   }
   return crc8;
}

#define DEFAULT_TEMP_10 (850)
#define MAX_CHAIN_LOOPS (100)

#ifdef SUPPORTS_CHAIN_PROTOCOL
#if defined ( __CC_ARM )
#pragma anon_unions
#endif
typedef struct {
    union {
        uint64_t    registrationId; // 64bit id (family, romid, & crc)
        struct {
            uint8_t     family;
            uint8_t     romid[6];       // 48bit id
            uint8_t     crc;
        };
    }reg;
    int16_t       Temp10;
    int16_t       TempLow10;
    int16_t       TempHigh10;
    uint8_t        TempStatus;
} mnt_tempsensor_t;

static mnt_tempsensor_t sensorChain[MAX_TEMP_SENSORS] = { 0 };

uint64_t get1WSensorRegId(uint8_t index) {
    return sensorChain[index].reg.registrationId;
}

void set1WSensorRegId(uint8_t index, uint64_t registrationId) {
    sensorChain[index].reg.registrationId = registrationId;
}

uint8_t get1WSensorFamily(uint8_t index) {
    return sensorChain[index].reg.family;
}

void set1WSensorFamily(uint8_t index, uint8_t family) {
    sensorChain[index].reg.family = family;
}

uint8_t get1WSensorStatus(uint8_t index) {
    return sensorChain[index].TempStatus;
}

void set1WSensorStatus(uint8_t index, uint8_t TempStatus) {
    sensorChain[index].TempStatus = TempStatus;
}

int16_t get1WSensorTemp(uint8_t index) {
    return sensorChain[index].Temp10;
}

void store1WSensorTemp(uint8_t index, int16_t Temp10) {
    sensorChain[index].Temp10 = Temp10;
}

int16_t get1WSensorTempLow(uint8_t index) {
    return sensorChain[index].TempLow10;
}

void store1WSensorTempLow(uint8_t index, int16_t Temp10) {
    sensorChain[index].TempLow10 = Temp10;
}

int16_t get1WSensorTempHigh(uint8_t index) {
    return sensorChain[index].TempHigh10;
}

void store1WSensorTempHigh(uint8_t index, int16_t Temp10) {
    sensorChain[index].TempHigh10 = Temp10;
}

uint64_t pack64Bit(uint8_t *scratchpad) {
    uint8_t i = 0;
    uint64_t regid = 0;
    for (i = 0; i < 8; i++) {
        regid += ((uint64_t)*(scratchpad + i) << (i * 8));
    }
    return regid;
}

static unsigned char OWChainOn(void);
static unsigned char OWChainOff(void);
static unsigned char OWChainDone(void);
static unsigned char OWFullChainDone(void);
static void OWWriteRomId(uint64_t write_data);

/**********************************************************************
* Function:        unsigned char OWChainOn(void)
* PreCondition:    1W must be present see OWReset()
* Input:           None
* Output:          Return the True is slave set chain state.
* Overview:        This function sets chain state on after reset
***********************************************************************/
static unsigned char OWChainOn(void)
{
    uint8_t acknowledgement = 0;
    uint8_t index = 0;

//	NRF_LOG_RAW_INFO("ONEBUS: %s\n", __func__);
//    NRF_LOG_FLUSH();
    OWWriteByte((unsigned char)SKIP_ROM);             // Skip ROM
    OWWriteByte((unsigned char)CHAIN_COMMAND);        // Send Chain command
    OWWriteByte((unsigned char)CHAIN_ON);             // Send Chain ON command
    OWWriteByte((unsigned char)(~CHAIN_ON));          // Send Chain ON command inverted
    do {
        acknowledgement = OWReadByte();  // Read response
        index++; // Ensure an exit strategy if device does not support chaining
    } while (acknowledgement != CHAIN_ACK && index < MAX_CHAIN_LOOPS);

    return (acknowledgement == CHAIN_ACK);
}

/**********************************************************************
* Function:        unsigned char OWChainOff(void)
* PreCondition:    1W must be present see OWReset()
* Input:           None
* Output:          Return the True is slave set chain state.
* Overview:        This function sets chain state on after reset
***********************************************************************/
static unsigned char OWChainOff(void)
{
    uint8_t acknowledgement = 0;
    uint8_t index = 0;

//	NRF_LOG_RAW_INFO("ONEBUS: %s\n", __func__);
//    NRF_LOG_FLUSH();
    if (get1WSensorFamily(0) == CHAIN_FAMILY) {
        OWWriteByte(SKIP_ROM);             // Skip ROM
        OWWriteByte(CHAIN_COMMAND);        // Send Chain command
        OWWriteByte(CHAIN_OFF);            // Send Chain Done command
        OWWriteByte(~CHAIN_OFF);           // Send Chain Done command inverted
        do {
            acknowledgement = OWReadByte();  // Read response
            index++; // Ensure an exit strategy if device does not support chaining
        } while (acknowledgement != CHAIN_ACK && index < MAX_CHAIN_LOOPS);
        return (acknowledgement == CHAIN_ACK);
    } else { 
        return true;
    }
}

/**********************************************************************
* Function:        unsigned char OWChainDone(void)
* PreCondition:    1W must be present see OWReset()
* Input:           None
* Output:          Return the True is slave set chain state.
* Overview:        This function sets chain state on after reset
***********************************************************************/
static unsigned char OWChainDone(void)
{
    uint8_t acknowledgement = 0xFF;
    uint8_t index = 0;
    
//	NRF_LOG_RAW_INFO("ONEBUS: %s\n", __func__);
//    NRF_LOG_FLUSH();
    if (get1WSensorFamily(0) == CHAIN_FAMILY) {
        OWWriteByte((unsigned char)CHAIN_COMMAND);      // Send Chain command
        OWWriteByte((unsigned char)CHAIN_DONE);         // Send Chain Done command
        OWWriteByte((unsigned char)(~CHAIN_DONE));      // Send Chain Done command inverted
        do {
            acknowledgement = OWReadByte();  // Read response
            index++; // Ensure an exit strategy if device does not support chaining
        } while (acknowledgement != CHAIN_ACK && index < MAX_CHAIN_LOOPS);

//        NRF_LOG_RAW_INFO("ONEBUS: %s ack:0x%02x\n", __func__, acknowledgement);
//        NRF_LOG_FLUSH();
        return (acknowledgement == CHAIN_ACK);
    } else {
        return true;
    }
}

/**********************************************************************
* Function:        unsigned char OWFullChainDone(void)
* PreCondition:    1W must be present see OWReset()
* Input:           None
* Output:          Return the True is slave set chain state.
* Overview:        This function sets chain state on after reset
***********************************************************************/
static unsigned char OWFullChainDone(void)
{
    uint8_t acknowledgement = 0xFF;
    uint8_t index = 0;
    
//	NRF_LOG_RAW_INFO("ONEBUS: %s\n", __func__);
//    NRF_LOG_FLUSH();
    if (get1WSensorFamily(0) == CHAIN_FAMILY) {
        if (OWReset()) {
            OWWriteByte(SKIP_ROM);                          // Skip ROM
            OWWriteByte((unsigned char)CHAIN_COMMAND);      // Send Chain command
            OWWriteByte((unsigned char)CHAIN_DONE);         // Send Chain Done command
            OWWriteByte((unsigned char)(~CHAIN_DONE));      // Send Chain Done command inverted
            do {
                acknowledgement = OWReadByte();  // Read response
                index++; // Ensure an exit strategy if device does not support chaining
            } while (acknowledgement != CHAIN_ACK && index < MAX_CHAIN_LOOPS);
        }
        return (acknowledgement == CHAIN_ACK);
    }
    else {
        return true;
    }
}

void OWDiscovery(void)
{
    uint8_t	 i, sensor, CRC = 0;
    uint8_t scratchpad[8] = { 0 };
    uint64_t regid = 0;

//	NRF_LOG_RAW_INFO("ONEBUS: %s\n", __func__);
//    NRF_LOG_FLUSH();
    for (sensor = 0; sensor < MAX_TEMP_SENSORS; sensor++) {
        set1WSensorRegId(sensor, 0); // Initialize to zero to allow new discovery
    }

    if (OWReset()) {
        OWChainOn();
    }
    if (OWReset()) {
        // Read first device
        OWWriteByte(CONDITIONAL_READ_ROM);                // Read RomID device to determine family type

        CRC = 0;
        for (i = 0; i < 8; i++) {
            scratchpad[i] = OWReadByte();  // Read Scratchpad one byte at a time
            CRC = doCRC8(scratchpad[i], CRC);
        }
        if (CRC == 0) {
            regid = pack64Bit(scratchpad);
            set1WSensorRegId(0, regid);
            NRF_LOG_RAW_INFO("ONEBUS: %s sensor:%d ID:0x%08X%08X\n",
                __func__, 0, 
                ((get1WSensorRegId(0) & 0xFFFFFFFF00000000) >> 32), 
                ((get1WSensorRegId(0) & 0x00000000FFFFFFFF) >> 0));
            NRF_LOG_FLUSH();
        } else {
            // No Chain Sensors detected try non-chain
            if (OWReset()) {
                // Read first device
                OWWriteByte(READ_ROM);                // Read RomID device to determine family type

                CRC = 0;
                for (i = 0; i < 8; i++) {
                    scratchpad[i] = OWReadByte();  // Read Scratchpad one byte at a time
                    CRC = doCRC8(scratchpad[i], CRC);
                }
                if (CRC == 0) {
                    regid = pack64Bit(scratchpad);
                    set1WSensorRegId(0, regid);
                }
            }
        }
        if (get1WSensorFamily(0) == CHAIN_FAMILY) { // If first device is a chainable device check there are more
            if (OWChainDone()) {
                regid = 0;
                for (sensor = 1; sensor < MAX_TEMP_SENSORS; sensor++) {
                    set1WSensorRegId(sensor, 0); // Clear out data to find again
                    if (OWReset()) {
                        OWWriteByte(CONDITIONAL_READ_ROM);                    // Read next ROM

                        CRC = 0;
                        for (i = 0; i < 8; i++) {
                            scratchpad[i] = OWReadByte();  // Read Scratchpad one byte at a time
                            CRC = doCRC8(scratchpad[i], CRC);
                        }
                        if (CRC == 0) {
                            regid = pack64Bit(scratchpad);
                            set1WSensorRegId(sensor, regid);
                            OWChainDone();
                            NRF_LOG_RAW_INFO("ONEBUS: %s sensor:%d ID:0x%08X%08X\n",
                                __func__, sensor, 
                                ((get1WSensorRegId(sensor) & 0xFFFFFFFF00000000) >> 32), 
                                ((get1WSensorRegId(sensor) & 0x00000000FFFFFFFF) >> 0));
                            NRF_LOG_FLUSH();
                        } else {
                            break;  // No more devices
                        }
                    }
                }
            }
            OWChainOff();
        }
    }
    else {
        // No Sensors detected
        for (i = 0; i < MAX_TEMP_SENSORS; i++) {
            set1WSensorRegId(i, 0);
        }
        NRF_LOG_RAW_INFO("ONEBUS: %s Failed to read sensor(s)\n", __func__);
        NRF_LOG_FLUSH();
    }
    if (get1WSensorFamily(0) == CHAIN_FAMILY) {
        if (OWReset()) {
            OWChainOff();  // Ensure we reset the chain
        }
    }
	NRF_LOG_RAW_INFO("ONEBUS: %s SensFam:0x%02X\n", __func__, get1WSensorFamily(0));
    NRF_LOG_FLUSH();
}

#endif

void OWInit(void) {
    int index = 0;
    // Initialize the Sensor chain storage
	NRF_LOG_RAW_INFO("ONEBUS: %s\n", __func__);
    NRF_LOG_FLUSH();
    for (index = 0; index < MAX_TEMP_SENSORS; index++) {
        // MNT-155, initialize the structure
        memset(&sensorChain[index], 0, sizeof(mnt_tempsensor_t));
        set1WSensorRegId(index, 0);
        store1WSensorTemp(index, TEMP_NO_AVAILABLE);
        store1WSensorTempLow(index, TEMP_NO_THRESHOLD);
        store1WSensorTempHigh(index, TEMP_NO_THRESHOLD);
    }
    
    OneBusInit();
    OWDiscovery();
    // goto Sleep
    OneBusSleepControl(SLEEP); // NALAMCU-31
}

/**********************************************************************
 * Function:        void OW_write_byte (unsigned char write_data)
 * PreCondition:    None
 * Input:           Send byte to 1-wire slave device
 * Output:          None
 * Overview:        This function used to transmit a complete byte to slave device.
 *
 ***********************************************************************/
static void OWWriteRomId(uint64_t write_data)
{
    unsigned char loop;
    uint8_t byte;

//	NRF_LOG_RAW_INFO("ONEBUS: %s LSB order ", __func__);
    for (loop = 0; loop < sizeof(uint64_t); loop++)
    {
        byte = (unsigned char)((write_data >> (loop * 8)) & 0xFF);
        OWWriteByte(byte);    //Sending LS-byte first
//    	NRF_LOG_RAW_INFO("%02x", byte);
    }
//    NRF_LOG_RAW_INFO("\n");
//    NRF_LOG_FLUSH();
}

static uint8_t doCRC8(uint8_t inData, uint8_t seed)
{
	uint8_t bitsLeft;
	uint8_t temp;
 
	for (bitsLeft = 8; bitsLeft > 0; bitsLeft--) {
		temp = ((seed ^ inData) & 0x01);
		if (temp == 0) {
			seed >>= 1;
		} else {
			seed ^= 0x18;
			seed >>= 1;
			seed |= 0x80;
		}
		inData >>= 1;
	}
	return seed;    
}

void ow_temp_match_read(void)
{
    uint8_t	i, CRC, newStatus, sensor;
    volatile uint8_t scratchpad[16];
    int16_t	temperature;

	NRF_LOG_RAW_INFO("ONEBUS: %s\n", __func__);
    NRF_LOG_FLUSH();
    OWLevel(MODE_STANDARD);
    for (sensor = 0; sensor < MAX_TEMP_SENSORS; sensor++) {
        if (OWReset()) {

            OWWriteByte(MATCH_ROM);                    // MATCH ROM
            OWWriteRomId(get1WSensorRegId(sensor));
            OWWriteByte(READ_SCRATCHPAD);             // Read Scratchpad command

            CRC = 0;
            NRF_LOG_RAW_INFO("ONEBUS: %s ScratchPad LSB order ", __func__);
            for (i = 0; i < 9; i++) {
                scratchpad[i] = OWReadByte();  // Read Scratchpad one byte at a time
                CRC = doCRC8(scratchpad[i], CRC);
            	NRF_LOG_RAW_INFO("%02x", scratchpad[i]);
            }
            NRF_LOG_RAW_INFO("\n");
            //if (CRC == 0) {
            temperature = ((int16_t)(scratchpad[0] + (scratchpad[1] << 8)));
            if (((int16_t)0x7d0 >= temperature) && 
                ((int16_t)0xffa8 <= temperature)) { // Temperature must be a valid range -55 to 125 value
                NRF_LOG_RAW_INFO("ONEBUS: %s sensor:%d, temp:%d\n", __func__, sensor, temperature);
                NRF_LOG_FLUSH();
                temperature = (temperature * 10) / 16;
                if (temperature != DEFAULT_TEMP_10) {
                    // We have a valid (not default) temperature
                    monet_data.Temp10 = temperature;
                    // PUMAMCU-108
                    newStatus = (get1WSensorStatus(sensor) == TEMP_NO_AVAILABLE) ? TEMP_NORMAL : get1WSensorStatus(sensor); // Retrieve current status if not unknown
                    if ((get1WSensorStatus(sensor) != TEMP_LOW) &&
                        (get1WSensorTempLow(sensor) != (int16_t)TEMP_NO_THRESHOLD) &&
                        (temperature < get1WSensorTempLow(sensor))) {
                        newStatus = TEMP_LOW;
                    }
                    if ((get1WSensorStatus(sensor) != TEMP_HIGH) &&
                        (get1WSensorTempHigh(sensor) != (int16_t)TEMP_NO_THRESHOLD) &&
                        (temperature > get1WSensorTempHigh(sensor))) {
                        newStatus = TEMP_HIGH;
                    }
                    // PUMAMCU-108
                    if ((temperature <= get1WSensorTempHigh(sensor)) && 
                        (temperature >= get1WSensorTempLow(sensor)) && 
                        get1WSensorStatus(sensor) != TEMP_NORMAL) {
                        newStatus = TEMP_NORMAL; // Reset back to normal mode
                    }
                    if (newStatus != get1WSensorStatus(sensor)) { // If status changes indicate such
                        set1WSensorStatus(sensor, newStatus);
                        monet_gpio.Intstatus |= 1 << INT_TEMP_ONEWIRE;
                    }
                }
            }
            else {
                monet_data.Temp10 = TEMP_NO_SENSOR;
                store1WSensorTemp(sensor, monet_data.Temp10);
            }
        }
        store1WSensorTemp(sensor, monet_data.Temp10);
    }
    // End of timing sensitive, enable IRQ
    __enable_irq(); // PUMAMCU-115
}

void ow_temp_start_conversion(void)
{
	NRF_LOG_RAW_INFO("ONEBUS: %s\n", __func__);
    NRF_LOG_FLUSH();
    if (get1WSensorFamily(0) == CHAIN_FAMILY) {
        if (OWReset()) {
            OWChainOff(); // Disable chain
        }
    }
    if (OWReset()) {
        // set strong pullup enable
        cSPU = CONFIG_SPU;

		// Start Conversion
		OWWriteByte(SKIP_ROM);                    // Skip ROM
		OWWriteBytePower(START_TEMP_CONVERSION);  // Start conversion
	} else {
		monet_data.Temp10 = TEMP_NO_SENSOR;
	}
}

void oneWire_oneSecond(void) {
	static uint8_t readTimer = 0;

	// Check if we have thresholds
	if (!monet_data.phonePowerOn) {
        if ((get1WSensorTempLow(0) == (uint16_t)TEMP_NO_THRESHOLD) &&
            (get1WSensorTempHigh(0) == (uint16_t)TEMP_NO_THRESHOLD) && 
            (get1WSensorTempLow(1) == (uint16_t)TEMP_NO_THRESHOLD) &&
            (get1WSensorTempHigh(1) == (uint16_t)TEMP_NO_THRESHOLD)) {
            return;
        }
	}

	switch (readTimer) {
    case 0:
        // Wakeup
        OneBusSleepControl(WAKE); // NALAMCU-31
		ow_temp_start_conversion();
		break;
	case 1:
        ow_temp_match_read();
        // Sleep
        OneBusSleepControl(SLEEP); // NALAMCU-31
        break;
	case 11:
	case 60:
        // Reset state machine
		readTimer = 0xFF;
		break;
	}
	readTimer++;
}

#ifdef HARDWARE_IIC_DRV_SUPPORT
	// To avoid compiler warnings
#else
static void ExtIicDisableAllInterrupt(void)
{
	// SystemDisableAllInterrupt() ;// Disable Interrupt
}

static void ExtIicEnableAllInterrupt(void)
{
	// SystemEnableAllInterrupt();	// Enable Interrupt
}
#endif

void I2c_WriteRegister(uint8 address, uint8 reg,uint8 val)
{
	#ifdef HARDWARE_IIC_DRV_SUPPORT
	if(platform_imu_i2c_2_write((uint8_t)address,(uint8_t)reg,(uint8_t *)&val,1) != 0)
	{
		NRF_LOG_RAW_INFO("ONEBUS: Write Byte Fail\n");
		NRF_LOG_RAW_INFO("ONEBUS: addr:0x%02x, reg:0x%02x, val:%u\n", address, reg, val);
        NRF_LOG_FLUSH();
	}
	#else
	ExtIicDisableAllInterrupt() ;// Disable Interrupt
	I2C_start();
	I2C_write(address&I2C_WRITE);
	I2C_write(reg);
	I2C_write(val);
	I2C_stop();
	ExtIicEnableAllInterrupt();	// Enable Interrupt
	#endif
}

u8  I2C_ReadRegister(uint8 address, uint8 reg)
{
	u8 rdata;
	
	#ifdef HARDWARE_IIC_DRV_SUPPORT
	if(platform_imu_i2c_2_read((uint8_t)address,(uint8_t)reg,(u8 *)&rdata,1) != 0)
	{
		NRF_LOG_RAW_INFO("ONEBUS: Read Byte Fail\n");
		NRF_LOG_RAW_INFO("ONEBUS: addr:0x%02x, reg:0x%02x, val:%u\n", address, reg, rdata);
        NRF_LOG_FLUSH();
	}
	#else
	ExtIicDisableAllInterrupt() ;// Disable Interrupt
	I2C_start();
	I2C_write(address&I2C_WRITE);
	I2C_write(reg);
	I2C_start();
	I2C_write(address|I2C_READ);
	rdata=I2C_read(NACK);
	I2C_stop();
	ExtIicEnableAllInterrupt();	// Enable Interrupt
	#endif
	
	return(rdata);
}

#ifdef HARDWARE_IIC_DRV_SUPPORT
static void I2C_Data_Transmit(uint8 address, uint8 wdata)
{
	if(nrf_drv_twi_tx(&m_twi2,(uint8_t)address,(u8 *)&wdata,1,true) != 0)
	{
		NRF_LOG_RAW_INFO("ONEBUS: Transmit Fail\n");
		NRF_LOG_RAW_INFO("ONEBUS: addr:0x%02x, data:%u\n", address, wdata);
        NRF_LOG_FLUSH();
	}
}

static u8 I2C_Data_Receive(uint8 address)
{
	u8 rdata;
	if(nrf_drv_twi_rx(&m_twi2, (uint8_t)address,(u8 *)&rdata,1) != 0)
	{
		NRF_LOG_RAW_INFO("ONEBUS: Receive Fail\n");
		NRF_LOG_RAW_INFO("ONEBUS: addr:0x%02x, data:%u\n", address, rdata);
        NRF_LOG_FLUSH();
	}
	return rdata;
}
#endif

void OneBusSleepControl(uint8_t Status)
{
//	configGPIO(GPIO_ONE_BUS_SLPZ, PIN_STATUS(0, 0, 1, (Status & 0x1)));
	configGPIO(GPIO_ONE_BUS_SLPZ, PIN_STATUS(0, 1, 1, (Status & 0x1)));	// Push-Pull mode is tested better than Open-Drain mode. Push-Pull (>80 uA), Open-Drain (>300 uA). The nRF52840 Pull-Up resistor is ~13 kohm, that leaks current about 3.3V/13000 ohm = 254 uA
    pf_gpio_write(GPIO_ONE_BUS_SLPZ, (Status & 0x1));
    
    NRF_LOG_RAW_INFO("ONEBUS: %s(%d) is %s\n", __func__, Status, (Status & 0x1) ? "Awake" : "Sleeping");
    NRF_LOG_FLUSH();
}

void OWdelay(u32 Delay)
{   
  while(Delay--)   
  {   
    u8 j=10;   
    while(j--);   
  }   
}   

void OneBusInit(void)
{
#ifndef HARDWARE_IIC_DRV_SUPPORT
	OB_I2C_Init();
#endif
	// Print Out
	NRF_LOG_RAW_INFO("%s\n", __func__);
    NRF_LOG_FLUSH();
	// Wakeup
	OneBusSleepControl(WAKE); // NALAMCU-31
    // Check if device is awake
    DS2482_detect();
}

//--------------------------------------------------------------------------
// DS2428 Detect routine that sets the I2C address and then performs a
// device reset followed by writing the configuration byte to default values:
//   1-Wire speed (c1WS) = standard (0)
//   Strong pullup (cSPU) = off (0)
//   Presence pulse masking (cPPM) = off (0)
//   Active pullup (cAPU) = on (CONFIG_APU = 0x01)
//
// Returns: TRUE if device was detected and written
//          FALSE device not detected or failure to write configuration byte
//
u8 DS2482_detect(void)
{
	// reset the DS2482 ON selected address
	if (!DS2482_reset())
	{
		// Print Out
		NRF_LOG_RAW_INFO("ONEBUS: Reset Err\n");
        NRF_LOG_FLUSH();
		return FALSE;
	}

	// default configuration
	c1WS = FALSE;
	cSPU = FALSE;
	cPPM = FALSE;
	cAPU = CONFIG_APU;


	// write the default configuration setup
	if (!DS2482_write_config(c1WS | cSPU | cPPM | cAPU))
	{
		// Print Out
		NRF_LOG_RAW_INFO("ONEBUS: Wr cfg Err\n");
        NRF_LOG_FLUSH();
		return FALSE;
	}

    NRF_LOG_RAW_INFO("ONEBUS: %s was found\n", __func__);
    NRF_LOG_FLUSH();
	return TRUE;
}

//--------------------------------------------------------------------------
// Perform a device reset on the DS2482
//
// Returns: TRUE if device was reset
//          FALSE device not detected or failure to perform reset
//
u8 DS2482_reset(void)
{
	u8 status;

	// Device Reset
	//   S AD,0 [A] DRST [A] Sr AD,1 [A] [SS] A\ P
	//  [] indicates from slave
	//  SS status byte to read to verify state

#ifdef HARDWARE_IIC_DRV_SUPPORT
	status = I2C_ReadRegister(I2C_address, CMD_DRST);
#else
	I2C_start();
	I2C_write(I2C_address & I2C_WRITE);
	I2C_write(CMD_DRST);
	I2C_rep_start();
	I2C_write(I2C_address | I2C_READ);
	status = I2C_read(NACK);
	I2C_stop();
#endif
	// Print Out
	NRF_LOG_RAW_INFO("ONEBUS: %s status:0x%02x\n", __func__, status);
    NRF_LOG_FLUSH();
	// check for failure due to incorrect read back of status
	return ((status & 0xF7) == 0x10);
}

//--------------------------------------------------------------------------
// Write the configuration register in the DS2482. The configuration
// options are provided in the lower nibble of the provided config byte.
// The uppper nibble in bitwise inverted when written to the DS2482.
//
// Returns:  TRUE: config written and response correct
//           FALSE: response incorrect
//
u8 DS2482_write_config(u8 config)
{
	u8 read_config;

	// Write configuration (Case A)
	//   S AD,0 [A] WCFG [A] CF [A] Sr AD,1 [A] [CF] A\ P
	//  [] indicates from slave
	//  CF configuration byte to write
	#ifdef HARDWARE_IIC_DRV_SUPPORT
	I2c_WriteRegister(I2C_address, CMD_WCFG, config | (~config << 4));
	read_config = I2C_ReadRegister(I2C_address, CMD_WCFG);
	#else
	I2C_start();
	I2C_write(I2C_address & I2C_WRITE);
	I2C_write(CMD_WCFG);
	I2C_write(config | (~config << 4));
	I2C_rep_start();
	I2C_write(I2C_address | I2C_READ);
	read_config = I2C_read(NACK);
	I2C_stop();
	#endif
	
	// check for failure due to incorrect read back
	if (config != read_config)
	{
		// handle error
		// ...
		DS2482_reset();
		// Print Out
		NRF_LOG_RAW_INFO("ONEBUS: Cfg Err(0x%02x 0x%02x)\n", config, read_config);
        NRF_LOG_FLUSH();
		return FALSE;
	}

//    NRF_LOG_RAW_INFO("ONEBUS: %s (0x%02x:0x%02x)\n", __func__, config, read_config);
//    NRF_LOG_FLUSH();
	return TRUE;
}

//--------------------------------------------------------------------------
// Select the 1-Wire channel on a DS2482-800.
//
// Returns: TRUE if channel selected
//          FALSE device not detected or failure to perform select
//
u8 DS2482_channel_select(u8 channel)
{
	u8 ch, ch_read, check;

	// Channel Select (Case A)
	//   S AD,0 [A] CHSL [A] CC [A] Sr AD,1 [A] [RR] A\ P
	//  [] indicates from slave
	//  CC channel value
	//  RR channel read back

	switch (channel)
	{
		default: case 0: ch = 0xF0; ch_read = 0xB8; break;
		case 1: ch = 0xE1; ch_read = 0xB1; break;
		case 2: ch = 0xD2; ch_read = 0xAA; break;
		case 3: ch = 0xC3; ch_read = 0xA3; break;
		case 4: ch = 0xB4; ch_read = 0x9C; break;
		case 5: ch = 0xA5; ch_read = 0x95; break;
		case 6: ch = 0x96; ch_read = 0x8E; break;
		case 7: ch = 0x87; ch_read = 0x87; break;
	};

	#ifdef HARDWARE_IIC_DRV_SUPPORT
	I2c_WriteRegister(I2C_address, CMD_CHSL, ch);
	check = I2C_Data_Receive(I2C_address);
	#else
	I2C_start();
	I2C_write(I2C_address & I2C_WRITE);
	I2C_write(CMD_CHSL);


	I2C_write(ch);
	I2C_rep_start();
	I2C_write(I2C_address | I2C_READ);
	check = I2C_read(NACK);
	I2C_stop();
	#endif

	// check for failure due to incorrect read back of channel
	return (check == ch_read);
}

//--------------------------------------------------------------------------
// Reset all of the devices on the 1-Wire Net and return the result.
//
// Returns: TRUE(1):  presence pulse(s) detected, device(s) reset
//          FALSE(0): no presence pulses detected
//
u8 OWReset(void)
{
	u8 status;
	u8 poll_count = 0;

   // Ensure SPU is not on
   DS2482_write_config(c1WS | cPPM | cAPU); // Make sure SPU is not configured
   
	// 1-Wire reset (Case B)
	//   S AD,0 [A] 1WRS [A] Sr AD,1 [A] [Status] A [Status] A\ P
	//                                   \--------/
	//                       Repeat until 1WB bit has changed to 0
	//  [] indicates from slave
	#ifdef HARDWARE_IIC_DRV_SUPPORT
	I2C_Data_Transmit(I2C_address, CMD_1WRS);

	status = I2C_Data_Receive(I2C_address);
	do
	{
		status = I2C_Data_Receive(I2C_address);
	}
	while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));
//	NRF_LOG_RAW_INFO("ONEBUS: %s status:0x%02x, poll_count:%d\n", __func__, status, poll_count);
//    NRF_LOG_FLUSH();
	#else
	I2C_start();
	I2C_write(I2C_address & I2C_WRITE);
	I2C_write(CMD_1WRS);
	I2C_rep_start();
	I2C_write(I2C_address | I2C_READ);

	// loop checking 1WB bit for completion of 1-Wire operation
	// abort if poll limit reached
	status = I2C_read(ACK);
	do
	{
		status = I2C_read((status & STATUS_1WB)?ACK:NACK);
	}
	while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));

	I2C_stop();
	#endif

	// check for failure due to poll limit reached
	if (poll_count >= POLL_LIMIT)
	{
		// handle error
		// ...
		DS2482_reset();
		// Print Out
		NRF_LOG_RAW_INFO("ONEBUS: Rest Err\n");
        NRF_LOG_FLUSH();
		return FALSE;
	}

	// check for short condition
	if (status & STATUS_SD)
		short_detected = TRUE;
	else
		short_detected = FALSE;

	// check for presence detect
	if (status & STATUS_PPD)
		return TRUE;
	else
		return FALSE;
}

//--------------------------------------------------------------------------
// Send 1 bit of communication to the 1-Wire Net.
// The parameter 'sendbit' least significant bit is used.
//
// 'sendbit' - 1 bit to send (least significant byte)
//
void OWWriteBit(u8 sendbit)
{
   OWTouchBit(sendbit);
}

//--------------------------------------------------------------------------
// Reads 1 bit of communication from the 1-Wire Net and returns the
// result
//
// Returns:  1 bit read from 1-Wire Net
//
u8 OWReadBit(void)
{
   return OWTouchBit(0x01);
}

//--------------------------------------------------------------------------
// Send 1 bit of communication to the 1-Wire Net and return the
// result 1 bit read from the 1-Wire Net. The parameter 'sendbit'
// least significant bit is used and the least significant bit
// of the result is the return bit.
//
// 'sendbit' - the least significant bit is the bit to send
//
// Returns: 0:   0 bit read from sendbit
//          1:   1 bit read from sendbit
//
u8 OWTouchBit(u8 sendbit)
{
	u8 status;
	u8 poll_count = 0;

	// 1-Wire bit (Case B)
	//   S AD,0 [A] 1WSB [A] BB [A] Sr AD,1 [A] [Status] A [Status] A\ P
	//                                          \--------/
	//                           Repeat until 1WB bit has changed to 0
	//  [] indicates from slave
	//  BB indicates byte containing bit value in msbit

	#ifdef HARDWARE_IIC_DRV_SUPPORT
	I2c_WriteRegister(I2C_address, CMD_1WSB, sendbit ? 0x80 : 0x00);

	status = I2C_Data_Receive(I2C_address);
	do
	{
		status = I2C_Data_Receive(I2C_address);
	}
	while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));
	#else
	I2C_start();
	I2C_write(I2C_address & I2C_WRITE);
	I2C_write(CMD_1WSB);
	I2C_write(sendbit ? 0x80 : 0x00);
	I2C_rep_start();
	I2C_write(I2C_address | I2C_READ);

	// loop checking 1WB bit for completion of 1-Wire operation
	// abort if poll limit reached
	status = I2C_read(ACK);
	do
	{
		status = I2C_read((status & STATUS_1WB)?ACK:NACK);
	}
	while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));

	I2C_stop();
	#endif
	
	// check for failure due to poll limit reached
	if (poll_count >= POLL_LIMIT)
	{
		// handle error
		// ...
		DS2482_reset();
		return 0;
	}

	// return bit state
	if (status & STATUS_SBR)
		return 1;
	else
		return 0;
}

//--------------------------------------------------------------------------
// Send 8 bits of communication to the 1-Wire Net and verify that the
// 8 bits read from the 1-Wire Net are the same (write operation).
// The parameter 'sendbyte' least significant 8 bits are used.
//
// 'sendbyte' - 8 bits to send (least significant byte)
//
// Returns:  TRUE: bytes written and echo was the same
//           FALSE: echo was not the same
//
void OWWriteByte(u8 sendbyte)
{
	u8 status;
	u8 poll_count = 0;

	// 1-Wire Write Byte (Case B)
	//   S AD,0 [A] 1WWB [A] DD [A] Sr AD,1 [A] [Status] A [Status] A\ P
	//                                          \--------/
	//                             Repeat until 1WB bit has changed to 0
	//  [] indicates from slave
	//  DD data to write
	#ifdef HARDWARE_IIC_DRV_SUPPORT
	I2c_WriteRegister(I2C_address, CMD_1WWB, sendbyte);

	status = I2C_Data_Receive(I2C_address);
	do
	{
		status = I2C_Data_Receive(I2C_address);
	}
	while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));
	#else
	I2C_start();
	I2C_write(I2C_address & I2C_WRITE);
	I2C_write(CMD_1WWB);
	I2C_write(sendbyte);
	I2C_rep_start();
	I2C_write(I2C_address | I2C_READ);

	// loop checking 1WB bit for completion of 1-Wire operation
	// abort if poll limit reached
	status = I2C_read(ACK);
	do
	{
		status = I2C_read((status & STATUS_1WB)?ACK:NACK);
	}
	while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));

	I2C_stop();
	#endif
	// check for failure due to poll limit reached
	if (poll_count >= POLL_LIMIT)
	{
		// handle error
		// ...
		DS2482_reset();
	}
}

//--------------------------------------------------------------------------
// Send 8 bits of read communication to the 1-Wire Net and return the
// result 8 bits read from the 1-Wire Net.
//
// Returns:  8 bits read from 1-Wire Net
//
u8 OWReadByte(void)
{
	u8 data, status;
	u8 poll_count = 0;

	/*/ 1-Wire Read Bytes (Case C)
	//   S AD,0 [A] 1WRB [A] Sr AD,1 [A] [Status] A [Status] A\
	//                                   \--------/
	//                     Repeat until 1WB bit has changed to 0
	//   Sr AD,0 [A] SRP [A] E1 [A] Sr AD,1 [A] DD A\ P
	//
	//  [] indicates from slave
	//  DD data read */
	#ifdef HARDWARE_IIC_DRV_SUPPORT
	I2C_Data_Transmit(I2C_address, CMD_1WRB);

	status = I2C_Data_Receive(I2C_address);
	do
	{
		status = I2C_Data_Receive(I2C_address);
	}
	while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));
	#else

	I2C_start();
	I2C_write(I2C_address & I2C_WRITE);
	I2C_write(CMD_1WRB);
	I2C_rep_start();
	I2C_write(I2C_address | I2C_READ);

	// loop checking 1WB bit for completion of 1-Wire operation
	// abort if poll limit reached
	status = I2C_read(ACK);
	do
	{
		status = I2C_read((status & STATUS_1WB)?ACK:NACK);
	}
	while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));
	#endif
	// check for failure due to poll limit reached
	if (poll_count >= POLL_LIMIT)
	{
		// handle error
		// ...
		DS2482_reset();
		return 0;
	}

	#ifdef HARDWARE_IIC_DRV_SUPPORT
	I2c_WriteRegister(I2C_address, CMD_SRP, 0xE1);
	data = I2C_Data_Receive(I2C_address);
	#else
	I2C_rep_start();
	I2C_write(I2C_address & I2C_WRITE);
	I2C_write(CMD_SRP);
	I2C_write(0xE1);
	I2C_rep_start();
	I2C_write(I2C_address | I2C_READ);
	data =  I2C_read(NACK);
	I2C_stop();
	#endif
	
	return data;
}

//--------------------------------------------------------------------------
// The 'OWBlock' transfers a block of data to and from the
// 1-Wire Net. The result is returned in the same buffer.
//
// 'tran_buf' - pointer to a block of unsigned
//              chars of length 'tran_len' that will be sent
//              to the 1-Wire Net
// 'tran_len' - length in bytes to transfer
//
void OWBlock(u8 *tran_buf, u16 tran_len)
{
   u16 i;

   for (i = 0; i < tran_len; i++)
      tran_buf[i] = OWTouchByte(tran_buf[i]);
}

//--------------------------------------------------------------------------
// Send 8 bits of communication to the 1-Wire Net and return the
// result 8 bits read from the 1-Wire Net. The parameter 'sendbyte'
// least significant 8 bits are used and the least significant 8 bits
// of the result are the return byte.
//
// 'sendbyte' - 8 bits to send (least significant byte)
//
// Returns:  8 bits read from sendbyte
//
u8 OWTouchByte(u8 sendbyte)
{
   if (sendbyte == 0xFF)
      return OWReadByte();
   else
   

{
      OWWriteByte(sendbyte);
      return sendbyte;
   }
}

//--------------------------------------------------------------------------
// Find the 'first' devices on the 1-Wire network
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : no device present
//
u8 OWFirst(void)
{
   // reset the search state
   LastDiscrepancy = 0;
   LastDeviceFlag = FALSE;
   LastFamilyDiscrepancy = 0;

   return OWSearch();
}

//--------------------------------------------------------------------------
// Find the 'next' devices on the 1-Wire network
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
u8 OWNext(void)
{
   // leave the search state alone
   return OWSearch();
}

//--------------------------------------------------------------------------
// The 'OWSearch' function does a general search. This function
// continues from the previous search state. The search state
// can be reset by using the 'OWFirst' function.
// This function contains one parameter 'alarm_only'.
// When 'alarm_only' is TRUE (1) the find alarm command
// 0xEC is sent instead of the normal search command 0xF0.
// Using the find alarm command 0xEC will limit the search to only
// 1-Wire devices that are in an 'alarm' state.
//
// Returns:   TRUE (1) : when a 1-Wire device was found and its
//                       Serial Number placed in the global ROM
//            FALSE (0): when no new device was found.  Either the
//                       last search was the last device or there
//                       are no devices on the 1-Wire Net.
//
u8 OWSearch(void)
{
   u8 crc8;
   u8 ROM_NO[8];
   u8 id_bit_number;
   u8 last_zero, rom_byte_number, search_result;
   u8 id_bit, cmp_id_bit;
   u8 rom_byte_mask, search_direction, status;

   // initialize for search
   crc8 = 0;
   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = FALSE;

   // if the last call was not the last one
   if (!LastDeviceFlag)
   {
      // 1-Wire reset
      if (!OWReset())
      {
         // reset the search
         LastDiscrepancy = 0;
         LastDeviceFlag = FALSE;
         LastFamilyDiscrepancy = 0;
         // Print Out
         NRF_LOG_RAW_INFO("ONEBUS: Search Err\n");
         NRF_LOG_FLUSH();
         return FALSE;
      }

      // issue the search command
      OWWriteByte(0xF0);

      // loop to do the search
      do
      {
         // if this discrepancy if before the Last Discrepancy
         // on a previous next then pick the same as last time
         if (id_bit_number < LastDiscrepancy)
         {
            if ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0)
               search_direction = 1;
            else
               search_direction = 0;
         }
         else
         {
            // if equal to last pick 1, if not then pick 0
            if (id_bit_number == LastDiscrepancy)
               search_direction = 1;
            else
               search_direction = 0;
         }

         // Perform a triple operation on the DS2482 which will perform
         // 2 read bits and 1 write bit
         status = DS2482_search_triplet(search_direction);

         // check bit results in status byte
         id_bit = ((status & STATUS_SBR) == STATUS_SBR);
         cmp_id_bit = ((status & STATUS_TSB) == STATUS_TSB);
         search_direction = ((status & STATUS_DIR) == STATUS_DIR) ? (u8)1 : (u8)0;

         // check for no devices on 1-Wire
         if ((id_bit) && (cmp_id_bit))
            break;
         else
         {
            if ((!id_bit) && (!cmp_id_bit) && (search_direction == 0))
            {
               last_zero = id_bit_number;

               // check for Last discrepancy in family
               if (last_zero < 9)
                  LastFamilyDiscrepancy = last_zero;
            }

            // set or clear the bit in the ROM byte rom_byte_number
            // with mask rom_byte_mask
            if (search_direction == 1)
               ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
               ROM_NO[rom_byte_number] &= (u8)~rom_byte_mask;

            // increment the byte counter id_bit_number
            // and shift the mask rom_byte_mask
            id_bit_number++;
            rom_byte_mask <<= 1;

            // if the mask is 0 then go to new SerialNum byte rom_byte_number
            // and reset mask
            if (rom_byte_mask == 0)
            {
               rom_byte_number++;
               rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7


      crc8 = calc_crc8(ROM_NO, rom_byte_number);  // accumulate the CRC

      // if the search was successful then
      if (!((id_bit_number < 65) || (crc8 != 0)))
      {
         // search successful so set LastDiscrepancy,LastDeviceFlag
         // search_result
         LastDiscrepancy = last_zero;

         // check for last device
         if (LastDiscrepancy == 0)
            LastDeviceFlag = TRUE;

         search_result = TRUE;
      }
   }

   // if no device found then reset counters so next
   // 'search' will be like a first

   if (!search_result || (ROM_NO[0] == 0))
   {
      LastDiscrepancy = 0;
      LastDeviceFlag = FALSE;
      LastFamilyDiscrepancy = 0;
      search_result = FALSE;
   }

   return search_result;
}

//--------------------------------------------------------------------------
// Use the DS2482 help command '1-Wire triplet' to perform one bit of a
//-Wire search.
//This command does two read bits and one write bit. The write bit
// is either the default direction (all device have same bit) or in case of
// a discrepancy, the 'search_direction' parameter is used.
//
// Returns ï¿½C The DS2482 status byte result from the triplet command
//
u8 DS2482_search_triplet(u8 search_direction)
{
	u8 status;
	u8 poll_count = 0;

	// 1-Wire Triplet (Case B)
	//   S AD,0 [A] 1WT [A] SS [A] Sr AD,1 [A] [Status] A [Status] A\ P
	//                                         \--------/
	//                           Repeat until 1WB bit has changed to 0
	//  [] indicates from slave
	//  SS indicates byte containing search direction bit value in msbit

	#ifdef HARDWARE_IIC_DRV_SUPPORT
	I2c_WriteRegister(I2C_address, CMD_1WT, search_direction ? 0x80 : 0x00);

	status = I2C_Data_Receive(I2C_address);
	do
	{
		status = I2C_Data_Receive(I2C_address);
	}
	while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));
	#else
	I2C_start();
	I2C_write(I2C_address & I2C_WRITE);
	I2C_write(CMD_1WT);
	I2C_write(search_direction ? 0x80 : 0x00);
	I2C_rep_start();
	I2C_write(I2C_address | I2C_READ);

	// loop checking 1WB bit for completion of 1-Wire operation
	// abort if poll limit reached
	status = I2C_read(ACK);
	do
	{
		status = I2C_read((status & STATUS_1WB)?ACK:NACK);
	}
	while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));

	I2C_stop();
	#endif

	// check for failure due to poll limit reached
	if (poll_count >= POLL_LIMIT)
	{
		// handle error
		// ...
		DS2482_reset();
		return 0;
	}

	// return status byte
	return status;
}

//--------------------------------------------------------------------------
// Set the 1-Wire Net communication speed.
//
// 'new_speed' - new speed defined as
//                MODE_STANDARD   0x00
//                MODE_OVERDRIVE  0x01
//
// Returns:  current 1-Wire Net speed
//
u8 OWSpeed(u8 new_speed)
{
   // set the speed
   if (new_speed == MODE_OVERDRIVE)
      c1WS = CONFIG_1WS;
   else
      c1WS = FALSE;

   // write the new config
   DS2482_write_config(c1WS | cSPU | cPPM | cAPU);

   return new_speed;
}

//--------------------------------------------------------------------------
// Set the 1-Wire Net line level pullup to normal. The DS2482 only
// allows enabling strong pullup on a bit or byte event. Consequently this
// function only allows the MODE_STANDARD argument. To enable strong pullup
// use OWWriteBytePower or OWReadBitPower.
//
// 'new_level' - new level defined as
//                MODE_STANDARD     0x00
//
// Returns:  current 1-Wire Net level
//
u8 OWLevel(u8 new_level)
{
   // function only will turn back to non-strong pullup
   if (new_level != MODE_STANDARD)
      return MODE_STRONG;

   // clear the strong pullup bit in the global config state
   cSPU = FALSE;

   // write the new config
   DS2482_write_config(c1WS | cSPU | cPPM | cAPU);

   return MODE_STANDARD;
}

//--------------------------------------------------------------------------
// Send 1 bit of communication to the 1-Wire Net and verify that the
// response matches the 'applyPowerResponse' bit and apply power delivery
// to the 1-Wire net.  Note that some implementations may apply the power
// first and then turn it off if the response is incorrect.
//
// 'applyPowerResponse' - 1 bit response to check, if correct then start
//                        power delivery
//
// Returns:  TRUE: bit written and response correct, strong pullup now on
//           FALSE: response incorrect
//
u8 OWReadBitPower(u8 applyPowerResponse)
{
   u8 rdbit;

   // set strong pullup enable
   cSPU = CONFIG_SPU;

   // write the new config
   if (!DS2482_write_config(c1WS | cSPU | cPPM | cAPU))
   {
      // Print Out
      NRF_LOG_RAW_INFO("ONEBUS: Rd bit Err\n");
      NRF_LOG_FLUSH();
      return FALSE;
   }

   // perform read bit
   rdbit = OWReadBit();

   // check if response was correct, if not then turn off strong pullup
   if (rdbit != applyPowerResponse)
   {
      OWLevel(MODE_STANDARD);
      // Print Out
      NRF_LOG_RAW_INFO("ONEBUS: Response Err\n");
      NRF_LOG_FLUSH();
      return FALSE;
   }

   return TRUE;
}

//--------------------------------------------------------------------------
// Send 8 bits of communication to the 1-Wire Net and verify that the
// 8 bits read from the 1-Wire Net are the same (write operation).
// The parameter 'sendbyte' least significant 8 bits are used. After the
// 8 bits are sent change the level of the 1-Wire net.
//
// 'sendbyte' - 8 bits to send (least significant bit)
//
// Returns:  TRUE: bytes written and echo was the same, strong pullup now on
//           FALSE: echo was not the same
//
u8 OWWriteBytePower(u8 sendbyte)
{
   // set strong pullup enable
   cSPU = CONFIG_SPU;

   // write the new config
   if (!DS2482_write_config(c1WS | cSPU | cPPM | cAPU))
   {
      // Print Out
      NRF_LOG_RAW_INFO("ONEBUS: Wr bit Err\n");
      NRF_LOG_FLUSH();
      return FALSE;
   }

   // perform write byte
   OWWriteByte(sendbyte);

   return TRUE;
}

//return 0: 1 wire not busy   
//return 1: 1 wire busy   
u8 Check_Busy(void)
{
	u8 temp;

	I2c_WriteRegister(DS2482, CMD_SRP, 0xF0);
	temp = I2C_ReadRegister(DS2482, CMD_SRP);
		
	// Check 1 wire busy bit
	return(temp&0x01);   
}   
   
//Entrance parameters:Get DS18S20 8Byte ID
char* Get_1Wire_ID(void)
{   
	u8 idx;   
	u8 errtime = POLL_LIMIT;
	u8 idata[10] = "UNKNOWN";
	// char *idstring = (char*)idata;
	
	OWReset();
	OWdelay(10);
	while(Check_Busy()) // Check 1 wire devcie busy status
	{
		errtime--;
		if(!errtime) 
		{
			break;
		}
	}

	OWWriteByte(READ_ROM);

	memset((void*)idata,0,sizeof(idata));
	for(idx =0; idx < 8; idx++)
	{   
		idata[idx] = OWReadByte();
	}   
	// Print Out
	NRF_LOG_RAW_INFO("ONEBUS: TempID %s\n", idata);
    NRF_LOG_FLUSH();

	return NULL;
}   
   
//Entrance parameters: number(>=1) ds18s20 on the bus
u16 Read_Temperature(void)
{
    static uint8_t	 sensor = 0;
	u8 errtime = POLL_LIMIT;
	u8 templow;
	u8 temphigh;

    if (get1WSensorFamily(0) == CHAIN_FAMILY) {
        if (OWReset()) {
            OWChainOn();  // Ensure we start with first device
        }
    }

    for (sensor = 0; sensor < MAX_TEMP_SENSORS; sensor++) {
        OWReset();
        OWdelay(20);

        OWWriteByte(SKIP_ROM);    //Skip ROM
        while(Check_Busy())
        {
            errtime--;
            if(!errtime) 
            {
                break;
            }
        }

        OWWriteByte(START_TEMP_CONVERSION); //Start all temperature conversion 
        while(Check_Busy())
        {
            errtime--;
            if(!errtime) 
            {
                break;
            }
        }

        OWdelay(100); // Waiting for conversion to complete

        OWReset();
        OWdelay(20);

        OWWriteByte(READ_SCRATCHPAD); // Read temperature
        while(Check_Busy())
        {
            errtime--;
            if(!errtime) 
            {
                break;
            }
        }
        OWdelay(5);   

        // Read two byte temperature
        //Low
        templow = OWReadByte();
        //HIGH
        temphigh = OWReadByte();

        OWReset();   
#ifdef SUPPORTS_CHAIN_PROTOCOL
        store1WSensorTemp(sensor, monet_data.Temp10);
        OWFullChainDone(); // Send done to enable next in the chain
#endif
    }
	return(temphigh * 256 + templow);   
}   
   
   
// show temperature string 
void Show_Temperature(void)
{   
	u8 String[9] = {'\0'};
	u16 temperature;   

	temperature= Read_Temperature();   
	if(temperature>0x8000)   
	{   
		temperature=65535-temperature+1;   
		String[0]='-';   
	}   
	else
	{
		String[0]='+'; 
	}

	if(temperature&0x0001)
	{
		String[5]='5';//Half a degree Celsius 
	}
	else
	{
		String[5]='0';
	}
		
	temperature=(temperature&0xff)/2;   
	String[1]=(temperature/100)+0x30;   
	String[2]=((temperature/10)%10)+0x30;   
	String[3]=(temperature%10)+0x30;   
	String[4]='.';   
	String[6]=' ';   
	String[7]='C';   
	String[8]='\n';   

	// Print Out
	NRF_LOG_RAW_INFO("ONEBUS: Temp %s\n", String);
    NRF_LOG_FLUSH();
}

/*******************************************************************************
                        Copyrights (C) Asiatelco Technologies Co., 2003-2016. All rights reserved
                                                             End Of The File
*******************************************************************************/
