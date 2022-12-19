/*******************************************************************************
* File Name          : onebus.c
* Author               : Jevon
* Description        : This file provides all the obd functions.

* History:
*  02/23/2017 : onebus V1.00
*******************************************************************************/

#ifndef __ONEBUS_H
#define __ONEBUS_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#define TEMP_NO_THRESHOLD	0x8000
#define TEMP_NO_SENSOR		0x8001
#define TEMP_NO_AVAILABLE	0x8002

#define SUPPORTS_CHAIN_PROTOCOL 1

// OneWire Commands
#define SKIP_ROM              (0xCC)
#define READ_ROM              (0x33)
#define CONDITIONAL_READ_ROM  (0x0F)
#define MATCH_ROM             (0x55)
#define READ_SCRATCHPAD       (0xBE)
#define WRITE_SCRATCHPAD      (0x4E)
#define COPY_SCRATCHPAD       (0x48)
#define START_TEMP_CONVERSION (0x44)

#ifdef SUPPORTS_CHAIN_PROTOCOL
#define CHAIN_COMMAND         (0x99)
#define CHAIN_OFF             (0x3C)
#define CHAIN_ON              (0x5A)
#define CHAIN_DONE            (0x96)

#define CHAIN_ACK             (0xAA)
#define CHAIN_FAMILY          (0x42)
#define MAX_TEMP_SENSORS (2)

uint8_t     get1WSensorFamily(uint8_t index);
void        set1WSensorFamily(uint8_t index, uint8_t family);
int16_t     get1WSensorTemp(uint8_t index);
void        store1WSensorTemp(uint8_t index, int16_t Temp10);
int16_t     get1WSensorTempLow(uint8_t index);
void        store1WSensorTempLow(uint8_t index, int16_t Temp10);
int16_t     get1WSensorTempHigh(uint8_t index);
void        store1WSensorTempHigh(uint8_t index, int16_t Temp10);
uint8_t     get1WSensorStatus(uint8_t index);
void        set1WSensorStatus(uint8_t index, uint8_t TempStatus);
uint64_t    get1WSensorRegId(uint8_t index);
void        OWDiscovery(void);
void        oneWire_oneSecond(void);
void        OWInit(void);
#endif // SUPPORTS_CHAIN_PROTOCOL

enum {
	TEMP_NORMAL,
	TEMP_LOW,
	TEMP_HIGH
};

// SCL - PB10
// SDA - PB11
#define I2C_READ  			0x01
#define I2C_WRITE 		0xFE

#define HARDWARE_IIC_DRV_SUPPORT (1)

#ifdef HARDWARE_IIC_DRV_SUPPORT
#define I2C_OP_TIMEOUT 	1000
#endif

// Variable Declared 

#define u8 uint8_t
#define u16 uint16_t
#define uint8 uint8_t
#define u32 uint32_t

#ifndef DISABLE
#define DISABLE 1
#endif

#ifndef WAKE
#define WAKE 1 // NALAMCU-31
#endif

#ifndef SLEEP
#define SLEEP 0 // NALAMCU-31
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0//1
#endif

typedef enum{
  ACK 	= 1,
  NACK 	= 2,
  NOACK 	= 3,
}etAck;

//Function Declares
extern void OB_I2C_Init(void);
extern void I2C_start(void);
extern void I2C_rep_start(void);
extern void I2C_stop(void);
extern void I2C_WriteACK(etAck ack);
extern u8 I2C_WaitACK(void);
extern u8 I2C_write(u8 wdata);
extern u8 I2C_read(etAck ack);
extern void I2c_WriteRegister(uint8 address, uint8 reg,uint8 val);
extern u8  I2C_ReadRegister(uint8 address, uint8 reg);
extern void I2C_ReadSerialData(u8 address, uint8 reg,u8 count,u8 * buff);
extern void OneBusSleepControl(uint8_t Status);
extern void OWdelay(u32 Delay);
extern void OneBusInit(void);
extern u8 DS2482_detect(void);
extern u8 DS2482_reset(void);
extern u8 DS2482_write_config(u8 config);
extern u8 DS2482_channel_select(u8 channel);
extern u8 OWReset(void);
extern void OWWriteBit(u8 sendbit);
extern u8 OWReadBit(void);
extern u8 OWTouchBit(u8 sendbit);
extern void OWWriteByte(u8 sendbyte);
extern u8 OWReadByte(void);
extern void OWBlock(u8 *tran_buf, u16 tran_len);
extern u8 OWTouchByte(u8 sendbyte);
extern u8 OWFirst(void);
extern u8 OWNext(void);
extern u8 OWSearch(void);
extern u8 DS2482_search_triplet(u8 search_direction);
extern u8 OWSpeed(u8 new_speed);
extern u8 OWLevel(u8 new_level);
extern u8 OWReadBitPower(u8 applyPowerResponse);
extern u8 OWWriteBytePower(u8 sendbyte);
extern u8 Check_Busy(void);
extern char* Get_1Wire_ID(void);
extern u16 Read_Temperature(void);
extern void Show_Temperature(void);

#ifdef __cplusplus
}
#endif
#endif /*__POWERCTL_H */

/*******************************************************************************
End Of The File
*******************************************************************************/

