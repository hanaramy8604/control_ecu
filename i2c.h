// i2c.h
#ifndef I2C_H_
#define I2C_H_

#include "std_types.h"
#include "common_macros.h"
#include <avr/io.h>
#include <avr/interrupt.h>

/*******************************************************************************
 *                              Preprocessor Macros                             *
 *******************************************************************************/

/* I2C Status Bits in the TWSR Register */
#define TW_START         0x08  // start has been sent
#define TW_REP_START     0x10  // repeated start
#define TW_MT_SLA_W_ACK  0x18  // Master transmit (slave address + Write request) + ACK received
#define TW_MT_SLA_R_ACK  0x40  // Master transmit (slave address + Read request) + ACK received
#define TW_MT_DATA_ACK   0x28  // Master transmit data and ACK received from Slave
#define TW_MR_DATA_ACK   0x50  // Master received data and send ACK to slave
#define TW_MR_DATA_NACK  0x58  // Master received data but doesn't send ACK to slave

/*******************************************************************************
 *                         User Defined Data Types                              *
 *******************************************************************************/

/* Define address type as uint8 */
typedef uint8 TWI_AddressType;

/* Define baud rate type as enum with standard I2C speeds */
typedef enum {
    TWI_BITRATE_100K = 100000UL,   // Standard Mode: 100 kbit/s
    TWI_BITRATE_400K = 400000UL,   // Fast Mode: 400 kbit/s
    TWI_BITRATE_1M   = 1000000UL,  // Fast Mode Plus: 1 Mbit/s
    TWI_BITRATE_3_4M = 3400000UL   // High Speed Mode: 3.4 Mbit/s
} TWI_BaudRateType;

/* TWI Configuration Structure */
typedef struct {
    TWI_AddressType address;       // Slave address (if MC acts as slave)
    TWI_BaudRateType bit_rate;     // SCL frequency (bit rate)
} TWI_ConfigType;

/*******************************************************************************
 *                            Functions Prototypes                              *
 *******************************************************************************/

void TWI_init(const TWI_ConfigType *Config_Ptr);
void TWI_start(void);
void TWI_stop(void);
void TWI_write(uint8 data);
uint8 TWI_readWithACK(void);
uint8 TWI_readWithNACK(void);
uint8 TWI_getStatus(void);

#endif /* I2C_H_ */
