// external_eeprom.c
#include "external_eeprom.h"
#include "i2c.h"
#include <util/delay.h>

/*******************************************************************************
 *                              Preprocessor Macros                             *
 *******************************************************************************/
#define EEPROM_WRITE_CYCLE_TIME_MS  5   // Typical EEPROM write cycle time

/*******************************************************************************
 *                          Functions Definitions                               *
 *******************************************************************************/

void EEPROM_init(void) {
    /* Configure I2C with proper structure members */
    TWI_ConfigType twi_config = {
        .address = 0x01,              // MC address if it acts as slave (not critical for master-only)
        .bit_rate = TWI_BITRATE_400K  // 400 kHz Fast Mode
    };

    /* Initialize I2C module */
    TWI_init(&twi_config);
}

uint8 EEPROM_writeByte(uint16 u16addr, uint8 u8data) {
    /* Send the Start Bit */
    TWI_start();
    if (TWI_getStatus() != TW_START)
        return ERROR;

    /*
     * Send the device address with A10-A8 bits from memory address
     * EEPROM device address: 1010 A10 A9 A8 R/W
     * For write operation: R/W = 0
     */
    TWI_write((uint8)(0xA0 | ((u16addr & 0x0700) >> 7)));
    if (TWI_getStatus() != TW_MT_SLA_W_ACK)
        return ERROR;

    /* Send the required memory location address (lower 8 bits) */
    TWI_write((uint8)(u16addr));
    if (TWI_getStatus() != TW_MT_DATA_ACK)
        return ERROR;

    /* Write byte to EEPROM */
    TWI_write(u8data);
    if (TWI_getStatus() != TW_MT_DATA_ACK)
        return ERROR;

    /* Send the Stop Bit */
    TWI_stop();

    /*
     * Wait for EEPROM write cycle to complete
     * Typical write cycle time is 5ms for most EEPROMs
     */
    _delay_ms(EEPROM_WRITE_CYCLE_TIME_MS);

    return SUCCESS;
}

uint8 EEPROM_readByte(uint16 u16addr, uint8 *u8data) {
    /* Send the Start Bit */
    TWI_start();
    if (TWI_getStatus() != TW_START)
        return ERROR;

    /*
     * Send the device address with A10-A8 bits from memory address
     * For write operation (to set address pointer): R/W = 0
     */
    TWI_write((uint8)(0xA0 | ((u16addr & 0x0700) >> 7)));
    if (TWI_getStatus() != TW_MT_SLA_W_ACK)
        return ERROR;

    /* Send the required memory location address (lower 8 bits) */
    TWI_write((uint8)(u16addr));
    if (TWI_getStatus() != TW_MT_DATA_ACK)
        return ERROR;

    /* Send the Repeated Start Bit */
    TWI_start();
    if (TWI_getStatus() != TW_REP_START)
        return ERROR;

    /*
     * Send the device address with A10-A8 bits from memory address
     * For read operation: R/W = 1
     */
    TWI_write((uint8)(0xA0 | ((u16addr & 0x0700) >> 7) | 0x01));
    if (TWI_getStatus() != TW_MT_SLA_R_ACK)
        return ERROR;

    /* Read byte from memory without sending ACK (last byte) */
    *u8data = TWI_readWithNACK();
    if (TWI_getStatus() != TW_MR_DATA_NACK)
        return ERROR;

    /* Send the Stop Bit */
    TWI_stop();

    return SUCCESS;
}
