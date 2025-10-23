// i2c.c
#include "i2c.h"

/*******************************************************************************
 *                          Functions Definitions                               *
 *******************************************************************************/

void TWI_init(const TWI_ConfigType *Config_Ptr) {
    uint8 twbr_value;

    /*
     * Bit Rate calculation:
     * SCL_frequency = (F_CPU) / (16 + 2 * TWBR * Prescaler)
     * For Prescaler = 1 (TWSR = 0):
     * TWBR = ((F_CPU / SCL_frequency) - 16) / 2
     */

    /* Calculate TWBR value with prescaler = 1 */
    if (F_CPU > (Config_Ptr->bit_rate * 16UL)) {
        twbr_value = (uint8)(((F_CPU / Config_Ptr->bit_rate) - 16) / 2);
    } else {
        /* If calculation would be invalid, default to 100kHz */
        twbr_value = (uint8)(((F_CPU / TWI_BITRATE_100K) - 16) / 2);
    }

    /* Set bit rate register */
    TWBR = twbr_value;

    /* Set prescaler to 1 by clearing TWPS1 and TWPS0 bits */
    TWSR = 0x00;

    /*
     * Set TWI slave address (used if this MC acts as a slave)
     * Bit 0 in TWAR is for General Call Recognition: set to 0 (disabled)
     */
    TWAR = (Config_Ptr->address << 1);  // Shift left, bit 0 = 0

    /* Enable TWI module */
    TWCR = (1 << TWEN);
}

void TWI_start(void) {
    /*
     * Clear the TWINT flag before sending the start bit TWINT=1
     * Send the start bit by TWSTA=1
     * Enable TWI Module TWEN=1
     */
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

    /* Wait for TWINT flag set in TWCR Register (start bit sent successfully) */
    while (BIT_IS_CLEAR(TWCR, TWINT));
}

void TWI_stop(void) {
    /*
     * Clear the TWINT flag before sending the stop bit TWINT=1
     * Send the stop bit by TWSTO=1
     * Enable TWI Module TWEN=1
     */
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void TWI_write(uint8 data) {
    /* Put data on TWI data register */
    TWDR = data;

    /*
     * Clear the TWINT flag before sending the data TWINT=1
     * Enable TWI Module TWEN=1
     */
    TWCR = (1 << TWINT) | (1 << TWEN);

    /* Wait for TWINT flag set in TWCR Register (data sent successfully) */
    while (BIT_IS_CLEAR(TWCR, TWINT));
}

uint8 TWI_readWithACK(void) {
    /*
     * Clear the TWINT flag before reading the data TWINT=1
     * Enable sending ACK after reading data TWEA=1
     * Enable TWI Module TWEN=1
     */
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);

    /* Wait for TWINT flag set in TWCR Register (data received successfully) */
    while (BIT_IS_CLEAR(TWCR, TWINT));

    /* Read and return data */
    return TWDR;
}

uint8 TWI_readWithNACK(void) {
    /*
     * Clear the TWINT flag before reading the data TWINT=1
     * Enable TWI Module TWEN=1
     */
    TWCR = (1 << TWINT) | (1 << TWEN);

    /* Wait for TWINT flag set in TWCR Register (data received successfully) */
    while (BIT_IS_CLEAR(TWCR, TWINT));

    /* Read and return data */
    return TWDR;
}

uint8 TWI_getStatus(void) {
    uint8 status;

    /* Mask to eliminate first 3 bits and get the last 5 bits (status bits) */
    status = TWSR & 0xF8;

    return status;
}
