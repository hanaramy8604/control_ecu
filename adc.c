/******************************************************************************
*
* Module: ADC
*
* File Name: adc.c
*
* Description: Source file for the ATmega16 ADC driver
*
* Author: Hana Ramy
*
*******************************************************************************/

#include "micro_config.h" /* To use the ADC Registers */
#include "adc.h"
#include "common_macros.h" /* To use the macros like SET_BIT */


/*******************************************************************************
* Functions Definitions *
*******************************************************************************/


void ADC_init(const ADC_ConfigType * Config_Ptr)
{
    /* ADMUX Register Configuration:
     * REFS1:0  = Configure based on reference voltage
     * ADLAR    = 0 (Right adjusted)
     * MUX4:0   = 00000 (Channel 0 as initial)
     */
    ADMUX = 0;

    /* Configure Reference Voltage (REFS1:0) */
    switch(Config_Ptr->ref_volt)
    {
        case ADC_AREF:
            /* REFS1:0 = 00 - External AREF */
            break;
        case ADC_AVCC:
            /* REFS1:0 = 01 - AVCC with external capacitor */
            ADMUX |= (1<<REFS0);
            break;
        case ADC_INTERNAL_2_56V:
            /* REFS1:0 = 11 - Internal 2.56V */
            ADMUX |= (1<<REFS1) | (1<<REFS0);
            break;
    }

    /* ADCSRA Register Configuration:
     * ADEN     = 1 (Enable ADC)
     * ADIE     = 0 (Disable ADC Interrupt)
     * ADATE    = 0 (Disable Auto Trigger)
     * ADPS2:0  = Configure prescaler
     */
    ADCSRA = (1<<ADEN);
    ADCSRA |= (Config_Ptr->prescaler & 0x07);
}



uint16 ADC_readChannel(uint8 channel_num)
{
channel_num &= 0x07; // Input channel number must be
ADMUX&= 0xE0;
	ADMUX|=channel_num;
	SET_BIT(ADCSRA,ADSC);
	while(BIT_IS_CLEAR(ADCSRA,ADIF)){ // polling to wait until the flag gets cleared and break for the interrupt of ADC
	}
	SET_BIT(ADCSRA,ADIF);


	return ADC; // return the ADC value read from channel we give
}
