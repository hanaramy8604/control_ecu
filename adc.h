/******************************************************************************
*
* Module: ADC
*
* File Name: adc.h
*
* Description: header file for the ATmega16 ADC driver
*
* Author: Hana Ramy
*
*******************************************************************************/

#ifndef ADC_H_
#define ADC_H_

#include "std_types.h"

/*******************************************************************************
* Definitions *
*******************************************************************************/
#define ADC_MAXIMUM_VALUE 1023
#define ADC_REF_VOLT_VALUE 5



/*******************************************************************************
 *                         Types Declaration                                   *
 *******************************************************************************/

typedef enum
{
    ADC_AREF,              /* External AREF, Internal Vref turned off */
    ADC_AVCC,              /* AVCC with external capacitor at AREF pin */
    ADC_INTERNAL_2_56V     /* Internal 2.56V Voltage Reference */
} ADC_ReferenceVoltage;

typedef enum
{
    ADC_PRESCALER_2 = 1,
    ADC_PRESCALER_4 = 2,
    ADC_PRESCALER_8 = 3,
    ADC_PRESCALER_16 = 4,
    ADC_PRESCALER_32 = 5,
    ADC_PRESCALER_64 = 6,
    ADC_PRESCALER_128 = 7
} ADC_Prescaler;

typedef struct
{
    ADC_ReferenceVoltage ref_volt;
    ADC_Prescaler prescaler;
} ADC_ConfigType;



/*******************************************************************************
* Functions Prototypes *
*******************************************************************************/



/*
* Description :
* Function responsible for initialize the ADC driver.
*/
void ADC_init(const ADC_ConfigType * Config_Ptr);


/*
* Description :
* Function responsible for read analog data from a certain ADC channel
* and convert it to digital using the ADC driver.
*/
uint16 ADC_readChannel(uint8 channel_num);

#endif /* ADC_H_ */
