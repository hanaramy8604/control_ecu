#include"ultrasonic.h"
#include "micro_config.h"
#include "std_types.h"

uint8 edge_count=0;
uint16 time_value=0;
uint16 ultrasonic_distance=0;



void ultrasonic_init(void){
	//Initialization of the ultra-sonic sensor
	ICU_ConfigType config={F_CPU_8,RAISING}; //configure the ICU based on what we need
	ICU_setCallBack(ultrasonic_edgeProcessing); //defining the call-back function
	ICU_init(&config);//initialize the ICU
	GPIO_setupPinDirection(trigger_PORTID,trigger_PINID,PIN_OUTPUT); //define the trigger as output pin
	GPIO_writePin(trigger_PORTID,trigger_PINID,LOGIC_LOW);
}
void ultrasonic_edgeProcessing(void){
	/*Function mainly used to determine the edge
	 * calculate the time between raising and falling edge
	 */

	edge_count++;
		if(edge_count == 1)
		{
			/*Clear the timer counter register to start measurements from the
			 * first detected rising edge
			 */
			ICU_clearTimerValue();
			/* Detect falling edge */
			ICU_setEdgeDetectionType(FALLING);
		}
		else if(edge_count == 2)
			{
				/* Store the High time value */
				time_value= ICU_getInputCaptureValue();
				/* Detect rising edge */
				ICU_setEdgeDetectionType(RAISING);
			     edge_count=0;
			}
}
void ultrasonic_trigger(void){
	// initiate the trigger to send pulses
	GPIO_writePin(trigger_PORTID,trigger_PINID,LOGIC_HIGH);
    _delay_us(20);
    GPIO_writePin(trigger_PORTID,trigger_PINID,LOGIC_LOW);
}
uint16 ultrasonic_readDistance(void){
	//calculate the distance of the sensor
	ultrasonic_trigger();
ultrasonic_distance= (uint16)(time_value/TIME_CONSTANT);
return ultrasonic_distance;
}


