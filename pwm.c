// pwm.c
#include "pwm.h"
#include "gpio.h"
#include "micro_config.h"

void PWM_Timer0_Start(uint8 duty_cycle) {
    // Initialize Timer Counter to 0
    TCNT0 = 0;

    // Convert duty cycle percentage (0-100) to compare value (0-255)
    OCR0 = (uint8)((duty_cycle * 255UL) / 100);

    // Configure OC0 (PB3) as output pin
    GPIO_setupPinDirection(PORTB_ID, PIN3_ID, PIN_OUTPUT);

    // Configure Timer0:
    // WGM00=1, WGM01=1 -> Fast PWM mode
    // COM01=1, COM00=0 -> Non-inverting mode (Clear OC0 on compare match)
    // CS01=1, CS00=1 -> Prescaler F_CPU/64
    TCCR0 = (1 << WGM00) | (1 << WGM01) | (1 << COM01) | (1 << CS01) | (1 << CS00);
}
