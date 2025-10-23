// motor.c
#include "gpio.h"
#include "motor.h"
#include "pwm.h"

void motor_init(void) {
    // Configure direction control pins as output
    GPIO_setupPinDirection(motor_IN1_PORTID, motor_IN1_PINID, PIN_OUTPUT);
    GPIO_setupPinDirection(motor_IN2_PORTID, motor_IN2_PINID, PIN_OUTPUT);

    // Stop motor initially
    GPIO_writePin(motor_IN1_PORTID, motor_IN1_PINID, LOGIC_LOW);
    GPIO_writePin(motor_IN2_PORTID, motor_IN2_PINID, LOGIC_LOW);

    // Start PWM at 100% duty cycle (maximum speed as required)
    PWM_Timer0_Start(100);
}

void motor_rotate(motor_state status, uint8 speed) {
    // Update PWM duty cycle
    PWM_Timer0_Start(speed);

    // Set motor direction
    switch (status) {
        case CW:
            GPIO_writePin(motor_IN1_PORTID, motor_IN1_PINID, LOGIC_HIGH);
            GPIO_writePin(motor_IN2_PORTID, motor_IN2_PINID, LOGIC_LOW);
            break;
        case ACW:
            GPIO_writePin(motor_IN1_PORTID, motor_IN1_PINID, LOGIC_LOW);
            GPIO_writePin(motor_IN2_PORTID, motor_IN2_PINID, LOGIC_HIGH);
            break;
        case STOP:
        default:
            GPIO_writePin(motor_IN1_PORTID, motor_IN1_PINID, LOGIC_LOW);
            GPIO_writePin(motor_IN2_PORTID, motor_IN2_PINID, LOGIC_LOW);
            break;
    }
}
