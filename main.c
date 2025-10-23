/******************************************************************************
 *
 * File Name: main.c (Control ECU - FINAL FIX)
 *
 * Description: Vehicle Fault Detection and Logging System - Control ECU
 *              FINAL: Fixed ultrasonic lag and window button operation
 *
 * Author: VFDLS Team
 *
 *******************************************************************************/

#include "micro_config.h"
#include "std_types.h"
#include "gpio.h"
#include "uart.h"
#include "adc.h"
#include "lm35_sensor.h"
#include "ultrasonic.h"
#include "pwm.h"
#include "external_eeprom.h"

/*******************************************************************************
 *                              Definitions                                    *
 *******************************************************************************/

/* UART Commands from HMI ECU */
#define CMD_START_MONITORING       0x01
#define CMD_SEND_VALUES           0x02
#define CMD_SEND_FAULTS           0x03
#define CMD_STOP_MONITORING       0x04

/* Acknowledgment bytes */
#define ACK                       0xAA
#define READY                     0x55

/* Fault Detection Thresholds */
#define TEMP_THRESHOLD            90
#define DISTANCE_THRESHOLD        10

/* DTC Codes */
#define DTC_DISTANCE_P001         0x01
#define DTC_TEMPERATURE_P002      0x02

/* Window Button Pins - FINAL CORRECTED */
#define WINDOW1_OPEN_PORT         PORTA_ID
#define WINDOW1_OPEN_PIN          PIN3_ID
#define WINDOW1_CLOSE_PORT        PORTA_ID
#define WINDOW1_CLOSE_PIN         PIN4_ID

#define WINDOW2_OPEN_PORT         PORTA_ID
#define WINDOW2_OPEN_PIN          PIN5_ID
#define WINDOW2_CLOSE_PORT        PORTA_ID
#define WINDOW2_CLOSE_PIN         PIN6_ID

/* Window Motor Pins */
#define WINDOW1_MOTOR_PORT        PORTB_ID
#define WINDOW1_MOTOR_IN1         PIN0_ID
#define WINDOW1_MOTOR_IN2         PIN1_ID

#define WINDOW2_MOTOR_PORT        PORTD_ID
#define WINDOW2_MOTOR_IN1         PIN3_ID
#define WINDOW2_MOTOR_IN2         PIN4_ID

/* EEPROM */
#define EEPROM_FAULT_COUNT_ADDR   0x0000
#define EEPROM_FAULT_START_ADDR   0x0001
#define EEPROM_MAX_FAULTS         100

/* Button States */
#define BUTTON_PRESSED            LOGIC_LOW
#define BUTTON_RELEASED           LOGIC_HIGH

/*******************************************************************************
 *                           Global Variables                                  *
 *******************************************************************************/

volatile uint8 g_monitoringActive = 0;
volatile uint8 g_temperature = 0;
volatile uint16 g_distance = 0;
volatile uint8 g_window1State = 0;
volatile uint8 g_window2State = 0;
volatile uint8 g_distanceFaultLogged = 0;
volatile uint8 g_temperatureFaultLogged = 0;
volatile uint8 g_faultCount = 0;

/*******************************************************************************
 *                        Functions Prototypes                                 *
 *******************************************************************************/

void System_Init(void);
void UART_Init_Config(void);
void ADC_Init_Config(void);
void ReadSensors(void);
void DetectFaults(void);
void LogFaultToEEPROM(uint8 dtc);
void HandleWindowControl(void);
void SendSensorData(void);
void SendFaultCodes(void);
void LoadFaultCount(void);

/*******************************************************************************
 *                            Main Function                                    *
 *******************************************************************************/

int main(void)
{
    uint8 command;

    /* Initialize system */
    System_Init();

    /* Load fault count */
    LoadFaultCount();

    /* Enable global interrupts for ICU */
    sei();

    /* Stabilization delay */
    _delay_ms(100);

    while(1)
    {
        /* CRITICAL FIX: Handle window control continuously in background
         * This allows windows to work regardless of menu state */
        HandleWindowControl();

        /* Check for UART command (non-blocking check) */
        if(BIT_IS_SET(UCSRA, RXC))  /* Data available */
        {
            command = UART_recieveByte();

            switch(command)
            {
                case CMD_START_MONITORING:
                    UART_sendByte(ACK);
                    g_monitoringActive = 1;
                    g_distanceFaultLogged = 0;
                    g_temperatureFaultLogged = 0;
                    break;

                case CMD_SEND_VALUES:
                    /* Read fresh sensor values */
                    ReadSensors();
                    /* Send data immediately */
                    SendSensorData();
                    break;

                case CMD_SEND_FAULTS:
                    SendFaultCodes();
                    break;

                case CMD_STOP_MONITORING:
                    UART_sendByte(ACK);
                    g_monitoringActive = 0;
                    break;

                default:
                    break;
            }
        }

        /* If monitoring is active, check sensors and faults */
        if(g_monitoringActive)
        {
            ReadSensors();
            DetectFaults();
            _delay_ms(100);  /* 100ms cycle time */
        }
        else
        {
            _delay_ms(50);  /* Shorter delay when not monitoring */
        }
    }

    return 0;
}

/*******************************************************************************
 *                        Functions Definitions                                *
 *******************************************************************************/

void System_Init(void)
{
    /* Initialize UART */
    UART_Init_Config();

    /* Initialize ADC */
    ADC_Init_Config();

    /* Initialize Ultrasonic (includes ICU) */
    ultrasonic_init();

    /* Initialize EEPROM */
    EEPROM_init();

    /* Initialize PWM at 100% duty cycle for motors */
    PWM_Timer0_Start(100);

    /* CRITICAL: Configure window button pins as INPUT with internal pull-up */
    /* Window 1 buttons */
    GPIO_setupPinDirection(WINDOW1_OPEN_PORT, WINDOW1_OPEN_PIN, PIN_INPUT);


    GPIO_setupPinDirection(WINDOW1_CLOSE_PORT, WINDOW1_CLOSE_PIN, PIN_INPUT);


    /* Window 2 buttons */
    GPIO_setupPinDirection(WINDOW2_OPEN_PORT, WINDOW2_OPEN_PIN, PIN_INPUT);


    GPIO_setupPinDirection(WINDOW2_CLOSE_PORT, WINDOW2_CLOSE_PIN, PIN_INPUT);


    /* Configure motor control pins as OUTPUT */
    GPIO_setupPinDirection(WINDOW1_MOTOR_PORT, WINDOW1_MOTOR_IN1, PIN_OUTPUT);
    GPIO_setupPinDirection(WINDOW1_MOTOR_PORT, WINDOW1_MOTOR_IN2, PIN_OUTPUT);
    GPIO_setupPinDirection(WINDOW2_MOTOR_PORT, WINDOW2_MOTOR_IN1, PIN_OUTPUT);
    GPIO_setupPinDirection(WINDOW2_MOTOR_PORT, WINDOW2_MOTOR_IN2, PIN_OUTPUT);

    /* Initialize motors to STOP state */
    GPIO_writePin(WINDOW1_MOTOR_PORT, WINDOW1_MOTOR_IN1, LOGIC_LOW);
    GPIO_writePin(WINDOW1_MOTOR_PORT, WINDOW1_MOTOR_IN2, LOGIC_LOW);
    GPIO_writePin(WINDOW2_MOTOR_PORT, WINDOW2_MOTOR_IN1, LOGIC_LOW);
    GPIO_writePin(WINDOW2_MOTOR_PORT, WINDOW2_MOTOR_IN2, LOGIC_LOW);
}

void UART_Init_Config(void)
{
    UART_ConfigType uart_config = {
        .bit_data = UART_8_BIT,
        .parity = UART_NO_PARITY,
        .stop_bit = UART_1_STOP_BIT,
        .baud_rate = 9600
    };

    UART_init(&uart_config);
}

void ADC_Init_Config(void)
{
    ADC_ConfigType adc_config = {
        .ref_volt = ADC_AVCC,
        .prescaler = ADC_PRESCALER_64
    };

    ADC_init(&adc_config);
}

void ReadSensors(void)
{
    /* Read temperature */
    g_temperature = LM35_getTemperature();

    /* Read distance - this now includes built-in delay */
    g_distance = ultrasonic_readDistance();
}

void DetectFaults(void)
{
    /* Check distance fault */
    if(g_distance < DISTANCE_THRESHOLD)
    {
        if(!g_distanceFaultLogged)
        {
            LogFaultToEEPROM(DTC_DISTANCE_P001);
            g_distanceFaultLogged = 1;
        }
    }
    else
    {
        g_distanceFaultLogged = 0;
    }

    /* Check temperature fault */
    if(g_temperature > TEMP_THRESHOLD)
    {
        if(!g_temperatureFaultLogged)
        {
            LogFaultToEEPROM(DTC_TEMPERATURE_P002);
            g_temperatureFaultLogged = 1;
        }
    }
    else
    {
        g_temperatureFaultLogged = 0;
    }
}

void LogFaultToEEPROM(uint8 dtc)
{
    uint16 write_address;

    if(g_faultCount >= EEPROM_MAX_FAULTS)
    {
        return;
    }

    write_address = EEPROM_FAULT_START_ADDR + g_faultCount;
    EEPROM_writeByte(write_address, dtc);

    g_faultCount++;
    EEPROM_writeByte(EEPROM_FAULT_COUNT_ADDR, g_faultCount);
}

void HandleWindowControl(void)
{
    /* Local variables to store button states */
    uint8 win1_open_btn, win1_close_btn;
    uint8 win2_open_btn, win2_close_btn;

    /* Read all button states */
    win1_open_btn = GPIO_readPin(WINDOW1_OPEN_PORT, WINDOW1_OPEN_PIN);
    win1_close_btn = GPIO_readPin(WINDOW1_CLOSE_PORT, WINDOW1_CLOSE_PIN);
    win2_open_btn = GPIO_readPin(WINDOW2_OPEN_PORT, WINDOW2_OPEN_PIN);
    win2_close_btn = GPIO_readPin(WINDOW2_CLOSE_PORT, WINDOW2_CLOSE_PIN);

    /* === WINDOW 1 CONTROL === */
    if(win1_open_btn == BUTTON_PRESSED && win1_close_btn == BUTTON_RELEASED)
    {
        /* Only OPEN button pressed - Rotate CCW (Open direction) */
        GPIO_writePin(WINDOW1_MOTOR_PORT, WINDOW1_MOTOR_IN1, LOGIC_LOW);
        GPIO_writePin(WINDOW1_MOTOR_PORT, WINDOW1_MOTOR_IN2, LOGIC_HIGH);
        g_window1State = 1;  /* Open */
    }
    else if(win1_close_btn == BUTTON_PRESSED && win1_open_btn == BUTTON_RELEASED)
    {
        /* Only CLOSE button pressed - Rotate CW (Close direction) */
        GPIO_writePin(WINDOW1_MOTOR_PORT, WINDOW1_MOTOR_IN1, LOGIC_HIGH);
        GPIO_writePin(WINDOW1_MOTOR_PORT, WINDOW1_MOTOR_IN2, LOGIC_LOW);
        g_window1State = 0;  /* Closed */
    }
    else
    {
        /* No button pressed OR both pressed - STOP motor */
        GPIO_writePin(WINDOW1_MOTOR_PORT, WINDOW1_MOTOR_IN1, LOGIC_LOW);
        GPIO_writePin(WINDOW1_MOTOR_PORT, WINDOW1_MOTOR_IN2, LOGIC_LOW);
    }

    /* === WINDOW 2 CONTROL === */
    if(win2_open_btn == BUTTON_PRESSED && win2_close_btn == BUTTON_RELEASED)
    {
        /* Only OPEN button pressed - Rotate CCW */
        GPIO_writePin(WINDOW2_MOTOR_PORT, WINDOW2_MOTOR_IN1, LOGIC_LOW);
        GPIO_writePin(WINDOW2_MOTOR_PORT, WINDOW2_MOTOR_IN2, LOGIC_HIGH);
        g_window2State = 1;  /* Open */
    }
    else if(win2_close_btn == BUTTON_PRESSED && win2_open_btn == BUTTON_RELEASED)
    {
        /* Only CLOSE button pressed - Rotate CW */
        GPIO_writePin(WINDOW2_MOTOR_PORT, WINDOW2_MOTOR_IN1, LOGIC_HIGH);
        GPIO_writePin(WINDOW2_MOTOR_PORT, WINDOW2_MOTOR_IN2, LOGIC_LOW);
        g_window2State = 0;  /* Closed */
    }
    else
    {
        /* No button pressed OR both pressed - STOP motor */
        GPIO_writePin(WINDOW2_MOTOR_PORT, WINDOW2_MOTOR_IN1, LOGIC_LOW);
        GPIO_writePin(WINDOW2_MOTOR_PORT, WINDOW2_MOTOR_IN2, LOGIC_LOW);
    }
}

void SendSensorData(void)
{
    /* Send ready signal */
    UART_sendByte(READY);

    /* Send temperature */
    UART_sendByte(g_temperature);

    /* Send distance (high byte, then low byte) */
    UART_sendByte((uint8)(g_distance >> 8));
    UART_sendByte((uint8)(g_distance & 0xFF));

    /* Send window states */
    UART_sendByte(g_window1State);
    UART_sendByte(g_window2State);
}

void SendFaultCodes(void)
{
    uint8 i;
    uint8 fault_code;
    uint16 read_address;

    /* Send ready signal */
    UART_sendByte(READY);

    /* Send fault count */
    UART_sendByte(g_faultCount);

    /* Send each fault code */
    for(i = 0; i < g_faultCount; i++)
    {
        read_address = EEPROM_FAULT_START_ADDR + i;
        EEPROM_readByte(read_address, &fault_code);
        UART_sendByte(fault_code);
        _delay_ms(10);
    }
}

void LoadFaultCount(void)
{
    uint8 stored_count;

    if(EEPROM_readByte(EEPROM_FAULT_COUNT_ADDR, &stored_count) == SUCCESS)
    {
        if(stored_count <= EEPROM_MAX_FAULTS)
        {
            g_faultCount = stored_count;
        }
        else
        {
            g_faultCount = 0;
            EEPROM_writeByte(EEPROM_FAULT_COUNT_ADDR, 0);
        }
    }
    else
    {
        g_faultCount = 0;
        EEPROM_writeByte(EEPROM_FAULT_COUNT_ADDR, 0);
    }
}
