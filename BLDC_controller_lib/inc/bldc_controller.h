/**
 * @file bldc_controller.h
 * @brief Header file for BLDC motor controller functionality.
 *
 * This file defines the main structures, constants, and function prototypes
 * for controlling a BLDC motor. It includes platform-specific includes and
 * provides a uniform interface for motor control across different hardware.
 */


#ifndef BLDC_CONTROLLER_H
#define BLDC_CONTROLLER_H

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "bldc_pinout.h"


/***************************************
 * Firmware includes based on the selected hardware platform.
 ***************************************/

#ifdef STM32_HAL

    #include "main.h"
    #include "gpio.h"
    #include "adc.h"
    #include "tim.h"
    #include "usart.h"

#elif RASPBERRY_PICO

    #include <stdio.h>
    #include "pico/stdlib.h"
    #include "hardware/pwm.h"
    #include "hardware/clocks.h"
    #include "hardware/irq.h"
    #include "hardware/adc.h"
    #include "hardware/gpio.h"

#elif ARDUINO

#elif ESP32

#endif


// Proportions
#define CURRENT_SCALING     ((int)(3.3 / 0.001 / 20 / 4096 * 1000)) 
#define VOLTAGE_SCALING     ((int)(3.3 / 4096 * (47 + 2.2) / 2.2 * 1000))

// Parameter
#define HALL_OVERSAMPLE     ((unsigned int)(8))
#define HALL_IDENTIFY_DUTY_CYCLE    ((unsigned int)(25)) 
#define F_PWM   ((unsigned int)(16000))
#define DUTY_CYCLE_MAX      ((unsigned int)(255)) 

// Current cutoff
#define FULL_SCALE_CURRENT_MA       ((int)(30000))

// Throttle limits
#define THROTTLE_LOW    ((unsigned int)(200)) 
#define THROTTLE_HIGH   ((unsigned int)(2540)) 


//ERROR
#define bldc_return_state   int8_t
#define ERROR_DEFAULT   (int8_t)(0)
#define ERROR_OK        (int8_t)(0)
#define ERROR           (int8_t)(1)


#define BLDC_HALL_A_PIN      ((uint16_t)HALL_A_PIN)
#define BLDC_HALL_B_PIN      ((uint16_t)HALL_B_PIN)
#define BLDC_HALL_C_PIN      ((uint16_t)HALL_C_PIN)

#define BLDC_H_A_PIN         ((uint16_t)H_A_PIN)
#define BLDC_H_B_PIN         ((uint16_t)H_B_PIN)
#define BLDC_H_C_PIN         ((uint16_t)H_C_PIN)

#define BLDC_L_A_PIN         ((uint16_t)L_A_PIN)
#define BLDC_L_B_PIN         ((uint16_t)L_B_PIN)
#define BLDC_L_C_PIN         ((uint16_t)L_C_PIN)

#define BLDC_THROTTLE_PIN    ((uint16_t)THROTTLE_PIN)

#define BLDC_VOLTAGE_PIN     ((uint16_t)VOLTAGE_PIN)
#define BLDC_CURRENT_PIN     ((uint16_t)CURRENT_PIN)

#define BLDC_LED_PIN         ((uint16_t)LED_PIN)



#ifdef STM32_HAL

    #define BLDC_HALL_A_PORT         ((GPIO_TypeDef *)STM_HALL_A_PORT)
    #define BLDC_HALL_B_PORT         ((GPIO_TypeDef *)STM_HALL_B_PORT)
    #define BLDC_HALL_C_PORT         ((GPIO_TypeDef *)STM_HALL_C_PORT)

    #define BLDC_H_A_PORT            ((GPIO_TypeDef *)STM_H_A_PORT)
    #define BLDC_H_B_PORT            ((GPIO_TypeDef *)STM_H_B_PORT)
    #define BLDC_H_C_PORT            ((GPIO_TypeDef *)STM_H_C_PORT)

    #define BLDC_L_A_PORT            ((GPIO_TypeDef *)STM_L_A_PORT)
    #define BLDC_L_B_PORT            ((GPIO_TypeDef *)STM_L_B_PORT)
    #define BLDC_L_C_PORT            ((GPIO_TypeDef *)STM_L_C_PORT)

    #define BLDC_THROTTLE_HANDLER    ((ADC_HandleTypeDef *)STM_THROTTLE_HANDLER)
    #define BLDC_VOLTAGE_HANDLER     ((ADC_HandleTypeDef *)STM_VOLTAGE_HANDLER)
    #define BLDC_CURRENT_HANDLER     ((ADC_HandleTypeDef *)STM_CURRENT_HANDLER)

    #define BLDC_PWM_A_HANDLER       ((TIM_HandleTypeDef *)STM_PWM_A_HANDLER)
    #define BLDC_PWM_B_HANDLER       ((TIM_HandleTypeDef *)STM_PWM_B_HANDLER)
    #define BLDC_PWM_C_HANDLER       ((TIM_HandleTypeDef *)STM_PWM_C_HANDLER)

    #define BLDC_UART_HANDLER        ((UART_HandleTypeDef *)STM_UART_HANDLER)

    #define BLDC_LED_PORT            ((GPIO_TypeDef *)LED_PORT)

#elif RASPBERRY_PICO

    #define BLDC_PWM_A_HANDLER       ((uint)PICO_PWM_A_HANDLER)
    #define BLDC_PWM_B_HANDLER       ((uint)PICO_PWM_B_HANDLER)
    #define BLDC_PWM_C_HANDLER       ((uint)PICO_PWM_C_HANDLER)

#elif ARDUINO

#elif ESP32

#endif

/**
 * @brief BLDC motor control structure.
 *
 * This structure holds all the necessary variables for controlling
 * a BLDC motor including PWM values, sensor readings, and control parameters.
 */

typedef struct {

    int pwm_A;
    int pwm_B;
    int pwm_C;
    int adc_current_sense_error;
    int duty_cycle;
    int voltage_mv;
    int current_ma;
    int throttle_pwm;
    unsigned int hallToMotor[8];

}   bldc_t;

extern bldc_t bldc; /**< Bldc data structure */

bldc_return_state bldc_init();
bldc_return_state get_halls(unsigned int* hall);
bldc_return_state process_halls();
bldc_return_state writePhases(uint16_t ah, uint16_t bh, uint16_t ch, uint16_t al, uint16_t bl, uint16_t cl);
bldc_return_state write_pd_table(unsigned int halls, unsigned int duty);

/***************************************
 * ADC functions 
 ***************************************/
bldc_return_state read_throttle(int* data);
bldc_return_state read_voltage(int* data);
bldc_return_state read_current(int* data);

/***************************************
 * IRQ functions 
 ***************************************/
void pwm_irq();
void adc_irq();
bldc_return_state printData(char * message);
bldc_return_state set_pwm(uint16_t ah, uint16_t bh, uint16_t ch, uint16_t al, uint16_t bl, uint16_t cl);

#ifdef STM32_HAL
/**-----------------------------------
 * STM32 HAL specific functions definitions.
 -----------------------------------**/
    bldc_return_state gpio_read(int* data, GPIO_TypeDef* port, uint16_t pin);

    bldc_return_state adc_read(int* data, ADC_HandleTypeDef* port, uint16_t pin);

    bldc_return_state start_adc_irq();

    bldc_return_state start_tim_irq();
#else
/**-----------------------------------
 * RASPBERRY PICO, ARDUINO and ESP32 specific functions definitions.
 -----------------------------------**/
    bldc_return_state gpio_read(int* data, uint16_t pin);

    bldc_return_state adc_read(int* data, ADC_TypeDef* port, uint16_t pin);

#endif

#endif //BLDC_CONTROLLER_H










/*
To do:
Me quede en la definicion de pwm para ARDUINO y ESP32
Definir slice A y B de los 3 timers y seguir implementando el handle de errores en las funciones

primero las definiciones que cambian en hardware

Definir las variables de puertos, pines y handler de harware en bldc_pinout

*/
