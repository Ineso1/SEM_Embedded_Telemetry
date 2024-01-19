#include "bldc_controller.h"

bldc_t bldc = {0};  // Initialize all fields to zero

/**
 * @brief Read hall sensor values and oversample them.
 * 
 * This function reads the state of the hall sensors multiple times (based on HALL_OVERSAMPLE)
 * and determines the most likely state by majority voting. It supports different hardware platforms.
 * @param hall Pointer to store the oversampled hall sensor value.
 * @return bldc_return_state The state of the operation (success or error).
 */
bldc_return_state get_halls(unsigned int * hall){
	unsigned int hallCounts[] = {0, 0, 0};

	// Read all the Hall pins repeatedly and tally the results
	for (unsigned int i = 0; i < HALL_OVERSAMPLE; i++) {
		int h_a, h_b, h_c;
        #ifdef STM32_HAL
            gpio_read(&h_a, BLDC_HALL_A_PORT, BLDC_HALL_A_PIN);
            gpio_read(&h_b, BLDC_HALL_B_PORT, BLDC_HALL_B_PIN);
            gpio_read(&h_c, BLDC_HALL_C_PORT, BLDC_HALL_C_PIN);
        #else
            gpio_read(&h_a, BLDC_HALL_A_PIN);
            gpio_read(&h_b, BLDC_HALL_B_PIN);
            gpio_read(&h_c, BLDC_HALL_C_PIN);
        #endif
		hallCounts[0] += h_a;
		hallCounts[1] += h_b;
		hallCounts[2] += h_c;
	}

	*hall = 0;

	// If votes >= threshold, set the corresponding bit to 1
	if (hallCounts[0] > HALL_OVERSAMPLE / 2)
		*hall |= (1 << 0);
	if (hallCounts[1] > HALL_OVERSAMPLE / 2)
		*hall |= (1 << 1);
	if (hallCounts[2] > HALL_OVERSAMPLE / 2)
		*hall |= (1 << 2);

    #ifdef DEBUG_MSG
        char message[50];
        snprintf(message, sizeof(message), "Hall Value: %u\r\n", *hall);
        printData(message);
    #endif

	return ERROR_OK;
}

/**
 * @brief Process hall sensor signals and control motor phases.
 * 
 * This function processes the hall sensor signals to determine the motor's position and speed.
 * It then adjusts the motor phase control based on throttle PWM value.
 * @return bldc_return_state The state of the process (success or error).
 */
bldc_return_state process_halls(){
    for(unsigned int j = 0; j < 200; j++)
    {
	    unsigned int hall;
		get_halls(&hall);
	    write_pd_table(hall, bldc.throttle_pwm);
    }
    bldc.throttle_pwm = 0;
    #ifdef STM32_HAL
        start_adc_irq();
    #endif
    return ERROR_OK;
}

/**
 * @brief Write PWM values to motor phases.
 * 
 * Sets the PWM values for high and low side transistors of each motor phase.
 * @param ah PWM value for high side of phase A.
 * @param bh PWM value for high side of phase B.
 * @param ch PWM value for high side of phase C.
 * @param al PWM value for low side of phase A.
 * @param bl PWM value for low side of phase B.
 * @param cl PWM value for low side of phase C.
 * @return bldc_return_state The state of the operation (success or error).
 */
bldc_return_state writePhases(uint16_t ah, uint16_t bh, uint16_t ch, uint16_t al, uint16_t bl, uint16_t cl){
	set_pwm(ah, bh, ch, al, bl, cl);
	return ERROR_OK;
}

/**
 * @brief Write phase duty cycle based on hall sensor values.
 * 
 * Adjusts the motor phase control based on the current hall sensor readings and specified duty cycle.
 * @param halls Current hall sensor value.
 * @param duty Specified duty cycle for motor control.
 * @return bldc_return_state The state of the operation (success or error).
 */
bldc_return_state write_pd_table(unsigned int halls, unsigned int duty){
	if(duty > 255){
		duty = 0;
	}
	if(duty < 40){
		bldc.throttle_pwm = 0;
		duty = 0;
		halls = 8;
	}

	unsigned int complement = 255 - duty - 6;

	#ifdef BACKWARDS
		switch(halls){
			case 1: // Case 001
			writePhases(0, 0, duty, 255, 0, complement);
			break;
			case 2: // Case 010
			writePhases(0, duty, 0, 0, complement, 255);
			break;
			case 3: // Case 011
			writePhases(0, duty, 0, 255, complement, 0);
			break;
			case 4: // Case 100
			writePhases(duty, 0, 0, complement, 255, 0);
			break;
			case 5: // Case 101
			writePhases(0, 0, duty, 0, 255, complement);
			break;
			case 6: // Case 110
			writePhases(duty, 0, 0, complement, 0, 255);
			break;
			default: // Case 000 or error
			writePhases(0, 0, 0, 255, 255, 255);
		}

	#else
	
		switch(halls){
			case 1: // Case 001
			writePhases(duty, 0, 0, complement, 0, 255);
			break;
			case 2: // Case 010
			writePhases(0, 0, duty, 0, 255, complement);
			break;
			case 3: // Case 011
			writePhases(duty, 0, 0, complement, 255, 0);
			break;
			case 4: // Case 100
			writePhases(0, duty, 0, 255, complement, 0);
			break;
			case 5: // Case 101
			writePhases(0, duty, 0, 0, complement, 255);
			break;
			case 6: // Case 110
			writePhases(0, 0, duty, 255, 0, complement);
			break;
			default: // Case 000 or error
			writePhases(0, 0, 0, 255, 255, 255);
		}

	#endif
	return ERROR_OK;
}

/**
 * @brief Read throttle value from ADC.
 * 
 * Reads the throttle value from the ADC, scales it, and updates the global BLDC structure.
 * @param data Pointer to store the scaled throttle value.
 * @return bldc_return_state The state of the operation (success or error).
 */
bldc_return_state read_throttle(int* data){
	int throttle_adc;
    #ifdef STM32_HAL
	    adc_read(&throttle_adc, BLDC_THROTTLE_HANDLER, BLDC_THROTTLE_PIN);
    #else
	    read_adc(&throttle_adc, BLDC_THROTTLE_PIN);
    #endif
	throttle_adc = (throttle_adc - THROTTLE_LOW) * 255;
	if(throttle_adc < 0){
		throttle_adc *= -1;
	}

	throttle_adc /= (THROTTLE_HIGH - THROTTLE_LOW);
	bldc.throttle_pwm = throttle_adc;

	#ifdef DEBUG_MSG
		char message[50];
		snprintf(message, sizeof(message), "Throttle Value: %u\r\n", throttle_adc);
		printData(message);
	#endif

    if (throttle_adc >= 255) // Bound the output between 0 and 255
        bldc.throttle_pwm = 255;

    if (throttle_adc <= 0)
        bldc.throttle_pwm = 0;

    *data = bldc.throttle_pwm;
    return ERROR_OK;
}

/**
 * @brief Read voltage value from ADC.
 * 
 * Reads the voltage value from the ADC and updates the global BLDC structure after scaling.
 * @param data Pointer to store the scaled voltage value.
 * @return bldc_return_state The state of the operation (success or error).
 */
bldc_return_state read_voltage(int* data){
	adc_read(data, BLDC_CURRENT_HANDLER, BLDC_CURRENT_PIN);
	bldc.voltage_mv = *data * CURRENT_SCALING;
	/*Scale and limits*/
	return ERROR_OK;
}

/**
 * @brief Read current value from ADC.
 * 
 * Reads the current value from the ADC and updates the global BLDC structure after scaling.
 * @param data Pointer to store the scaled current value.
 * @return bldc_return_state The state of the operation (success or error).
 */
bldc_return_state read_current(int* data){
	adc_read(data, BLDC_VOLTAGE_HANDLER, BLDC_VOLTAGE_PIN);
	bldc.current_ma = *data * VOLTAGE_SCALING;
	/*Scale and limits*/
	/*Function to protect over current*/
	return ERROR_OK;
}

/**
 * @brief PWM interrupt handler.
 * 
 * Handles PWM-related interrupt processing. This function is platform-dependent.
 */
void pwm_irq() {
    #ifdef STM32_HAL
	    start_adc_irq();
    #elif RASPBERRY_PICO
        adc_select_input(0);
        adc_run(true);
        pwm_clear_irq(BLDC_PWM_A_HANDLER);
        while(!adc_fifo_is_empty())
        adc_fifo_get();
    #endif
}

/**
 * @brief ADC interrupt handler.
 * 
 * Handles ADC-related interrupt processing. Reads throttle, voltage, current, and updates
 * the motor control based on these values. This function is platform-dependent.
 */
void adc_irq() {
	int throttle = 0, voltage = 0, current = 0;
	read_throttle(&throttle);
	read_voltage(&voltage);
	read_current(&current);
	unsigned int halls;
	get_halls(&halls);
	write_pd_table(halls, bldc.throttle_pwm);

	#ifdef DEBUG_MSG
		char message[50];
		snprintf(message, sizeof(message), "Throttle Value: %u\r\n", bldc.throttle_pwm);
		printData(message);
	#endif

    #ifdef STM32_HAL
	    start_tim_irq();
    #endif
}



#ifdef STM32_HAL

    /**
     * @brief Initialize the BLDC motor controller for STM32 HAL platform.
     * 
     * Sets up timers and PWM for motor control, initializes peripherals for the STM32 platform.
     * @return bldc_return_state The state of initialization (success or error).
     */
    bldc_return_state bldc_init(){
        __HAL_TIM_SET_COMPARE(BLDC_PWM_A_HANDLER, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(BLDC_PWM_A_HANDLER, TIM_CHANNEL_2, 255);
        __HAL_TIM_SET_COMPARE(BLDC_PWM_B_HANDLER, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(BLDC_PWM_B_HANDLER, TIM_CHANNEL_2, 255);
        __HAL_TIM_SET_COMPARE(BLDC_PWM_C_HANDLER, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(BLDC_PWM_C_HANDLER, TIM_CHANNEL_2, 255);

        HAL_TIM_PWM_Start(BLDC_PWM_A_HANDLER, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start_IT(BLDC_PWM_A_HANDLER, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(BLDC_PWM_B_HANDLER, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(BLDC_PWM_B_HANDLER, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(BLDC_PWM_C_HANDLER, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(BLDC_PWM_C_HANDLER, TIM_CHANNEL_2);

        return ERROR_OK;
    }
    
    /**
     * @brief Read GPIO pin value on STM32 HAL platform.
     * 
     * Reads the value of a specified GPIO pin on the STM32 platform.
     * @param data Pointer to store the read GPIO value.
     * @param port GPIO port of the pin.
     * @param pin GPIO pin number.
     * @return bldc_return_state The state of GPIO reading (success or error).
     */
    bldc_return_state gpio_read(int* data, GPIO_TypeDef* port, uint16_t pin) {
        *data = HAL_GPIO_ReadPin(port, pin);
        return ERROR_OK;
    }

    /**
     * @brief Read ADC value on STM32 HAL platform.
     * 
     * Reads the value from an ADC channel on the STM32 platform.
     * @param data Pointer to store the read ADC value.
     * @param port ADC handler for the channel.
     * @param pin ADC pin number.
     * @return bldc_return_state The state of ADC reading (success or error).
     */
    bldc_return_state adc_read(int* data, ADC_HandleTypeDef* port, uint16_t pin) {
        *data = HAL_ADC_GetValue(port);
        return ERROR_OK;
    }
    
    /**
     * @brief Print data message on STM32 HAL platform.
     * 
     * Transmits a string message over UART for debugging or informational purposes on STM32.
     * @param message The string message to be printed.
     * @return bldc_return_state The state of the print operation (success or error).
     */
    bldc_return_state printData(char * message){
        HAL_StatusTypeDef status;
        status = HAL_UART_Transmit(BLDC_UART_HANDLER, (uint8_t *)message, strlen(message), 10);
        if (status != HAL_OK)
        {
            return ERROR;
        }
        return ERROR_OK;
    }

    /**
     * @brief Set PWM values for motor phases on STM32 HAL platform.
     * 
     * Sets the PWM values for high and low side transistors of each motor phase for STM32.
     * @param ah PWM value for high side of phase A.
     * @param bh PWM value for high side of phase B.
     * @param ch PWM value for high side of phase C.
     * @param al PWM value for low side of phase A.
     * @param bl PWM value for low side of phase B.
     * @param cl PWM value for low side of phase C.
     * @return bldc_return_state The state of setting PWM values (success or error).
     */
    bldc_return_state set_pwm(uint16_t ah, uint16_t bh, uint16_t ch, uint16_t al, uint16_t bl, uint16_t cl){
    	__HAL_TIM_SET_COMPARE(BLDC_PWM_A_HANDLER, TIM_CHANNEL_1, ah);
        __HAL_TIM_SET_COMPARE(BLDC_PWM_A_HANDLER, TIM_CHANNEL_2, 255 - al);
        __HAL_TIM_SET_COMPARE(BLDC_PWM_B_HANDLER, TIM_CHANNEL_1, bh);
        __HAL_TIM_SET_COMPARE(BLDC_PWM_B_HANDLER, TIM_CHANNEL_2, 255 - bl);
        __HAL_TIM_SET_COMPARE(BLDC_PWM_C_HANDLER, TIM_CHANNEL_1, ch);
        __HAL_TIM_SET_COMPARE(BLDC_PWM_C_HANDLER, TIM_CHANNEL_2, 255 - cl);
        return ERROR_OK;
    }

    /**
     * @brief Start ADC interrupt on STM32 HAL platform.
     * 
     * Initiates the ADC interrupt for throttle readings on the STM32 platform.
     * @return bldc_return_state The state of starting ADC interrupt (success or error).
     */
    bldc_return_state start_adc_irq(){
        HAL_ADC_Stop(BLDC_THROTTLE_HANDLER);
        HAL_ADC_Start_IT(BLDC_THROTTLE_HANDLER);
        return ERROR_OK;
    }

    /**
     * @brief Start timer interrupt on STM32 HAL platform.
     * 
     * Initiates the timer interrupt for PWM control on the STM32 platform.
     * @return bldc_return_state The state of starting timer interrupt (success or error).
     */
    bldc_return_state start_tim_irq(){
		HAL_TIM_PWM_Stop(BLDC_PWM_A_HANDLER, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start_IT(BLDC_PWM_A_HANDLER, TIM_CHANNEL_2);
		return ERROR_OK;
    }

    /**
     * @brief Callback for TIM PWM pulse finished on STM32 HAL platform.
     * 
     * Invoked when a PWM pulse is finished. It calls the pwm_irq function.
     * @param htim Handle to the TIM_HandleTypeDef structure.
     */
    void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
    	if(htim->Instance == BLDC_PWM_A_HANDLER->Instance){
    		HAL_GPIO_TogglePin(BLDC_LED_PORT, BLDC_LED_PIN);
    		pwm_irq();
    	}
    }

    /**
     * @brief Callback for ADC conversion complete on STM32 HAL platform.
     * 
     * Invoked when an ADC conversion is complete. It calls the adc_irq function.
     * @param hadc Handle to the ADC_HandleTypeDef structure.
     */
    void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
		if(hadc->Instance == BLDC_THROTTLE_HANDLER->Instance){
			adc_irq();
		}
	}

#elif RASPBERRY_PICO

    /**
     * @brief Initialize the BLDC motor controller for Raspberry Pi Pico platform.
     * 
     * Sets up GPIO, PWM, ADC, and IRQ for motor control on the Raspberry Pi Pico.
     * @return bldc_return_state The state of initialization (success or error).
     */
    bldc_return_state bldc_init(){
        stdio_init_all();

        //  Initialize Led
        gpio_init(BLDC_LED_PIN);
        gpio_set_dir(BLDC_LED_PIN, GPIO_OUT);

        //  Initialize hall sensor pins
        gpio_init(BLDC_HALL_A_PIN);
        gpio_set_dir(BLDC_HALL_A_PIN, GPIO_IN);
        gpio_init(BLDC_HALL_B_PIN);
        gpio_set_dir(BLDC_HALL_B_PIN, GPIO_IN);
        gpio_init(BLDC_HALL_B_PIN);
        gpio_set_dir(BLDC_HALL_B_PIN, GPIO_IN);

        //  Initialize predriver PWM pins
        gpio_set_function(BLDC_H_A_PIN, GPIO_FUNC_PWM);
        gpio_set_function(BLDC_L_A_PIN, GPIO_FUNC_PWM);
        gpio_set_function(BLDC_H_B_PIN, GPIO_FUNC_PWM);
        gpio_set_function(BLDC_L_B_PIN, GPIO_FUNC_PWM);
        gpio_set_function(BLDC_H_C_PIN, GPIO_FUNC_PWM);
        gpio_set_function(BLDC_L_C_PIN, GPIO_FUNC_PWM);

        //  PWM configuration
        float pwm_divider = (float)(clock_get_hz(clk_sys)) / (F_PWM * 255 * 2);
        pwm_config config = pwm_get_default_config();
        pwm_config_set_clkdiv(&config, pwm_divider);
        pwm_config_set_wrap(&config, 255 - 1);
        pwm_config_set_phase_correct(&config, true);
        pwm_config_set_output_polarity(&config, false, true);

        pwm_init(BLDC_PWM_A_HANDLER, &config, false);
        pwm_init(BLDC_PWM_B_HANDLER, &config, false);
        pwm_init(BLDC_PWM_C_HANDLER, &config, false);

        pwm_set_mask_enabled(0x07);

        //  Initialize IRQ
        pwm_clear_irq(BLDC_PWM_A_HANDLER);
        pwm_set_irq_enabled(BLDC_PWM_A_HANDLER, true);
        irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_irq);
        irq_set_priority(PWM_IRQ_WRAP, 0);
        irq_set_enabled(PWM_IRQ_WRAP, true);

        //  Initialize ADC
        adc_init();
        adc_gpio_init(BLDC_CURRENT_PIN);  
        adc_gpio_init(BLDC_VOLTAGE_PIN);
        adc_gpio_init(BLDC_THROTTLE_PIN);
        adc_set_round_robin(0b111);
        adc_fifo_setup(true, false, 3, false, false);
        irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_irq);
        irq_set_priority(ADC_IRQ_FIFO, 0);
        adc_irq_set_enabled(true);
        irq_set_enabled(ADC_IRQ_FIFO, true);

        //  Adjust isense
        sleep_ms(500);
        for(uint i = 0; i < 100; i++){ 
            bldc.adc_current_sense_error += bldc.current_ma;   //adc_current_sense_error es el error cuando no hay corriente circulando por el motor 
        }
        sleep_ms(10);
        bldc.adc_current_sense_error /= 100;

        return ERROR_OK;
    }

/**
     * @brief Read GPIO pin value on Raspberry Pi Pico platform.
     * 
     * Reads the value of a specified GPIO pin on the Raspberry Pi Pico.
     * @param data Pointer to store the read GPIO value.
     * @param pin GPIO pin number.
     * @return bldc_return_state The state of GPIO reading (success or error).
     */
    bldc_return_state gpio_read(int* data, uint16_t pin) {
        *data = gpio_get(pin);
    }

    /**
     * @brief Print data message on Raspberry Pi Pico platform.
     * 
     * Sends a string message over UART for debugging or informational purposes on Raspberry Pi Pico.
     * @param message The string message to be printed.
     * @return bldc_return_state The state of the print operation (success or error).
     */
    bldc_return_state printData(char* message) {
        int transmitted = uart_puts(UART_ID, message);
        if (transmitted < 0) {
            return ERROR;
        }
        return ERROR_OK;
    }

    /**
     * @brief Set PWM values for motor phases on Raspberry Pi Pico platform.
     * 
     * Sets the PWM values for high and low side transistors of each motor phase for Raspberry Pi Pico.
     * @param ah PWM value for high side of phase A.
     * @param bh PWM value for high side of phase B.
     * @param ch PWM value for high side of phase C.
     * @param al PWM value for low side of phase A.
     * @param bl PWM value for low side of phase B.
     * @param cl PWM value for low side of phase C.
     * @return bldc_return_state The state of setting PWM values (success or error).
     */
    bldc_return_state set_pwm(uint16_t ah, uint16_t bh, uint16_t ch, uint16_t al, uint16_t bl, uint16_t cl){
        pwm_set_both_levels(BLDC_PWM_A_HANDLER , ah, 255 - al);
        pwm_set_both_levels(BLDC_PWM_B_HANDLER , bh, 255 - bl);
        pwm_set_both_levels(BLDC_PWM_C_HANDLER , ch, 255 - cl);
    }

#elif ARDUINO

    /**
     * @brief Read GPIO pin value on Arduino platform.
     * 
     * Reads the value of a specified GPIO pin on the Arduino.
     * @param data Pointer to store the read GPIO value.
     * @param pin GPIO pin number.
     * @return bldc_return_state The state of GPIO reading (success or error).
     */
    bldc_return_state gpio_read(int* data, uint16_t pin) {
            *data = digitalRead(pin);
    }

    /**
     * @brief Print data message on Arduino platform.
     * 
     * Sends a string message over Serial for debugging or informational purposes on Arduino.
     * @param message The string message to be printed.
     * @return bldc_return_state The state of the print operation (success or error).
     */
    bldc_return_state printData(char* message) {
        if (!Serial) {
            return ERROR;
        }
        Serial.print(message);
        return ERROR_OK;
    }

    /**
     * @brief Set PWM values for motor phases on Arduino platform.
     * 
     * Placeholder for setting the PWM values for motor phases on Arduino. To be implemented.
     * @param ah PWM value for high side of phase A.
     * @param bh PWM value for high side of phase B.
     * @param ch PWM value for high side of phase C.
     * @param al PWM value for low side of phase A.
     * @param bl PWM value for low side of phase B.
     * @param cl PWM value for low side of phase C.
     * @return bldc_return_state The state of setting PWM values (success or error).
     */                                             
    bldc_return_state set_pwm(uint16_t ah, uint16_t bh, uint16_t ch, uint16_t al, uint16_t bl, uint16_t cl){
    }

#elif ESP32

    /**
     * @brief Read GPIO pin value on ESP32 platform.
     * 
     * Reads the value of a specified GPIO pin on the ESP32.
     * @param data Pointer to store the read GPIO value.
     * @param pin GPIO pin number.
     * @return bldc_return_state The state of GPIO reading (success or error).
     */
    bldc_return_state gpio_read(int* data, uint16_t pin) {
            *data = digitalRead(pin);
    }

    /**
     * @brief Print data message on ESP32 platform.
     * 
     * Sends a string message over Serial for debugging or informational purposes on ESP32.
     * @param message The string message to be printed.
     * @return bldc_return_state The state of the print operation (success or error).
     */
    bldc_return_state printData(char* message) {
        if (!Serial) {
            return ERROR;
        }
        Serial.print(message);
        return ERROR_OK;
    }

    /**
     * @brief Set PWM values for motor phases on ESP32 platform.
     * 
     * Placeholder for setting the PWM values for motor phases on ESP32. To be implemented.
     * @param ah PWM value for high side of phase A.
     * @param bh PWM value for high side of phase B.
     * @param ch PWM value for high side of phase C.
     * @param al PWM value for low side of phase A.
     * @param bl PWM value for low side of phase B.
     * @param cl PWM value for low side of phase C.
     * @return bldc_return_state The state of setting PWM values (success or error).
     */
    bldc_return_state set_pwm(uint16_t ah, uint16_t bh, uint16_t ch, uint16_t al, uint16_t bl, uint16_t cl){
    }

#endif
