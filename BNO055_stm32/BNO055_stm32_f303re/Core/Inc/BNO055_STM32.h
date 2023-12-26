#ifndef BNO055_STM32_H
#define BNO055_STM32_H

#include "BNO055.h"
#include "i2c.h"
#include "main.h"

I2C_HandleTypeDef* i2c_dev;

void bno055_set_i2c_handler(I2C_HandleTypeDef* hi2c){
    i2c_dev = hi2c;
}

void bno055_writeData(uint8_t reg, uint8_t data) {
    uint8_t txdata[2] = {reg, data};
    uint8_t status;
    status = HAL_I2C_Master_Transmit(i2c_dev, BNO055_I2C_ADDR_LO << 1, txdata, sizeof(txdata), 10);
    if (status == HAL_OK) {
        return;
    }

    if (status == HAL_ERROR) {
        printf("HAL_I2C_Master_Transmit HAL_ERROR\r\n");
    } else if (status == HAL_TIMEOUT) {
        printf("HAL_I2C_Master_Transmit HAL_TIMEOUT\r\n");
    } else if (status == HAL_BUSY) {
        printf("HAL_I2C_Master_Transmit HAL_BUSY\r\n");
    } else {
        printf("Unknown status data %d", status);
    }

    uint32_t error = HAL_I2C_GetError(i2c_dev);
    if (error == HAL_I2C_ERROR_NONE) {
        return;
    } else if (error == HAL_I2C_ERROR_BERR) {
        printf("HAL_I2C_ERROR_BERR\r\n");
    } else if (error == HAL_I2C_ERROR_ARLO) {
        printf("HAL_I2C_ERROR_ARLO\r\n");
    } else if (error == HAL_I2C_ERROR_AF) {
        printf("HAL_I2C_ERROR_AF\r\n");
    } else if (error == HAL_I2C_ERROR_OVR) {
        printf("HAL_I2C_ERROR_OVR\r\n");
    } else if (error == HAL_I2C_ERROR_DMA) {
        printf("HAL_I2C_ERROR_DMA\r\n");
    } else if (error == HAL_I2C_ERROR_TIMEOUT) {
        printf("HAL_I2C_ERROR_TIMEOUT\r\n");
    }

    HAL_I2C_StateTypeDef state = HAL_I2C_GetState(i2c_dev);
    if (state == HAL_I2C_STATE_RESET) {
        printf("HAL_I2C_STATE_RESET\r\n");
    } else if (state == HAL_I2C_STATE_READY) {
        printf("HAL_I2C_STATE_RESET\r\n");
    } else if (state == HAL_I2C_STATE_BUSY) {
        printf("HAL_I2C_STATE_BUSY\r\n");
    } else if (state == HAL_I2C_STATE_BUSY_TX) {
        printf("HAL_I2C_STATE_BUSY_TX\r\n");
    } else if (state == HAL_I2C_STATE_BUSY_RX) {
        printf("HAL_I2C_STATE_BUSY_RX\r\n");
    } else if (state == HAL_I2C_STATE_LISTEN) {
        printf("HAL_I2C_STATE_LISTEN\r\n");
    } else if (state == HAL_I2C_STATE_BUSY_TX_LISTEN) {
        printf("HAL_I2C_STATE_BUSY_TX_LISTEN\r\n");
    } else if (state == HAL_I2C_STATE_BUSY_RX_LISTEN) {
        printf("HAL_I2C_STATE_BUSY_RX_LISTEN\r\n");
    } else if (state == HAL_I2C_STATE_ABORT) {
        printf("HAL_I2C_STATE_ABORT\r\n");
    } else if (state == HAL_I2C_STATE_TIMEOUT) {
        printf("HAL_I2C_STATE_TIMEOUT\r\n");
    } else if (state == HAL_I2C_STATE_ERROR) {
        printf("HAL_I2C_STATE_ERROR\r\n");
    }
}

void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len) {
    HAL_I2C_Master_Transmit(i2c_dev, BNO055_I2C_ADDR_LO << 1, &reg, 1, 100);
    HAL_I2C_Master_Receive(i2c_dev, BNO055_I2C_ADDR_LO << 1, data, len, 100);
}

#endif
