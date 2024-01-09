#include "BNO055.h"

I2C_HandleTypeDef* i2c_dev;

bno055_conf_t default_bno055_config = {
    .pwr_mode = POWER_MODE_NORMAL, // Default power mode
    .op_mode = OPERATION_MODE_NDOF, // Default operation mode (9 Degrees of Freedom)
    .axis_remap_conf = AXIS_REMAP_CONFIG_P0, // Default axis remap configuration
    .axis_remap_sign = AXIS_REMAP_SIGN_P0, // Default axis remap sign
    .acc_g_range = ACC_CONFIG_4G, // Default accelerometer G range
    .acc_bandwidth = ACC_CONFIG_62_5Hz, // Default accelerometer bandwidth
    .acc_operation_mode = ACC_CONFIG_NORMAL, // Default accelerometer operation mode
    .gyr_range = GYR_CONFIG_250DPS, // Default gyroscope range
    .gyr_bandwidth = GYR_CONFIG_116Hz, // Default gyroscope bandwidth
    .gyr_op_mode = GYR_CONFIG_NORMAL, // Default gyroscope operation mode
    .mag_data_rate = MAG_CONFIG_10Hz, // Default magnetometer data rate
    .mag_op_mode = MAG_CONFIG_REGULAR, // Default magnetometer operation mode
    .mag_pwr_mode = MAG_CONFIG_NORMAL, // Default magnetometer power mode
    .unit_sel = ACCELERATION_M_S2 | ANGULAR_RATE_DPS | EULER_ANGLES_DEG | TEMPERATURE_C // Default units
};

bno055_verification_t default_bno055_verification = {
    .chip_id = 0,
    .sw_rev_id = 0,
    .page_id = 0,
    .acc_id = 0,
    .mag_id = 0,
    .gyr_id = 0,
    .bl_rev_id = 0
};

void bno055_set_i2c_handler(I2C_HandleTypeDef* hi2c){
    i2c_dev = hi2c;
}

uint8_t bno055_writeData(uint8_t* txdata) {
    uint8_t status;
    status = HAL_I2C_Master_Transmit(i2c_dev, BNO055_I2C_ADDR_LO << 1, txdata, sizeof(txdata), 10);
    if (status == HAL_OK) {
        return 0;
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
        return 1;
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
    return 1;
}


uint8_t bno055_readData(uint8_t reg, uint8_t *data, uint8_t len) {
	uint8_t status;
	status = HAL_I2C_Master_Transmit(i2c_dev, BNO055_I2C_ADDR_LO << 1, &reg, 1, 10);
	if (status != HAL_OK) {
		return 1;
	}
    bno055_delay(10);
    status = HAL_I2C_Master_Receive(i2c_dev, BNO055_I2C_ADDR_LO << 1, data, len, 10);
    bno055_delay(10);
    if (status == HAL_OK) {
		return 0;
	}
	else{
		return 1;
	}
}


void bno055_delay(uint32_t ms){
    HAL_Delay(ms);
}

BNO055_FUNC_RETURN bno055_init(bno055_conf_t * bno055_conf, bno055_verification_t * bno055_verification){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t conf_page0 [2] = {BNO055_PAGE_ID, 0x00};
    uint8_t op_mode_conf [2] = {BNO055_OPR_MODE, OPERATION_MODE_CONFIG};

    ret += bno055_writeData(conf_page0);
    bno055_delay(10);

    ret +=bno055_writeData(op_mode_conf);
	bno055_delay(550);

	uint8_t conf_page1 [2] = {BNO055_PAGE_ID, 0x01};
	uint8_t acc_conf [2] = {BNO055_ACC_CONFIG, bno055_conf->acc_operation_mode << 5 | bno055_conf->acc_bandwidth << 2 | bno055_conf->acc_g_range };
	uint8_t gyr_conf0 [2] = {BNO055_GYRO_CONFIG_0, bno055_conf->gyr_bandwidth << 3 | bno055_conf->gyr_range };
	uint8_t gyr_conf1 [2] = {BNO055_GYRO_CONFIG_1, bno055_conf->gyr_op_mode };
	uint8_t mag_conf [2] = {BNO055_MAG_CONFIG, bno055_conf->mag_pwr_mode << 5 | bno055_conf->mag_op_mode << 3 | bno055_conf->mag_data_rate };

	ret += bno055_writeData(conf_page1);
	bno055_delay(10);


	ret += bno055_writeData(acc_conf);
	bno055_delay(10);

	ret += bno055_writeData(gyr_conf0);
	bno055_delay(10);

	ret += bno055_writeData(gyr_conf1);
	bno055_delay(10);

	ret += bno055_writeData(mag_conf);
	bno055_delay(10);


    uint8_t pwr_mode [2] = {BNO055_PWR_MODE, bno055_conf->pwr_mode};
    uint8_t op_mode [2] = {BNO055_OPR_MODE, bno055_conf->op_mode};
    uint8_t axis_remap_conf [2] = {BNO055_AXIS_MAP_CONFIG, bno055_conf->axis_remap_conf};
    uint8_t axis_remap_sign [2] = {BNO055_AXIS_MAP_SIGN, bno055_conf->axis_remap_sign};

    uint8_t unit_sel [2] = {BNO055_UNIT_SEL, bno055_conf->unit_sel};

    ret += bno055_writeData(conf_page0);
	bno055_delay(10);

	ret += bno055_writeData(pwr_mode);
    bno055_delay(10);

    ret += bno055_writeData(axis_remap_conf);
    bno055_delay(10);

    ret += bno055_writeData(axis_remap_sign);
    bno055_delay(10);

    ret += bno055_writeData(unit_sel);
    bno055_delay(10);

    ret += bno055_writeData(op_mode);
    bno055_delay(10);

    uint8_t sw_id[2] = {0, 0};
    uint8_t data = 0;

    ret += bno055_readData(BNO055_CHIP_ID, &data, 1);
    bno055_verification->chip_id = data;
    bno055_delay(10);

    ret += bno055_readData(BNO055_ACC_ID, &data, 1);
    bno055_verification->acc_id = data;
    bno055_delay(10);

    ret += bno055_readData(BNO055_MAG_ID, &data, 1);
    bno055_verification->mag_id = data;
    bno055_delay(10);

    ret += bno055_readData(BNO055_GYR_ID, &data, 1);
    bno055_verification->gyr_id = data;
    bno055_delay(10);

    ret += bno055_readData(BNO055_BL_REV_ID, &data, 1);
    bno055_verification->bl_rev_id = data;
    bno055_delay(10);

    ret += bno055_readData(BNO055_SW_REV_ID_LSB, sw_id, 2);
    bno055_verification->sw_rev_id = (uint16_t)((sw_id[1] << 8)|(sw_id[0]));
    bno055_delay(10);

    ret += bno055_readData(BNO055_PAGE_ID, &data, 1);
    bno055_verification->page_id = data;
    bno055_delay(100);

    return ret;
}

BNO055_FUNC_RETURN bno055_read_acc_x(uint16_t* acc_x){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_ACC_DATA_X_LSB, data, 2);
    *acc_x = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_acc_y(uint16_t* acc_y){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_ACC_DATA_Y_LSB, data, 2);
    *acc_y = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_acc_z(uint16_t* acc_z){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_ACC_DATA_Z_LSB, data, 2);
    *acc_z = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_acc_xyz(bno055_acc_t* acc_xyz){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[6] = {0,0,0,0,0,0};
    ret += bno055_readData(BNO055_ACC_DATA_X_LSB, data, 6);
    acc_xyz->x = (float)(uint16_t)((data[1] << 8)|(data[0]));
    acc_xyz->y = (float)(uint16_t)((data[3] << 8)|(data[2]));
    acc_xyz->z = (float)(uint16_t)((data[5] << 8)|(data[4]));
    return ret;
}

BNO055_FUNC_RETURN bno055_read_mag_x(uint16_t* mag_x){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_MAG_DATA_X_LSB, data, 2);
    *mag_x = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_mag_y(uint16_t* mag_y){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_MAG_DATA_Y_LSB, data, 2);
    *mag_y = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_mag_z(uint16_t* mag_z){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_MAG_DATA_Z_LSB, data, 2);
    *mag_z = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_mag_xyz(bno055_mag_t* mag_xyz){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[6] = {0,0,0,0,0,0};
    ret += bno055_readData(BNO055_MAG_DATA_X_LSB, data, 6);
    mag_xyz->x = (float)(uint16_t)((data[1] << 8)|(data[0]));
    mag_xyz->y = (float)(uint16_t)((data[3] << 8)|(data[2]));
    mag_xyz->z = (float)(uint16_t)((data[5] << 8)|(data[4]));
    return ret;
}

BNO055_FUNC_RETURN bno055_read_gyr_x(uint16_t* gyr_x){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_MAG_DATA_Z_LSB, data, 2);
    *gyr_x += (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_gyr_y(uint16_t* gyr_y){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_MAG_DATA_Z_LSB, data, 2);
    *gyr_y = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_gyr_z(uint16_t* gyr_z){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_MAG_DATA_Z_LSB, data, 2);
    *gyr_z = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_gyr_xyz(bno055_gyr_t* gyr_xyz){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[6] = {0,0,0,0,0,0};
    ret += bno055_readData(BNO055_MAG_DATA_X_LSB, data, 6);
    gyr_xyz->x = (float)(uint16_t)((data[1] << 8)|(data[0]));
    gyr_xyz->y = (float)(uint16_t)((data[3] << 8)|(data[2]));
    gyr_xyz->z = (float)(uint16_t)((data[5] << 8)|(data[4]));
    return ret;
}

BNO055_FUNC_RETURN bno055_read_euler_h(uint16_t* euler_h){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_EUL_HEADING_LSB, data, 2);
    *euler_h = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_euler_r(uint16_t* euler_r){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_EUL_ROLL_LSB, data, 2);
    *euler_r = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_euler_p(uint16_t* euler_p){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_EUL_PITCH_LSB, data, 2);
    *euler_p = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_euler_hrp(bno055_euler_t* euler_hrp){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[6] = {0,0,0,0,0,0};
    ret += bno055_readData(BNO055_EUL_HEADING_LSB, data, 6);
    euler_hrp->h = (float)(uint16_t)((data[1] << 8)|(data[0]));
    euler_hrp->r = (float)(uint16_t)((data[3] << 8)|(data[2]));
    euler_hrp->p = (float)(uint16_t)((data[5] << 8)|(data[4]));
    return ret;
}

BNO055_FUNC_RETURN bno055_read_quaternion_w(uint16_t* quaternion_w){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_QUA_DATA_W_LSB, data, 2);
    *quaternion_w = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_quaternion_x(uint16_t* quaternion_x){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_QUA_DATA_X_LSB, data, 2);
    *quaternion_x = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_quaternion_y(uint16_t* quaternion_y){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_QUA_DATA_Y_LSB, data, 2);
    *quaternion_y = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_quaternion_z(uint16_t* quaternion_z){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_QUA_DATA_Z_LSB, data, 2);
    *quaternion_z = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_quaternion_wxyz(bno055_quaternion_t* quaternion_wxyz){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[8] = {0,0,0,0,0,0,0,0};
    ret += bno055_readData(BNO055_EUL_HEADING_LSB, data, 8);
    quaternion_wxyz->w = (float)(uint16_t)((data[1] << 8)|(data[0]));
    quaternion_wxyz->x = (float)(uint16_t)((data[3] << 8)|(data[2]));
    quaternion_wxyz->y = (float)(uint16_t)((data[5] << 8)|(data[4]));
    quaternion_wxyz->z = (float)(uint16_t)((data[7] << 8)|(data[6]));
    return ret;
}

BNO055_FUNC_RETURN bno055_read_linear_acc_x(uint16_t* linear_acc_x){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_LIA_DATA_X_LSB, data, 2);
    *linear_acc_x = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_linear_acc_y(uint16_t* linear_acc_y){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_LIA_DATA_Y_LSB, data, 2);
    *linear_acc_y = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_linear_acc_z(uint16_t* linear_acc_z){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_LIA_DATA_Z_LSB, data, 2);
    *linear_acc_z = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_linear_acc_xyz(bno055_linear_acc_t* linear_acc_xyz){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[6] = {0,0,0,0,0,0};
    ret += bno055_readData(BNO055_EUL_HEADING_LSB, data, 6);
    linear_acc_xyz->x = (float)(uint16_t)((data[1] << 8)|(data[0]));
    linear_acc_xyz->y = (float)(uint16_t)((data[3] << 8)|(data[2]));
    linear_acc_xyz->z = (float)(uint16_t)((data[5] << 8)|(data[4]));
    return ret;
}

BNO055_FUNC_RETURN bno055_read_gravity_x(uint16_t* gravity_x){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_GRV_DATA_X_LSB, data, 2);
    *gravity_x = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_gravity_y(uint16_t* gravity_y){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_GRV_DATA_Y_LSB, data, 2);
    *gravity_y = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_gravity_z(uint16_t* gravity_z){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[2] = {0,0};
    ret += bno055_readData(BNO055_GRV_DATA_Z_LSB, data, 2);
    *gravity_z = (uint16_t)((data[1] << 8)|(data[0]));
    return ret;
}
BNO055_FUNC_RETURN bno055_read_gravity_xyz(bno055_gravity_t* gravity_xyz){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    uint8_t data[6] = {0,0,0,0,0,0};
    ret += bno055_readData(BNO055_GRV_DATA_X_LSB, data, 6);
    gravity_xyz->x = (float)(uint16_t)((data[1] << 8)|(data[0]));
    gravity_xyz->y = (float)(uint16_t)((data[3] << 8)|(data[2]));
    gravity_xyz->z = (float)(uint16_t)((data[5] << 8)|(data[4]));
    return ret;
}

BNO055_FUNC_RETURN bno055_get_acc_calib_status(){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    return ret;
}
BNO055_FUNC_RETURN bno055_get_mag_calib_status(){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    return ret;
}
BNO055_FUNC_RETURN bno055_get_gyr_calib_status(){
    BNO055_FUNC_RETURN ret = ERROR_DEFAULT;
    return ret;
}


/**
 * Get the correct chip id
 * Make sure on every function to have the right page
 * Get calibration
 * Set the units on the Init
 * Make validation on the Init
 * Make sure every function has its return value
 * Handle each funtion its return value
 * Make the begining confic struct
 * Struct to handle page, chip id, etc.
 */
