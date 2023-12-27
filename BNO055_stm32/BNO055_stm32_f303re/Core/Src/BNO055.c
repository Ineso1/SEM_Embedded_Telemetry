#include "BNO055.h"

BNO055_FUNC_RETURN bno055_init(bno055_conf_t * bno055_conf){

    uint8_t conf_page0 [2] = {BNO055_PAGE_ID, 0x00};
    uint8_t pwr_mode [2] = {BNO055_PWR_MODE, bno055_conf->pwr_mode};
    uint8_t op_mode [2] = {BNO055_OPR_MODE, bno055_conf->op_mode};
    uint8_t axis_remap_conf [2] = {BNO055_AXIS_MAP_CONFIG, bno055_conf->axis_remap_conf};
    uint8_t axis_remap_sign [2] = {BNO055_AXIS_MAP_SIGN, bno055_conf->axis_remap_sign};
    
    uint8_t unit_sel [2] = {BNO055_OPR_MODE, bno055_conf->unit_sel};

    bno055_writeData(&conf_page0);
    bno055_delay(10);

    bno055_writeData(&pwr_mode);
    bno055_delay(10);

    bno055_writeData(&op_mode);
    bno055_delay(10);

    bno055_writeData(&axis_remap_conf);
    bno055_delay(10);

    bno055_writeData(&axis_remap_sign);
    bno055_delay(10);

    uint8_t conf_page1 [2] = {BNO055_PAGE_ID, 0x01};
    uint8_t acc_conf [2] = {BNO055_ACC_CONFIG, bno055_conf->acc_operation_mode << 5 | bno055_conf->acc_bandwidth << 2 | bno055_conf->acc_g_range };
    uint8_t gyr_conf0 [2] = {BNO055_GYRO_CONFIG_0, bno055_conf->gyr_bandwidth << 3 | bno055_conf->gyr_range };
    uint8_t gyr_conf1 [2] = {BNO055_GYRO_CONFIG_1, bno055_conf->gyr_op_mode };
    uint8_t mag_conf [2] = {BNO055_MAG_CONFIG, bno055_conf->mag_pwr_mode << 5 | bno055_conf->mag_op_mode << 3 | bno055_conf->mag_data_rate };

    bno055_writeData(&conf_page1);
    bno055_delay(10);

    bno055_writeData(&acc_conf);
    bno055_delay(10);

    bno055_writeData(&gyr_conf0);
    bno055_delay(10);

    bno055_writeData(&gyr_conf1);
    bno055_delay(10);

    bno055_writeData(&mag_conf);
    bno055_delay(10);

}

BNO055_FUNC_RETURN bno055_read_acc_x(uint16_t* acc_x){
    uint8_t data[2] = {0,0};
    bno055_readData(BNO055_ACC_DATA_X_LSB, data, 2);
    *acc_x = (uint16_t)((data[1] << 8)|(data[0]));
}
BNO055_FUNC_RETURN bno055_read_acc_y(uint16_t* acc_y){
    uint8_t data[2] = {0,0};
    bno055_readData(BNO055_ACC_DATA_Y_LSB, data, 2);    
    *acc_y = (uint16_t)((data[1] << 8)|(data[0]));
}
BNO055_FUNC_RETURN bno055_read_acc_z(uint16_t* acc_z){
    uint8_t data[2] = {0,0};
    bno055_readData(BNO055_ACC_DATA_Z_LSB, data, 2);
    *acc_z = (uint16_t)((data[1] << 8)|(data[0]));
}
BNO055_FUNC_RETURN bno055_read_acc_xyz(bno055_acc_t* acc_xyz){
    uint8_t data[6] = {0,0,0,0,0,0};
    bno055_readData(BNO055_ACC_DATA_X_LSB, data, 6);
    acc_xyz->x = (float)(uint16_t)((data[1] << 8)|(data[0]));
    acc_xyz->y = (float)(uint16_t)((data[3] << 8)|(data[2]));
    acc_xyz->z = (float)(uint16_t)((data[5] << 8)|(data[4]));
}

BNO055_FUNC_RETURN bno055_read_mag_x(uint16_t* mag_x){
    uint8_t data[2] = {0,0};
    bno055_readData(BNO055_MAG_DATA_X_LSB, data, 2);
    *mag_x = (uint16_t)((data[1] << 8)|(data[0]));
}
BNO055_FUNC_RETURN bno055_read_mag_y(uint16_t* mag_y){
    uint8_t data[2] = {0,0};
    bno055_readData(BNO055_MAG_DATA_Y_LSB, data, 2);
    *mag_y = (uint16_t)((data[1] << 8)|(data[0]));
}
BNO055_FUNC_RETURN bno055_read_mag_z(uint16_t* mag_z){
    uint8_t data[2] = {0,0};
    bno055_readData(BNO055_MAG_DATA_Z_LSB, data, 2);
    *mag_z = (uint16_t)((data[1] << 8)|(data[0]));
}
BNO055_FUNC_RETURN bno055_read_mag_xyz(bno055_mag_t* mag_xyz){
    uint8_t data[6] = {0,0,0,0,0,0};
    bno055_readData(BNO055_MAG_DATA_X_LSB, data, 6);
    mag_xyz->x = (float)(uint16_t)((data[1] << 8)|(data[0]));
    mag_xyz->y = (float)(uint16_t)((data[3] << 8)|(data[2]));
    mag_xyz->z = (float)(uint16_t)((data[5] << 8)|(data[4]));
}

BNO055_FUNC_RETURN bno055_read_gyr_x(uint16_t* gyr_x){
    uint8_t data[2] = {0,0};
    bno055_readData(BNO055_MAG_DATA_Z_LSB, data, 2);
    *gyr_x = (uint16_t)((data[1] << 8)|(data[0]));
}
BNO055_FUNC_RETURN bno055_read_gyr_y(uint16_t* gyr_y){
    uint8_t data[2] = {0,0};
    bno055_readData(BNO055_MAG_DATA_Z_LSB, data, 2);
    *gyr_y = (uint16_t)((data[1] << 8)|(data[0]));
}
BNO055_FUNC_RETURN bno055_read_gyr_z(uint16_t* gyr_z){
    uint8_t data[2] = {0,0};
    bno055_readData(BNO055_MAG_DATA_Z_LSB, data, 2);
    *gyr_z = (uint16_t)((data[1] << 8)|(data[0]));
}
BNO055_FUNC_RETURN bno055_read_gyr_xyz(bno055_gyr_t* gyr_xyz){
    uint8_t data[6] = {0,0,0,0,0,0};
    bno055_readData(BNO055_MAG_DATA_X_LSB, data, 6);
    gyr_xyz->x = (float)(uint16_t)((data[1] << 8)|(data[0]));
    gyr_xyz->y = (float)(uint16_t)((data[3] << 8)|(data[2]));
    gyr_xyz->z = (float)(uint16_t)((data[5] << 8)|(data[4]));
}

BNO055_FUNC_RETURN bno055_read_euler_h(){
    
}
BNO055_FUNC_RETURN bno055_read_euler_r(){
    
}
BNO055_FUNC_RETURN bno055_read_euler_p(){
    
}
BNO055_FUNC_RETURN bno055_read_euler_hrp(){
    
}

BNO055_FUNC_RETURN bno055_read_quaternion_w(){
    
}
BNO055_FUNC_RETURN bno055_read_quaternion_x(){
    
}
BNO055_FUNC_RETURN bno055_read_quaternion_y(){
    
}
BNO055_FUNC_RETURN bno055_read_quaternion_z(){
    
}
BNO055_FUNC_RETURN bno055_read_quaternion_wxyz(){
    
}

BNO055_FUNC_RETURN bno055_read_linear_acc_x(){
    
}
BNO055_FUNC_RETURN bno055_read_linear_acc_y(){
    
}
BNO055_FUNC_RETURN bno055_read_linear_acc_z(){
    
}
BNO055_FUNC_RETURN bno055_read_linear_acc_xyz(){
    
}

BNO055_FUNC_RETURN bno055_read_gravity_x(){
    
}
BNO055_FUNC_RETURN bno055_read_gravity_y(){
    
}
BNO055_FUNC_RETURN bno055_read_gravity_z(){
    
}
BNO055_FUNC_RETURN bno055_read_gravity_xyz(){
    
}

BNO055_FUNC_RETURN bno055_get_acc_calib_status(){
    
}
BNO055_FUNC_RETURN bno055_get_mag_calib_status(){
    
}
BNO055_FUNC_RETURN bno055_get_gyr_calib_status(){
    
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