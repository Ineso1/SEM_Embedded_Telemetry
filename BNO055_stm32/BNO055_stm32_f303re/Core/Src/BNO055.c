#include "BNO055.h"

BNO055_FUNC_RETURN bno055_init(bno055_conf_t * bno055_conf){
    uint8_t pwr_mode [2] = {BNO055_PWR_MODE, bno055_conf->pwr_mode};
    uint8_t op_mode [2] = {BNO055_OPR_MODE, bno055_conf->op_mode};
    uint8_t axis_remap_conf [2] = {BNO055_AXIS_MAP_CONFIG, bno055_conf->axis_remap_conf};
    uint8_t axis_remap_sign [2] = {BNO055_AXIS_MAP_SIGN, bno055_conf->axis_remap_sign};

    uint8_t acc_conf [2] = {BNO055_ACC_CONFIG, bno055_conf->acc_operation_mode << 5 | bno055_conf->acc_bandwidth << 2 | bno055_conf->acc_g_range };
    uint8_t gyr_conf0 [2] = {BNO055_GYRO_CONFIG_0, bno055_conf->gyr_bandwidth << 3 | bno055_conf->gyr_range };
    uint8_t gyr_conf1 [2] = {BNO055_GYRO_CONFIG_1, bno055_conf->gyr_op_mode };
    uint8_t mag_conf [2] = {BNO055_MAG_CONFIG, bno055_conf->mag_pwr_mode << 5 | bno055_conf->mag_op_mode << 3 | bno055_conf->mag_data_rate };

    uint8_t unit_sel [2] = {BNO055_OPR_MODE, bno055_conf->unit_sel};



}

BNO055_FUNC_RETURN bno055_write_register(){
    
}
BNO055_FUNC_RETURN bno055_read_register(){
    
}

BNO055_FUNC_RETURN bno055_read_acc_x(){
    
}
BNO055_FUNC_RETURN bno055_read_acc_y(){
    
}
BNO055_FUNC_RETURN bno055_read_acc_z(){
    
}
BNO055_FUNC_RETURN bno055_read_acc_xyz(){
    
}

BNO055_FUNC_RETURN bno055_read_mag_x(){
    
}
BNO055_FUNC_RETURN bno055_read_mag_y(){
    
}
BNO055_FUNC_RETURN bno055_read_mag_z(){
    
}
BNO055_FUNC_RETURN bno055_read_mag_xyz(){
    
}

BNO055_FUNC_RETURN bno055_read_gyr_x(){
    
}
BNO055_FUNC_RETURN bno055_read_gyr_y(){
    
}
BNO055_FUNC_RETURN bno055_read_gyr_z(){
    
}
BNO055_FUNC_RETURN bno055_read_gyr_xyz(){
    
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
