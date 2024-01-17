#ifndef BNO055_H
#define BNO055_H

#include <stdint.h>
#include "i2c.h"
#include "main.h"


#define BNO055_FUNC_RETURN uint8_t
#define ERROR_DEFAULT           (uint8_t)(0)

#define START_BYTE				0xAA
#define RESPONSE_BYTE			0xBB
#define ERROR_BYTE		  		0xEE

#define BNO055_I2C_ADDR_HI		0x29
#define BNO055_I2C_ADDR_LO		0x28

#define ERROR_WRITE_SUCCESS		0x01		// Everything worker as expected
#define ERROR_WRITE_FAIL		0x03		// Check connection, protocol settings and operation more of BNO055
#define ERROR_REGMAP_INV_ADDR	0x04		// Invalid register address
#define ERROR_REGMAP_WRITE_DIS	0x05		// Register is read-only
#define ERROR_WRONG_START_BYTE	0x06		// Check if the first byte
#define ERROR_BUS_OVERRUN_ERR	0x07		// Resend the command, BNO055 was not able to clear the receive buffer
#define ERROR_MAX_LEN_ERR		0x08		// Split the command, max fire size can be up to 128 bytes
#define ERROR_MIN_LEN_ERR		0x09		// Min length of data is less than 1
#define ERROR_RECV_CHAR_TIMEOUT	0x0A		// Decrease the waiting time between sending of two bytes of one frame

#define REG_WRITE				0x00
#define REG_READ				0x01


/** Datasheet registers **/

/** PAGE 0 **/

//Register definition start
#define BNO055_CHIP_ID          0x00
#define BNO055_ACC_ID           0x01
#define BNO055_MAG_ID           0x02
#define BNO055_GYR_ID          0x03
#define BNO055_SW_REV_ID_LSB    0x04
#define BNO055_SW_REV_ID_MSB    0x05
#define BNO055_BL_REV_ID        0x06
#define BNO055_PAGE_ID          0x07

//Accelerometer data register
#define BNO055_ACC_DATA_X_LSB   0x08
#define BNO055_ACC_DATA_X_MSB   0x09
#define BNO055_ACC_DATA_Y_LSB   0x0A
#define BNO055_ACC_DATA_Y_MSB   0x0B
#define BNO055_ACC_DATA_Z_LSB   0x0C
#define BNO055_ACC_DATA_Z_MSB   0x0D

//Magnetometer data register
#define BNO055_MAG_DATA_X_LSB   0x0E
#define BNO055_MAG_DATA_X_MSB   0x0F
#define BNO055_MAG_DATA_Y_LSB   0x10
#define BNO055_MAG_DATA_Y_MSB   0x11
#define BNO055_MAG_DATA_Z_LSB   0x12
#define BNO055_MAG_DATA_Z_MSB   0x13

//Gyroscope data register
#define BNO055_GYR_DATA_X_LSB   0x14
#define BNO055_GYR_DATA_X_MSB   0x15
#define BNO055_GYR_DATA_Y_LSB   0x16
#define BNO055_GYR_DATA_Y_MSB   0x17
#define BNO055_GYR_DATA_Z_LSB   0x18
#define BNO055_GYR_DATA_Z_MSB   0x19

//Euler data register
#define BNO055_EUL_HEADING_LSB  0x1A
#define BNO055_EUL_HEADING_MSB  0x1B
#define BNO055_EUL_ROLL_LSB     0x1C
#define BNO055_EUL_ROLL_MSB     0x1D
#define BNO055_EUL_PITCH_LSB    0x1E
#define BNO055_EUL_PITCH_MSB    0x1F

//Quaternion data register
#define BNO055_QUA_DATA_W_LSB   0x20
#define BNO055_QUA_DATA_W_MSB   0x21
#define BNO055_QUA_DATA_X_LSB   0x22
#define BNO055_QUA_DATA_X_MSB   0x23
#define BNO055_QUA_DATA_Y_LSB   0x24
#define BNO055_QUA_DATA_Y_MSB   0x25
#define BNO055_QUA_DATA_Z_LSB   0x26
#define BNO055_QUA_DATA_Z_MSB   0x27

//Linear acceleration data register
#define BNO055_LIA_DATA_X_LSB   0x28
#define BNO055_LIA_DATA_X_MSB   0x29
#define BNO055_LIA_DATA_Y_LSB   0x2A
#define BNO055_LIA_DATA_Y_MSB   0x2B
#define BNO055_LIA_DATA_Z_LSB   0x2C
#define BNO055_LIA_DATA_Z_MSB   0x2D

//Gravity data register
#define BNO055_GRV_DATA_X_LSB   0x2E
#define BNO055_GRV_DATA_X_MSB   0x2F
#define BNO055_GRV_DATA_Y_LSB   0x30
#define BNO055_GRV_DATA_Y_MSB   0x31
#define BNO055_GRV_DATA_Z_LSB   0x32
#define BNO055_GRV_DATA_Z_MSB   0x33

//Temperature data register
#define BNO055_TEMP             0x34

//Status registers
#define BNO055_CALIB_STAT       0x35
#define BNO055_ST_RESULT        0x36
#define BNO055_INT_STATUS       0x37
#define BNO055_SYS_CLK_STATUS   0x38
#define BNO055_SYS_STATUS       0x39
#define BNO055_SYS_ERR          0x3A

//Unit selection register
#define BNO055_UNIT_SEL         0x3B

//Mode register
#define BNO055_OPR_MODE         0x3D
#define BNO055_PWR_MODE         0x3E
#define BNO055_SYS_TRIGGER      0x3F
#define BNO055_TEMP_SOURCE      0x40

//Axis remap register
#define BNO055_AXIS_MAP_CONFIG  0x41
#define BNO055_AXIS_MAP_SIGN    0x42

//Matrix registers
#define BNO055_SIC_MATRIX_LSB0 0x43
#define BNO055_SIC_MATRIX_MSB0 0x44
#define BNO055_SIC_MATRIX_LSB1 0x45
#define BNO055_SIC_MATRIX_MSB1 0x46
#define BNO055_SIC_MATRIX_LSB2 0x47
#define BNO055_SIC_MATRIX_MSB2 0x48
#define BNO055_SIC_MATRIX_LSB3 0x49
#define BNO055_SIC_MATRIX_MSB3 0x4A
#define BNO055_SIC_MATRIX_LSB4 0x4B
#define BNO055_SIC_MATRIX_MSB4 0x4C
#define BNO055_SIC_MATRIX_LSB5 0x4D
#define BNO055_SIC_MATRIX_MSB5 0x4E
#define BNO055_SIC_MATRIX_LSB6 0x4F
#define BNO055_SIC_MATRIX_MSB6 0x50
#define BNO055_SIC_MATRIX_LSB7 0x51
#define BNO055_SIC_MATRIX_MSB7 0x52
#define BNO055_SIC_MATRIX_LSB8 0x53
#define BNO055_SIC_MATRIX_MSB8 0x54

//Accelerometer offset register
#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A

//Magnetometer offset register
#define BNO055_MAG_OFFSET_X_LSB 0x5B
#define BNO055_MAG_OFFSET_X_MSB 0x5C
#define BNO055_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_MAG_OFFSET_Z_MSB 0x60

//Gyroscope offset register
#define BNO055_GYR_OFFSET_X_LSB 0x61
#define BNO055_GYR_OFFSET_X_MSB 0x62
#define BNO055_GYR_OFFSET_Y_LSB 0x63
#define BNO055_GYR_OFFSET_Y_MSB 0x64
#define BNO055_GYR_OFFSET_Z_LSB 0x65
#define BNO055_GYR_OFFSET_Z_MSB 0x66

//Radius register
#define BNO055_ACC_RADIUS_LSB   0x67
#define BNO055_ACC_RADIUS_MSB   0x68
#define BNO055_MAG_RADIUS_LSB   0x69
#define BNO055_MAG_RADIUS_MSB   0x6A


/** PAGE 1 **/

//Page id register
#define BNO055_PAGE_ID          0x07

//config register
#define BNO055_ACC_CONFIG       0x08
#define BNO055_MAG_CONFIG       0x09
#define BNO055_GYRO_CONFIG_0    0x0A
#define BNO055_GYRO_CONFIG_1    0x0B
#define BNO055_ACC_SLEEP_CONFIG 0x0C
#define BNO055_GYR_SLEEP_CONFIG 0x0D
#define BNO055_INT_MSK          0x0F
#define BNO055_INT_EN           0x10

#define BNO055_ACC_AM_THRES     0x11
#define BNO055_ACC_INT_SETTINGS 0x12
#define BNO055_ACC_HG_DURATION  0x13
#define BNO055_ACC_HG_THRESH    0x14
#define BNO055_ACC_NM_THRESH    0x15
#define BNO055_ACC_NM_SET       0x16

#define BNO055_GYR_INT_SETTINGS 0x17
#define BNO055_GYR_HR_X_SET     0x18
#define BNO055_GYR_DUR_X        0x19
#define BNO055_GYR_HR_Y_SET     0x1A
#define BNO055_GYR_DUR_Y        0x1B
#define BNO055_GYR_HR_Z_SET     0x1C
#define BNO055_GYR_DUR_Z        0x1D
#define BNO055_GYR_AM_THRESH    0x1E
#define BNO055_GYR_AM_SET       0x1F

/** Power mode settings --> page 20 
    [BNO055_PWR_MODE]:xxxxxxxx
 **/
typedef enum {
    POWER_MODE_NORMAL = 0X00,
    POWER_MODE_LOWPOWER = 0X01,
    POWER_MODE_SUSPEND = 0X02
} bno055_powermode_t;

/** Operation mode settings  --> page 22 
    [BNO055_OPR_MODE]:xxxxxxxx
 **/
typedef enum {
    OPERATION_MODE_CONFIG = 0X00,
    OPERATION_MODE_ACCONLY = 0X01,
    OPERATION_MODE_MAGONLY = 0X02,
    OPERATION_MODE_GYRONLY = 0X03,
    OPERATION_MODE_ACCMAG = 0X04,
    OPERATION_MODE_ACCGYRO = 0X05,
    OPERATION_MODE_MAGGYRO = 0X06,
    OPERATION_MODE_AMG = 0X07,
    OPERATION_MODE_IMU = 0X08,
    OPERATION_MODE_COMPASS = 0X09,
    OPERATION_MODE_M4G = 0X0A,
    OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
    OPERATION_MODE_NDOF = 0X0C
} bno055_opmode_t;

/** Axis remap settings --> page 26 
    [BNO055_AXIS_MAP_CONFIG]:xxxxxxxx
 **/
typedef enum {
    AXIS_REMAP_CONFIG_P0 = 0x21,
    AXIS_REMAP_CONFIG_P1 = 0x24,
    AXIS_REMAP_CONFIG_P2 = 0x24,
    AXIS_REMAP_CONFIG_P3 = 0x21,
    AXIS_REMAP_CONFIG_P4 = 0x24,
    AXIS_REMAP_CONFIG_P5 = 0x21,
    AXIS_REMAP_CONFIG_P6 = 0x21,
    AXIS_REMAP_CONFIG_P7 = 0x24
} bno055_axis_remap_config_t;

/** Axis remap sign settings --> page 26 
    [BNO055_AXIS_MAP_SIGN]:xxxxxxxx
 **/
typedef enum {
    AXIS_REMAP_SIGN_P0 = 0x04,
    AXIS_REMAP_SIGN_P1 = 0x00,
    AXIS_REMAP_SIGN_P2 = 0x06,
    AXIS_REMAP_SIGN_P3 = 0x02,
    AXIS_REMAP_SIGN_P4 = 0x03,
    AXIS_REMAP_SIGN_P5 = 0x01,
    AXIS_REMAP_SIGN_P6 = 0x07,
    AXIS_REMAP_SIGN_P7 = 0x05
} bno055_axis_remap_sign_t;

/** Accelerometer configuration settings --> page 29 
    [BNO055_ACC_CONFIG]:xxxxxxxx
 **/
typedef enum{
    ACC_CONFIG_2G = 0x00,
    ACC_CONFIG_4G = 0x01,
    ACC_CONFIG_8G = 0x02,
    ACC_CONFIG_16G = 0x03,
} bno055_acc_g_range_config_t;

typedef enum{
    ACC_CONFIG_7_81Hz = 0x00,
    ACC_CONFIG_15_63Hz = 0x04,
    ACC_CONFIG_31_25Hz = 0x08,
    ACC_CONFIG_62_5Hz = 0x0C,
    ACC_CONFIG_125Hz = 0x10,
    ACC_CONFIG_250Hz = 0x14,
    ACC_CONFIG_500Hz = 0x18,
    ACC_CONFIG_1000Hz = 0x1C,
} bno055_acc_bandwidth_config_t;

typedef enum{
    ACC_CONFIG_NORMAL = 0x0,
    ACC_CONFIG_SUSPEND = 0x20,
    ACC_CONFIG_LOW_POWER_1 = 0x40,
    ACC_CONFIG_STANDBY = 0x60,
    ACC_CONFIG_LOW_POWER_2 = 0x80,
    ACC_CONFIG_DEEP_SUSPEND = 0xA0,
} bno055_acc_operation_mode_config_t;

/** Gyroscope configuration settings --> page 30 
    [BNO055_GYR_CONFIG_0]:xxxxxxxx
    [BNO055_GYR_CONFIG_1]:xxxxxxxx  //This for the operation mode
 **/
typedef enum{
    GYR_CONFIG_2000DPS = 0x00,
    GYR_CONFIG_1000DPS = 0x01,
    GYR_CONFIG_500DPS = 0x02,
    GYR_CONFIG_250DPS = 0x03,
    GYR_CONFIG_125DPS = 0x04,
} bno055_gyr_range_config_t;

typedef enum{
    GYR_CONFIG_523Hz = 0x00,
    GYR_CONFIG_230Hz = 0x08,
    GYR_CONFIG_116Hz = 0x10,
    GYR_CONFIG_47Hz = 0x18,
    GYR_CONFIG_23Hz = 0x20,
    GYR_CONFIG_12Hz = 0x28,
    GYR_CONFIG_64Hz = 0x30,
    GYR_CONFIG_32Hz = 0x38,
} bno055_gyr_bandwidth_config_t;

typedef enum{
    GYR_CONFIG_NORMAL = 0x0,
    GYR_CONFIG_FAST_POWER_UP = 0x01,
    GYR_CONFIG_DEEP_SUSPEND = 0x02,
    GYR_CONFIG_SUSPEND = 0x03,
    GYR_CONFIG_ADVANCED_POWERSAVE = 0x04,
} bno055_gyr_operation_mode_config_t;

/** Magnetometer configuration settings --> page 31 
    [BNO055_MAG_CONFIG]:xxxxxxxx
 **/
typedef enum{
    MAG_CONFIG_2Hz = 0x00,
    MAG_CONFIG_6Hz = 0x01,
    MAG_CONFIG_8Hz = 0x02,
    MAG_CONFIG_10Hz = 0x03,
    MAG_CONFIG_15Hz = 0x04,
    MAG_CONFIG_20Hz = 0x05,
    MAG_CONFIG_25Hz = 0x06,
    MAG_CONFIG_30Hz = 0x07,
} bno055_mag_data_output_rate_config_t;

typedef enum{
    MAG_CONFIG_LOW_POWER = 0x00,
    MAG_CONFIG_REGULAR = 0x08,
    MAG_CONFIG_ENHANCED_REGULAR = 0x10,
    MAG_CONFIG_HIGH_ACCURACY = 0x18,
} bno055_mag_operation_mode_config_t;

typedef enum{
    MAG_CONFIG_NORMAL = 0x0,
    MAG_CONFIG_SLEEP = 0x20,
    MAG_CONFIG_SUSPEND = 0x40,
    MAG_CONFIG_FORCE_MODE = 0x60,
} bno055_mag_power_mode_config_t;

/** Unit selection settings --> page 32
    [BNO055_UNIT_SEL]: xxxxxxxx
 **/

/** 
 * Table 3-11: Unit selection for BNO055 sensor
 * | Data                     | Units            | [Reg Addr]: Register Value   |
 * |--------------------------|------------------|-----------------------------|
 * | Acceleration             | m/s²             | [UNIT_SEL] : xxxxxxx0b      |
 * |                          | mg               | [UNIT_SEL] : xxxxxxx1b      |
 * | Linear Acceleration,     | m/s²             | [UNIT_SEL] : xxxxxxx0b      |
 * | Gravity vector           |                  |                             |
 * |                          |                  |                             |
 * | Magnetic Field Strength  | Micro Tesla      | NA                          |
 * | Angular Rate             | Dps              | [UNIT_SEL] : xxxxxxx0xb     |
 * |                          | Rps              | [UNIT_SEL] : xxxxxxx1xb     |
 * | Euler Angles             | Degrees          | [UNIT_SEL] : xxxxxx0xxb     |
 * |                          | Radians          | [UNIT_SEL] : xxxxxx1xxb     |
 * | Quaternion               | Quaternion units | NA                          |
 * | Temperature              | °C               | [UNIT_SEL] : xxx0xxxxb      |
 * |                          | °F               | [UNIT_SEL] : xxx1xxxxb      |
 *
 * Note: 'x' in the Register Value column represents a placeholder for actual bits.
 *       NA indicates that no register selection is needed for the unit.
 **/

typedef enum {
    ACCELERATION_M_S2 = 0x00, 
    ACCELERATION_MG = 0x01, 

    LINEAR_ACCELERATION_M_S2 = 0x00,

    ANGULAR_RATE_DPS = 0x00,
    ANGULAR_RATE_RPS = 0x02,

    EULER_ANGLES_DEG = 0x00,
    EULER_ANGLES_RAD = 0x04,

    TEMPERATURE_C = 0x00,
    TEMPERATURE_F = 0x10
    
} bno055_unit_selection_t;

/** Data output selection in fusion data settings --> page 32
    [BNO055_UNIT_SEL]: xxxxxxxx
 **/
typedef enum {
    WINDOWS = 0x00,
    ANDROID = 0x80
} bno055_data_output_format_t;

/** Data output selection in fusion data settings --> page 33
    
    [BNO055_ACC_OFFSET_X_LSB]: 0x00
    [BNO055_ACC_OFFSET_X_MSB]: 0x00
    [BNO055_ACC_OFFSET_Y_LSB]: 0x00
    [BNO055_ACC_OFFSET_Y_MSB]: 0x00
    [BNO055_ACC_OFFSET_Z_LSB]: 0x00
    [BNO055_ACC_OFFSET_Z_MSB]: 0x00

    [BNO055_MAG_OFFSET_X_LSB]: 0x00
    [BNO055_MAG_OFFSET_X_MSB]: 0x00
    [BNO055_MAG_OFFSET_Y_LSB]: 0x00
    [BNO055_MAG_OFFSET_Y_MSB]: 0x00
    [BNO055_MAG_OFFSET_Z_LSB]: 0x00
    [BNO055_MAG_OFFSET_Z_MSB]: 0x00

    [BNO055_GYR_OFFSET_X_LSB]: 0x00
    [BNO055_GYR_OFFSET_X_MSB]: 0x00
    [BNO055_GYR_OFFSET_Y_LSB]: 0x00
    [BNO055_GYR_OFFSET_Y_MSB]: 0x00
    [BNO055_GYR_OFFSET_Z_LSB]: 0x00
    [BNO055_GYR_OFFSET_Z_MSB]: 0x00
    
    [BNO055_ACC_RADIUS_LSB]: 0x00
    [BNO055_ACC_RADIUS_MSB]: 0x00

    [BNO055_MAG_RADIUS_LSB]: 0x00
    [BNO055_MAG_RADIUS_MSB]: 0x00

 **/
typedef struct {
  int16_t acc_offset_x; 
  int16_t acc_offset_y; 
  int16_t acc_offset_z; 

  int16_t mag_offset_x; 
  int16_t mag_offset_y; 
  int16_t mag_offset_z; 

  int16_t gyr_offset_x;
  int16_t gyr_offset_y;
  int16_t gyr_offset_z;

  int16_t acc_radius; 
  int16_t mag_radius; 

} bno055_offsets_t;

typedef struct
{
    float x; 
    float y;
    float z;
} bno055_acc_t;

typedef struct
{
    float x;
    float y;
    float z;
} bno055_mag_t;

typedef struct
{
    float x; 
    float y; 
    float z; 
} bno055_gyr_t;

typedef struct
{
    float h;
    float r;
    float p;
} bno055_euler_t;

typedef struct
{
    float w; 
    float x; 
    float y; 
    float z; 
} bno055_quaternion_t;

typedef struct
{
    float x;
    float y;
    float z;
} bno055_linear_acc_t;

typedef struct
{
    float x;
    float y;
    float z;
} bno055_gravity_t;

typedef struct{
    bno055_powermode_t                      pwr_mode;
    bno055_opmode_t                         op_mode;
    bno055_axis_remap_config_t              axis_remap_conf;
    bno055_axis_remap_sign_t                axis_remap_sign;
    bno055_acc_g_range_config_t             acc_g_range;
    bno055_acc_bandwidth_config_t           acc_bandwidth;
    bno055_acc_operation_mode_config_t      acc_operation_mode;
    bno055_gyr_range_config_t               gyr_range;
    bno055_gyr_bandwidth_config_t           gyr_bandwidth;
    bno055_gyr_operation_mode_config_t      gyr_op_mode;
    bno055_mag_data_output_rate_config_t    mag_data_rate;
    bno055_mag_operation_mode_config_t      mag_op_mode;
    bno055_mag_power_mode_config_t          mag_pwr_mode;
    bno055_unit_selection_t                 unit_sel;

} bno055_conf_t;

typedef struct{
    uint8_t chip_id; /**< chip_id of bno055 */
    uint16_t sw_rev_id; /**< software revision id of bno055 */
    uint8_t page_id; /**< page_id of bno055 */
    uint8_t acc_id; /**< accel revision id of bno055 */
    uint8_t mag_id; /**< mag revision id of bno055 */
    uint8_t gyr_id; /**< gyro revision id of bno055 */
    uint8_t bl_rev_id; /**< boot loader revision id of bno055 */
}  bno055_verification_t;

void bno055_set_i2c_handler(I2C_HandleTypeDef *);
uint8_t bno055_writeData(uint8_t *);
uint8_t bno055_readData(uint8_t, uint8_t *, uint8_t);
void bno055_delay(uint32_t);

BNO055_FUNC_RETURN bno055_init(bno055_conf_t *, bno055_verification_t *);

BNO055_FUNC_RETURN bno055_read_acc_x(uint16_t *);
BNO055_FUNC_RETURN bno055_read_acc_y(uint16_t *);
BNO055_FUNC_RETURN bno055_read_acc_z(uint16_t *);
BNO055_FUNC_RETURN bno055_read_acc_xyz(bno055_acc_t *);

BNO055_FUNC_RETURN bno055_read_mag_x(uint16_t *);
BNO055_FUNC_RETURN bno055_read_mag_y(uint16_t *);
BNO055_FUNC_RETURN bno055_read_mag_z(uint16_t *);
BNO055_FUNC_RETURN bno055_read_mag_xyz(bno055_mag_t *);

BNO055_FUNC_RETURN bno055_read_gyr_x(uint16_t *);
BNO055_FUNC_RETURN bno055_read_gyr_y(uint16_t *);
BNO055_FUNC_RETURN bno055_read_gyr_z(uint16_t *);
BNO055_FUNC_RETURN bno055_read_gyr_xyz(bno055_gyr_t *);

BNO055_FUNC_RETURN bno055_read_euler_h(uint16_t *);
BNO055_FUNC_RETURN bno055_read_euler_r(uint16_t *);
BNO055_FUNC_RETURN bno055_read_euler_p(uint16_t *);
BNO055_FUNC_RETURN bno055_read_euler_hrp(bno055_euler_t *);

BNO055_FUNC_RETURN bno055_read_quaternion_w(uint16_t *);
BNO055_FUNC_RETURN bno055_read_quaternion_x(uint16_t *);
BNO055_FUNC_RETURN bno055_read_quaternion_y(uint16_t *);
BNO055_FUNC_RETURN bno055_read_quaternion_z(uint16_t *);
BNO055_FUNC_RETURN bno055_read_quaternion_wxyz(bno055_quaternion_t *);

BNO055_FUNC_RETURN bno055_read_linear_acc_x(uint16_t *);
BNO055_FUNC_RETURN bno055_read_linear_acc_y(uint16_t *);
BNO055_FUNC_RETURN bno055_read_linear_acc_z(uint16_t *);
BNO055_FUNC_RETURN bno055_read_linear_acc_xyz(bno055_linear_acc_t *);

BNO055_FUNC_RETURN bno055_read_gravity_x(uint16_t *);
BNO055_FUNC_RETURN bno055_read_gravity_y(uint16_t *);
BNO055_FUNC_RETURN bno055_read_gravity_z(uint16_t *);
BNO055_FUNC_RETURN bno055_read_gravity_xyz(bno055_gravity_t *);

BNO055_FUNC_RETURN bno055_get_acc_calib_status();
BNO055_FUNC_RETURN bno055_get_mag_calib_status();
BNO055_FUNC_RETURN bno055_get_gyr_calib_status();

extern bno055_conf_t default_bno055_config;
extern bno055_verification_t default_bno055_verification;

#endif // BNO055_H
