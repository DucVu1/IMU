
/*
 * MPU9250.h
 *
 *  Created on: Jul 20, 2023
 *      Author: Duc
 */

#ifndef SRC_MPU9250_H_
#define SRC_MPU9250_H_
//Mathematical value
#define PI 3.141592654
#define Mu 0.01
#define Time_constant 0.00025 //4000Hz frequency
#define Time_constant_filter 0.5 //should inspect to get a better number (read Low cost IMU implementation)
#define g 9.8
#define trust 0.95
#define cutoff 0.95
#define Counter_limit 100000

//checking register
#define Device_WAI 0x75 // return should be 0x71 or 113
#define AK8963_WAI 0x0   // return should be 0x48 or 72
#define Device_Address 0x68
//Gyroscope register and Parameter
#define FS_GYRO_250 0
#define FS_GYRO_500 8
#define FS_GYRO_1000 16
#define FS_GYRO_2000 24
#define GYRO_X_H 67
#define GYRO_X_L 68
#define GYRO_Y_H 69
#define GYRO_Y_L 70
#define GYRO_Z_H 71
#define GYRO_Z_L 72
#define REG_CONFIG_GYRO 27
#define Scale_Constant_Gyro 131.0
//accelerometer register and Parameter
#define FS_ACC_2G 0
#define FS_ACC_4G 8
#define FS_ACC_8G 16
#define FS_ACC_16G 24
#define ACC_X_H 59
#define ACC_X_L 60
#define ACC_Y_H 61
#define ACC_Y_L 62
#define ACC_Z_H 63
#define ACC_Z_L 64
#define REG_CONFIG_ACC 28
#define Scale_Constant_Acc 16384.0

#define REG_CONFIG_PWR1 107
#define USER_CTRL 106
#define General_Timeout 100


#define TEMP_H 65
#define TEMP_L 66

// magnetometer register
#define I2C_Master_Control 36
#define I2C_Master_Control_Config 0x5D
#define USER_CTRL_Config 0x00
#define RST_I2C_Master 0x02
#define I2C_Master_Interface_Enable 0x20
#define Bypass_Config 0x02
#define Bypass_Config_Off 0x0
#define REG_Bypass 55
#define Mag_setup 0x1F
#define CTRL_1 0x0A
#define CTRL_1_Config_FuseRom 0x1F //fuse ROM mode:cause we need to read the adjustment sensitivity and 16 bit output
#define CTRL_1_Config_Continuous1  0x12 // continuous mode 1 sensor is measured at 8Hz
#define CTRL_1_Config_Continuous2 0x16	//continuous mode 2 sensor is measured at 100Hz
#define MAG_X_L 3
#define MAG_X_H 4
#define MAG_Y_L 5
#define MAG_Y_H 6
#define MAG_Z_L 7
#define MAG_Z_H 8
#define ASAX 10
#define ASAY 11
#define ASAZ 12
#define REG_ST1 2
#define REG_ST2 9
#define AK8963_Address 0x0C

//Slave Configuration
//Slave0
#define I2C_SLV0_Dataout 99
#define I2C_SLV0_ADDR 37
#define I2C_SLV0_REG 38 //the content will be the register address that we will read from AK8963
#define I2C_SLV0_CTRL 39
#define I2C_SLV_ADDR_Config 0b10001100
#define I2C_SLV_CTRL_Config 0b10000001
//Slave1
#define I2C_SLV1_Dataout 100
#define I2C_SLV1_ADDR 40
#define I2C_SLV1_REG 41 //the content will be the register address that we will read from AK8963
#define I2C_SLV1_CTRL 42
//Slave2
#define I2C_SLV2_Dataout 101
#define I2C_SLV2_ADDR 43
#define I2C_SLV2_REG 44 //the content will be the register address that we will read from AK8963
#define I2C_SLV2_CTRL 45



void mpu9250_init();
void mpu9250_read();
void magnetometer_init();
void mpu9250_calibrate_gyro();
double** mpu9250_calibrate_accel();
double** mpu9250_calibrate_magneto();
double** Multiply_Mag();
double** Multiply();
void mpu9250_angel();
void Complimentary_filter();
void Lowpass_filter();
#endif /* SRC_MPU9250_H_ */
