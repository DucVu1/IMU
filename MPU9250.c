/*
 * MPU9250.c
 *
 *  Created on: Jul 20, 2023
 *      Author: Duc
 */

#include "MPU9250.h"
#include "main.h"
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "math.h"
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
int starter = 0;
double x_gyro_calibrate_para=0;
double y_gyro_calibrate_para=0;
double z_gyro_calibrate_para=0;
void freeMatrix(double** matrix, int row) {
    for (int i = 0; i < row; i++) {
        free(matrix[i]);
    }
    free(matrix);
}
void mpu9250_calibrate_gyro( double x_gyror,double y_gyror, double z_gyror){
	static double x_gyro_cal, y_gyro_cal, z_gyro_cal;
	static int count=0;
	if (count == 0){
		x_gyro_cal=0;
		y_gyro_cal=0;
		z_gyro_cal=0;
	}
	x_gyro_cal += x_gyror;
	y_gyro_cal += y_gyror;
	z_gyro_cal += z_gyror;
	count=count+1;
    if (count == 200) {
        x_gyro_calibrate_para = x_gyro_cal /count;
        y_gyro_calibrate_para = y_gyro_cal / count;
        z_gyro_calibrate_para = z_gyro_cal / count;
    }
}
double** mpu9250_calibrate_magneto(double x_magr,double y_magr, double z_magr){
	 double **measurement_matrix = (double**)malloc(3 * sizeof(double*));
	    for(int i=0;i<3;i++){
	    measurement_matrix[i] = (double*)malloc(1 * sizeof(double));}

	    // Update the array indexing and calibration values accordingly
	    measurement_matrix[0][0] = x_magr + 19.026427;
	    measurement_matrix[1][0] = y_magr - 90.319273;
	    measurement_matrix[2][0] = z_magr - 90.319273;

	    double **calibrated_matrix = Multiply_Mag(measurement_matrix, 3, 1, 3);
	    freeMatrix(measurement_matrix, 3); // Free the memory allocated for measurement_matrix
	    return calibrated_matrix;
}
double** Multiply_Mag(double** matrix1, int row, int column1, int column2) {
    double calibrate_parameter[3][3] = {
        {0.207995, 0.032704, 0.001643},
        {0.032704, 0.180287, 0.003490},
        {0.001643, 0.003490, 0.264669}
    };

    // Allocate memory for the resulting matrix
    double** multiplied_matrix = (double**)malloc(row * sizeof(double*));
    for (int i = 0; i < row; i++) {
        multiplied_matrix[i] = (double*)malloc(column2 * sizeof(double));
    }

    // Perform matrix multiplication
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < column2; j++) {
            multiplied_matrix[i][j] = 0;
            for (int k = 0; k < column1; k++) {
                multiplied_matrix[i][j] += matrix1[i][k] * calibrate_parameter[k][j];
            }
        }
    }

    return multiplied_matrix;
}
double** mpu9250_calibrate_accel(double x_accr, double y_accr, double z_accr) {
    double **measurement_matrix = (double**)malloc(3 * sizeof(double*));
    for(int i=0;i<3;i++){
    measurement_matrix[i] = (double*)malloc(1 * sizeof(double));}

    // Update the array indexing and calibration values accordingly
    measurement_matrix[0][0] = x_accr - 0.038108;
    measurement_matrix[1][0] = y_accr - 0.011325;
    measurement_matrix[2][0] = z_accr + 0.047890;

    double **calibrated_matrix = Multiply(measurement_matrix, 3, 1, 3);
    freeMatrix(measurement_matrix, 3); // Free the memory allocated for measurement_matrix
    return calibrated_matrix;
}

double** Multiply(double** matrix1, int row, int column1, int column2) {
    double calibrate_parameter[3][3] = {
        {0.979159, 0.057901, -0.011943},
        {0.057901, 1.010893, 0.006137},
        {-0.011943, 0.006137, 0.996744}
    };

    // Allocate memory for the resulting matrix
    double** multiplied_matrix = (double**)malloc(row * sizeof(double*));
    for (int i = 0; i < row; i++) {
        multiplied_matrix[i] = (double*)malloc(column2 * sizeof(double));
    }

    // Perform matrix multiplication
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < column2; j++) {
            multiplied_matrix[i][j] = 0;
            for (int k = 0; k < column1; k++) {
                multiplied_matrix[i][j] += matrix1[i][k] * calibrate_parameter[k][j];
            }
        }
    }

    return multiplied_matrix;
}

void mpu9250_init(){
	//check if the device is connected
	 HAL_StatusTypeDef	ret = HAL_I2C_IsDeviceReady(&hi2c1, (Device_Address<<1)+0, 2, General_Timeout);
		if(ret !=HAL_OK){
			printf("The device is not ready. Check again\n");
		}

	//congfic accelerometer
	uint8_t temp_data =FS_ACC_2G;
	HAL_StatusTypeDef ret2 = HAL_I2C_Mem_Write(&hi2c1, (Device_Address<<1)+0, REG_CONFIG_ACC, 1, &temp_data, 1, General_Timeout);
		if(ret2 !=HAL_OK){
				printf("The device accelerometer is not written. Check again\n");
			}

	//config gyroscope
	temp_data =FS_GYRO_250;
	HAL_StatusTypeDef ret3 = HAL_I2C_Mem_Write(&hi2c1, (Device_Address<<1)+0, REG_CONFIG_GYRO, 1, &temp_data, 1, General_Timeout);
		if(ret3 !=HAL_OK){
				printf("The device gyroscope is not written. Check again\n");
			}
	//config power management register
	temp_data = 0;
	HAL_StatusTypeDef ret4 = HAL_I2C_Mem_Write(&hi2c1, (Device_Address<<1)+0, REG_CONFIG_PWR1, 1, &temp_data, 1, General_Timeout);
		if(ret4 !=HAL_OK){
				printf("The device fail exiting sleep mode. Check again\n");
		}
		magnetometer_init();
}

//magnetomert configuration
void magnetometer_init(){
	uint8_t temp_data;
	//Turn off Sensor Master I2C Interface using the USER_CTRL Register
	temp_data =USER_CTRL_Config;
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&hi2c1, (Device_Address<<1)+0, USER_CTRL, 1, &temp_data, 1, General_Timeout);
		if(ret !=HAL_OK){
				printf("The device Sensor I2C master interface is not disabled. Check again\n");
			}
	//Turn on Bypass Register
	temp_data =Bypass_Config;
	HAL_StatusTypeDef ret1 = HAL_I2C_Mem_Write(&hi2c1, (Device_Address<<1)+0, REG_Bypass, 1, &temp_data, 1, General_Timeout);
		if(ret1 !=HAL_OK){
				printf("The device Bypass register is not enabled. Check again\n");
			}
	//Configure mode for the magnetometer (Continuous2)
	temp_data =CTRL_1_Config_Continuous2;
	HAL_StatusTypeDef ret2 = HAL_I2C_Mem_Write(&hi2c1, (AK8963_Address<<1)+0, CTRL_1, 1, &temp_data, 1, General_Timeout);
		if(ret2 !=HAL_OK){
				printf("The device Magnetometer mode is not configured yet. Check again\n");
			}

}
void Complimentary_filter(double*roll, double*pitch, double*yaw, double acc_roll,double acc_pitch,double gyro_roll,double gyro_pitch, double time)
{
	double alpha;
	static int counter = 0;
	alpha = Time_constant_filter/(Time_constant_filter+time);
	if(counter ==0){
		*roll = 0; // This mean that we assume the object is at rest during the first measure
		*pitch = 0;
		counter =1;
	}
	*roll = alpha * (*roll + gyro_roll)-(1-alpha)*acc_roll;
	*pitch = alpha * (*pitch + gyro_pitch)-(1-alpha)*acc_pitch;
}
void mpu9250_angel(double accx, double accy, double accz,double gyrox,double gyroy,double gyroz,double* roll,double* pitch, double* yaw, double*acc_roll, double*acc_pitch, double*gyro_roll, double*gyro_pitch, int timer_val){
	int sign;
	static double previous_angle_roll_gyro = 0;
	static double previous_angle_pitch_gyro = 0;
	double time = timer_val*Time_constant;
	if (accz>0){
		sign = 1;
	}
	else{
		sign = -1;
	}
    // Calculate roll angle
    *acc_roll = atan2(accy, sign*sqrt(pow(accx, 2) + pow(accz, 2)));
    // Calculate pitch angle
    *acc_pitch = atan2(-accx, sqrt(pow(accy, 2) + pow(accz, 2)));
    if(starter == 1){
    	*gyro_roll = gyrox * time + previous_angle_roll_gyro;
    	*gyro_pitch = gyroy * time + previous_angle_pitch_gyro;
    	previous_angle_roll_gyro = *gyro_roll;
    	previous_angle_pitch_gyro = *gyro_pitch;
    	Complimentary_filter(roll, pitch,yaw,*acc_roll,*acc_pitch, *gyro_roll, *gyro_pitch, time);
    	}
}
//Accelerometer and Gyroscope and Magnetometer read
void mpu9250_read(uint32_t previous_time){
	uint32_t current_time;
 	static int counter = 0;
	counter = counter+1;
	if(counter ==200){
		starter =1;
	}
	double roll, pitch, yaw, roll_acc, pitch_acc, roll_gyro, pitch_gyro;

	// declare variables
	uint8_t acc_mea_x[2],acc_mea_y[2],acc_mea_z[2],gyro_mea_x[2],gyro_mea_y[2],gyro_mea_z[2];
	uint8_t readData,mag_mea_x[2],mag_mea_y[2],mag_mea_z[2],overflow_check;
	int16_t x_acc,z_acc,y_acc,x_gyro,y_gyro,z_gyro,x_mag,y_mag,z_mag;

	 //Read data from Accelerometer and Gyroscope

	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, ACC_X_H, 1, &acc_mea_x[0], 1, General_Timeout);
	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, ACC_X_L, 1, &acc_mea_x[1], 1, General_Timeout);
	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, GYRO_X_H, 1, &gyro_mea_x[0], 1, General_Timeout);
	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, GYRO_X_L, 1, &gyro_mea_x[1], 1, General_Timeout);
	x_acc = ((int16_t)acc_mea_x[0]<<8)+acc_mea_x[1];
	x_gyro = ((int16_t)gyro_mea_x[0]<<8)+gyro_mea_x[1];

	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, ACC_Y_H, 1, &acc_mea_y[0], 1, General_Timeout);
	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, ACC_Y_L, 1, &acc_mea_y[1], 1, General_Timeout);
	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, GYRO_Y_H, 1, &gyro_mea_y[0], 1, General_Timeout);
	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, GYRO_Y_L, 1, &gyro_mea_y[1], 1, General_Timeout);
	y_acc = ((int16_t)acc_mea_y[0]<<8)+acc_mea_y[1];
	y_gyro = ((int16_t)gyro_mea_y[0]<<8)+gyro_mea_y[1];

	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, ACC_Z_H, 1, &acc_mea_z[0], 1, General_Timeout);
	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, ACC_Z_L, 1, &acc_mea_z[1], 1, General_Timeout);
	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, GYRO_Z_H, 1, &gyro_mea_z[0], 1, General_Timeout);
	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, GYRO_Z_L, 1, &gyro_mea_z[1], 1, General_Timeout);
	z_acc = ((int16_t)acc_mea_z[0]<<8)+acc_mea_z[1];
	z_gyro = ((int16_t)gyro_mea_z[0]<<8)+gyro_mea_z[1];

	HAL_I2C_Mem_Read(&hi2c1, (AK8963_Address<<1)+0, REG_ST1, 1, &readData, 1, General_Timeout);
	if( (readData & 0x01) == 0x01 ){
		HAL_I2C_Mem_Read(&hi2c1, (AK8963_Address<<1)+0, MAG_X_L, 1, &mag_mea_x[0], 1, General_Timeout);
		HAL_I2C_Mem_Read(&hi2c1, (AK8963_Address<<1)+0, MAG_X_H, 1, &mag_mea_x[1], 1, General_Timeout);
		HAL_I2C_Mem_Read(&hi2c1, (AK8963_Address<<1)+0, MAG_Y_L, 1, &mag_mea_x[0], 1, General_Timeout);
		HAL_I2C_Mem_Read(&hi2c1, (AK8963_Address<<1)+0, MAG_Y_H, 1, &mag_mea_y[1], 1, General_Timeout);
		HAL_I2C_Mem_Read(&hi2c1, (AK8963_Address<<1)+0, MAG_Z_L, 1, &mag_mea_z[0], 1, General_Timeout);
		HAL_I2C_Mem_Read(&hi2c1, (AK8963_Address<<1)+0, MAG_Z_H, 1, &mag_mea_z[1], 1, General_Timeout);
		HAL_I2C_Mem_Read(&hi2c1, (AK8963_Address<<1)+0, REG_ST2, 1, &overflow_check, 1, General_Timeout);

		if(!(overflow_check & 0x08)) {
		   x_mag = ((int16_t)mag_mea_x[1] << 8) | mag_mea_x[0];
		   y_mag = ((int16_t)mag_mea_y[1] << 8) | mag_mea_y[0];
		   z_mag = ((int16_t)mag_mea_z[1] << 8) | mag_mea_z[0];
		}
		else {
			printf("\r\n");
		}
	}
	//Scale to the desire (not calibrated)

	double x_accr = x_acc/Scale_Constant_Acc;
	double z_accr = z_acc/Scale_Constant_Acc;
	double y_accr = y_acc/Scale_Constant_Acc;
	double x_gyror = x_gyro/Scale_Constant_Gyro;
	double z_gyror = z_gyro/Scale_Constant_Gyro;
	double y_gyror = y_gyro/Scale_Constant_Gyro;
	//Calibrate data

	double **calibrated_magnetometer = mpu9250_calibrate_magneto((double)x_mag,(double)y_mag,(double)z_mag);
	double **calibrated_accelerometer = mpu9250_calibrate_accel((double)x_accr,(double)y_accr,(double)z_accr);
	mpu9250_calibrate_gyro(x_gyror,y_gyror,z_gyror);
	double x_gyro_calibrated = x_gyror- x_gyro_calibrate_para;
	double y_gyro_calibrated = y_gyror- y_gyro_calibrate_para;
	double z_gyro_calibrated = z_gyror- z_gyro_calibrate_para;
	current_time = abs(__HAL_TIM_GET_COUNTER(&htim2) - previous_time);
	mpu9250_angel(calibrated_accelerometer[0][0], calibrated_accelerometer[1][0],calibrated_accelerometer[2][0],x_gyro_calibrated,y_gyro_calibrated,z_gyro_calibrated,&roll,&pitch,&yaw, &roll_acc, &pitch_acc, &roll_gyro, &pitch_gyro, current_time);
	//recalculate the angle to degree
	roll_acc = (roll_acc/PI)*180;
	pitch_acc = (pitch_acc/PI)*180;
	//print angel data
	printf("Timer: %.5f",(double)current_time*Time_constant);
	printf("Roll_acc: %.5f  ",roll_acc);
	printf("Pitch_acc: %.5f  \n",pitch_acc);
	if(starter == 1){
	printf("Roll_gyro: %.5f  ",roll_gyro);
	printf("Pitch_gyro: %.5f  \n",pitch_gyro);
	printf("Roll: %.5f  ",roll);
	printf("Pitch %.5f  \n",pitch);
	}

	//print raw data
    printf("Calibrated acc: %.5f ", calibrated_accelerometer[0][0]*g);
    printf(" %.5f  ", calibrated_accelerometer[1][0]*g);
    printf(" %.5f  ", calibrated_accelerometer[2][0]*g);
	if(starter ==1){
	    printf("Calibrated gyro: %.5f  ", x_gyro_calibrated);
	    printf(" %.5f   ", y_gyro_calibrated);
	    printf(" %.5f   ", z_gyro_calibrated);
	}
	else{
	    printf(" %.5f   ", x_gyror);
	    printf(" %.5f   ", y_gyror);
	    printf(" %.5f   ", z_gyror);
	}
    printf("Calibrated mag: %.5f  ", calibrated_magnetometer[0][0]);
    printf(" %.5f  ", calibrated_magnetometer[1][0]);
    printf(" %.5f    \n", calibrated_magnetometer[2][0]);
}
