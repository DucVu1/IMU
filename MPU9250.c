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
extern I2C_HandleTypeDef hi2c1;
int counter = 0;
int starter = 0;
double x_gyro_calibrate_para=0;
double y_gyro_calibrate_para=0;
double z_gyro_calibrate_para=0;
double x_gyro_cal;
double y_gyro_cal;
double z_gyro_cal;
double Mag_Cal_X;
double Mag_Cal_Y;
double Mag_Cal_Z;

int count=0 ;
void freeMatrix(double** matrix, int row) {
    for (int i = 0; i < row; i++) {
        free(matrix[i]);
    }
    free(matrix);
}
void mpu9250_calibrate_gyro( double x_gyror,double y_gyror, double z_gyror){
	if (count == 0){
		x_gyro_cal=0;
		y_gyro_cal=0;
		z_gyro_cal=0;
	}
	x_gyro_cal += x_gyror;
	y_gyro_cal += y_gyror;
	z_gyro_cal += z_gyror;
	count=count+1;
    if (count == 100) {
        x_gyro_calibrate_para = x_gyro_cal /count;
        y_gyro_calibrate_para = y_gyro_cal / count;
        z_gyro_calibrate_para = z_gyro_cal / count;
    }
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
 HAL_StatusTypeDef	ret = HAL_I2C_IsDeviceReady(&hi2c1, (Device_Address<<1)+0, 2, 100);
	if(ret !=HAL_OK){
		printf("The device is not ready. Check again\n");
	}

//congfic accelerometer
uint8_t temp_data =FS_ACC_2G;
HAL_StatusTypeDef ret2 = HAL_I2C_Mem_Write(&hi2c1, (Device_Address<<1)+0, REG_CONFIG_ACC, 1, &temp_data, 1, 100);
	if(ret2 !=HAL_OK){
			printf("The device accelerometer is not written. Check again\n");
		}

//config gyroscope
temp_data =FS_GYRO_250;
HAL_StatusTypeDef ret3 = HAL_I2C_Mem_Write(&hi2c1, (Device_Address<<1)+0, REG_CONFIG_GYRO, 1, &temp_data, 1, 100);
	if(ret3 !=HAL_OK){
			printf("The device gyroscope is not written. Check again\n");
		}
//config power management register
temp_data = 0;
HAL_StatusTypeDef ret4 = HAL_I2C_Mem_Write(&hi2c1, (Device_Address<<1)+0, REG_CONFIG_PWR1, 1, &temp_data, 1, 100);
	if(ret4 !=HAL_OK){
			printf("The device fail exiting sleep mode. Check again\n");
	}

}



//Accelerometer and Gyroscope
void mpu9250_read(){
	counter = counter+1;
	if(counter ==100){
		starter =1;
	}

	float Scale_Constant_Acc=16384.0;
	float Scale_Constant_Gyro = 131.0;
	float Scale_Constant_Mag =1 ;

	// declare variables
	uint8_t acc_mea_x[2],acc_mea_y[2],acc_mea_z[2],gyro_mea_x[2],gyro_mea_y[2],gyro_mea_z[2];
	uint8_t mag_mea_x[2],mag_mea_y[2],mag_mea_z[2];
	int16_t x_acc,z_acc,y_acc,x_gyro,y_gyro,z_gyro,x_mag,y_mag,z_mag;

	 //Read data from Accelerometer and Gyroscope

	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, ACC_X_H, 1, &acc_mea_x[0], 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, ACC_X_L, 1, &acc_mea_x[1], 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, GYRO_X_H, 1, &gyro_mea_x[0], 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, GYRO_X_L, 1, &gyro_mea_x[1], 1, 100);
	x_acc = ((int16_t)acc_mea_x[0]<<8)+acc_mea_x[1];
	x_gyro = ((int16_t)gyro_mea_x[0]<<8)+gyro_mea_x[1];

	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, ACC_Y_H, 1, &acc_mea_y[0], 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, ACC_Y_L, 1, &acc_mea_y[1], 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, GYRO_Y_H, 1, &gyro_mea_y[0], 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, GYRO_Y_L, 1, &gyro_mea_y[1], 1, 100);
	y_acc = ((int16_t)acc_mea_y[0]<<8)+acc_mea_y[1];
	y_gyro = ((int16_t)gyro_mea_y[0]<<8)+gyro_mea_y[1];

	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, ACC_Z_H, 1, &acc_mea_z[0], 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, ACC_Z_L, 1, &acc_mea_z[1], 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, GYRO_Z_H, 1, &gyro_mea_z[0], 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, (Device_Address<<1)+0, GYRO_Z_L, 1, &gyro_mea_z[1], 1, 100);
	z_acc = ((int16_t)acc_mea_z[0]<<8)+acc_mea_z[1];
	z_gyro = ((int16_t)gyro_mea_z[0]<<8)+gyro_mea_z[1];


	double x_accr = x_acc/Scale_Constant_Acc;
	double z_accr = z_acc/Scale_Constant_Acc;
	double y_accr = y_acc/Scale_Constant_Acc;
	double x_gyror = x_gyro/Scale_Constant_Gyro;
	double z_gyror = z_gyro/Scale_Constant_Gyro;
	double y_gyror = y_gyro/Scale_Constant_Gyro;

	//Calibrate data

	double **calibrated_accelerometer = mpu9250_calibrate_accel((double)x_accr,(double)y_accr,(double)z_accr);
	mpu9250_calibrate_gyro(x_gyror,y_gyror,z_gyror);

    printf("Calibrated acc: %.5f  ", calibrated_accelerometer[0][0]*9.8);
    printf(" %.5f  ", calibrated_accelerometer[1][0]*9.8);
    printf(" %.5f    ", calibrated_accelerometer[2][0]*9.8);
	if(starter ==1){
	    printf("Calibrated gyro: %.5f  ", x_gyror-x_gyro_calibrate_para);
	    printf(" %.5f  ", y_gyror-y_gyro_calibrate_para);
	    printf(" %.5f  \n", z_gyror-z_gyro_calibrate_para);
	}
	else{
	    printf(" %.5f   ", x_gyror);
	    printf(" %.5f   ", y_gyror);
	    printf(" %.5f   \n", z_gyror);
	}
}
