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
#include "Kalman_filter.h"

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;

int starter = 0;
double x_gyro_calibrate_para = 0;
double y_gyro_calibrate_para = 0;
double z_gyro_calibrate_para = 0;
Kalman kalman_roll;
Kalman kalman_pitch;

void freeMatrix(double** matrix, int row) {
    for (int i = 0; i < row; i++) {
        free(matrix[i]);
    }
    free(matrix);
}

void mpu9250_calibrate_gyro(double x_gyror, double y_gyror, double z_gyror) {
    static double x_gyro_cal = 0;
    static double y_gyro_cal = 0;
    static double z_gyro_cal = 0;
    static int count = 0;

    if (count == 0) {
        x_gyro_cal = 0;
        y_gyro_cal = 0;
        z_gyro_cal = 0;
    }

    x_gyro_cal += x_gyror;
    y_gyro_cal += y_gyror;
    z_gyro_cal += z_gyror;
    count = count + 1;

    if (count == Sampling) {
        x_gyro_calibrate_para = x_gyro_cal / count;
        y_gyro_calibrate_para = y_gyro_cal / count;
        z_gyro_calibrate_para = z_gyro_cal / count;
    }
}

double** mpu9250_calibrate_magneto(double x_magr, double y_magr, double z_magr) {
    double **measurement_matrix = (double**)malloc(3 * sizeof(double*));
    for (int i = 0; i < 3; i++) {
        measurement_matrix[i] = (double*)malloc(1 * sizeof(double));
    }
    double **calibrate_parameter = (double**)malloc(3 * sizeof(double*));
     for (int i = 0; i < 3; i++) {
    	 calibrate_parameter[i] = (double*)malloc(1 * sizeof(double));
     }
	calibrate_parameter[0][0] = 0.207995;
	calibrate_parameter[0][1] = 0.032704;
	calibrate_parameter[0][2] = 0.001643;

	calibrate_parameter[1][0] = 0.032704;
	calibrate_parameter[1][1] = 0.180287;
	calibrate_parameter[1][2] = 0.003490;

	calibrate_parameter[2][0] = 0.001643;
	calibrate_parameter[2][1] = 0.003490;
	calibrate_parameter[2][2] = 0.264669;
    // Update the array indexing and calibration values accordingly
    measurement_matrix[0][0] = x_magr + 19.026427;
    measurement_matrix[1][0] = y_magr - 90.319273;
    measurement_matrix[2][0] = z_magr - 90.319273;

    double **calibrated_matrix = Multiply(measurement_matrix,calibrate_parameter, 3, 3, 1);
    freeMatrix(measurement_matrix, 3); // Free the memory allocated for measurement_matrix

    return calibrated_matrix;
}
double** mpu9250_calibrate_accel(double x_accr, double y_accr, double z_accr) {
    double **measurement_matrix = (double**)malloc(3 * sizeof(double*));
    for (int i = 0; i < 3; i++) {
        measurement_matrix[i] = (double*)malloc(1 * sizeof(double));
    }
    double **calibrate_parameter = (double**)malloc(3 * sizeof(double*));
       for (int i = 0; i < 3; i++) {
      	 calibrate_parameter[i] = (double*)malloc(1 * sizeof(double));
       }
   calibrate_parameter[0][0] = 1.005453;
   calibrate_parameter[0][1] = 0.000134;
   calibrate_parameter[0][2] = 0.007665;

   calibrate_parameter[1][0] = 0.000134;
   calibrate_parameter[1][1] = 1.004245;
   calibrate_parameter[1][2] = -0.000883;

   calibrate_parameter[2][0] = 0.007665;
   calibrate_parameter[2][1] = -0.000883;
   calibrate_parameter[2][2] = 0.992212;

    // Update the array indexing and calibration values accordingly
    measurement_matrix[0][0] = x_accr + 0.007122;
    measurement_matrix[1][0] = y_accr + 0.011151;
    measurement_matrix[2][0] = z_accr - 0.064440;

    double **calibrated_matrix = Multiply(measurement_matrix,calibrate_parameter, 3, 3, 1);
    freeMatrix(measurement_matrix, 3); // Free the memory allocated for measurement_matrix

    return calibrated_matrix;
}

double** Multiply(double** matrix1,double** calibrate_parameter, int row, int column1, int column2) {
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

void mpu9250_init() {
    // Check if the device is connected
    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, (Device_Address << 1) + 0, 2, General_Timeout);
    if (ret != HAL_OK) {
        printf("The device is not ready. Check again\n");
    }

    // Config accelerometer
    uint8_t temp_data = FS_ACC_2G;
    HAL_StatusTypeDef ret2 = HAL_I2C_Mem_Write(&hi2c1, (Device_Address << 1) + 0, REG_CONFIG_ACC, 1, &temp_data, 1, General_Timeout);
    if (ret2 != HAL_OK) {
        printf("The device accelerometer is not written. Check again\n");
    }

    // Config gyroscope
    temp_data = FS_GYRO_250;
    HAL_StatusTypeDef ret3 = HAL_I2C_Mem_Write(&hi2c1, (Device_Address << 1) + 0, REG_CONFIG_GYRO, 1, &temp_data, 1, General_Timeout);
    if (ret3 != HAL_OK) {
        printf("The device gyroscope is not written. Check again\n");
    }

    // Config power management register
    temp_data = 0;
    HAL_StatusTypeDef ret4 = HAL_I2C_Mem_Write(&hi2c1, (Device_Address << 1) + 0, REG_CONFIG_PWR1, 1, &temp_data, 1, General_Timeout);
    if (ret4 != HAL_OK) {
        printf("The device fail exiting sleep mode. Check again\n");
    }

    // magnetometer_init();

    Kalman_init(&kalman_roll);
    Kalman_init(&kalman_pitch);
    Kalman_setAngle(&kalman_roll, 0);
    Kalman_setAngle(&kalman_pitch, 0);
    Kalman_setRmeasure(&kalman_pitch,0.0121);
    Kalman_setRmeasure(&kalman_roll,0.1849);
    Kalman_setQbias(&kalman_pitch,0.04);
    Kalman_setQangle(&kalman_pitch,0.00179776); //IF WE REMOVE Delay should re calculate
    Kalman_setQbias(&kalman_roll,0.0144);
    Kalman_setQangle(&kalman_roll,0.000647193);
}

// magnetometer configuration
void magnetometer_init() {
    uint8_t temp_data;

    // Turn off Sensor Master I2C Interface using the USER_CTRL Register
    temp_data = USER_CTRL_Config;
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&hi2c1, (Device_Address << 1) + 0, USER_CTRL, 1, &temp_data, 1, General_Timeout);
    if (ret != HAL_OK) {
        printf("The device Sensor I2C master interface is not disabled. Check again\n");
    }

    // Turn on Bypass Register
    temp_data = Bypass_Config;
    HAL_StatusTypeDef ret1 = HAL_I2C_Mem_Write(&hi2c1, (Device_Address << 1) + 0, REG_Bypass, 1, &temp_data, 1, General_Timeout);
    if (ret1 != HAL_OK) {
        printf("The device Bypass register is not enabled. Check again\n");
    }

    // Configure mode for the magnetometer (Continuous2)
    temp_data = CTRL_1_Config_Continuous2;
    HAL_StatusTypeDef ret2 = HAL_I2C_Mem_Write(&hi2c1, (AK8963_Address << 1) + 0, CTRL_1, 1, &temp_data, 1, General_Timeout);
    if (ret2 != HAL_OK) {
        printf("The device Magnetometer mode is not configured yet. Check again\n");
    }
}

// Lowpass_filter for accelerometer
void Lowpass_filter(double *roll_acc, double previous_roll_acc, double *pitch_acc, double previous_pitch_acc) {
    *roll_acc = cutoff * (*roll_acc) + previous_roll_acc * (1 - cutoff);
    *pitch_acc = cutoff * (*pitch_acc) + previous_pitch_acc * (1 - cutoff);
}

void Complimentary_filter(double* angle, double gyro_data, double measured_angle, double time) {
    *angle = trust * (*angle + gyro_data * time) - (1 - trust) * measured_angle;
}

void mpu9250_angel(double accx, double accy, double accz,
                   double gyrox, double gyroy, double gyroz,
                   double magx, double magy, double magz,
                   double* roll, double* pitch, double* yaw,
                   double* roll_acc, double* pitch_acc, double* roll_gyro,
                   double* pitch_gyro, double *yaw_magneto, double* previous_roll_acc,
                   double* previous_pitch_acc, double *roll_kalman, double *pitch_kalman,
                   double time) {
    int Xm, Ym;
    static double previous_angle_roll_gyro = 0;
    static double previous_angle_pitch_gyro = 0;

    // Calculate roll angle
#ifdef RESTRICT_PITCH // Eq. 25 and 26
    *roll_acc = atan2(accy, accz) * RAD_TO_DEG;
    *pitch_acc = atan(-accx / sqrt(accy * accy + accz * accz)) * RAD_TO_DEG;
#else // Eq. 28 and 29
    *roll_acc = atan(accy / sqrt(accx * accx + accz * accz)) * RAD_TO_DEG;
    *pitch_acc = atan2(-accx, accz) * RAD_TO_DEG;
#endif

#ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((*roll_acc < -90 && *roll_kalman > 90) || (*roll_acc > 90 && *roll_kalman < -90)) {
        Kalman_setAngle(&kalman_roll, *roll_acc);
        *roll = *roll_acc;
        *roll_kalman = *roll_acc;
        *roll_gyro = *roll_acc;
    } else {
        *roll_kalman = Kalman_getAngle(&kalman_roll, *roll_acc, gyrox, time); // Calculate the angle using a Kalman filter
    }

    if (abs(*roll_kalman) > 90) {
        gyroy = -gyroy; // Invert rate, so it fits the restricted accelerometer reading
    }
    *pitch_kalman = Kalman_getAngle(&kalman_pitch, *pitch_acc, gyroy, time);
#else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((*pitch_acc < -90 && *pitch_kalman > 90) || (*pitch_acc > 90 && *pitch_kalman < -90)) {
    	Kalman_setAngle(&kalman_pitch,*pitch_acc);
    	*pitch = *pitch_acc;
        *pitch_kalman = *pitch_acc;
        *pitch_gyro = *pitch_acc;
    } else {
        *pitch_kalman = Kalman_getAngle(&kalman_pitch, *pitch_acc, gyroy, time); // Calculate the angle using a Kalman filter
    }

    if (abs(*pitch_kalman) > 90) {
        gyrox = -gyrox; // Invert rate, so it fits the restricted accelerometer reading
    }
    *roll_kalman = Kalman_getAngle(&kalman_roll, *roll_acc, gyrox, time); // Calculate the angle using a Kalman filter
#endif

    // Low_pass filter to removed noise from accelerometer calculation
    Lowpass_filter(roll_acc, *previous_roll_acc, pitch_acc, *previous_pitch_acc);
    *previous_roll_acc = (*roll_acc);
    *previous_pitch_acc = (*pitch_acc);

    if (starter == 1) {
        // Calculate angel from gyroscope
        *roll_gyro = gyrox * time + previous_angle_roll_gyro;
        *pitch_gyro = gyroy * time + previous_angle_pitch_gyro;
        previous_angle_roll_gyro = *roll_gyro;
        previous_angle_pitch_gyro = *pitch_gyro;

        // Calculate angel by sensor fusion
        Complimentary_filter(roll, gyrox, (*roll_acc), time);
        Complimentary_filter(pitch, gyroy, (*pitch_acc), time);

        // Calculate angel by magnetometer
        // Cross product to get the value of the Xm and Ym on 2D
        Xm = magx * cos((*pitch)) - magy * sin((*roll)) * sin((*pitch)) + magz * cos((*roll)) * sin((*pitch));
        Ym = magy * cos(*roll) + magz * sin(*roll);
        *yaw_magneto = atan2(Ym, Xm);
        Complimentary_filter(yaw, gyroz, *yaw_magneto, time);
    }
}
void mpu9250_read(uint32_t first_time){
	static uint32_t current_time, previous_time;
	double time;
 	static int counter = 0;
 	static double previous_roll_acc = 0;
 	static double previous_pitch_acc = 0;
 	static double roll = 0;
 	static double pitch =0;
	counter = counter+1;
	if(counter == Sampling){
		starter =1;
	}
	double yaw, roll_acc, pitch_acc, roll_gyro, pitch_gyro, yaw_magneto,roll_kalman, pitch_kalman;

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
	double x_gyro_calibrated = x_gyror - x_gyro_calibrate_para;
	double y_gyro_calibrated = y_gyror - y_gyro_calibrate_para;
	double z_gyro_calibrated = z_gyror - z_gyro_calibrate_para;
	//get time
	current_time = __HAL_TIM_GET_COUNTER(&htim2);
	if(counter == 1){
		previous_time = first_time;
		time = (abs((int)current_time - (int)previous_time)) * Time_constant;
		previous_time = current_time;
	}
	else{
		if(current_time >= previous_time){
			time = ((int)current_time - (int)previous_time)* Time_constant;
		}
		else {
			time = (Counter_limit -(int)previous_time + (int)current_time)* Time_constant;
		}
		previous_time = current_time;
	}
	mpu9250_angel(calibrated_accelerometer[0][0], calibrated_accelerometer[1][0],calibrated_accelerometer[2][0],
				x_gyro_calibrated,y_gyro_calibrated,z_gyro_calibrated,calibrated_magnetometer[0][0], calibrated_magnetometer[1][0],
				calibrated_magnetometer[2][0],&roll,&pitch,&yaw, &roll_acc, &pitch_acc, &roll_gyro, &pitch_gyro, &yaw_magneto,
				&previous_roll_acc, &previous_pitch_acc,&roll_kalman, &pitch_kalman, time);
	//print angel data
//		printf("%.5f  ", roll_acc);
//		printf("%.5f  \n",pitch_acc);
	if(starter == 1){
//		printf("Roll_gyro: %.5f  ",roll_gyro);
//		printf("Pitch_gyro: %.5f  \n",pitch_gyro);
		Kalman_printgain(&kalman_roll);
		printf(" %.5f ",roll_kalman);
		printf(" %.5f  \n",roll);
//		printf(" %.5f ",pitch_kalman);
//		printf(" %.5f \n ",pitch);
	}
//		printf("Yaw_magneto: %.5f  ",yaw_magneto);
//		printf("Yaw %.5f  \n", yaw);
//	printf(" %.5f  ", x_accr);
//	printf(" %.5f  ", y_accr);
//	printf(" %.5f  \n", z_accr);
	//print raw data
//	   printf("Calibrated acc: %.5f ", calibrated_accelerometer[0][0]*g);
//	   printf(" %.5f  ", calibrated_accelerometer[1][0]*g);
//	   printf(" %.5f  \n", calibrated_accelerometer[2][0]*g);
//	if(starter ==1){
//	    printf(" %.5f  ", x_gyro_calibrated);
//	    printf(" %.5f  \n", y_gyro_calibrated);
//	    printf(" %.5f  \n", z_gyro_calibrated);
//	}
//	else{
//	    printf(" %.5f   ", x_gyror);
//	    printf(" %.5f   ", y_gyror);
//	    printf(" %.5f  \n ", z_gyror);
//	}
//   printf("Calibrated mag: %.5f  ", calibrated_magnetometer[0][0]);
//   printf(" %.5f  ", calibrated_magnetometer[1][0]);
//   printf(" %.5f    \n", calibrated_magnetometer[2][0]);
}
