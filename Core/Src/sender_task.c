/*
 * sender_task.c
 *
 *  Created on: Jan 29, 2022
 *      Author: jan
 */



#include "sender_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "MPU6050.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;
extern osSemaphoreId_t Motion_SemaphoreHandle;
extern osMessageQueueId_t Send_Sensor_DataHandle;
extern osTimerId_t Check_IMU_Angle_TimerHandle;

struct imu_data_struct{
	float ax, ay, az;
	float gx, gy, gz;
	float roll, pitch;
}imu_data;


uint8_t interrupt_flag;


extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == IMU_INT_Pin)
	{
		uint8_t interrupts = MPU6050_GetIntStatusRegister();
		MPU6050_GetMotionStatusRegister();
		osSemaphoreRelease(Motion_SemaphoreHandle);
	}
}



// Timer_Callback 10s after end of scan loop ended
void Check_Angle(void *argument){

	float roll  = imu_data.roll;
	float pitch = imu_data.pitch;
	MPU6050_GetRollPitch( &imu_data.roll, &imu_data.pitch);

	float diff_roll = fabsf(roll - imu_data.roll);
	float diff_pitch = fabsf(pitch - imu_data.pitch);

	if(diff_pitch + diff_roll > 5){
		osSemaphoreRelease(Motion_SemaphoreHandle);
	}
}


void Start_Sender_task(void *argument){

	MPU6050_Init(&hi2c1);

	MPU6050_SetInterruptMode(MPU6050_INTMODE_ACTIVELOW);
	MPU6050_SetInterruptDrive(MPU6050_INTDRV_PUSHPULL);
	MPU6050_SetInterruptLatch(MPU6050_INTLATCH_WAITCLEAR);
	MPU6050_SetInterruptLatchClear(MPU6050_INTCLEAR_STATUSREAD);

	MPU6050_SetIntEnableRegister(0); // Disable all interrupts

	MPU6050_SetIntDataReadyEnabled(0);

	// Enable Motion interrupts
	MPU6050_SetDHPFMode(MPU6050_DHPF_5);

	MPU6050_SetIntMotionEnabled(1);
	MPU6050_SetIntZeroMotionEnabled(0);
	MPU6050_SetIntFreeFallEnabled(0);

	MPU6050_SetFreeFallDetectionDuration(2);
	MPU6050_SetFreeFallDetectionThreshold(5);

	MPU6050_SetMotionDetectionDuration(10);
	MPU6050_SetMotionDetectionThreshold(6);

	MPU6050_SetZeroMotionDetectionDuration(2);
	MPU6050_SetZeroMotionDetectionThreshold(4);

	const uint32_t time_sample_to_send = 10;

	osSemaphoreAcquire(Motion_SemaphoreHandle, osWaitForever);

	for(;;){

		osSemaphoreAcquire(Motion_SemaphoreHandle, osWaitForever);

		// Scan loop
		for(uint32_t time_stamp = 0; time_stamp < time_sample_to_send; ++time_stamp){
			osDelay(200);
			MPU6050_GetAccelerometerScaled(&imu_data.ax, &imu_data.ay, &imu_data.az);
			MPU6050_GetGyroscopeScaled(&imu_data.gx, &imu_data.gy, &imu_data.gz);
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

			MPU6050_GetRollPitch(&imu_data.roll, &imu_data.pitch);
			uint16_t angles[2];
			angles[0] = (uint16_t)(imu_data.roll*100.0f);
			angles[1] = (uint16_t)(imu_data.pitch*100.0f);

			osMessageQueuePut(Send_Sensor_DataHandle, &angles[0], 0U, 10);
		}

		osTimerStart(Check_IMU_Angle_TimerHandle, 2000U /* ms */);
	}
}
