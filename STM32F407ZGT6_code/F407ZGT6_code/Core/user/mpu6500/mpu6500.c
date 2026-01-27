/*
 * mpu6500.c
 *
 *  Created on: Jan 27, 2026
 *      Author: yufei
 */

#include "mpu6500.h"

int MPU6500_SPIInit() {
	int result;

	mpu_int_param.cb = mpu_data_ready;
	mpu_int_param.pin = MPU6500_INT_PIN;
	mpu_int_param.lp_exit = 1;
	mpu_int_param.active_low = 1;
	reg_int_cb(&mpu_int_param); // 注册回调
	// MPU 初始化（motion_driver_6.12 mpu_init 有参数）
	result = mpu_init(&mpu_int_param);  // 注意：必须传 int_param_s*
	printf("mpu_init(&mpu_int_param) = %d\r\n", result);
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

	//mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);

	// 加载 DMP 固件
	result = dmp_load_motion_driver_firmware();
	if (result) {
		printf("dmp_load_motion_driver_firmware failed: %d\n", result);
		return -1;
	}
	// 配置 DMP 输出
	//result = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP| DMP_FEATURE_ANDROID_ORIENT);
	result = dmp_enable_feature( DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL);
	printf("dmp_enable_feature = %d\r\n", result);
	result = dmp_set_fifo_rate(50); // 50 Hz
	result = mpu_set_dmp_state(1); // 打开 DMP
	printf("mpu_set_dmp_state = %d\r\n", result);
	result = mpu_set_int_latched(0); // 使能 MPU 中断
	printf("mpu_set_int_latched = %d\r\n", result);

	return 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == MPU6500_INT_PIN) {
		int_count++;
		if (int_count >= 4) {
			int_count = 0;
			if (mpu_int_param.cb) {
				mpu_int_param.cb();  // 调用你写的 mpu_data_ready()
			}
		}
	}
}

void mpu_data_ready(void) {
	mpu_dmp_int = 1;
}

void dmp_print_once() {
	do {
		dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
	} while (more);

	if (sensors & INV_WXYZ_QUAT) {
		// 这里再 printf / 处理
		const float scale = 1.0f / (1L << 30);
		float q_out[4];

		// 转 float
		float q0 = quat[0] * scale;
		float q1 = quat[1] * scale;
		float q2 = quat[2] * scale;
		float q3 = quat[3] * scale;

		// 计算模长
		float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

		// 防止除 0
		if (norm < 1e-6f) {
			q_out[0] = 1.0f;
			q_out[1] = 0.0f;
			q_out[2] = 0.0f;
			q_out[3] = 0.0f;
			return;
		}
		float inv = 1.0f / norm;
		q_out[0] = q0 * inv;
		q_out[1] = q1 * inv;
		q_out[2] = q2 * inv;
		q_out[3] = q3 * inv;
		quat_nom[0] = q_out[0];
		quat_nom[1] = -q_out[1];
		quat_nom[2] = -q_out[2];
		quat_nom[3] = q_out[3];
		//printf("%.5f, %.5f, %.5f, %.5f, ", quat_nom[0], quat_nom[1],
		//		quat_nom[2], quat_nom[3]);
	}
}
