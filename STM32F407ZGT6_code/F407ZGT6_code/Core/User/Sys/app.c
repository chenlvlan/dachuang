/*
 * app.c
 *
 *  Created on: Dec 11, 2025
 *      Author: yufei
 */

#include "app.h"

//#define MPU6500_INTERFACE_SPI
#define CLI_BUF_LEN 64

static char cli_buf[CLI_BUF_LEN];
static uint8_t cli_idx = 0;
static uint8_t cli_rx_char;

bool doMotionCtrlCycle = 0;
motorDataRead_t JMDataRead[4] = { 0 };
static mpu6500_handle_t g_mpu6500;
//----------------------------------------
//int16_t accel_raw[1][3];
//int16_t gyro_raw[1][3];
//float accel_g[1][3];
//float gyro_dps[1][3];
//uint16_t len = 1;
volatile uint8_t mpu_dmp_int = 0;
volatile uint32_t g_loop_delay_ms = 100;
//int32_t quat[4];
//float pitch, roll, yaw;
//------------------------------------------

void appSetup() {
	HVHP(1); //母线上电
	HAL_Delay(1000); //这个延时必须加，不然在上电（冷启动，不是按reset那种）后MPU6500会初始化失败
	MPU6500_SPIInit();
	cli_init();

	printf("CLI ready, type 'help'\r\n");
	HAL_Delay(150); //等待供电稳定
	CommCan_Init(&hcan1); //关节电机can1通信初始化
	CommCan_Init(&hcan2); //关节电机can2通信初始化
	HAL_Delay(100);

	//上电后的基本信息读取
	refreshAll(idLF);
	refreshAll(idLR);
	refreshAll(idRF);
	refreshAll(idRR);

	HAL_TIM_Base_Start_IT(&htim3); //运动控制环开始定时

	//警告：在限位块未安装的时候，严禁执行回原点程序，否则会导致撞机
	returnToOrigin(0.3, 0.2, 3000); //回原点

}

void appLoop() {
	//printf("loop begin  ");
	//这个是主程序
	if (doMotionCtrlCycle == 1) {
		doMotionCtrlCycle = 0;
		motionCtrlCycle();
	}
	//-----------------------------------------------------
	/*
	 mpu6500_read(&g_mpu6500, &accel_raw[1], &accel_g[1], &gyro_raw[1],
	 &gyro_dps[1], &len);

	 printf("ACC: \t%f\t%f\t%f g\r\n", accel_g[1][0], accel_g[1][1],
	 accel_g[1][2]);
	 printf("GYR: \t%f\t%f\t%f dps\r\n", gyro_dps[1][0], gyro_dps[1][1],
	 gyro_dps[1][2]);

	 if (mpu_dmp_int) {
	 mpu_dmp_int = 0;
	 if (mpu6500_dmp_read(&g_mpu6500, &accel_raw[1], &accel_g[1],
	 &gyro_raw[1], &gyro_dps[1], &quat, &pitch, &roll, &yaw, &len)
	 == 0) {
	 float q0 = quat[0] / 1073741824.0f;
	 float q1 = quat[1] / 1073741824.0f;
	 float q2 = quat[2] / 1073741824.0f;
	 float q3 = quat[3] / 1073741824.0f;
	 // 打印或用来调试
	 float q_norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

	 printf("=== DMP DATA ===\r\n");
	 printf("ACC_RAW : %6d %6d %6d\r\n", accel_raw[0][0],
	 accel_raw[0][1], accel_raw[0][2]);
	 printf("ACC_G   : %6.3f %6.3f %6.3f g\r\n", accel_g[0][0],
	 accel_g[0][1], accel_g[0][2]);

	 printf("GYRO_RAW: %6d %6d %6d\r\n", gyro_raw[0][0], gyro_raw[0][1],
	 gyro_raw[0][2]);
	 printf("GYRO_DPS: %6.2f %6.2f %6.2f dps\r\n", gyro_dps[0][0],
	 gyro_dps[0][1], gyro_dps[0][1]);

	 printf("QUAT    : %+.4f %+.4f %+.4f %+.4f\r\n", q0, q1, q2, q3);
	 printf("|Q|     : %.6f\r\n", q_norm);

	 printf("RPY(deg): R=%6.2f P=%6.2f Y=%6.2f\r\n", roll, pitch, yaw);
	 printf("----------------\r\n");

	 }
	 }
	 */
	//if (mpu_dmp_int) {
	//printf("\r\n1\r\n");
	mpu_dmp_int = 0;
	dmp_print_once(); // 这里才调用 mpu6500_dmp_read()
	//mpu6500_force_fifo_reset(&g_mpu6500);
	//printf("2\r\n");
	//}
	//printf("a cycle\r\n");
	//printf("loop end  ");
//----------------------------------------------------------------------------
	HAL_Delay(g_loop_delay_ms);

	//unsigned char msg[] = "1,10.0,1,-10.0\n";
	//HAL_UART_Transmit(&huart4, msg, sizeof(msg), 0xffff);
//printf("111\n");
}

//警告：这个是阻塞函数，实时状态下禁止使用
void refreshAll(uint8_t id) {
	JM_GetMotorInfo(id, 0);
	HAL_Delay(5);
	JM_GetRealTimeStatusInfo(id);
	HAL_Delay(5);
	JM_GetSoftwareInfo(id);
	HAL_Delay(5);
}

void returnToOrigin(float speed, float torque, uint32_t timeout) {

//.......
//在执行回原点前，先安全地保存现场，完成准备工作

	JM_SetPosVelModeMaxTorque(idLF, torque);	//限制最大力矩
	JM_SetPosVelModeMaxTorque(idLR, torque);
	JM_SetPosVelModeMaxTorque(idRF, torque);
	JM_SetPosVelModeMaxTorque(idRR, torque);

	HAL_Delay(100);
	speed = 0 - fabsf(speed);	//速度为负数才是往原点转
	JM_VelMode(idLF, speed);	//进行归零动作
	JM_VelMode(idLR, speed);
	JM_VelMode(idRF, speed);
	JM_VelMode(idRR, speed);

//float deltaPos[4][25];
//这里使用简化的回原点程序，就是在限定力矩的情况下，给充足的时间，并假定所有关节电机都转到
//了原点，时间一到直接设置当前为原点

	/*
	 uint32_t startTime = HAL_GetTick(); //开始时间
	 while (HAL_GetTick() - startTime <= timeout) { //在超时时间以内的话
	 //这里面也跑着运动控制环
	 if (doMotionCtrlCycle == 1) {
	 doMotionCtrlCycle == 0;

	 }
	 }
	 */
	HAL_Delay(timeout);
	JM_PosRelaMode(idRF, 0.2);	//右侧两个电机转至右边的腿分开
	JM_PosRelaMode(idRR, 0.2);
	HAL_Delay(500);
	JM_Restart(idLF);	//重启，读取新的位置
	JM_Restart(idLR);
	JM_Restart(idRF);
	JM_Restart(idRR);
	HAL_Delay(200);	//必须加延时，不然RR电机无法回零
	JM_ReturnToOrigin(idLF);	//转到原点，以展示回原点是否正确
	HAL_Delay(200);
	JM_ReturnToOrigin(idLR);
	HAL_Delay(200);
	JM_ReturnToOrigin(idRF);
	HAL_Delay(200);
	JM_ReturnToOrigin(idRR);
	HAL_Delay(200);
//回原点完毕，安全地恢复现场
//........
}

void motionCtrlCycle() {
//运动控制环

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		// ---- 这里执行你的 20ms 控制环 ----
		doMotionCtrlCycle = 1;
	}
}

/*
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
 if (GPIO_Pin == MPU6500_INT_PIN) {
 printf("INT triggered\n");
 uint8_t status;
 int d = 0;
 d = mpu6500_get_interrupt_status(&g_mpu6500, &status);
 //printf("mpu6500_get_interrupt_status = %d", d);
 mpu_dmp_int = 1;
 }
 }
 */
void MPU6500_SPIInit() {
	/*
	 //--------------------------------------------------------------

	 uint8_t res;
	 DRIVER_MPU6500_LINK_INIT(&g_mpu6500, mpu6500_handle_t);
	 DRIVER_MPU6500_LINK_SPI_INIT(&g_mpu6500, mpu6500_interface_spi_init);
	 DRIVER_MPU6500_LINK_SPI_DEINIT(&g_mpu6500, mpu6500_interface_spi_deinit);
	 DRIVER_MPU6500_LINK_SPI_READ(&g_mpu6500, mpu6500_interface_spi_read);
	 DRIVER_MPU6500_LINK_SPI_WRITE(&g_mpu6500, mpu6500_interface_spi_write);
	 DRIVER_MPU6500_LINK_DELAY_MS(&g_mpu6500, mpu6500_interface_delay_ms);
	 DRIVER_MPU6500_LINK_DEBUG_PRINT(&g_mpu6500, mpu6500_interface_debug_print);
	 DRIVER_MPU6500_LINK_RECEIVE_CALLBACK(&g_mpu6500,
	 mpu6500_interface_receive_callback);
	 g_mpu6500.iic_spi = MPU6500_INTERFACE_SPI;

	 res = mpu6500_init(&g_mpu6500);

	 if (mpu6500_dmp_load_firmware(&g_mpu6500) != 0) {
	 printf("dmp firmware load failed\n");
	 }
	 mpu6500_dmp_set_fifo_rate(&g_mpu6500, 200);   // 200Hz
	 mpu6500_dmp_set_feature(&g_mpu6500, MPU6500_DMP_FEATURE_6X_QUAT);
	 mpu6500_dmp_set_enable(&g_mpu6500, MPU6500_BOOL_TRUE);

	 if (res != 0) {
	 printf("mpu6500 init failed\r\n");
	 Error_Handler();
	 }

	 res = mpu6500_dmp_init(&g_mpu6500);
	 if (res != 0)
	 {
	 printf("mpu6500 dmp init failed\r\n");
	 Error_Handler();
	 }
	 mpu6500_set_accelerometer_range(&g_mpu6500, MPU6500_ACCELEROMETER_RANGE_2G);
	 mpu6500_set_gyroscope_range(&g_mpu6500, MPU6500_GYROSCOPE_RANGE_2000DPS);
	 mpu6500_set_sample_rate_divider(&g_mpu6500, (uint8_t) 1000);   // 1 kHz
	 //-------------------------------------------------------------------------
	 */
	/*
	 // 绑定接口
	 DRIVER_MPU6500_LINK_INIT(&g_mpu6500, mpu6500_handle_t);
	 DRIVER_MPU6500_LINK_SPI_INIT(&g_mpu6500, mpu6500_interface_spi_init);
	 DRIVER_MPU6500_LINK_SPI_DEINIT(&g_mpu6500, mpu6500_interface_spi_deinit);
	 DRIVER_MPU6500_LINK_SPI_READ(&g_mpu6500, mpu6500_interface_spi_read);
	 DRIVER_MPU6500_LINK_SPI_WRITE(&g_mpu6500, mpu6500_interface_spi_write);
	 DRIVER_MPU6500_LINK_DELAY_MS(&g_mpu6500, mpu6500_interface_delay_ms);
	 DRIVER_MPU6500_LINK_DEBUG_PRINT(&g_mpu6500, mpu6500_interface_debug_print);
	 DRIVER_MPU6500_LINK_RECEIVE_CALLBACK(&g_mpu6500,
	 mpu6500_interface_receive_callback);
	 g_mpu6500.iic_spi = MPU6500_INTERFACE_SPI;

	 // **必须先初始化 handle**
	 if (mpu6500_init(&g_mpu6500) != 0) {
	 printf("mpu6500 init failed\n");
	 Error_Handler();
	 }
	 */
	mpu6500_interface_t interface = MPU6500_INTERFACE_SPI;
	mpu6500_address_t addr_pin = MPU6500_ADDRESS_AD0_HIGH; // 取决于 CS 接法

	if (mpu6500_dmp_init(&g_mpu6500, interface, addr_pin, NULL,
	NULL, NULL) != 0) {
		printf("DMP init failed\n");
		Error_Handler();
	}
	mpu6500_dmp_set_fifo_rate(&g_mpu6500, 200); // 50Hz 而不是 200Hz
	//printf("mpu6500_dmp_set_fifo_rate = %d\r\n", c);
	//mpu6500_dmp_set_enable(&g_mpu6500, MPU6500_BOOL_TRUE);
	//mpu6500_set_interrupt_data_ready(&g_mpu6500, MPU6500_BOOL_TRUE);
	//mpu6500_set_interrupt(&g_mpu6500, MPU6500_INTERRUPT_DATA_READY, MPU6500_BOOL_TRUE);
	//mpu6500_set_interrupt_latch(&g_mpu6500, MPU6500_BOOL_FALSE);      // 50µs 脉冲模式
	//mpu6500_set_interrupt_read_clear(&g_mpu6500, MPU6500_BOOL_TRUE);  // 读状态清除中断
	//mpu6500_set_interrupt_latch(&g_mpu6500, MPU6500_BOOL_FALSE);

	//printf("mpu6500_dmp_set_enable = %d\r\n", c);
	//HAL_Delay(500);
	//printf("inited = %d, dmp_inited = %d\r\n", g_mpu6500.inited, g_mpu6500.dmp_inited);

}

/* 四元数打印函数 */
void dmp_print_once() {
	int16_t accel_raw[1][3];
	float accel_g[1][3];
	int16_t gyro_raw[1][3];
	float gyro_dps[1][3];
	int32_t quat[1][4];
	float pitch, roll, yaw;
	uint16_t len = 1;

	int tmp = 0;
	//float temperature;
	//mpu6500_read_temperature(&g_mpu6500, NULL, &temperature);
	//printf("temperature = %.2f\r\n", temperature);

	tmp = mpu6500_dmp_read(&g_mpu6500, accel_raw, accel_g, gyro_raw, gyro_dps,
			quat, &pitch, &roll, &yaw, &len);
	//tmp = mpu6500_dmp_read_all(&g_mpu6500, accel_raw, accel_g, gyro_raw, gyro_dps, &quat, &pitch, &roll, &yaw, len);
	//printf("inited = %d, dmp_inited = %d\r\n", g_mpu6500.inited, g_mpu6500.dmp_inited);
	if (tmp != 0) {
		printf("DMP read failed, error code %d\r\n", tmp);
		mpu6500_force_fifo_reset(&g_mpu6500);
		//printf("mpu6500_force_fifo_reset = %d\r\n", b);
		return;
	}

	float q0 = quat[0][0] / 1073741824.0f;
	float q1 = quat[0][1] / 1073741824.0f;
	float q2 = quat[0][2] / 1073741824.0f;
	float q3 = quat[0][3] / 1073741824.0f;
	float q_norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

	//printf("QUAT: %+.4f %+.4f %+.4f %+.4f |Q|: %.6f\r\n", q0, q1, q2, q3, q_norm);
	//printf("RPY: R=%6.2f P=%6.2f Y=%6.2f\r\n", roll, pitch, yaw);
	printf("%.5f, %.5f, %.5f\n", roll, pitch, yaw);
}
/*
 void mpu_receive_callback(uint8_t type) {
 mpu_dmp_int = 1;
 printf("mpu_receive_callback triggered\n");
 }

 void mpu_tap_callback(uint8_t count, uint8_t dir) {
 (void) count;
 (void) dir;
 }

 //orientation 回调（空函数
 void mpu_orient_callback(uint8_t orientation) {
 (void) orientation;
 }
 */

void cli_init(void) {
	HAL_UART_Receive_IT(&huart1, &cli_rx_char, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart != &huart1)
		return;

	//printf("aaa\r\n");
	if (cli_rx_char == '\r' || cli_rx_char == '\n') {
		cli_buf[cli_idx] = '\0';
		cli_idx = 0;

		cli_handle_command(cli_buf);
	} else {
		if (cli_idx < CLI_BUF_LEN - 1) {
			cli_buf[cli_idx++] = cli_rx_char;
		}
	}

	HAL_UART_Receive_IT(&huart1, &cli_rx_char, 1);
}

void cli_handle_command(char *cmd) {
	if (strlen(cmd) == 0)
		return;

	if (strcmp(cmd, "help") == 0) {
		printf("Commands:\r\n");
		printf("  delay           show delay(ms)\r\n");
		printf("  delay <num>     set delay(ms)\r\n");
	} else if (strncmp(cmd, "delay", 5) == 0) {
		uint32_t val;

		if (sscanf(cmd, "delay %lu", &val) == 1) {
			g_loop_delay_ms = val;
			printf("delay set to %lu ms\r\n", g_loop_delay_ms);
		} else {
			printf("delay = %lu ms\r\n", g_loop_delay_ms);
		}
	} else {
		printf("unknown cmd: %s\r\n", cmd);
	}
}
