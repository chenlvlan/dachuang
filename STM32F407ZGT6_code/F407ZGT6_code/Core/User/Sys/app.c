/*
 * app.c
 *
 *  Created on: Dec 11, 2025
 *      Author: yufei
 */

#include "app.h"
#include <math.h>
//#include "arm_math.h"

volatile bool doMotionCtrlCycle = 0;
legData_t legData_L = { .L1 = 90.0f, .L2 = 90.0f, .L3 = 130.0f, .L4 = 130.0f, .d =
		65.5f, .theta_f_max = 1.448623f, .theta_f_min = 0.0f, .theta_r_max =
		1.448623f, .theta_r_min = 0.0f, .x = 0.0f, .y = -190.0f };

legData_t legData_R = { .L1 = 90.0f, .L2 = 90.0f, .L3 = 130.0f, .L4 = 130.0f, .d =
		65.5f, .theta_f_max = 1.448623f, .theta_f_min = 0.0f, .theta_r_max =
		1.448623f, .theta_r_min = 0.0f, .x = 0.0f, .y = -190.0f };
wheelMotorData_t wheelMotorData = { .mode = WM_Torque };

float quat_nom[4] = { 0 };
float roll, yaw, pitch;

volatile uint8_t emergency_request = 0;
volatile float remote_forward_speed = 0.0f;  // m/s
volatile float remote_turn_angle = 0.0f;     // rad
volatile float remote_leg_delta = 0.0f;      // mm (对 yRef 的偏置)


controlData_t ctrlData;

void apply_remote_command(controlData_t *ctrlData);

static inline float clampf_local(float x, float min, float max) {
    return clampf(x, min, max);
}

// setter 实现（尽量短并做并发保护）
void set_remote_forward_speed(float forward_mps) {
    __disable_irq();
    remote_forward_speed = clampf_local(forward_mps, -1.0f, 1.0f); // 限幅如需改动
    __enable_irq();
}

void set_remote_turn_angle(float turn_rad) {
    __disable_irq();
    const float TURN_LIMIT = 0.785398f; // +/-45deg
    remote_turn_angle = clampf_local(turn_rad, -TURN_LIMIT, TURN_LIMIT);
    __enable_irq();
}

void set_remote_leg_delta(float leg_delta_mm) {
    __disable_irq();
    remote_leg_delta = clampf_local(leg_delta_mm, -100.0f, 100.0f); // mm limit
    __enable_irq();
}

void emergency_stop_motors(void) {
    __disable_irq();
    remote_forward_speed = 0.0f;
    remote_turn_angle = 0.0f;
    ctrlData.m0torque = 0.0f;
    ctrlData.m1torque = 0.0f;
    wheelMotorData.m0target = 0.0f;
    wheelMotorData.m1target = 0.0f;
    emergency_request = 1; // 标志，主循环处理真实发送
    __enable_irq();
}

/* 状态量（全局共享） */
//float pitch_rate;   // 可选，用于D项
//float pitch_avg = 0.0f;
//float x_ref;        // 轮子前后目标位置（输出）
//float pitch_lpf_alpha = 0.9f;  // 时间常数 ~1s
//float wheel_torque_cmd;  // 最终输出力矩
void HVHP(bool isEN) {
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, (GPIO_PinState) isEN);
}

void appSetup() {
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);   // 例：INT 接在 PA3
	HVHP(1); //母线上电
	WM_CommInit();
	WM_SendRestart(); //复位驱动器
	HAL_Delay(1000); //这个延时必须加，不然在上电（冷启动，不是按reset那种）后MPU6500会初始化失败
	mpu6500_SPIInit();
	cli_init();

	printf("CLI ready, type 'help'\r\n");
	//HAL_Delay(150); //等待供电稳定
	JM_CommInit();
	HAL_Delay(100);

	//上电后的基本信息读取
	JM_RefreshAll(idLF);
	JM_RefreshAll(idLR);
	JM_RefreshAll(idRF);
	JM_RefreshAll(idRR);

	HAL_TIM_Base_Start_IT(&htim3); //运动控制环开始定时

	//警告：在限位块未安装的时候，严禁执行回原点程序，否则会导致撞机
	JM_FindOrigin(0.4, 0.3, 3000); //回原点

	//开启6500的中断捕获
	HAL_NVIC_ClearPendingIRQ(EXTI3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	control_init();
	//control_comm_init();
	HAL_Delay(2000);
}

void appLoop() {
	if (emergency_request) {
	        emergency_request = 0;
	        WM_Send(&wheelMotorData); // 立即把已置零的目标下发到驱动
	    }
	if (mpu6500_isReady()) {
		mpu6500_DMPGet(&quat_nom[0]);
		quat2euler(quat_nom[0], quat_nom[1], quat_nom[2], quat_nom[3], &roll,
			&pitch, &yaw);

		set_remote_forward_speed(0.5f);//调试代码
		//set_remote_leg_delta(-10.0f);   // 把腿端抬高 10 mm（示例方向）
		//set_remote_turn_angle(0.1f);    // 右转 0.3 rad


		//printf("%.5f, %.5f, %.5f, ", roll, pitch, yaw);
		ctrlData.roll = roll;
		ctrlData.pitch = pitch;
		ctrlData.yaw = yaw;

		control_loop(&ctrlData);
		apply_remote_command(&ctrlData);//调用遥控控制


		// 1. 左腿赋值 + 逆解
		legData_L.x = ctrlData.xRefLeft;
		legData_L.y = ctrlData.yRefLeft;
		fivebar_inverse_kinematics(&legData_L);

		// 2. 右腿赋值 + 逆解
		legData_R.x = ctrlData.xRefRight;
		legData_R.y = ctrlData.yRefRight;
		fivebar_inverse_kinematics(&legData_R);


		//printf("%.5f, %.5f, %.5f, %.5f, %.5f\r\n", roll, pitch, yaw, legData.x,
		//		wheel_torque_cmd);

		/*
		 uint8_t tmp[16];
		 memcpy(&tmp[0], &roll, 4);
		 memcpy(&tmp[4], &pitch, 4);
		 memcpy(&tmp[8], &yaw, 4);
		 tmp[12] = 0x00;
		 tmp[13] = 0x00;
		 tmp[14] = 0x80;
		 tmp[15] = 0x7f;
		 HAL_UART_Transmit(&huart1, &tmp[0], 16, 0xffff);
		 */

		JM_PosAbsMode(idLF, legData_L.theta_f);
		JM_PosAbsMode(idRF, legData_R.theta_f);
		JM_PosAbsMode(idLR, legData_L.theta_r);
		JM_PosAbsMode(idRR, legData_R.theta_r);

		wheelMotorData.m0target = ctrlData.m0torque;
		wheelMotorData.m1target = ctrlData.m1torque;
		//WM_SendTorque(wheel_torque_cmd, wheel_torque_cmd);
		WM_Send(&wheelMotorData);
		WM_Receive(&wheelMotorData.m0velocity, &wheelMotorData.m0torque,
				&wheelMotorData.m1velocity, &wheelMotorData.m1torque);
		ctrlData.m0speed = wheelMotorData.m0velocity;
		ctrlData.m1speed = wheelMotorData.m1velocity;
		ctrlData.m0torque = wheelMotorData.m0torque;
		ctrlData.m1torque = wheelMotorData.m1torque;
		//WM_SendTorque(0.00114, 0.00114);
		//WM_Disable();
		//printf("%.5f, %.5f, %.5f, %.5f\r\n", wheelMotorData.m0velocity,
		//		wheelMotorData.m0torque, wheelMotorData.m1velocity,
		//		wheelMotorData.m1torque);
	}
	cli_poll();
}


void apply_remote_command(controlData_t *ctrlData) {
    // remote_forward_speed 已为 m/s； remote_turn_angle 已为 rad
    const float K_FORWARD = 0.08f;  // 速度 -> 扭矩系数（保留或重新标定）
    const float K_TURN = 0.1f;     // 转向 -> 扭矩差分系数
    const float K_LEG_TURN = 0.5f;

    // 读取（本函数在同一线程 context 中被调用，读写 remote_* 已用 volatile）
    float forward = remote_forward_speed;
    float turn = remote_turn_angle;
    float leg_delta = remote_leg_delta;

    // 映射到扭矩偏置
    float forward_bias = forward * K_FORWARD;
    float turn_bias = turn * K_TURN;

    // 合成扭矩（保留原有平衡扭矩 ctrlData->m?torque）
    ctrlData->m0torque = clampf(ctrlData->m0torque + forward_bias + turn_bias,
                                    -TORQUE_LIMIT, TORQUE_LIMIT);
    ctrlData->m1torque = clampf(ctrlData->m1torque + forward_bias - turn_bias,
                                    -TORQUE_LIMIT, TORQUE_LIMIT);


    ctrlData->xRefLeft  +=  turn * K_LEG_TURN;
    ctrlData->xRefRight -=  turn * K_LEG_TURN;


    // 把腿高度偏置融合到腿端参考值，注意坐标方向（示例：yRef 为负数向下）
    // 假设 STAND_Y_REF 是 mm 或已用同一单位（你的项目里 STAND_Y_REF 应与 legData.y 单位一致）
    float newYLeft = STAND_Y_REF + leg_delta;
    //float newYRight = STAND_Y_REF + leg_delta;


    // 限幅以防超出机械范围
    // ctrlData 的 yRefLeft/yRefRight 对应 compute.c 中使用的 xRef/yRef
    ctrlData->yRefLeft = clampf(newYLeft, -500.0f, 0.0f);  // 例子：-500..0 mm，请按实际改
    ctrlData->yRefRight = ctrlData->yRefLeft;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		// ---- 这里执行你的 20ms 控制环 ----
		doMotionCtrlCycle = 1;
	}
}
