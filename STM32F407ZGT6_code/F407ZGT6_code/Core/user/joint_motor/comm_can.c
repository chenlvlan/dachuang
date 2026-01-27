#include "../joint_motor/comm_can.h"

#include "can.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"

//#define hCan        hCan2
//注释掉了，使用带参数的初始化函数

//#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
//这个用内联函数替换掉了，避免warning

#define POS_MAX_Default     95.5f
#define VEL_MAX_DEFAULT     45.0f
#define T_MAX_DEFAULT       18.0f
#define KP_MAX              500.0f
#define KD_MAX              5.0f

float pos_max = POS_MAX_Default;
float vel_max = VEL_MAX_DEFAULT;
float t_max = T_MAX_DEFAULT;

CanRxMessage_t CanRxMessage;
uint8_t canTxBuf[8] = { 0 }; /**!< can数据发送缓存 */

static inline float LIMIT_MIN_MAX(float x, float min, float max) {
	if (x < min)
		return min;
	if (x > max)
		return max;
	return x;
}

/* Converts a float to an unsigned int, given range and number of bits */
int float_to_uint(float x, float x_min, float x_max, int bits) {
	float span = x_max - x_min;
	float offset = x_min;

	x = LIMIT_MIN_MAX(x, x_min, x_max);

	return (int) ((x - offset) * ((float) ((1 << bits) - 1)) / span);
}

/* converts unsigned int to float, given range and number of bits */
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
	float span = x_max - x_min;
	float offset = x_min;
	return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
}

void CommCan_Init(CAN_HandleTypeDef *hCan) {

	//CAN_FilterTypeDef sCAN_Filter;

	//sCAN_Filter.FilterBank = 0; /* 指定将被初始化的过滤器 */
	//sCAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK; /* 过滤模式为屏蔽位模式 */
	//sCAN_Filter.FilterScale = CAN_FILTERSCALE_16BIT; /* 指定滤波器的规模 */
	//sCAN_Filter.FilterIdHigh = 0;
	//sCAN_Filter.FilterIdLow = 0;
	//sCAN_Filter.FilterMaskIdHigh = 0;
	//sCAN_Filter.FilterMaskIdLow = 0;
	//sCAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	//sCAN_Filter.FilterActivation = ENABLE; /* 启用或禁用过滤器 */
	//sCAN_Filter.SlaveStartFilterBank = 0; /* 选择启动从过滤器组 */

	//HAL_CAN_ConfigFilter(hCan, &sCAN_Filter);

	CAN_FilterTypeDef filter;

	if (hCan->Instance == CAN1) {
		filter.FilterBank = 0; // 使用过滤器 0
	} else if (hCan->Instance == CAN2) {
		filter.FilterBank = 14;
	}

	filter.FilterMode = CAN_FILTERMODE_IDLIST;     // 列表模式
	filter.FilterScale = CAN_FILTERSCALE_16BIT;    // 每个Bank可以存四个ID
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterActivation = ENABLE;

	// 填入你的四个 ID（标准 ID 左移 5 位）
	filter.FilterIdHigh = (11 << 5);
	filter.FilterIdLow = (12 << 5);
	filter.FilterMaskIdHigh = (13 << 5);
	filter.FilterMaskIdLow = (14 << 5);

	if (hCan->Instance == CAN1) {
		filter.SlaveStartFilterBank = 14;
	}

	// 设置过滤器
	if (HAL_CAN_ConfigFilter(hCan, &filter) != HAL_OK) {
		Error_Handler();
	}

	HAL_CAN_Start(hCan); /* 开启CAN通信 */
	HAL_CAN_ActivateNotification(hCan, CAN_IT_RX_FIFO0_MSG_PENDING); /* 开启挂起中断允许 */
}

static void _Transmit(CAN_HandleTypeDef *canHandle, uint16_t dev_id,
		uint8_t cmd_id, uint8_t data_size, uint8_t *data) {
	CAN_TxHeaderTypeDef CAN_Header;
	uint32_t canTxMailbox;
	uint8_t i = 0;

	if (data_size <= 7) {
		canTxBuf[0] = cmd_id;

		if (data_size != 0) {
			if (data != NULL) {
				for (i = 0; i < data_size; i++) {
					canTxBuf[i + 1] = data[i];
				}
			} else {
				return;
			}
		}

		CAN_Header.StdId = dev_id; /* 指定标准标识符，该值在0x00-0x7FF */
		CAN_Header.IDE = CAN_ID_STD; /* 指定将要传输消息的标识符类型 */
		CAN_Header.RTR = CAN_RTR_DATA; /* 指定消息传输帧类型 */
		CAN_Header.DLC = (data_size + 1); /* 指定将要传输的帧长度 */

		if (HAL_CAN_AddTxMessage(canHandle, &CAN_Header, canTxBuf,
				(uint32_t*) &canTxMailbox) == HAL_OK) {

		}
	}
}

/* 重启电机 */
void CommCan_SysMotorRestart(CAN_HandleTypeDef *hCan, uint8_t dev_addr) {
	uint8_t tmp_buf[7];
	tmp_buf[0] = 0xFF;
	tmp_buf[1] = 0x00;
	tmp_buf[2] = 0xFF;
	tmp_buf[3] = 0x00;
	tmp_buf[4] = 0xFF;
	tmp_buf[5] = 0x00;
	tmp_buf[6] = 0xFF;
	_Transmit(hCan, dev_addr, eCAN_Restart, 7, tmp_buf);
}

/*   ****************************** 电机参数读取*********************************   */
/* 读取电机版本信息 */
void CommCan_GetVerInfo(CAN_HandleTypeDef *hCan, uint8_t dev_addr) {
	_Transmit(hCan, dev_addr, eCAN_GetVerInfo, 0, NULL);
}

/* 读取实时Q轴电流 */
void CommCan_GetIq(CAN_HandleTypeDef *hCan, uint8_t dev_addr) {
	_Transmit(hCan, dev_addr, eCAN_GetIq, 0, NULL);
}

/* 读取实时速度 */
void CommCan_GetVelocity(CAN_HandleTypeDef *hCan, uint8_t dev_addr) {
	_Transmit(hCan, dev_addr, eCAN_GetVelocity, 0, NULL);
}

/*读实时单圈绝对值角度、多圈绝对值角度 */
void CommCan_GetPosition(CAN_HandleTypeDef *hCan, uint8_t dev_addr) {
	_Transmit(hCan, dev_addr, eCAN_GetPosition, 0, NULL);
}

/* 读取电压、电流、温度、状态、故障实时状态信息 */
void CommCan_GetStatusInfo(CAN_HandleTypeDef *hCan, uint8_t dev_addr) {
	_Transmit(hCan, dev_addr, eCAN_GetStatusInfo, 0, NULL);
}

/* 读取电机硬件参数 */
void CommCan_GetMotorPara(CAN_HandleTypeDef *hCan, uint8_t dev_addr) {
	_Transmit(hCan, dev_addr, eCAN_GetMotorPara, 0, NULL);
}

/*   ****************************** 电机参数设置*********************************   */
/* 电机清除故障 */
void CommCan_SysMotorResetFault(CAN_HandleTypeDef *hCan, uint8_t dev_addr) {
	_Transmit(hCan, dev_addr, eCAN_ResetFault, 0, NULL);
}

/* 设置电机当前位置为原点*/
void CommCan_SetZeroByCurrentRawAngle(CAN_HandleTypeDef *hCan, uint8_t dev_addr) {
	_Transmit(hCan, dev_addr, eCAN_SetZeroByCurrentRawAngle, 0, NULL);
}

/* 设置位置模式最大速度 */
void CommCan_SetPosCtrlMaxVelocity(CAN_HandleTypeDef *hCan, uint8_t dev_addr,
		float target_param) {
	uint32_t target_val = (uint32_t) (target_param * 100);
	_Transmit(hCan, dev_addr, eCAN_SetPosCtrlMaxVelocity, 4,
			(uint8_t*) &target_val);
}

/* 设置位置或速度模式最大力矩 */
void CommCan_SetPosVelCtrlMaxIq(CAN_HandleTypeDef *hCan, uint8_t dev_addr,
		float target_param) {
	uint32_t target_val = (uint32_t) (target_param * 1000);
	_Transmit(hCan, dev_addr, eCAN_SetPosVelCtrlMaxIq, 4,
			(uint8_t*) &target_val);
}

/* 设置力矩速度电流斜率 */
void CommCan_SetIqCtrlSlope(CAN_HandleTypeDef *hCan, uint8_t dev_addr,
		float target_param) {
	uint32_t target_val = (uint32_t) (target_param * 1000);
	_Transmit(hCan, dev_addr, eCAN_SetIqCtrlSlope, 4, (uint8_t*) &target_val);
}

/* 设置速度模式下最大加速度 */
void CommCan_SetVelCtrlAcc(CAN_HandleTypeDef *hCan, uint8_t dev_addr,
		float target_param) {
	uint32_t target_val = (uint32_t) (target_param * 100);
	_Transmit(hCan, dev_addr, eCAN_SetVelCtrlAcc, 4, (uint8_t*) &target_val);
}

/* 位置环Kp参数设置*/
void CommCan_SetPosKpParam(CAN_HandleTypeDef *hCan, uint8_t dev_addr,
		float target_param) {
	_Transmit(hCan, dev_addr, eCAN_Pos_Kp, 4, (uint8_t*) &target_param);
}

/* 位置环Ki参数设置*/
void CommCan_SetPosKiParam(CAN_HandleTypeDef *hCan, uint8_t dev_addr,
		float target_param) {
	_Transmit(hCan, dev_addr, eCAN_Pos_Ki, 4, (uint8_t*) &target_param);
}

/* 速度环Kp参数设置*/
void CommCan_SetVelKpParam(CAN_HandleTypeDef *hCan, uint8_t dev_addr,
		float target_param) {
	_Transmit(hCan, dev_addr, eCAN_Vel_Kp, 4, (uint8_t*) &target_param);
}

/* 速度环Ki参数设置*/
void CommCan_SetVelKiParam(CAN_HandleTypeDef *hCan, uint8_t dev_addr,
		float target_param) {
	_Transmit(hCan, dev_addr, eCAN_Vel_Ki, 4, (uint8_t*) &target_param);
}

/* 力矩模式设置Q轴电流 */
void CommCan_SetIq(CAN_HandleTypeDef *hCan, uint8_t dev_addr, float target_Iq) {
	int32_t tmp_val = (int32_t) (target_Iq * 1000);
	_Transmit(hCan, dev_addr, eCAN_SetIqCtrlTragetVal, 4, (uint8_t*) &tmp_val);
}

/* 速度模式设置速度 */
void CommCan_SetVelocity(CAN_HandleTypeDef *hCan, uint8_t dev_addr,
		float target_vel) {
	int32_t tmp_val = (int32_t) (target_vel * 100);

	_Transmit(hCan, dev_addr, eCAN_SetVelCtrlTragetVal, 4, (uint8_t*) &tmp_val);
}

/* 绝对值位置控制，单位Count，一圈为16384Counts */
void CommCan_SetAbsPosition_Count(CAN_HandleTypeDef *hCan, uint8_t dev_addr,
		int32_t target_pos) {
	_Transmit(hCan, dev_addr, eCAN_SetAbsPosCtrlTragetVal, 4,
			(uint8_t*) &target_pos);
}

/* 相对值位置控制，单位Count，一圈为16384Counts */
void CommCan_SetRelateive_Count(CAN_HandleTypeDef *hCan, uint8_t dev_addr,
		int32_t target_pos) {
	_Transmit(hCan, dev_addr, eCAN_SetRelateiveTragetVal, 4,
			(uint8_t*) &target_pos);
}

/* 最短距离回到原点 */
void CommCan_ShortestHomePosition(CAN_HandleTypeDef *hCan, uint8_t dev_addr) {
	_Transmit(hCan, dev_addr, eCAN_ShortestHomePosition, 0, NULL);
}

/* 抱阀控制开关输出；参数0x00，抱闸失能开关断开；0x01，抱闸使能开关闭合；0xFF，读取抱闸开关状态 */
void CommCan_BreakCtrl(CAN_HandleTypeDef *hCan, uint8_t dev_addr,
		uint8_t open_or_close) {
	_Transmit(hCan, dev_addr, eCAN_BreakCtrl, 1, &open_or_close);
}

/* 电机失能 */
void CommCan_MotorDisableCtrl(CAN_HandleTypeDef *hCan, uint8_t dev_addr) {
	_Transmit(hCan, dev_addr, eCAN_DisableCtrl, 0, NULL);
}

/* Mit类型协议运控模式，读取参数*/
void CommCan_GetMitCtrlModePara(CAN_HandleTypeDef *hCan, uint8_t dev_addr) {
	_Transmit(hCan, dev_addr, eCAN_GetConfigMitCtrlModePara, 0, NULL);
}

/* Mit类型协议运控模式，配置参数*/
void CommCan_ConfigMitCtrlModePara(CAN_HandleTypeDef *hCan, uint8_t dev_addr,
		float pos_max_config, float vel_max_config, float t_max_config) {
	uint16_t config_data[3];
	config_data[0] = (uint16_t) (pos_max_config * 10);
	config_data[1] = (uint16_t) (vel_max_config * 100);
	config_data[2] = (uint16_t) (t_max_config * 100);
	_Transmit(hCan, dev_addr, eCAN_GetConfigMitCtrlModePara, 6,
			(uint8_t*) &config_data);
}

/* Mit类型协议运控模式，实时读取位置、速度、力矩和状态信息*/
void CommCan_GetMitCtrlModeRealTimeData(CAN_HandleTypeDef *hCan,
		uint8_t dev_addr) {
	_Transmit(hCan, dev_addr, eCAN_GetMitCtrlModeRealTimeData, 0, NULL);
}

/* 运控模式下运动控制命令 */
void CommCan_MotorRunCtrlMode_Control(CAN_HandleTypeDef *hCan,
		uint32_t dev_addr, float pos_target, float vel_target,
		float pos_gain_kp, float vel_gain_kd, float torque_target) {
	uint8_t tmp_buf[8];

	uint16_t position = float_to_uint(pos_target, -pos_max, pos_max, 16);
	uint16_t velocity = float_to_uint(vel_target, -vel_max, vel_max, 12);
	uint16_t kp = float_to_uint(pos_gain_kp, 0, KP_MAX, 12);
	uint16_t kd = float_to_uint(vel_gain_kd, 0, KD_MAX, 12);
	uint16_t torque = float_to_uint(torque_target, -t_max, t_max, 12);

	tmp_buf[0] = (position >> 8);
	tmp_buf[1] = (uint8_t) (position & 0xFF);
	tmp_buf[2] = (uint8_t) (velocity >> 4);
	tmp_buf[3] = (uint8_t) (((velocity & 0x0F) << 4) | (kp >> 8));
	tmp_buf[4] = (uint8_t) (kp & 0xFF);
	tmp_buf[5] = (uint8_t) (kd >> 4);
	tmp_buf[6] = (uint8_t) (((kd & 0x0F) << 4) | (torque >> 8));
	tmp_buf[7] = (uint8_t) (torque & 0xFF);

	/* 数据发送 */
	{
		CAN_TxHeaderTypeDef CAN_Header;
		uint32_t canTxMailbox;

		CAN_Header.StdId = 0x400 | dev_addr; /* 当前命令标识符的最高位Bit[10]需置1 */
		CAN_Header.IDE = CAN_ID_STD;
		CAN_Header.RTR = CAN_RTR_DATA;
		CAN_Header.DLC = sizeof(tmp_buf);

		if (HAL_CAN_AddTxMessage(hCan, &CAN_Header, tmp_buf,
				(uint32_t*) &canTxMailbox) == HAL_OK) {
		}
	}
}

