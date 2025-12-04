#ifndef _ZE300_485_H
#define _ZE300_485_H

#include <Arduino.h>
#include <HardwareSerial.h>

#define RS485_Serial Serial2
#define RX_LEN 256
#define dw digitalWrite

struct softwareInfo_t // 0x0a
{
	uint16_t bootVer;
	uint16_t appVer;
	uint16_t hardwareVer;
	uint8_t rs485SelfDefVer;
	uint8_t rs485ModbusVer;
	uint8_t canSelfDefVer;
	uint8_t canCanopenVer;
	uint8_t uid[12];
};
struct realTimeData_t // 0x0b
{
	float singleAbsAngle; // Deg
	float multiAbsAngle;  // Deg
	float mechVelocity;	  // Deg/s
	float qAxisCurrent;	  // A
	float torque;		  // N/m,derived
	float busVoltage;	  // V
	float busCurrent;	  // A
	uint8_t temperature;  // ℃
	uint8_t runStatus;	  // status
	uint8_t motorStatue;  // status
	uint8_t errorCode;
};
struct motorParameter_t // 0x12
{
	char name[16];
	uint8_t polarPairNumber; // Pair
	float phaseResistance;	 // Ohm
	float phaseInductance;	 // mH
	float torqueConstant;	 //(N*m)/A
	uint8_t reduceRatio;	 // Null
};
struct motionParameter_t // 0x14
{
	float positionLoop_Kp;				  // Null
	float positionLoop_Ki;				  // Null
	float maxVel_Of_PosMode;			  // Deg/s
	float velocityLoop_Kp;				  // Null
	float velocityLoop_Ki;				  // Null
	float maxQAxisCurr_of_PosModeVelMode; // A
	float maxTorque_Of_VelModePosMode;	  // N*m, derived
};

struct information_t
{
	softwareInfo_t softwareInfo;
	realTimeData_t realTimeData;
	motorParameter_t motorParameter;
	motionParameter_t motionParameter;
};
enum status
{
	off = 0,
	volatgeControl = 1,
	qAxisCurrentControl = 2,
	velocityControl = 3,
	positionControl = 4,
	motorDisable = 0,
};
struct defaultPara_t
{
	float tor_Const;
	float pos_Kp;
	float pos_Ki;
	float vel_Kp;
	float vel_Ki;
};
class ZE300
{
public:
	void init(long baudRate, int rxPin, int txPin, int tnow_pin);
	void writeData(uint8_t deviceAddr, uint8_t command, uint8_t *data, uint8_t length);
	void readData(uint8_t deviceAddr, uint8_t command, information_t *pInformation);
	void reboot(uint8_t deviceAddr);
	void resetError(uint8_t deviceAddr);
	void disableMotor(uint8_t deviceAddr, information_t *pInformation);
	void torqueControl(uint8_t deviceAddr, float targetTorque, float torqueAcceleration, information_t *pInformation);
	void velocityControl(uint8_t deviceAddr, float targetVelocity, float velocityAcceleration, information_t *pInformation);
	void absolutePosControl(uint8_t deviceAddr, float targetAbsolutePosition, information_t *pInformation);
	void relativePosControl(uint8_t deviceAddr, float targetRelativePosition, information_t *pInformation);
	void multiCloseLoopControl(uint8_t deviceAddr, float velocity_Of_PosMode, float torque_Of_VelModePosMode, information_t *pInformation);
	void refreshAllInformation(uint8_t deviceAddr, information_t *pInformation, bool isDisplay);
	void getSoftwareInfo(uint8_t deviceAddr, information_t *pInformation, bool isDisplay);
	void getRealTimeData(uint8_t deviceAddr, information_t *pInformation, bool isDisplay);
	void getMotorParameter(uint8_t deviceAddr, information_t *pInformation, bool isDisplay);
	void getMotionParameter(uint8_t deviceAddr, information_t *pInformation, bool isDisplay);
	void flushRXBuffer();
	uint16_t calcCRC(uint8_t *Frame, uint16_t Len);
	bool isDebug = 0;			   // 是否开启debug，串口显示发送的数据以及其他调试信息
	uint8_t idStartAt = 3;		   // 电机的ID从几开始（ID必须连续）
	uint8_t responseTimeDelay = 7; // 在发出一条需要或会有回复的数据后，多少毫秒内会回复完整

	enum
	{
		eRestart = 0x00,					// 软件复位
		eGetVerInfo = 0x0A,					// 获取Boot软件版本、应用软件版本、硬件版本、RS485协议版本、CAN协议版本、UID
		eGetRealtimeInfo = 0x0B,			// 读取实时数据（单圈绝对值角度、多圈绝对值角度、速度、Q轴电流、母线电压、母线电流、工作温度、运行状态、电机状态、故障码）
		eResetFault = 0x0F,					// 清除故障
		eGetUserPara = 0x10,				// 读取用户参数
		eWriteAndSaveUserPara = 0x11,		// 写用户参数
		eGetMotorPara = 0x12,				// 读取电机参数
		eWriteAndSaveMotorPara = 0x13,		// 写电机硬件参数
		eGetMotionCtrlPara = 0x14,			// 读取运动控制参数
		eWriteMotionCtrlPara = 0x15,		// 写入运动控制参数
		eWriteAndSaveMotionCtrlPara = 0x16, // 写入并保存运动控制参数
		eSetZeroByCurrentRawAngle = 0x1D,	// 设置当前位置为原点
		eStartCalibEncoder = 0x1E,			// 开始编码器校准
		eRestSysPara = 0x1F,				// 参数恢复默认值
		eTorqueCtrl = 0x20,					// 力矩控制
		eVelocityCtrl = 0x21,				// 速度控制
		eAbsolutePositionCtrl = 0x22,		// 绝对值位置控制
		eRelativePositionCtrl = 0x23,		// 相对位置控制
		eShortestHomePosition = 0x24,		// 最短距离回原点
		eDisableCtrl = 0x2F,				// 失能电机输出
		UART_TX_PROTOCOL_HEADER_V3 = 0xAE,
		UART_RX_PROTOCOL_HEADER_V3 = 0xAC,
		// RS485_TNOW_TX_LEVEL			= HIGH,
		// RS485_TNOW_RX_LEVEL			= LOW,
	};
};

#endif