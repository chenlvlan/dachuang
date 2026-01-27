// DengFOC V0.2
// 灯哥开源，遵循GNU协议，转载请著名版权！
// GNU开源协议（GNU General Public License, GPL）是一种自由软件许可协议，保障用户能够自由地使用、研究、分享和修改软件。
// 该协议的主要特点是，要求任何修改或衍生的作品必须以相同的方式公开发布，即必须开源。此外，该协议也要求在使用或分发软件时，必须保留版权信息和许可协议。GNU开源协议是自由软件基金会（FSF）制定和维护的一种协议，常用于GNU计划的软件和其他自由软件中。
// 仅在DengFOC官方硬件上测试过，欢迎硬件购买/支持作者，淘宝搜索店铺：灯哥开源
// 你的支持将是接下来做视频和持续开源的经费，灯哥在这里先谢谢大家了

#include "DengFOC.h"
#include <Arduino.h>
#include "esp_system.h" // 核心复位API
#include "esp_pm.h"		// 深度复位相关（可选）

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

struct motorCommand
{
	uint8_t mode;
	float m0target;
	float m1target;
};

motorCommand motorCmd;

uint8_t buf[16] = {0};
uint8_t *pBuf;
String inputString = "";
const bool isDebug = 0;
const uint8_t cmdSource = 1;
const float torqueConstant = 0.2;

void comm();
void parseCommand(String cmd);
void printPara();

void setup()
{
	pBuf = &buf[0];
	Serial.begin(115200);					   // 调试串口
	Serial1.begin(115200, SERIAL_8N1, 16, 17); // 通信串口

	DFOC_enable(); // 放在校准前
	DFOC_Vbus(24); // 设定驱动器供电电压
	DFOC_M0_alignSensor(11, -1);
	DFOC_M1_alignSensor(11, -1);
}

int count = 0;
void loop()
{
	runFOC();
	// DFOC_M0_setTorque(1);
	// DFOC_M1_setTorque(1);

	// 力位（加入电流环后）
	//  DFOC_M0_SET_ANGLE_PID(0.5,0,0.003,100000,0.1);
	//  DFOC_M0_SET_CURRENT_PID(1.25,50,0,100000);
	//  DFOC_M0_set_Force_Angle(serial_motor_target());
	//  DFOC_M1_SET_ANGLE_PID(0.5,0,0.003,100000,0.1);
	//  DFOC_M1_SET_CURRENT_PID(1.25,50,0,100000);
	//  DFOC_M1_set_Force_Angle(serial_motor_target());

	// 速度（加入电流环后）
	//  DFOC_M0_SET_VEL_PID(3,2,0,100000,0.5);
	//  DFOC_M0_SET_CURRENT_PID(0.5,50,0,100000);
	//  DFOC_M0_setVelocity(serial_motor_target());
	//  DFOC_M0_SET_VEL_PID(0.1,2,0,100000,0.5);
	//  DFOC_M0_SET_CURRENT_PID(0.5,50,0,100000);
	//  DFOC_M0_setVelocity(serial_motor_target());

	// //位置-速度-力（加入电流环后）
	// DFOC_M0_SET_ANGLE_PID(1, 0, 0, 100000, 30);
	// DFOC_M0_SET_VEL_PID(0.02, 1, 0, 100000, 0.5);
	// DFOC_M0_SET_CURRENT_PID(5, 200, 0, 100000);
	// DFOC_M0_set_Velocity_Angle(serial_motor_target());

	// //位置-速度-力（加入电流环后）
	// DFOC_M1_SET_ANGLE_PID(1, 0, 0, 100000, 30);
	// DFOC_M1_SET_VEL_PID(0.02, 1, 0, 100000, 0.5);
	// DFOC_M1_SET_CURRENT_PID(5, 200, 0, 100000);
	// DFOC_M1_set_Velocity_Angle(serial_motor_target());
	// 电流力矩

	DFOC_M0_SET_VEL_PID(0.002, 0, 0, 50000, 1);
	DFOC_M1_SET_VEL_PID(0.002, 0, 0, 50000, 1);
	DFOC_M0_SET_CURRENT_PID(5, 200, 0, 100000, 3);
	DFOC_M1_SET_CURRENT_PID(5, 200, 0, 100000, 3);

	// DFOC_M1_SET_CURRENT_PID(1.2, 0, 0, 100000, 5);
	// DFOC_M1_SET_CURRENT_PID(1.2, 0, 0, 100000, 5);

	if (motorCmd.mode == 0)
	{
		DFOC_disable();
	}
	else if (motorCmd.mode == 1)
	{
		DFOC_enable();
		DFOC_M0_setVelocity(motorCmd.m0target);
		DFOC_M1_setVelocity(motorCmd.m1target);
	}
	else if (motorCmd.mode == 2)
	{
		DFOC_enable();
		motorCmd.m0target = _constrain(motorCmd.m0target, -0.11, 0.11);
		motorCmd.m1target = _constrain(motorCmd.m1target, -0.11, 0.11); // 限制力矩
		DFOC_M0_setTorque(motorCmd.m0target / torqueConstant);
		DFOC_M1_setTorque(motorCmd.m1target / torqueConstant);
	}
	else if (motorCmd.mode == 255)
	{
		// ESP.restart();
		// esp_reset_hw();
		ESP.deepSleep(0);
	}
	// DFOC_M0_setTorque(serial_motor_target());
	// DFOC_M1_setTorque(serial_motor_target());
	// DFOC_M0_setVelocity(serial_motor_target());
	// DFOC_M1_setVelocity(serial_motor_target());

	count++;
	if (count > 100)
	{
		// Serial.println(motorCmd.m0target);
		count = 0;
		// Serial.printf("%f\n", DFOC_M0_Current());
		// Serial.printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", DFOC_M0_Current(), DFOC_M1_Current(), DFOC_M0_Angle(), DFOC_M0_Velocity(), serial_motor_target());
		// Serial.printf("%f,%f,%f\n", DFOC_M0_Angle(), S0_electricalAngle(),S1_electricalAngle());
		// Serial.printf("%f,%f,%f\n", DFOC_M0_Current(), DFOC_M1_Current(),serial_motor_target());
		Serial.printf("M0tar=%.3f\tI0=%.3f\tω0=%.3f\tM1tar=%.3f\tI1=%.3f\tω1=%.3f\n", motorCmd.m0target, DFOC_M0_Current(), DFOC_M0_Velocity(), motorCmd.m1target, DFOC_M1_Current(), DFOC_M1_Velocity());
	}
	// 接收串口
	// serialReceiveUserCommand();
	comm();

	// Serial.println(motorCmd.m0target);
}

void comm()
{
	if (cmdSource == 0)
	{
		while (Serial.available())
		{
			char inChar = (char)Serial.read();
			if (inChar == '\0')
			{
				continue; // ⭐⭐⭐ 关键
			}
			if (inChar == '\n')
			{
				parseCommand(inputString);
				inputString = "";
				printPara();
			}
			else if (inChar != '\r')
			{
				inputString += inChar;
			}
		}
	}
	else if (cmdSource == 1)
	{
		while (Serial1.available())
		{
			uint8_t tmp = Serial1.read();
			if (tmp == '\n')
			{
				if (isDebug)
				{
					/*
					 for (int i = 0; i < 9; i++)
					 {
						 Serial.print(buf[i], HEX);
					 }
					 Serial.print("\n");
					 */
				}
				pBuf = &buf[0]; // 指针归位
				motorCmd.mode = buf[0];
				uint32_t m0byte = (uint32_t)buf[4] << 24 | (uint32_t)buf[3] << 16 | (uint32_t)buf[2] << 8 | (uint32_t)buf[1];
				uint32_t m1byte = (uint32_t)buf[8] << 24 | (uint32_t)buf[7] << 16 | (uint32_t)buf[6] << 8 | (uint32_t)buf[5];
				memcpy(&motorCmd.m0target, &m0byte, 4);
				memcpy(&motorCmd.m1target, &m1byte, 4);
				if (isnanf(motorCmd.m0target) || isnanf(motorCmd.m1target))
				{
					motorCmd.m0target = 0;
					motorCmd.m1target = 0;
				}
				motorCmd.m1target = (0 - motorCmd.m1target);
				printPara();
			}
			else
			{
				*pBuf = tmp; // 接收到的字节写入缓存
				pBuf++;		 // 指针加一
			}
		}
	}
}

void parseCommand(String cmd)
{
	// 分割字符串
	cmd.trim();
	// Serial.print("\"");
	// if (isDebug)
	// CmdSource.println(cmd);
	// Serial.println("\"");
	int firstComma = cmd.indexOf(',');
	int secondComma = cmd.indexOf(',', firstComma + 1);

	if (firstComma == -1 || secondComma == -1)
	{
		// if (isDebug)
		Serial.println("command format error");
		return;
	}

	String modeStr = cmd.substring(0, firstComma);
	String m0targetStr = cmd.substring(firstComma + 1, secondComma);
	String m1targetStr = cmd.substring(secondComma + 1);

	motorCmd.mode = modeStr.toInt();
	motorCmd.m0target = m0targetStr.toFloat();
	motorCmd.m1target = m1targetStr.toFloat();
}

void printPara()
{
	if (isDebug)
	{
		Serial.printf("mode = %d\tM0 target = %.3f\tM1 target = %.3f\n", motorCmd.mode, motorCmd.m0target, motorCmd.m1target);
	}
}