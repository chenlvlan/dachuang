#include <ZE300_485.h>

typedef struct
{
	uint8_t head;			 /* 协议头 */
	uint8_t pack_num;		 /* 包序号 */
	uint8_t addr;			 /* RS485地址 */
	uint8_t cmd;			 /* 命令码 */
	uint8_t data_len;		 /* 数据字段长度 */
	uint8_t buf[RX_LEN - 5]; /* 数据段 */
} UartProtocolData_t;

defaultPara_t defPara{0.52, 1.0, 0.0, 0.5, 0.005};

uint8_t comm_uart_rx_buffer[RX_LEN] = {0};
uint8_t comm_uart_tx_buffer[RX_LEN] = {0};

static const uint8_t CRCHi[] = {
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40};

static const uint8_t CRCLo[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
	0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
	0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
	0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
	0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
	0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
	0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
	0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
	0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
	0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
	0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
	0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
	0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
	0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
	0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
	0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
	0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
	0x41, 0x81, 0x80, 0x40};

uint16_t ZE300::calcCRC(uint8_t *Frame, uint16_t Len)
{
	uint8_t ucCRCHi = 0xFF;
	uint8_t ucCRCLo = 0xFF;
	int iIndex;
	while (Len--)
	{
		iIndex = ucCRCLo ^ *(Frame++);
		ucCRCLo = (uint8_t)(ucCRCHi ^ CRCHi[iIndex]);
		ucCRCHi = CRCLo[iIndex];
	}
	return (uint16_t)(ucCRCHi << 8 | ucCRCLo);
}

// 初始化
void ZE300::init(long baudRate, int rxPin, int txPin, int tnow_pin)
{
	RS485_Serial.begin(baudRate);
	RS485_Serial.setPins(rxPin, txPin, -1, tnow_pin);
	RS485_Serial.setMode(MODE_RS485_HALF_DUPLEX);
	delay(100);
	for (int i = 0; i < 255; i++) // 清空串口缓存
	{
		RS485_Serial.read();
	}
	// tnowPin=tnow_pin;
	// pinMode(tnowPin,OUTPUT);
	// dw(tnowPin,RS485_TNOW_RX_LEVEL);
}

// 写数据给驱动器，设备地址，命令码，数据包，数据包长度
void ZE300::writeData(uint8_t deviceAddr, uint8_t command, uint8_t *data, uint8_t length)
{
	UartProtocolData_t *pData = (UartProtocolData_t *)comm_uart_tx_buffer;
	pData->head = UART_TX_PROTOCOL_HEADER_V3;
	pData->pack_num = 0;
	pData->addr = deviceAddr;
	pData->cmd = command;
	pData->data_len = length;
	memcpy(&pData->buf[0], data, length);

	uint16_t tmp_crc = calcCRC(comm_uart_tx_buffer, (length + 5));
	pData->buf[length] = (uint8_t)tmp_crc;
	pData->buf[length + 1] = (uint8_t)(tmp_crc >> 8);

	if (isDebug)
	{
		Serial.print("Transmitted = ");
		for (int i = 0; i < length + 7; i++)
		{
			if (comm_uart_tx_buffer[i] <= 0x0f)
			{
				Serial.print("0");
			}
			Serial.print(comm_uart_tx_buffer[i], HEX);
			Serial.print(" ");
		}
		Serial.print("\n");
	}

	// dw(tnowPin,RS485_TNOW_TX_LEVEL);
	RS485_Serial.write(&comm_uart_tx_buffer[0], length + 7);
	// delay(2);
	// dw(tnowPin,RS485_TNOW_RX_LEVEL);
	memset(&comm_uart_tx_buffer, 0x00, length + 7);
}

// 读驱动器返回的数据
void ZE300::readData(uint8_t deviceAddr, uint8_t command, information_t *pInformation)
{
	delay(responseTimeDelay);
	memset(&comm_uart_rx_buffer, 0x00, RX_LEN); // 缓存置零，准备接收新数据
	uint8_t expectLength;
	switch (command)
	{
	case eGetVerInfo:
		expectLength = 29;
		break;
	case eGetRealtimeInfo:
	case eTorqueCtrl:
	case eVelocityCtrl:
	case eAbsolutePositionCtrl:
	case eRelativePositionCtrl:
	case eShortestHomePosition:
	case eDisableCtrl:
		expectLength = 29;
		break;
	case eGetMotorPara:
	case eWriteAndSaveMotorPara:
		expectLength = 37;
		break;
	case eGetMotionCtrlPara:
	case eWriteMotionCtrlPara:
	case eWriteAndSaveMotionCtrlPara:
		expectLength = 31;
		break;
	default:
		break;
	}

	uint8_t length = RS485_Serial.available(); // 串口缓冲区数据的长度
	if (isDebug)
	{
		Serial.print("Real length = ");
		Serial.println(length);
		Serial.print("Expected length = ");
		Serial.println(expectLength);
		// Serial.println("");
	}

	// 临时修改为小于等于，应为大于等于
	if (length /* >= expectLength*/) // 长度大于等于指令码对应的返回数据长度才可能是完整数据
	{
		RS485_Serial.readBytes(&comm_uart_rx_buffer[0], expectLength); // 将串口缓冲区数据读入缓存
		if (isDebug)
		{
			Serial.print("\nReceived = ");
			for (int i = 0; i < length; i++)
			{
				if (comm_uart_rx_buffer[i] <= 0x0f)
				{
					Serial.print("0");
				}
				Serial.print(comm_uart_rx_buffer[i], HEX);
				Serial.print(" ");
			}
			Serial.println("");
		}
		if (calcCRC(&comm_uart_rx_buffer[0], length) == 0) // 如果CRC校验无误
		{
			UartProtocolData_t *pData = (UartProtocolData_t *)comm_uart_rx_buffer;		// 缓存转化为数据协议的格式，指针指向缓存
			if (pData->head == UART_RX_PROTOCOL_HEADER_V3 && pData->addr == deviceAddr) // 确认协议头和设备地址
			{
				if (isDebug)
				{
					Serial.println("Valid data");
				}
				switch (pData->cmd)
				{
				case eGetVerInfo:
				{ // 0x0a
					memcpy(&(pInformation->softwareInfo.bootVer), &(pData->buf[0]), 2);
					memcpy(&(pInformation->softwareInfo.appVer), &(pData->buf[2]), 2);
					memcpy(&(pInformation->softwareInfo.hardwareVer), &(pData->buf[4]), 2);
					pInformation->softwareInfo.rs485SelfDefVer = pData->buf[6];
					pInformation->softwareInfo.rs485ModbusVer = pData->buf[7];
					pInformation->softwareInfo.canSelfDefVer = pData->buf[8];
					pInformation->softwareInfo.canCanopenVer = pData->buf[9];
					// memcpy(&(pInformation->softwareInfo.rs485SelfDefVer), &(pData->buf[6]), 1);
					// memcpy(&(pInformation->softwareInfo.rs485ModbusVer), &(pData->buf[7]), 1);
					// memcpy(&(pInformation->softwareInfo.canSelfDefVer), &(pData->buf[8]), 1);
					// memcpy(&(pInformation->softwareInfo.canCanopenVer), &(pData->buf[9]), 1);
					memcpy(&(pInformation->softwareInfo.uid[0]), &(pData->buf[10]), 12);
				}
				break;

				case eGetRealtimeInfo:
				case eTorqueCtrl:
				case eVelocityCtrl:
				case eAbsolutePositionCtrl:
				case eRelativePositionCtrl:
				case eShortestHomePosition:
				case eDisableCtrl:
				{ // 0x0b
					uint16_t tmp_2u;
					int32_t tmp_4s;
					memcpy(&tmp_2u, &(pData->buf[0]), 2); // 单圈绝对值角度[0]-[1]
					pInformation->realTimeData.singleAbsAngle = float(float(tmp_2u * 360) / 16384);
					memcpy(&tmp_4s, &(pData->buf[2]), 4); // 多圈绝对值角度[2]-[5]
					pInformation->realTimeData.multiAbsAngle = float(float(tmp_4s * 360) / 16384);
					memcpy(&tmp_4s, &(pData->buf[6]), 4); // 机械速度[6]-[9]
					pInformation->realTimeData.mechVelocity = float(float(tmp_4s * 360) / 6000);
					memcpy(&tmp_4s, &(pData->buf[10]), 4); // Q轴电流[10]-[13]
					pInformation->realTimeData.qAxisCurrent = float(float(tmp_4s) / 1000);
					if (pInformation->motorParameter.torqueConstant == 0)
					{
						pInformation->realTimeData.torque = float(pInformation->realTimeData.qAxisCurrent * defPara.tor_Const);
					}
					else
					{
						pInformation->realTimeData.torque = float(pInformation->realTimeData.qAxisCurrent * pInformation->motorParameter.torqueConstant);
					}
					memcpy(&tmp_2u, &(pData->buf[14]), 2); // 母线电压[14]-[15]
					pInformation->realTimeData.busVoltage = float(float(tmp_2u) / 100);
					memcpy(&tmp_2u, &(pData->buf[16]), 2); // 母线电流[16]-[17]
					pInformation->realTimeData.busCurrent = float(float(tmp_2u) / 100);
					pInformation->realTimeData.temperature = pData->buf[18]; // 工作温度[18]
					pInformation->realTimeData.runStatus = pData->buf[19];	 // 运行模式[19]
					pInformation->realTimeData.motorStatue = pData->buf[20]; // 电机状态[20]
					pInformation->realTimeData.errorCode = pData->buf[21];	 // 故障码[21]
				}
				break;

				case eGetMotionCtrlPara:
				case eWriteMotionCtrlPara:
				case eWriteAndSaveMotionCtrlPara:
				{ // 0x14
					uint32_t tmp_4u;
					memcpy(&(pInformation->motionParameter.positionLoop_Kp), &(pData->buf[0]), 4);
					memcpy(&(pInformation->motionParameter.positionLoop_Ki), &(pData->buf[4]), 4);
					memcpy(&tmp_4u, &(pData->buf[8]), 4);
					pInformation->motionParameter.maxVel_Of_PosMode = float(float(tmp_4u * 360) / 6000);
					memcpy(&(pInformation->motionParameter.velocityLoop_Kp), &(pData->buf[12]), 4);
					memcpy(&(pInformation->motionParameter.velocityLoop_Ki), &(pData->buf[16]), 4);
					memcpy(&tmp_4u, &(pData->buf[20]), 4);
					pInformation->motionParameter.maxQAxisCurr_of_PosModeVelMode = float(float(tmp_4u) / 1000);
					if (pInformation->motorParameter.torqueConstant == 0)
					{
						pInformation->motionParameter.maxTorque_Of_VelModePosMode = float(pInformation->motionParameter.maxQAxisCurr_of_PosModeVelMode * defPara.tor_Const);
					}
					else
					{
						pInformation->motionParameter.maxTorque_Of_VelModePosMode = float(pInformation->motionParameter.maxQAxisCurr_of_PosModeVelMode * pInformation->motorParameter.torqueConstant);
					}
				}
				break;

				case eGetMotorPara:
				case eWriteAndSaveMotorPara:
				{
					memcpy(&(pInformation->motorParameter.name), &(pData->buf[0]), 16);
					memcpy(&(pInformation->motorParameter.polarPairNumber), &(pData->buf[16]), 1);
					memcpy(&(pInformation->motorParameter.phaseResistance), &(pData->buf[17]), 4);
					memcpy(&(pInformation->motorParameter.phaseInductance), &(pData->buf[21]), 4);
					memcpy(&(pInformation->motorParameter.torqueConstant), &(pData->buf[25]), 4);
					memcpy(&(pInformation->motorParameter.reduceRatio), &(pData->buf[29]), 1);
				}
				default:
					break;
				}
			}
		}
		else if (isDebug)
		{
			Serial.println("Invalid or incomplete data");
		}
	}
}

// 重启驱动器，要转到零位附近
void ZE300::reboot(uint8_t deviceAddr)
{
	writeData(deviceAddr, eRestart, NULL, 0);
}

// 复位错误
void ZE300::resetError(uint8_t deviceAddr)
{
	writeData(deviceAddr, eResetFault, NULL, 0);
	delay(responseTimeDelay);
	flushRXBuffer();
}

// 电机卸力（关闭电机输出）
void ZE300::disableMotor(uint8_t deviceAddr, information_t *pInformation)
{
	writeData(deviceAddr, eDisableCtrl, NULL, 0);
	readData(deviceAddr, eDisableCtrl, pInformation);
}

// 力矩控制，unit: N*m, (N*m)/s, (N*m)/A
void ZE300::torqueControl(uint8_t deviceAddr, float targetTorque, float torqueAcceleration, information_t *pInformation)
{
	byte data[8];
	int32_t targetTor = int32_t((targetTorque * 1000) / pInformation->motorParameter.torqueConstant);
	uint32_t torConst = uint32_t((abs(torqueAcceleration) * 1000) / pInformation->motorParameter.torqueConstant);
	memcpy(&data[0], &targetTor, 4);
	memcpy(&data[4], &torConst, 4);
	writeData(deviceAddr, eTorqueCtrl, &data[0], 8);
	readData(deviceAddr, eTorqueCtrl, pInformation);
}

// 速度控制，unit: Deg/s, (Deg/s)/s
void ZE300::velocityControl(uint8_t deviceAddr, float targetVelocity, float velocityAcceleration, information_t *pInformation)
{
	byte data[8];
	int32_t targetVel = int32_t(targetVelocity * 60 * 100);
	uint32_t velAcc = uint32_t(abs(velocityAcceleration) * 60 * 100);
	memcpy(&data[0], &targetVel, 4);
	memcpy(&data[4], &velAcc, 4);
	writeData(deviceAddr, eVelocityCtrl, &data[0], 8);
	readData(deviceAddr, eVelocityCtrl, pInformation);
}

// 绝对位置控制，unit: Deg
void ZE300::absolutePosControl(uint8_t deviceAddr, float targetAbsolutePosition, information_t *pInformation)
{
	byte data[4];
	int32_t targetAbsPos = int32_t((targetAbsolutePosition * 16384) / 360);
	memcpy(&data[0], &targetAbsPos, 4);
	writeData(deviceAddr, eAbsolutePositionCtrl, &data[0], 4);
	readData(deviceAddr, eAbsolutePositionCtrl, pInformation);
}

// 相对位置控制，unit: Deg
void ZE300::relativePosControl(uint8_t deviceAddr, float targetRelativePosition, information_t *pInformation)
{
	byte data[4];
	int32_t targetRelaPos = int32_t((targetRelativePosition * 16384) / 360);
	memcpy(&data[0], &targetRelaPos, 4);
	writeData(deviceAddr, eRelativePositionCtrl, &data[0], 4);
	readData(deviceAddr, eRelativePositionCtrl, pInformation);
}

// 多闭环控制，用于设置某种控制模式下另一个环的限制，unit: Deg/s, N*m
void ZE300::multiCloseLoopControl(uint8_t deviceAddr, float velocity_Of_PosMode, float torque_Of_VelModePosMode, information_t *pInformation)
{
	byte data[24];
	uint32_t vel = uint32_t((velocity_Of_PosMode * 60 * 100) / 360);
	uint32_t qAxisCurr = uint32_t((torque_Of_VelModePosMode * 1000) / pInformation->motorParameter.torqueConstant);
	memcpy(&data[0], &(pInformation->motionParameter.positionLoop_Kp), 4);
	memcpy(&data[4], &(pInformation->motionParameter.positionLoop_Ki), 4);
	memcpy(&data[8], &vel, 4);
	memcpy(&data[12], &(pInformation->motionParameter.velocityLoop_Kp), 4);
	memcpy(&data[16], &(pInformation->motionParameter.velocityLoop_Ki), 4);
	memcpy(&data[20], &qAxisCurr, 4);
	writeData(deviceAddr, eWriteMotionCtrlPara, &data[0], 24);
	readData(deviceAddr, eWriteMotionCtrlPara, pInformation);
}

void ZE300::refreshAllInformation(uint8_t deviceAddr, information_t *pInformation, bool isDisplay)
{
	if (isDisplay)
	{
		Serial.print("Information of ID ");
		Serial.print(deviceAddr);
		Serial.println(":");
	}
	getSoftwareInfo(deviceAddr, pInformation, isDisplay);
	getRealTimeData(deviceAddr, pInformation, isDisplay);
	getMotorParameter(deviceAddr, pInformation, isDisplay);
	getMotionParameter(deviceAddr, pInformation, isDisplay);
}

void ZE300::getSoftwareInfo(uint8_t deviceAddr, information_t *pInformation, bool isDisplay)
{
	writeData(deviceAddr, eGetVerInfo, NULL, 0);
	readData(deviceAddr, eGetVerInfo, pInformation);
	if (isDisplay)
	{
		Serial.println("");
		Serial.println("softwareInfo:");
		Serial.print("bootVer = ");
		Serial.println(pInformation->softwareInfo.bootVer);
		Serial.print("appVer = ");
		Serial.println(pInformation->softwareInfo.appVer);
		Serial.print("hardwareVer = ");
		Serial.println(pInformation->softwareInfo.hardwareVer, HEX);
		Serial.print("rs485SelfDefVer = ");
		Serial.println(pInformation->softwareInfo.rs485SelfDefVer);
		Serial.print("rs485ModbusVer = ");
		Serial.println(pInformation->softwareInfo.rs485ModbusVer);
		Serial.print("canSelfDefVer = ");
		Serial.println(pInformation->softwareInfo.canSelfDefVer);
		Serial.print("canCanopenVer = ");
		Serial.println(pInformation->softwareInfo.canCanopenVer);
		Serial.print("uid = ");
		for (int i = 0; i < 12; i++)
		{
			if (pInformation->softwareInfo.uid[i] <= 0x0f)
			{
				Serial.print("0");
			}
			Serial.print(pInformation->softwareInfo.uid[i], HEX);
		}
		Serial.println("");
	}
}

void ZE300::getRealTimeData(uint8_t deviceAddr, information_t *pInformation, bool isDisplay)
{
	writeData(deviceAddr, eGetRealtimeInfo, NULL, 0);
	readData(deviceAddr, eGetRealtimeInfo, pInformation);
	if (isDisplay)
	{
		Serial.println("");
		Serial.println("realTimeData:");
		Serial.print("singleAbsAngle = ");
		Serial.println(pInformation->realTimeData.singleAbsAngle);
		Serial.print("multiAbsAngle = ");
		Serial.println(pInformation->realTimeData.multiAbsAngle);
		Serial.print("mechVelocity = ");
		Serial.println(pInformation->realTimeData.mechVelocity);
		Serial.print("qAxisCurrent = ");
		Serial.println(pInformation->realTimeData.qAxisCurrent);
		Serial.print("torque = ");
		Serial.println(pInformation->realTimeData.torque);
		Serial.print("busVoltage = ");
		Serial.println(pInformation->realTimeData.busVoltage);
		Serial.print("busCurrent = ");
		Serial.println(pInformation->realTimeData.busCurrent);
		Serial.print("temperature = ");
		Serial.println(pInformation->realTimeData.temperature);
		Serial.print("runStatus = ");
		Serial.println(pInformation->realTimeData.runStatus);
		Serial.print("motorStatue = ");
		Serial.println(pInformation->realTimeData.motorStatue);
		Serial.print("errorCode = ");
		Serial.println(pInformation->realTimeData.errorCode);
		Serial.println("");
	}
}

void ZE300::getMotorParameter(uint8_t deviceAddr, information_t *pInformation, bool isDisplay)
{
	writeData(deviceAddr, eGetMotorPara, NULL, 0);
	readData(deviceAddr, eGetMotorPara, pInformation);
	if (isDisplay)
	{
		Serial.println("");
		Serial.println("motorParameter:");
		Serial.print("name = ");
		for (int i = 0; i < 16; i++)
		{
			if (pInformation->motorParameter.name[i] == 0)
			{
				break;
			}
			Serial.print(char(pInformation->motorParameter.name[i]));
		}
		Serial.println("");
		Serial.print("polarPairNumber = ");
		Serial.println(pInformation->motorParameter.polarPairNumber);
		Serial.print("phaseResistance = ");
		Serial.println(pInformation->motorParameter.phaseResistance);
		Serial.print("phaseInductance = ");
		Serial.println(pInformation->motorParameter.phaseInductance);
		Serial.print("torqueConstant = ");
		Serial.println(pInformation->motorParameter.torqueConstant);
		Serial.print("reduceRatio = ");
		Serial.println(pInformation->motorParameter.reduceRatio);
		Serial.println("");
	}
}

void ZE300::getMotionParameter(uint8_t deviceAddr, information_t *pInformation, bool isDisplay)
{
	writeData(deviceAddr, eGetMotionCtrlPara, NULL, 0);
	readData(deviceAddr, eGetMotionCtrlPara, pInformation);
	if (isDisplay)
	{
		Serial.println("");
		Serial.println("motionParameter:");
		Serial.print("positionLoop_Kp = ");
		Serial.println(pInformation->motionParameter.positionLoop_Kp);
		Serial.print("positionLoop_Ki = ");
		Serial.println(pInformation->motionParameter.positionLoop_Ki);
		Serial.print("maxVel_Of_PosMode = ");
		Serial.println(pInformation->motionParameter.maxVel_Of_PosMode);
		Serial.print("velocityLoop_Kp = ");
		Serial.println(pInformation->motionParameter.velocityLoop_Kp);
		Serial.print("velocityLoop_Ki = ");
		Serial.println(pInformation->motionParameter.velocityLoop_Ki);
		Serial.print("maxQAxisCurr_of_PosModeVelMode = ");
		Serial.println(pInformation->motionParameter.maxQAxisCurr_of_PosModeVelMode);
		Serial.print("maxTorque_Of_VelModePosMode = ");
		Serial.println(pInformation->motionParameter.maxTorque_Of_VelModePosMode);
		Serial.println("");
	}
}

void ZE300::flushRXBuffer()
{
	while (RS485_Serial.available())
	{
		RS485_Serial.read();
	}
}
