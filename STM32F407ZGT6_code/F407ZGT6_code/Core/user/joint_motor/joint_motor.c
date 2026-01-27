/*
 * joint_motor.c
 *
 *  Created on: Jan 27, 2026
 *      Author: yufei
 */

#include "joint_motor.h"

void JM_Disable(uint8_t id) {
	CommCan_MotorDisableCtrl(idToHandle(id), id);
}

void JM_SetPosVelModeMaxTorque(uint8_t id, float torque) {
	if (JMDataRead[idToIndex(id)].torqueConst == 0) { //力矩常数为0，那就现读
		JM_GetMotorInfo(id, 1); //使用强制模式，因为这个时候力矩常数为0，其实已经是异常情况了
	}
	CommCan_SetPosVelCtrlMaxIq(idToHandle(id), id,
			(torque / JMDataRead[idToIndex(id)].torqueConst));
}

void JM_GetSoftwareInfo(uint8_t id) {
	CommCan_GetVerInfo(idToHandle(id), id);
}

void JM_GetRealTimeStatusInfo(uint8_t id) {
	CommCan_GetStatusInfo(idToHandle(id), id);
}

void JM_GetMotorInfo(uint8_t id, bool forced) {
	CommCan_GetMotorPara(idToHandle(id), id);
	if (forced) {
		//bool safe = 0; //默认是异常的状态
		for (int i = 1; i < 1000; i++) {
			HAL_Delay(1);
			if (JMDataRead[idToIndex(id)].torqueConst != 0) {
				//读到数了
				//safe = 1;
				return;
			}
			if (i % 200 == 0) {
				//每隔200ms自动重新发送指令
				CommCan_GetMotorPara(idToHandle(id), id);
			}
		}
		//运行到这里说明出错了，关节电机失去通信了
	}
}

void JM_VelMode(uint8_t id, float speed) {
	CommCan_SetVelocity(idToHandle(id), id, radpsToRpm(speed));
}

void JM_PosRelaMode(uint8_t id, float position) {
	CommCan_SetRelateive_Count(idToHandle(id), id,
			position * 16384 / (2 * M_PI));
}

void JM_PosAbsMode(uint8_t id, float position) {
	CommCan_SetAbsPosition_Count(idToHandle(id), id,
			position * 16384 / (2 * M_PI));
}

void JM_GetTorque(uint8_t id) {
	if (JMDataRead[idToIndex(id)].torqueConst == 0) { //力矩常数为0，那就现读
		JM_GetMotorInfo(id, 1); //使用强制模式，因为这个时候力矩常数为0，其实已经是异常情况了
	}
	CommCan_GetIq(idToHandle(id), id);
}

void JM_GetPos(uint8_t id) {
	CommCan_GetPosition(idToHandle(id), id);
}

void JM_Restart(uint8_t id) {
	CommCan_SysMotorRestart(idToHandle(id), id);
}

void JM_ReturnToOrigin(uint8_t id) {
	CommCan_ShortestHomePosition(idToHandle(id), id);
}

/* can总线数据的接收 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *canHandle) {

	/*
	 //printf("a interrupt of can");
	 uint8_t rx_addr = 0;
	 uint8_t rx_cmd = 0;
	 */
	HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &CanRxMessage.RxHead,
			CanRxMessage.canRxBuf);
	uint8_t id = CanRxMessage.RxHead.StdId;
	switch (id) {
	case idLF:
		solveMotorCanRx(&JMDataRead[indexLF]);
		break;
	case idLR:
		solveMotorCanRx(&JMDataRead[indexLR]);
		break;
	case idRF:
		solveMotorCanRx(&JMDataRead[indexRF]);
		break;
	case idRR:
		solveMotorCanRx(&JMDataRead[indexRR]);
		break;
	}
	/*

	 rx_addr = CanRxMessage.RxHead.StdId;
	 rx_cmd = CanRxMessage.canRxBuf[0];

	 if (isDebug) {
	 printf("\r\n");
	 printf("rx_addr = %d\r\n", rx_addr);
	 }

	 switch (rx_cmd) {
	 case eCAN_GetVerInfo: {
	 uint16_t BootAppStatus;
	 uint16_t ApplicationStatus;
	 uint16_t HardwardStatus;
	 uint8_t CAN_SelfStatues;

	 memcpy(&BootAppStatus, &(CanRxMessage.canRxBuf[1]),
	 sizeof(BootAppStatus));
	 memcpy(&ApplicationStatus, &(CanRxMessage.canRxBuf[3]),
	 sizeof(ApplicationStatus));
	 memcpy(&HardwardStatus, &(CanRxMessage.canRxBuf[5]),
	 sizeof(HardwardStatus));
	 memcpy(&CAN_SelfStatues, &(CanRxMessage.canRxBuf[7]),
	 sizeof(CAN_SelfStatues));

	 if (isDebug) {
	 printf("Boot_Ver：%d\r\n", BootAppStatus);
	 printf("Sofatware_Ver: %d\r\n", ApplicationStatus);
	 printf("Hardware_Ver: %02X\r\n", HardwardStatus);
	 printf("CAN_AUX_Ver: %02X\r\n", CAN_SelfStatues);
	 }

	 }
	 break;

	 case eCAN_Pos_Kp: {
	 float param_data;
	 memcpy(&param_data, &(CanRxMessage.canRxBuf[1]), sizeof(param_data));

	 if (isDebug) {
	 printf("Pos_Kp：%f\r\n", param_data);
	 }

	 }
	 break;

	 case eCAN_Pos_Ki: {
	 float param_data;
	 memcpy(&param_data, &(CanRxMessage.canRxBuf[1]), sizeof(param_data));

	 if (isDebug) {
	 printf("Pos_Ki：%f\r\n", param_data);
	 }

	 }
	 break;

	 case eCAN_Vel_Kp: {
	 float param_data;
	 memcpy(&param_data, &(CanRxMessage.canRxBuf[1]), sizeof(param_data));

	 if (isDebug) {
	 printf("Vel_Kp：%f\r\n", param_data);
	 }

	 }
	 break;

	 case eCAN_Vel_Ki: {
	 float param_data;
	 memcpy(&param_data, &(CanRxMessage.canRxBuf[1]), sizeof(param_data));

	 if (isDebug) {
	 printf("Vel_Ki：%f\r\n", param_data);
	 }

	 }
	 break;

	 case eCAN_SetIqCtrlTragetVal:
	 case eCAN_GetIq: {
	 int32_t IqCurrentValue;
	 memcpy(&IqCurrentValue, &(CanRxMessage.canRxBuf[1]),
	 sizeof(IqCurrentValue));

	 if (isDebug) {
	 printf("Iq：%fA\r\n", IqCurrentValue * 0.001f);
	 }

	 }
	 break;

	 case eCAN_SetVelCtrlTragetVal:
	 case eCAN_GetVelocity: {
	 int32_t RealtimeVelocity;
	 memcpy(&RealtimeVelocity, &(CanRxMessage.canRxBuf[1]),
	 sizeof(RealtimeVelocity));

	 if (isDebug) {
	 printf("Velocity：%fRpm\r\n", RealtimeVelocity * 0.01);
	 }

	 }
	 break;

	 case eCAN_DisableCtrl:
	 case eCAN_GetStatusInfo: {
	 uint16_t BusVoltage_multi100;
	 uint16_t BusCurrent_multi100;
	 uint8_t Temperature;
	 uint8_t RunMode;
	 uint8_t SysFault;

	 memcpy(&BusVoltage_multi100, &(CanRxMessage.canRxBuf[1]),
	 sizeof(BusVoltage_multi100));
	 memcpy(&BusCurrent_multi100, &(CanRxMessage.canRxBuf[3]),
	 sizeof(BusCurrent_multi100));
	 memcpy(&Temperature, &(CanRxMessage.canRxBuf[5]), sizeof(Temperature));
	 memcpy(&RunMode, &(CanRxMessage.canRxBuf[6]), sizeof(RunMode));
	 memcpy(&SysFault, &(CanRxMessage.canRxBuf[7]), sizeof(SysFault));

	 if (isDebug) {
	 printf("Bus_Voltage:%.2fV\r\n", BusVoltage_multi100 * 0.01);
	 printf("Bus_Current:%.2fA\r\n", BusCurrent_multi100 * 0.01);
	 printf("Work_Temp:%.2f\r\n", Temperature * 1.0);
	 printf("Run_Mode:%d\r\n", RunMode);
	 printf("Fault_Code:%d\r\n", SysFault);
	 }

	 }
	 break;

	 case eCAN_ResetFault: {
	 uint8_t SysFault;
	 memcpy(&SysFault, &(CanRxMessage.canRxBuf[1]), sizeof(SysFault));

	 if (isDebug) {
	 printf("Fault_Code:%d\r\n", SysFault);
	 }

	 }
	 if (isDebug) {
	 printf("Pos_Max:%.2f\r\n", pos_max);
	 printf("Vel_Max:%.2f\r\n", vel_max);
	 printf("T_Max:%.2f\r\n", t_max);
	 }

	 }
	 break;

	 case eCAN_GetMitCtrlModeRealTimeData: {
	 uint8_t *pBuf = CanRxMessage.canRxBuf;

	 float curr_Pos = uint_to_float(((pBuf[1] << 8) | pBuf[2]), -pos_max,
	 pos_max, 16);
	 float curr_vel = uint_to_float(
	 ((pBuf[3] << 4) | ((pBuf[4] & 0xF0) >> 4)), -vel_max, vel_max,
	 12);
	 float curr_torque = uint_to_float((((pBuf[4] & 0x0F) << 8) | pBuf[5]),
	 -t_max, t_max, 12);
	 uint8_t status = pBuf[6];

	 if (isDebug) {
	 printf("current_pos:%.2f\r\n", curr_Pos);
	 printf("current_vel:%.2f\r\n", curr_vel);
	 printf("current_torque:%.2f\r\n", curr_torque);
	 printf("status:%02X\r\n", status);
	 }

	 }
	 break;

	 default:
	 break;
	 }
	 */
}

void solveMotorCanRx(motorDataRead_t *motorDataRead) {

	uint8_t rx_addr = CanRxMessage.RxHead.StdId;
	uint8_t rx_cmd = CanRxMessage.canRxBuf[0];

	if (isDebug) {
		printf("\r\n");
		printf("rx_addr = %d\r\n", rx_addr);
	}

	switch (rx_cmd) {
	case eCAN_GetVerInfo: {
		//uint16_t BootAppStatus;
		//uint16_t ApplicationStatus;
		//uint16_t HardwardStatus;
		//uint8_t CAN_SelfStatues;

		memcpy(&(motorDataRead->bootVer), &(CanRxMessage.canRxBuf[1]),
				sizeof(motorDataRead->bootVer));
		memcpy(&(motorDataRead->appVer), &(CanRxMessage.canRxBuf[3]),
				sizeof(motorDataRead->appVer));
		memcpy(&(motorDataRead->hardwareVer), &(CanRxMessage.canRxBuf[5]),
				sizeof(motorDataRead->hardwareVer));
		memcpy(&(motorDataRead->canSelfDefVer), &(CanRxMessage.canRxBuf[7]),
				sizeof(motorDataRead->canSelfDefVer));

		if (isDebug) {
			printf("Boot_Ver：%d\r\n", motorDataRead->bootVer);
			printf("Sofatware_Ver: %d\r\n", motorDataRead->appVer);
			printf("Hardware_Ver: %02X\r\n", motorDataRead->hardwareVer);
			printf("CAN_AUX_Ver: %02X\r\n", motorDataRead->canSelfDefVer);
		}

	}
		break;

	case eCAN_Pos_Kp: {
		//float param_data;
		memcpy(&(motorDataRead->posKp), &(CanRxMessage.canRxBuf[1]),
				sizeof(motorDataRead->posKp));

		if (isDebug) {
			printf("Pos_Kp：%f\r\n", motorDataRead->posKp);
		}

	}
		break;

	case eCAN_Pos_Ki: {
		//float param_data;
		memcpy(&(motorDataRead->posKi), &(CanRxMessage.canRxBuf[1]),
				sizeof(motorDataRead->posKi));

		if (isDebug) {
			printf("Pos_Ki：%f\r\n", motorDataRead->posKi);
		}

	}
		break;

	case eCAN_Vel_Kp: {
		//float param_data;
		memcpy(&(motorDataRead->velKp), &(CanRxMessage.canRxBuf[1]),
				sizeof(motorDataRead->velKp));

		if (isDebug) {
			printf("Vel_Kp：%f\r\n", motorDataRead->velKp);
		}

	}
		break;

	case eCAN_Vel_Ki: {
		//float param_data;
		memcpy(&(motorDataRead->velKi), &(CanRxMessage.canRxBuf[1]),
				sizeof(motorDataRead->velKi));

		if (isDebug) {
			printf("Vel_Ki：%f\r\n", motorDataRead->velKi);
		}

	}
		break;

	case eCAN_SetIqCtrlTragetVal:
	case eCAN_GetIq: {
		int32_t IqCurrentValue;
		memcpy(&IqCurrentValue, &(CanRxMessage.canRxBuf[1]),
				sizeof(IqCurrentValue));
		motorDataRead->iq = IqCurrentValue * 0.001f;
		motorDataRead->torque = motorDataRead->iq * motorDataRead->torqueConst;
		if (isDebug) {
			printf("Iq：%fA\r\n", motorDataRead->iq);
		}

	}
		break;

	case eCAN_SetVelCtrlTragetVal:
	case eCAN_GetVelocity: {
		int32_t RealtimeVelocity;
		memcpy(&RealtimeVelocity, &(CanRxMessage.canRxBuf[1]),
				sizeof(RealtimeVelocity));
		motorDataRead->mechVel = rpmToRadps(RealtimeVelocity * 0.01);
		if (isDebug) {
			printf("Velocity：%fRad/s\r\n", motorDataRead->mechVel);
		}

	}
		break;

	case eCAN_DisableCtrl:
	case eCAN_GetStatusInfo: {
		uint16_t BusVoltage_multi100;
		uint16_t BusCurrent_multi100;
		//uint8_t Temperature;
		//uint8_t RunMode;
		//uint8_t SysFault;

		memcpy(&BusVoltage_multi100, &(CanRxMessage.canRxBuf[1]),
				sizeof(BusVoltage_multi100));
		memcpy(&BusCurrent_multi100, &(CanRxMessage.canRxBuf[3]),
				sizeof(BusCurrent_multi100));
		memcpy(&(motorDataRead->temperature), &(CanRxMessage.canRxBuf[5]),
				sizeof(motorDataRead->temperature));
		memcpy(&(motorDataRead->motorStatue), &(CanRxMessage.canRxBuf[6]),
				sizeof(motorDataRead->motorStatue));
		memcpy(&(motorDataRead->errorCode), &(CanRxMessage.canRxBuf[7]),
				sizeof(motorDataRead->errorCode));

		motorDataRead->busVolt = BusVoltage_multi100 * 0.01;
		motorDataRead->busCurr = BusCurrent_multi100 * 0.01;

		if (isDebug) {
			printf("Bus_Voltage:%.2fV\r\n", motorDataRead->busVolt);
			printf("Bus_Current:%.2fA\r\n", motorDataRead->busCurr);
			printf("Work_Temp:%d\r\n", motorDataRead->temperature);
			printf("Run_Mode:%d\r\n", motorDataRead->motorStatue);
			printf("Fault_Code:%d\r\n", motorDataRead->errorCode);
		}

	}
		break;

	case eCAN_ResetFault: {
		//uint8_t SysFault;
		memcpy(&(motorDataRead->errorCode), &(CanRxMessage.canRxBuf[1]),
				sizeof(motorDataRead->errorCode));

		if (isDebug) {
			printf("Fault_Code:%d\r\n", motorDataRead->errorCode);
		}

	}
		break;

	case eCAN_GetMotorPara: {
		//uint8_t motor_pole;
		//float moment_constant;
		//uint8_t reduction_ratio;

		memcpy(&(motorDataRead->polarPair), &(CanRxMessage.canRxBuf[1]),
				sizeof(motorDataRead->polarPair));
		memcpy(&(motorDataRead->torqueConst), &(CanRxMessage.canRxBuf[2]),
				sizeof(motorDataRead->torqueConst));
		memcpy(&(motorDataRead->reduceRatio), &(CanRxMessage.canRxBuf[6]),
				sizeof(motorDataRead->reduceRatio));

		if (isDebug) {
			printf("Npp:%d\r\n", motorDataRead->polarPair);
			printf("Kt:%f(N*m)/A\r\n", motorDataRead->torqueConst);
			printf("Gear ratio:%d\r\n", motorDataRead->reduceRatio);
		}

	}
		break;

	case eCAN_SetZeroByCurrentRawAngle: {
		uint8_t offset_angle;
		memcpy(&offset_angle, &(CanRxMessage.canRxBuf[1]),
				sizeof(offset_angle));
		motorDataRead->offsetAngle = degToRad(offset_angle);
		if (isDebug) {
			printf("Mec_offset:%.2fRad\r\n", motorDataRead->offsetAngle);
		}

	}
		break;

	case eCAN_SetPosCtrlMaxVelocity: {
		uint32_t pos_outlimit;
		memcpy(&pos_outlimit, &(CanRxMessage.canRxBuf[1]),
				sizeof(pos_outlimit));

		motorDataRead->posModeMaxVel = rpmToRadps(pos_outlimit * 0.01);
		if (isDebug) {
			printf("PosCtrlMaxVelocity:%fRad/s\r\n",
					motorDataRead->posModeMaxVel);
		}

	}
		break;

	case eCAN_SetPosVelCtrlMaxIq: {
		uint32_t pos_vel_maxiq;
		memcpy(&pos_vel_maxiq, &(CanRxMessage.canRxBuf[1]),
				sizeof(pos_vel_maxiq));

		motorDataRead->posVelModeMaxIq = pos_vel_maxiq * 0.001;
		if (isDebug) {
			printf("PosVelCtrlMaxIq:%fA\r\n", motorDataRead->posVelModeMaxIq);
		}

	}
		break;

	case eCAN_SetIqCtrlSlope: {
		uint32_t iqslope;
		memcpy(&iqslope, &(CanRxMessage.canRxBuf[1]), sizeof(iqslope));

		motorDataRead->iqSlope = iqslope * 0.001;
		if (isDebug) {
			printf("IqCtrlSlope:%fA/s\r\n", motorDataRead->iqSlope);
		}

	}
		break;

	case eCAN_SetVelCtrlAcc: {
		uint32_t vel_acc;
		memcpy(&vel_acc, &(CanRxMessage.canRxBuf[1]), sizeof(vel_acc));

		motorDataRead->velModeAcc = rpmToRadps(vel_acc * 0.01);
		if (isDebug) {
			printf("VelCtrlAcc:%fRad/s^2\r\n", motorDataRead->velModeAcc);
		}

	}
		break;

	case eCAN_BreakCtrl: {
		if (CanRxMessage.canRxBuf[1] == 0) {
			motorDataRead->motorBreak = 0;
		} else {
			motorDataRead->motorBreak = 1;
		}

		if (isDebug) {
			printf("Break_state:%s\r\n",
					(motorDataRead->motorBreak == 0) ? "Disable" : "Enable");
		}

	}
		break;

	case eCAN_GetPosition:
	case eCAN_SetAbsPosCtrlTragetVal:
	case eCAN_SetRelateiveTragetVal:
	case eCAN_ShortestHomePosition: {
		uint16_t SingleAbsPosCtrlTragetVal;
		int32_t MultiAbsPosCtrlTragetVal;

		memcpy(&SingleAbsPosCtrlTragetVal, &(CanRxMessage.canRxBuf[1]),
				sizeof(SingleAbsPosCtrlTragetVal));
		memcpy(&MultiAbsPosCtrlTragetVal, &(CanRxMessage.canRxBuf[3]),
				sizeof(MultiAbsPosCtrlTragetVal));

		motorDataRead->singleAbsAngle = degToRad(
				SingleAbsPosCtrlTragetVal * (360.0f / 16384));
		motorDataRead->multiAbsAngle = degToRad(
				MultiAbsPosCtrlTragetVal * (360.0f / 16384));
		if (isDebug) {
			printf("SingleAbsPos:%fRad\r\n", motorDataRead->singleAbsAngle);
			printf("MultiAbsPos:%fRad\r\n", motorDataRead->multiAbsAngle);
		}

	}
		break;

		//以下的都是MIT运动协议，不使用所以没有改

		/*
		 case eCAN_GetConfigMitCtrlModePara: {
		 uint16_t pos_max_get, vel_max_get, t_max_get;

		 memcpy(&pos_max_get, &(CanRxMessage.canRxBuf[1]), sizeof(pos_max_get));
		 memcpy(&vel_max_get, &(CanRxMessage.canRxBuf[3]), sizeof(vel_max_get));
		 memcpy(&t_max_get, &(CanRxMessage.canRxBuf[5]), sizeof(t_max_get));

		 pos_max = pos_max_get * 0.1f;
		 vel_max = vel_max_get * 0.01f;
		 t_max = t_max_get * 0.01f;

		 if (isDebug) {
		 printf("Pos_Max:%.2f\r\n", pos_max);
		 printf("Vel_Max:%.2f\r\n", vel_max);
		 printf("T_Max:%.2f\r\n", t_max);
		 }

		 }
		 break;

		 case eCAN_GetMitCtrlModeRealTimeData: {
		 uint8_t *pBuf = CanRxMessage.canRxBuf;

		 float curr_Pos = uint_to_float(((pBuf[1] << 8) | pBuf[2]), -pos_max,
		 pos_max, 16);
		 float curr_vel = uint_to_float(
		 ((pBuf[3] << 4) | ((pBuf[4] & 0xF0) >> 4)), -vel_max, vel_max,
		 12);
		 float curr_torque = uint_to_float((((pBuf[4] & 0x0F) << 8) | pBuf[5]),
		 -t_max, t_max, 12);
		 uint8_t status = pBuf[6];

		 if (isDebug) {
		 printf("current_pos:%.2f\r\n", curr_Pos);
		 printf("current_vel:%.2f\r\n", curr_vel);
		 printf("current_torque:%.2f\r\n", curr_torque);
		 printf("status:%02X\r\n", status);
		 }

		 }
		 break;
		 */

	default:
		break;
	}
}

