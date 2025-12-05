#ifndef __COMM_CAN_H
#define __COMM_CAN_H

#include "main.h"
#include "../User/Sys/misc.h"

typedef struct
{
    CAN_RxHeaderTypeDef RxHead;     /**< can通信协议头 */
    uint8_t canRxBuf[8];            /**< can通信接收到的数据包 */
}CanRxMessage_t;


/* 命令枚举 */
enum{
    eCAN_Restart                    = 0x00,     /**< 软件复位 */
    eCAN_GetVerInfo                 = 0xA0,     /**< 获取Boot软件版本、应用软件版本、硬件版本、CAN-AUX协议版本 */
    eCAN_GetIq                      = 0xA1,     /**< 读取实时Q轴电流 */
    eCAN_GetVelocity                = 0xA2,     /**< 读取实时速度 */
    eCAN_GetPosition                = 0xA3,     /**< 读实时单圈绝对值角度、多圈绝对值角度 */
    eCAN_GetStatusInfo              = 0xAE,     /**< 读取电压、电流、温度、状态、故障实时状态信息 */
    eCAN_ResetFault                 = 0xAF,     /**< 清除故障 */
    
    eCAN_GetMotorPara               = 0xB0,     /**< 读取电机硬件参数 */
    eCAN_SetZeroByCurrentRawAngle   = 0xB1,     /**< 设置当前位置为原点 */
	
    eCAN_SetPosCtrlMaxVelocity      = 0xB2,     /**< 设置位置模式最大速度 */
    eCAN_SetPosVelCtrlMaxIq         = 0xB3,     /**< 设置位置环/速度环模式最大的Q轴电流 */
    eCAN_SetIqCtrlSlope             = 0xB4,     /**< 设置Q轴电流控制电流的斜率 */
    eCAN_SetVelCtrlAcc              = 0xB5,     /**< 设置速度控制时的加速度 */
    
	eCAN_Pos_Kp						= 0xB6,     /**< 位置闭环PID-kp */
	eCAN_Pos_Ki						= 0xB7,     /**< 位置闭环PID-ki */
	eCAN_Vel_Kp						= 0xB8,     /**< 速度闭环PID-kp */
	eCAN_Vel_Ki						= 0xB9,     /**< 速度闭环PID-ki */
	
    eCAN_SetIqCtrlTragetVal         = 0xC0,     /**< Q轴电流控制目标值 */
    eCAN_SetVelCtrlTragetVal        = 0xC1,     /**< 速度控制的目标值 */
    eCAN_SetAbsPosCtrlTragetVal     = 0xC2,     /**< 绝对值位置控制的目标值 */
    eCAN_SetRelateiveTragetVal      = 0xC3,     /**< 相对位置控制的目标值 */
    eCAN_ShortestHomePosition       = 0xC4,     /**< 最短距离回原点 */
    eCAN_BreakCtrl									= 0xCE,			/**< 抱阀开关输出控制 */
    eCAN_DisableCtrl                = 0xCF,     /**< 失能电机输出 */
    
    eCAN_GetConfigMitCtrlModePara   = 0xF0,     /**< 配置MIT模式控制参数 */
    eCAN_GetMitCtrlModeRealTimeData = 0xF1,     /**< 读取MIT模式控制实时数据 */
};


void CommCan_Init(CAN_HandleTypeDef *hCan);

void CommCan_SysMotorRestart(CAN_HandleTypeDef *hCan,uint8_t dev_addr);                             /* 重启电机 */
void CommCan_GetVerInfo(CAN_HandleTypeDef *hCan,uint8_t dev_addr);                                  /* 读取电机版本信息 */
void CommCan_GetIq(CAN_HandleTypeDef *hCan,uint8_t dev_addr);                                       /* 读取实时Q轴电流 */
void CommCan_GetVelocity(CAN_HandleTypeDef *hCan,uint8_t dev_addr);                                 /* 读取实时速度 */
void CommCan_GetPosition(CAN_HandleTypeDef *hCan,uint8_t dev_addr);                                 /*读实时单圈绝对值角度、多圈绝对值角度 */
void CommCan_GetStatusInfo(CAN_HandleTypeDef *hCan,uint8_t dev_addr);                               /* 读取电压、电流、温度、状态、故障实时状态信息 */
void CommCan_GetMotorPara(CAN_HandleTypeDef *hCan,uint8_t dev_addr);                                /* 读取电机硬件参数 */
void CommCan_SysMotorResetFault(CAN_HandleTypeDef *hCan,uint8_t dev_addr);                          /* 电机清除故障 */
void CommCan_SetZeroByCurrentRawAngle(CAN_HandleTypeDef *hCan,uint8_t dev_addr);                    /* 设置电机当前位置为原点*/
void CommCan_SetPosCtrlMaxVelocity(CAN_HandleTypeDef *hCan,uint8_t dev_addr, float target_param);   /* 设置位置模式最大速度 */
void CommCan_SetPosVelCtrlMaxIq(CAN_HandleTypeDef *hCan,uint8_t dev_addr, float target_param);      /* 设置位置或速度模式最大力矩 */
void CommCan_SetIqCtrlSlope(CAN_HandleTypeDef *hCan,uint8_t dev_addr, float target_param);          /* 设置力矩速度电流斜率 */
void CommCan_SetVelCtrlAcc(CAN_HandleTypeDef *hCan,uint8_t dev_addr, float target_param);           /* 设置速度模式下最大加速度 */
void CommCan_SetPosKpParam(CAN_HandleTypeDef *hCan,uint8_t dev_addr, float target_param );          /* 位置环Kp参数设置*/
void CommCan_SetPosKiParam(CAN_HandleTypeDef *hCan,uint8_t dev_addr, float target_param );          /* 位置环Ki参数设置*/
void CommCan_SetVelKpParam(CAN_HandleTypeDef *hCan,uint8_t dev_addr, float target_param );          /* 速度环Kp参数设置*/
void CommCan_SetVelKiParam(CAN_HandleTypeDef *hCan,uint8_t dev_addr, float target_param );          /* 速度环Ki参数设置*/
void CommCan_SetIq(CAN_HandleTypeDef *hCan,uint8_t dev_addr, float target_Iq);                      /* 力矩模式设置Q轴电流 */
void CommCan_SetVelocity(CAN_HandleTypeDef *hCan,uint8_t dev_addr, float target_vel);               /* 速度模式设置速度 */
void CommCan_SetAbsPosition_Count(CAN_HandleTypeDef *hCan,uint8_t dev_addr, int32_t traget_pos);    /* 绝对值位置控制 */
void CommCan_SetRelateive_Count(CAN_HandleTypeDef *hCan,uint8_t dev_addr, int32_t target_pos);      /* 相对值位置控制，单位Count，一圈为16384Counts */
void CommCan_ShortestHomePosition(CAN_HandleTypeDef *hCan,uint8_t dev_addr);                        /* 最短距离回到原点 */
void CommCan_BreakCtrl(CAN_HandleTypeDef *hCan,uint8_t dev_addr, uint8_t open_or_close);            /* 抱闸输出控制*/
void CommCan_MotorDisableCtrl(CAN_HandleTypeDef *hCan,uint8_t dev_addr);                            /* 电机失能 */

void CommCan_GetMitCtrlModePara(CAN_HandleTypeDef *hCan,uint8_t dev_addr);
void CommCan_ConfigMitCtrlModePara(CAN_HandleTypeDef *hCan,uint8_t dev_addr, float config_pos_max, float config_vel_max, float config_t_max);
void CommCan_GetMitCtrlModeRealTimeData(CAN_HandleTypeDef *hCan,uint8_t dev_addr);
void CommCan_MotorRunCtrlMode_Control(CAN_HandleTypeDef *hCan,uint32_t dev_addr, float pos_target, float vel_target, float pos_gain_kp, float vel_gain_kd, float torque_target);

#endif
