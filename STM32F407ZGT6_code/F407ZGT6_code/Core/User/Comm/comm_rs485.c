#include "comm_rs485.h"
#include "usart.h"
#include "crc16_modbus.h"
#include "stdio.h"
#include "string.h"

uint8_t comm_uart_rx_buffer[RX_LEN] = {0};
uint8_t comm_uart_tx_buffer[RX_LEN] = {0};

void CommRs485_Init(void)
{
    UART_IDLEIT_Enable(&huart3);
    UART_Receive_DMA(&huart3, comm_uart_rx_buffer, RX_LEN);
    UART_EN_RS485_RX(&huart3);
}

/* 数据发送完成中断 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef  *huart)
{
    UART_EN_RS485_RX(huart);
}

static void _TransmitProtocolData(UartProtocolData_t* pData)
{
    uint16_t tmp_crc = CalcCRC16_Modbus(comm_uart_tx_buffer, (pData->data_len + 5));
    pData->buf[pData->data_len] = (uint8_t)tmp_crc;
    pData->buf[pData->data_len + 1] = (uint8_t)(tmp_crc>>8);
    
    UART_EN_RS485_TX(&huart3);
    if (HAL_OK != UART_Transmit_DMA(&huart3, comm_uart_tx_buffer, pData->data_len +7))
    {
        UART_EN_RS485_RX(&huart3);
    }
}


/* 发送不带参数的命令 */
static void _SendNoParaCmd(uint8_t dev_addr, uint8_t cmd)
{
	UartProtocolData_t *pData = (UartProtocolData_t*)comm_uart_tx_buffer;
    pData->head = UART_TX_PROTOCOL_HEADER_V3;
    pData->pack_num = 0;
    pData->addr = dev_addr;
    pData->cmd = cmd;
    pData->data_len = 0;
    
    _TransmitProtocolData(pData);
}


/* 系统复位 */
void CommRs485_Restart(uint8_t dev_addr)
{
    _SendNoParaCmd(dev_addr, eRestart);
}

/* 获取版本信息 */
void CommRs485_GetVerInfo(uint8_t dev_addr)
{
    _SendNoParaCmd(dev_addr, eGetVerInfo);
}

/* 获取实时信息 */
void CommRs485_GetRealtimeInfo(uint8_t dev_addr)
{
    _SendNoParaCmd(dev_addr, eGetRealtimeInfo);
}

/* 清除故障 */
void CommRs485_ResetFault(uint8_t dev_addr)
{
    _SendNoParaCmd(dev_addr, eResetFault);
	
}

/* 读取运动控制参数 */
void CommRs485_GetMotionCtrlPara(uint8_t dev_addr)
{
    _SendNoParaCmd(dev_addr, eGetMotionCtrlPara);
	
}

/* 写运动控制参数 */
void CommRs485_WriteMotionCtrlPara(uint8_t dev_addr, MotionCtrlPara_t pid , bool is_save)
{
	UartProtocolData_t *pData = (UartProtocolData_t*)comm_uart_tx_buffer;
    uint32_t tmp_val;
    
    pData->head = UART_TX_PROTOCOL_HEADER_V3;
    pData->pack_num = 0;
    pData->addr = dev_addr;
    pData->cmd = (is_save == true) ? eWriteAndSaveMotionCtrlPara : eWriteMotionCtrlPara ;
    pData->data_len =0x18;
    
    memcpy(&pData->buf[0], &pid.pos_kp, sizeof(pid.pos_kp));
    memcpy(&pData->buf[4], &pid.pos_ki, sizeof(pid.pos_ki));
    
    tmp_val = pid.pos_out_limit*100;
    memcpy(&pData->buf[8], &tmp_val, sizeof(tmp_val));
    
    memcpy(&pData->buf[12], &pid.vel_kp, sizeof(pid.vel_kp));
    memcpy(&pData->buf[16], &pid.vel_ki, sizeof(pid.vel_ki));
    
    tmp_val = pid.vel_out_limit*1000;
    memcpy(&pData->buf[20], &tmp_val, sizeof(tmp_val));
    
    _TransmitProtocolData(pData);
}


/* 设置当前位置为原点 */
void CommRs485_SetZeroByCurrentRawAngle(uint8_t dev_addr)
{
    _SendNoParaCmd(dev_addr, eSetZeroByCurrentRawAngle);
	
}

/* 参数恢复默认值 */
void CommRs485_RestSysPara(uint8_t dev_addr)
{
    _SendNoParaCmd(dev_addr, eRestSysPara);
}

/* Q轴电流控制*/
void CommRs485_Torque(uint8_t dev_addr, float target_Iq, float target_Slope)
{
    UartProtocolData_t *pData = (UartProtocolData_t*)comm_uart_tx_buffer;
    
    int32_t tmp_iq = (int32_t)(target_Iq*1000);
    uint32_t tmp_slope = (uint32_t)(target_Slope*1000);
    
    pData->head = UART_TX_PROTOCOL_HEADER_V3;
    pData->pack_num = 0;
    pData->addr = dev_addr;
    pData->cmd = eTorqueCtrl;
    pData->data_len = 8;
    
    memcpy(&pData->buf[0], &tmp_iq, sizeof(tmp_iq));
    memcpy(&pData->buf[4], &tmp_slope, sizeof(tmp_slope));
    
    _TransmitProtocolData(pData);
}

/* 速度控制，输入的target_vel单位为Rpm，target_acc单位为Rpm/s*/
void CommRs485_VelocityCtrl(uint8_t dev_addr, float target_vel, float target_acc)
{
    UartProtocolData_t *pData = (UartProtocolData_t*)comm_uart_tx_buffer;
    
    int32_t tmp_vel = (int32_t)(target_vel*100);
    uint32_t tmp_acc = (uint32_t)(target_acc*100);
    
    pData->head = UART_TX_PROTOCOL_HEADER_V3;
    pData->pack_num = 0;
    pData->addr = dev_addr;
    pData->cmd = eVelocityCtrl;
    pData->data_len = 8;
    
    memcpy(&pData->buf[0], &tmp_vel, sizeof(tmp_vel));
    memcpy(&pData->buf[4], &tmp_acc, sizeof(tmp_acc));
    
    _TransmitProtocolData(pData);
}

/* 绝对位置控制，单位为count，一圈16384 */
void CommRs485_AbsolutePositionCtrl(uint8_t dev_addr, int32_t target_count)
{
    UartProtocolData_t *pData = (UartProtocolData_t*)comm_uart_tx_buffer;
        
    pData->head = UART_TX_PROTOCOL_HEADER_V3;
    pData->pack_num = 0;
    pData->addr = dev_addr;
    pData->cmd = eAbsolutePositionCtrl;
    pData->data_len = 4;
    
    memcpy(&pData->buf[0], &target_count, sizeof(target_count));
    
    _TransmitProtocolData(pData);
}

/* 相对位置控制，单位为count，一圈16384 */
void CommRs485_RelativePositionCtrl(uint8_t dev_addr, int32_t target_count)
{
    UartProtocolData_t *pData = (UartProtocolData_t*)comm_uart_tx_buffer;
    
    pData->head = UART_TX_PROTOCOL_HEADER_V3;
    pData->pack_num = 0;
    pData->addr = dev_addr;
    pData->cmd = eRelativePositionCtrl;
    pData->data_len = 4;
    
    memcpy(&pData->buf[0], &target_count, sizeof(target_count));
    
    _TransmitProtocolData(pData);
}

/* 设置最短距离返回原点 */
void CommRs485_ShortestHomePosition(uint8_t dev_addr)
{
    UartProtocolData_t *pData = (UartProtocolData_t*)comm_uart_tx_buffer;
    
    pData->head = UART_TX_PROTOCOL_HEADER_V3;
    pData->pack_num = 0;
    pData->addr = dev_addr;
    pData->cmd = eShortestHomePosition;
    pData->data_len = 0;
    
    _TransmitProtocolData(pData);
}


/* 抱阀控制开关输出控制 */
void CommRs485_eBreakCtrl(uint8_t dev_addr, uint16_t target_state)
{
    UartProtocolData_t *pData = (UartProtocolData_t*)comm_uart_tx_buffer;
    
    int32_t tmp_state = (int32_t)(target_state);
    
    pData->head = UART_TX_PROTOCOL_HEADER_V3;
    pData->pack_num = 0;
    pData->addr = dev_addr;
    pData->cmd = eBreakCtrl;
    pData->data_len = 1;
    
    memcpy(&pData->buf[0], &tmp_state, sizeof(tmp_state));
    
    _TransmitProtocolData(pData);
}


/* 参数恢复默认值 */
void CommRs485_DisableCtrl(uint8_t dev_addr)
{
    _SendNoParaCmd(dev_addr, eDisableCtrl);
}

/* 串口空闲中断 */
void UART_IDLE_IT_Callback(UART_HandleTypeDef *huart)
{
    uint8_t size = 0;
    
    if(UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
    {
        UART_DMAStop(huart);
        UART_CLEAR_IDLEFLAG(huart);
        
        size = (RX_LEN - UART_DMA_GetReceiveCounter(huart));
        if(size != 0)
        {
            
            if((size >= 7) && (CalcCRC16_Modbus(comm_uart_rx_buffer, size) == 0))
            {
                UartProtocolData_t *pData = (UartProtocolData_t*)comm_uart_rx_buffer;
                if(pData->head == UART_RX_PROTOCOL_HEADER_V3)
                {
                    switch(pData->cmd)
                    {
                        case eGetVerInfo:
						{
							BoardVerInfo_t ver_info;
                            
                            memcpy(&ver_info.boot_ver, &(pData->buf[0]), sizeof(ver_info.software_ver));
                            memcpy(&ver_info.software_ver, &(pData->buf[2]), sizeof(ver_info.software_ver));
                            memcpy(&ver_info.hardware_ver, &(pData->buf[4]), sizeof(ver_info.hardware_ver));
                            memcpy(&ver_info.aux_rs485_ver, &(pData->buf[6]), sizeof(ver_info.aux_rs485_ver));
                            memcpy(&ver_info.modbus_rs485_ver, &(pData->buf[7]), sizeof(ver_info.modbus_rs485_ver));
                            memcpy(&ver_info.aux_can_ver, &(pData->buf[8]), sizeof(ver_info.aux_can_ver));
                            memcpy(&ver_info.canopen_ver, &(pData->buf[9]), sizeof(ver_info.canopen_ver));
                            memcpy(&ver_info.UID, &(pData->buf[10]), sizeof(ver_info.UID));
                            
                            printf("\r\n");
							printf("Boot软件版本 :%d\r\n", ver_info.boot_ver);
							printf("应用软件版本 :%d\r\n", ver_info.software_ver);
							printf("硬件版本     :%02X\r\n", ver_info.hardware_ver);
							printf("RS48-自定义  :%d\r\n", ver_info.aux_rs485_ver);
							printf("RS485-Modbus :%d\r\n", ver_info.modbus_rs485_ver);
							printf("CAN-自定义   :%d\r\n", ver_info.aux_can_ver);
							printf("Can-Canopen  :%d\r\n", ver_info.canopen_ver);
							printf("UID唯一序列号:");
							for(int i = 0; i < 12; i++)
							{
								printf("%02X",ver_info.UID[i]);
							}
							printf("\r\n");
                            printf("\r\n");
						}
						break;
                        
                        case eGetRealtimeInfo:
                        case eTorqueCtrl:
                        case eVelocityCtrl:
                        case eAbsolutePositionCtrl:
                        case eRelativePositionCtrl:
                        case eShortestHomePosition:
                        case eDisableCtrl:
                        {
                            RealTimeInfo_t realtime_info;
                            
                            memcpy(&realtime_info.MecRawAngle, &(pData->buf[0]), sizeof(realtime_info.MecRawAngle));                    /* 单圈绝对值角度[0]-[1] */
                            memcpy(&realtime_info.MecPosition, &(pData->buf[2]), sizeof(realtime_info.MecPosition));                    /* 多圈绝对值角度[2]-[5] */
                            memcpy(&realtime_info.MecVel_Mulit100, &(pData->buf[6]), sizeof(realtime_info.MecVel_Mulit100));            /* 机械速度[6]-[9] */
                            memcpy(&realtime_info.Iq_Mulit1000, &(pData->buf[10]), sizeof(realtime_info.Iq_Mulit1000));                 /* Q轴电流[10]-[13] */
                            memcpy(&realtime_info.BusVoltage_multi100, &(pData->buf[14]), sizeof(realtime_info.BusVoltage_multi100));   /* 母线电压[14]-[15] */
                            memcpy(&realtime_info.BusCurrent_multi100, &(pData->buf[16]), sizeof(realtime_info.BusCurrent_multi100));   /* 母线电流[16]-[17] */
                            realtime_info.Temperature = pData->buf[18];     /* 工作温度[18] */
                            realtime_info.RunMode = pData->buf[19];         /* 运行模式[19] */
                            realtime_info.CtrlState = pData->buf[20];       /* 电机状态[20] */
                            realtime_info.SysFault = pData->buf[21];        /* 故障码[21] */
                            
                            printf("\r\n");
                            printf("Addr:%d\r\n", pData->addr);
                            printf("MecRawAngle:%f°\r\n", (realtime_info.MecRawAngle/16384.0*360));
                            printf("MecPosition:%f°\r\n", (realtime_info.MecPosition/16384.0*360));
                            printf("MecVelocity:%fRpm\r\n", (realtime_info.MecVel_Mulit100/100.0));
                            printf("Iq:%fA\r\n", (realtime_info.Iq_Mulit1000/1000.0));
                            printf("BusVoltage:%fV\r\n", (realtime_info.BusVoltage_multi100/100.0));
                            printf("BusCurrent:%fA\r\n", (realtime_info.BusCurrent_multi100/100.0));
                            printf("Temperature:%d℃\r\n", realtime_info.Temperature);
                            printf("RunMode:%d\r\n", realtime_info.RunMode);
                            printf("CtrlState:%d\r\n", realtime_info.CtrlState);
                            printf("SysFault:%2x\r\n", realtime_info.SysFault);
                        }
						break;
						
						case eResetFault: 
						{
							uint8_t Fault_data = pData->buf[0];
                            
                            printf("\r\n");
                            printf("Fault_code:%02X\r\n",Fault_data);
                            printf("\r\n");

						}
						break;
						
						
						case eGetMotionCtrlPara:
                        case eWriteMotionCtrlPara:
                        case eWriteAndSaveMotionCtrlPara:
						{
							MotionCtrlPara_t pid;
                            uint32_t tmp_val;
							
                            memcpy(&pid.pos_kp, &(pData->buf[0]), sizeof(pid.pos_kp));
                            memcpy(&pid.pos_ki, &(pData->buf[4]), sizeof(pid.pos_ki));
                            
                            memcpy(&tmp_val, &(pData->buf[8]), sizeof(tmp_val));
                            pid.pos_out_limit = tmp_val*0.01f;
                            
                            memcpy(&pid.vel_kp, &(pData->buf[12]), sizeof(pid.vel_kp));
                            memcpy(&pid.vel_ki, &(pData->buf[16]), sizeof(pid.vel_ki));
                            
                            memcpy(&tmp_val, &(pData->buf[20]), sizeof(tmp_val));
                            pid.vel_out_limit = tmp_val*0.001f;
							
                            printf("\r\n");
                            printf("pos_kp:%f\r\n", pid.pos_kp);
                            printf("pos_ki:%f\r\n", pid.pos_ki);
                            printf("pos_out_limit:%f\r\n", pid.pos_out_limit);
                            printf("velocity_kp:%f\r\n", pid.vel_kp);
                            printf("velocity_ki:%f\r\n", pid.vel_ki);
                            printf("velocity_outline:%f\r\n", pid.vel_out_limit);
                            printf("\r\n");
						}
						break;
                        
                        default:
                            break;
                    }
                }
            }
        }
        
        UART_CLEAR_OREFLAG(huart);
        UART_Receive_DMA(huart, comm_uart_rx_buffer, RX_LEN);
    }
}



