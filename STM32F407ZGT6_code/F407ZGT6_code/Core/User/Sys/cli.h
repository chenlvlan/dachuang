/*
 * cli.h
 *
 *  Created on: Jan 22, 2026
 *      Author: yufei
 */

#ifndef USER_SYS_CLI_H_
#define USER_SYS_CLI_H_

#include <stdint.h>

#include "compute.h"
#include "../mpu6500/mpu6500.h"
#include "../joint_motor/joint_motor.h"
#include "../wheel_motor/wheel_motor.h"

extern legData_t legData;
extern arm_pid_instance_f32 pid_pitch;
extern arm_pid_instance_f32 pid_speed;
//#include "midware.h"

/* ================= 配置 ================= */

#define CLI_BUF_LEN        64
#define CLI_MAX_TOKENS     8

/* ============== 接口函数 ================= */

/* 在 main 初始化时调用一次 */
void cli_init(void);

/* 在 main 的 while(1) 中周期调用 */
void cli_poll(void);

/* UART 接收中断里会用到（你不直接调用） */
void cli_uart_rx_char(uint8_t ch);

#endif /* USER_SYS_CLI_H_ */
