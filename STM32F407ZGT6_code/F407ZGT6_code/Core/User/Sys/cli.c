/*
 * cli.c
 *
 *  Created on: Jan 22, 2026
 *      Author: yufei
 */

#include "cli.h"

#include "usart.h"     // huart1
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ================= 内部变量 ================= */

static volatile uint8_t cli_cmd_ready = 0;

static uint8_t cli_rx_char;
static char cli_buf[CLI_BUF_LEN];
static char cli_line[CLI_BUF_LEN];
static uint16_t cli_idx = 0;

/* ================= 内部声明 ================= */

static int cli_tokenize(char *line, char *argv[], int max_tokens);
static void cli_process(char *line);

/* 命令处理函数 */
static void cli_cmd_help(int argc, char *argv[]);
static void cli_cmd_set(int argc, char *argv[]);
static void cli_cmd_get(int argc, char *argv[]);
static void cli_cmd_ik(int argc, char *argv[]);
static void cli_cmd_pid(int argc, char *argv[]);

/* ================= 对外接口 ================= */

void cli_init(void) {
	cli_idx = 0;
	cli_cmd_ready = 0;

	HAL_UART_Receive_IT(&huart1, &cli_rx_char, 1);
}

/* 在 main while(1) 中调用 */
void cli_poll(void) {
	if (cli_cmd_ready) {
		cli_cmd_ready = 0;
		cli_process(cli_line);
	}
}

/* UART 中断里调用 */
void cli_uart_rx_char(uint8_t ch) {
	if (ch == '\r' || ch == '\n') {

		cli_buf[cli_idx] = '\0';

		if (!cli_cmd_ready) {
			strcpy(cli_line, cli_buf);
			cli_cmd_ready = 1;
		}

		cli_idx = 0;
	} else {
		if (cli_idx < CLI_BUF_LEN - 1) {
			cli_buf[cli_idx++] = ch;
		}
	}
}

/* ================= UART 回调（放在 cli.c 或 main.c 均可） ================= */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		cli_uart_rx_char(cli_rx_char);
		HAL_UART_Receive_IT(&huart1, &cli_rx_char, 1);
	}
}

/* ================= CLI 核心 ================= */

static int cli_tokenize(char *line, char *argv[], int max_tokens) {
	int argc = 0;

	while (*line && argc < max_tokens) {
		while (*line == ' ')
			line++;
		if (*line == '\0')
			break;

		argv[argc++] = line;

		while (*line && *line != ' ')
			line++;
		if (*line) {
			*line = '\0';
			line++;
		}
	}
	return argc;
}

static void cli_process(char *line) {
	char *argv[CLI_MAX_TOKENS];
	int argc = cli_tokenize(line, argv, CLI_MAX_TOKENS);

	if (argc == 0)
		return;

	if (strcmp(argv[0], "help") == 0) {
		cli_cmd_help(argc, argv);
	} else if (strcmp(argv[0], "set") == 0) {
		cli_cmd_set(argc, argv);
	} else if (strcmp(argv[0], "get") == 0) {
		cli_cmd_get(argc, argv);
	} else if (strcmp(argv[0], "ik") == 0) {
		cli_cmd_ik(argc, argv);
	} else if (strcmp(argv[0], "pid") == 0) {
		cli_cmd_pid(argc, argv);
	} else {
		printf("Unknown command\r\n");
	}
}

/* ================= 示例命令实现 ================= */

/* -------- help -------- */
static void cli_cmd_help(int argc, char *argv[]) {
	(void) argc;
	(void) argv;

	printf("Commands:\r\n");
	printf("  help\r\n");
	printf("  set <var> <value>\r\n");
	printf("  get <var>\r\n");
	printf("  ik <x> <y>\r\n");
}

/* -------- set -------- */
static float target_x = 0.0f;
static float target_y = 0.0f;

static void cli_cmd_set(int argc, char *argv[]) {
	if (argc < 3) {
		printf("Usage: set <x|y> <value>\r\n");
		return;
	}

	float v = atoff(argv[2]);

	if (strcmp(argv[1], "x") == 0) {
		target_x = v;
		printf("x = %.3f\r\n", target_x);
	} else if (strcmp(argv[1], "y") == 0) {
		target_y = v;
		printf("y = %.3f\r\n", target_y);
	} else {
		printf("Unknown var\r\n");
	}
}

/* -------- get -------- */
static void cli_cmd_get(int argc, char *argv[]) {
	if (argc < 2) {
		printf("Usage: get <x|y>\r\n");
		return;
	}

	if (strcmp(argv[1], "x") == 0) {
		printf("x = %.3f\r\n", target_x);
	} else if (strcmp(argv[1], "y") == 0) {
		printf("y = %.3f\r\n", target_y);
	} else {
		printf("Unknown var\r\n");
	}
}

/* -------- ik（示例占位） -------- */
static void cli_cmd_ik(int argc, char *argv[]) {
	if (argc < 3) {
		printf("Usage: ik <x> <y>\r\n");
		return;
	}

	legData.x = atof(argv[1]);
	legData.y = atof(argv[2]);
//printf("begin calc\r\n");
	fivebar_inverse_kinematics(&legData);
// 这里后面直接接你的 fivebar_inverse_kinematics
	printf("x=%.3f, y=%.3f, theta_f=%.2f Deg, theta_r=%.2f Deg, state=%d\r\n",
			legData.x, legData.y, legData.theta_f * 57.29578f,
			legData.theta_r * 57.29578f, legData.status);
	JM_PosAbsMode(idLF, legData.theta_f);
	JM_PosAbsMode(idRF, legData.theta_f);
	JM_PosAbsMode(idLR, legData.theta_r);
	JM_PosAbsMode(idRR, legData.theta_r);
}

static void cli_cmd_pid(int argc, char *argv[]) {
	if (argc < 2) {
		printf("Usage: pid <controller name> <Kp> <Ki> <Kd>\r\n");
		return;
	}

	if (strcmp(argv[1], "pitch") == 0) {
		if (argc == 2) {
			//pid pitch
			printf("Current PID of pitch: Kp = %.3e, Ki = %.3e, Kd = %.3e\r\n",
					pid_pitch.Kp, pid_pitch.Ki, pid_pitch.Kd);
		} else if (argc == 5) {
			pid_pitch.Kp = atoff(argv[2]);
			pid_pitch.Ki = atoff(argv[3]);
			pid_pitch.Kd = atoff(argv[4]);
			arm_pid_init_f32(&pid_pitch, 0);
			printf("Set PID of pitch: Kp = %.3e, Ki = %.3e, Kd = %.3e\r\n",
								pid_pitch.Kp, pid_pitch.Ki, pid_pitch.Kd);
		} else {
			printf("Need full parameter :)\r\n");
		}
	} else {
		printf("Unknown controller\r\n");
	}
}
