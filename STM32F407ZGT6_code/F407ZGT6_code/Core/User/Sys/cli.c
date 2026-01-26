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

	float v = atof(argv[2]);

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

	float x = atof(argv[1]);
	float y = atof(argv[2]);

	FiveBarGeom_t g;
	g.L1 = 90.0f;
	g.L2 = 90.0f;
	g.L3 = 130.0f;
	g.L4 = 130.0f;
	g.d = 65.5f;
	FiveBarLimit_t lim;
	lim.theta_f_min = 0.0f;
	lim.theta_f_max = 1.448623f;
	lim.theta_r_min = 0.0f;
	lim.theta_r_max = 1.448623f;
	float theta_f, theta_r;
	IKStatus_t iks;
	printf("begin calc\r\n");
	iks = fivebar_inverse_kinematics(x, y, &g, &lim, &theta_f, &theta_r);
	/* 这里后面直接接你的 fivebar_inverse_kinematics */
	printf("x=%.3f, y=%.3f, theta_f=%.2f Deg, theta_r=%.2f Deg, state=%d\r\n", x, y,
			theta_f*57.29578f, theta_r*57.29578f, iks);
	JM_PosAbsMode(idLF, theta_f);
	JM_PosAbsMode(idLR, theta_r);
	JM_PosAbsMode(idRF, theta_f);
	JM_PosAbsMode(idRR, theta_r);
}
