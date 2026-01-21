

#ifndef _INV_MPU_H_
#define _INV_MPU_H_
#include "mpu9250.h"

//定义输出速度
#define DEFAULT_MPU_HZ (100) // 100Hz
#define COMPASS_READ_MS (100)

#define INV_X_GYRO (0x40)
#define INV_Y_GYRO (0x20)
#define INV_Z_GYRO (0x10)
#define INV_XYZ_GYRO (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL (0x08)
#define INV_XYZ_COMPASS (0x01)

struct int_param_s
{
#if defined EMPL_TARGET_MSP430 || defined MOTION_DRIVER_TARGET_MSP430
    void (*cb)(void);
    unsigned short pin;
    unsigned char lp_exit;
    unsigned char active_low;
#elif defined EMPL_TARGET_UC3L0
    unsigned long pin;
    void (*cb)(volatile void *);
    void *arg;
#elif defined EMPL_TARGET_STM32F4
    void (*cb)(void);
#endif
};

#define MPU_INT_STATUS_DATA_READY (0x0001)
#define MPU_INT_STATUS_DMP (0x0002)
#define MPU_INT_STATUS_PLL_READY (0x0004)
#define MPU_INT_STATUS_I2C_MST (0x0008)
#define MPU_INT_STATUS_FIFO_OVERFLOW (0x0010)
#define MPU_INT_STATUS_ZMOT (0x0020)
#define MPU_INT_STATUS_MOT (0x0040)
#define MPU_INT_STATUS_FREE_FALL (0x0080)
#define MPU_INT_STATUS_DMP_0 (0x0100)
#define MPU_INT_STATUS_DMP_1 (0x0200)
#define MPU_INT_STATUS_DMP_2 (0x0400)
#define MPU_INT_STATUS_DMP_3 (0x0800)
#define MPU_INT_STATUS_DMP_4 (0x1000)
#define MPU_INT_STATUS_DMP_5 (0x2000)

/* Set up APIs */
int mpu_init(struct int_param_s *int_param);
int mpu_init_slave(void);
int mpu_set_bypass(unsigned char bypass_on);

/* Configuration APIs */
int mpu_lp_accel_mode(unsigned short rate);
int mpu_lp_motion_interrupt(unsigned short thresh, unsigned char time,
                            unsigned short lpa_freq);
int mpu_set_int_level(unsigned char active_low);
int mpu_set_int_latched(unsigned char enable);

int mpu_set_dmp_state(unsigned char enable);
int mpu_get_dmp_state(unsigned char *enabled);

int mpu_get_lpf(unsigned short *lpf);
int mpu_set_lpf(unsigned short lpf);

int mpu_get_gyro_fsr(unsigned short *fsr);
int mpu_set_gyro_fsr(unsigned short fsr);

int mpu_get_accel_fsr(unsigned char *fsr);
int mpu_set_accel_fsr(unsigned char fsr);

int mpu_get_compass_fsr(unsigned short *fsr);

int mpu_get_gyro_sens(float *sens);
int mpu_get_accel_sens(unsigned short *sens);

int mpu_get_sample_rate(unsigned short *rate);
int mpu_set_sample_rate(unsigned short rate);
int mpu_get_compass_sample_rate(unsigned short *rate);
int mpu_set_compass_sample_rate(unsigned short rate);

int mpu_get_fifo_config(unsigned char *sensors);
int mpu_configure_fifo(unsigned char sensors);

int mpu_get_power_state(unsigned char *power_on);
int mpu_set_sensors(unsigned char sensors);

int mpu_read_6500_accel_bias(long *accel_bias);
int mpu_set_gyro_bias_reg(long *gyro_bias);
int mpu_set_accel_bias_6500_reg(const long *accel_bias);
int mpu_read_6050_accel_bias(long *accel_bias);
int mpu_set_accel_bias_6050_reg(const long *accel_bias);

/* Data getter/setter APIs */
int mpu_get_gyro_reg(short *data, unsigned long *timestamp);
int mpu_get_accel_reg(short *data, unsigned long *timestamp);
int mpu_get_compass_reg(short *data, unsigned long *timestamp);
int mpu_get_temperature(long *data, unsigned long *timestamp);

int mpu_get_int_status(short *status);
int mpu_read_fifo(short *gyro, short *accel, unsigned long *timestamp,
                  unsigned char *sensors, unsigned char *more);
int mpu_read_fifo_stream(unsigned short length, unsigned char *data,
                         unsigned char *more);
int mpu_reset_fifo(void);

int mpu_write_mem(unsigned short mem_addr, unsigned short length,
                  unsigned char *data);
int mpu_read_mem(unsigned short mem_addr, unsigned short length,
                 unsigned char *data);
int mpu_load_firmware(unsigned short length, const unsigned char *firmware,
                      unsigned short start_addr, unsigned short sample_rate);

int mpu_reg_dump(void);
int mpu_read_reg(unsigned char reg, unsigned char *data);
int mpu_run_self_test(long *gyro, long *accel);
int mpu_run_6500_self_test(long *gyro, long *accel, unsigned char debug);
int mpu_register_tap_cb(void (*func)(unsigned char, unsigned char));

//自行添加的一些函数
void mget_ms(unsigned long *time);
unsigned short inv_row_2_scale(const signed char *row);
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
uint8_t run_self_test(void);
uint8_t mpu_dmp_init(void);
uint8_t mpu_dmp_get_data(float *pitch, float *roll, float *yaw);
uint8_t mpu_mpl_get_data(float *pitch, float *roll, float *yaw);
#endif /* #ifndef _INV_MPU_H_ */

/*
static int set_int_enable(unsigned char enable) 模块中断使能函数
int mpu_reg_dump(void) 测试打印函数
int mpu_read_reg(unsigned char reg, unsigned char *data) 3.向芯片读寄存器值，除了MEMERY和FIFO
int mpu_init(void) MPU6050的初始化
int mpu_lp_accel_mode(unsigned char rate) 进入低功耗模式
int mpu_get_gyro_reg(short *data, unsigned long *timestamp) 获取新的原始陀螺仪数据
int mpu_get_accel_reg(short *data, unsigned long *timestamp获取新的原始加速度数据
int mpu_get_temperature(long *data, unsigned long *timestamp) 获取新的温度数据
int mpu_set_accel_bias(const long *accel_bias) 偏差配置函数
int mpu_reset_fifo(void) 重置FIFO函数
int mpu_get_gyro_fsr(unsigned short *fsr) 获得陀螺仪全尺寸范围函数
int mpu_set_gyro_fsr(unsigned short fsr) 设置陀螺仪全尺寸范围函数
int mpu_get_accel_fsr(unsigned char *fsr) 获得加速度全尺寸范围函数
int mpu_set_accel_fsr(unsigned char fsr) 配置加速度全尺寸范围函数
int mpu_get_lpf(unsigned short *lpf) .获得DLPF范围函数
int mpu_set_lpf(unsigned short lpf) 配置DLPF范围函数
int mpu_get_sample_rate(unsigned short *rate) 获得采样频率范围函数
int mpu_set_sample_rate(unsigned short rate) 配置采样频率范围函数
int mpu_get_compass_sample_rate(unsigned short *rate) 获得罗盘采样频率范围函数
int mpu_set_compass_sample_rate(unsigned short rate) 配置罗盘采样频率范围函数
int mpu_get_gyro_sens(float *sens) 获得陀螺仪灵敏度比例因子函数
int mpu_get_accel_sens(unsigned short *sens) 获得加速计灵敏度比例因子函数
int mpu_get_fifo_config(unsigned char *sensors) 获得开启的FIFO通道函数
int mpu_configure_fifo(unsigned char sensors) 配置开启FIFO通道函数
int mpu_get_power_state(unsigned char *power_on) 获得芯片工作状态
int mpu_set_sensors(unsigned char sensors) 配置传感器的时钟和工作状态函数
int mpu_get_int_status(short *status).获得中断状态函数
int mpu_read_fifo(short *gyro, short *accel, unsigned long *timestamp,unsigned char *sensors, unsigned char *more) 获得FIFO数据函数
int mpu_read_fifo_stream(unsigned short length, unsigned char *data,unsigned char *more) 获得FIFO数据长度函数
int mpu_set_bypass(unsigned char bypass_on) 设置旁路模式函数
int mpu_set_int_level(unsigned char active_low) 设置中断优先级函数
int mpu_set_int_latched(unsigned char enable) 设置中断锁存函数-
设置自检函数
static int get_st_biases(long *gyro, long *accel, unsigned char hw_test) 获取所有的偏差值函数
int mpu_run_self_test(long *gyro, long *accel) 行自检值函数
int mpu_write_mem(unsigned short mem_addr, unsigned short length,unsigned char *data) 向DMP写记忆函数
int mpu_read_mem(unsigned short mem_addr, unsigned short length,unsigned char *data) 向DMP读记忆函数
int mpu_load_firmware(unsigned short length, const unsigned char *firmware,unsigned short start_addr, unsigned short sample_rate) 加载并验证DMP映像函数
int mpu_set_dmp_state(unsigned char enable) DMP状态控制函数
int mpu_get_dmp_state(unsigned char *enabled) DMP状态读取函数
*/

// printf("开始设置\n");
// MPUQMC5883L_register_write_byte(Conrtol_Reg2, 0X80);
// MPUQMC5883L_register_write_byte(Conrtol_Reg1, 0X00);
// vTaskDelay(30);
// uint8_t write_data = 0;
// MPUQMC5883L_register_write_byte(Period_Reg, 0X02);
// vTaskDelay(20);
// MPUQMC5883L_register_read(Period_Reg, &write_data, 1);
// vTaskDelay(20);
// printf("REDA DATA IS:%d\n", write_data);
// while (write_data != 0X02)
//     ;
// printf("初始化复位时间\n");
// MPUQMC5883L_register_write_byte(Conrtol_Reg1, 0X11);
// vTaskDelay(20);
// MPUQMC5883L_register_read(Conrtol_Reg1, &write_data, 1);
// vTaskDelay(20);
// printf("REDA DATA IS:%d\n", write_data);
// while (write_data != 0X11)
//     ;
// printf("设置连续转化\n");

// MPUQMC5883L_register_read(Status_Reg1, &DRDY, 1);
// printf(" Status Register：%d \n",DRDY);
// DRDY=DRDY&0x01;
// if (DRDY==0x01)
