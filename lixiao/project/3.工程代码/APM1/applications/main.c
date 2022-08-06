/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-20     Abbcc        first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "stdio.h"
#include "sensor_init.h"
#include <stdlib.h>
#include <ipc/pipe.h>
#include "string.h"
#include "rtdbg.h"
#include <ipc/pipe.h>
#include <onenet.h>
#include <u8g2_port.h>
/*1.-全局变量  */
Temp_Humi_val Sensor_val = { 0, 0, };//AHT10 数据
/* defined the LED2 pin: PE6 */
#define LED2_PIN    GET_PIN(E, 6)
#define LED3_PIN    GET_PIN(E, 5)
#define KEY1_PIN    GET_PIN(A, 1)
#define OLED_I2C_PIN_SCL                    26  // PB10
#define OLED_I2C_PIN_SDA                    27  // PB11

static rt_thread_t Sensor_flag = RT_NULL;
static void Sensor(void* parameter);            //AHT10 数据读取任务
static rt_thread_t OneNet_flag = RT_NULL;
static void OneNet(void* parameter);            //数据上传任务
static rt_thread_t Oled_flag = RT_NULL;
static void Oled(void* parameter);              //数据显示任务
static rt_thread_t mpu_flag = RT_NULL;
void motion_entry(void *parameter);             //mpu6050 数据读取任务
static rt_thread_t led_flag = RT_NULL;
void led_entry(void *parameter);             //点灯任务

static rt_mq_t sensor_mq = RT_NULL;         //传输传感器值的消息队列
//static rt_sem_t upload_flag = RT_NULL;      //上传数据的同步信号量
int main(void)
{
    /* set LED2 pin mode to output */
    rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);
    rt_err_t result;

    /* 初始化消息队列 */
    sensor_mq = rt_mq_create("THsensor", 20, 40, RT_IPC_FLAG_FIFO);
    if (sensor_mq != RT_NULL)
        rt_kprintf("Sensor_mq create success\r\n");
//    upload_flag = rt_sem_create("upload", 0, RT_IPC_FLAG_FIFO);
//    if (upload_flag != RT_NULL)
//        rt_kprintf("Upload_flag create success\r\n");
    mpu_flag = rt_thread_create("mpu", motion_entry, RT_NULL, 1024*8,RT_THREAD_PRIORITY_MAX - 5, 100);
    if (mpu_flag != RT_NULL)
        rt_thread_startup(mpu_flag);
    led_flag = rt_thread_create("led", led_entry, RT_NULL, 1024,RT_THREAD_PRIORITY_MAX - 5, 100);
    if (led_flag != RT_NULL)
        rt_thread_startup(led_flag);
    Oled_flag = rt_thread_create("Oled", Oled, RT_NULL, 1024*8,RT_THREAD_PRIORITY_MAX - 5, 100);
    if (Oled_flag != RT_NULL)
        rt_thread_startup(Oled_flag);
    Sensor_flag = rt_thread_create("Sensor", Sensor, RT_NULL, 2048,RT_THREAD_PRIORITY_MAX - 6, 100);
    if (Sensor_flag != RT_NULL)
        rt_thread_startup(Sensor_flag);
    rt_thread_mdelay(10000);
    OneNet_flag = rt_thread_create("OneNet", OneNet, RT_NULL, 1024*8,RT_THREAD_PRIORITY_MAX - 5, 100);
    if (OneNet_flag != RT_NULL)
        rt_thread_startup(OneNet_flag);
}
float temp, humi;
extern float pitch1;
char buf1[2];
char buf2[2];
char buf3[2];
static void Sensor(void* parameter)
{
    aht10_device_t AHT10 = aht10_init("i2c1");
    while (1)
    {
        temp = aht10_read_temperature(AHT10);
        humi = aht10_read_humidity(AHT10);
        Sensor_val.AHT10_temp_val = aht10_read_temperature(AHT10);
        Sensor_val.AHT10_humi_val = aht10_read_humidity(AHT10);
//        rt_sem_release(upload_flag);//释放上传数据信号量
//        rt_mq_send(sensor_mq, &Sensor_val, sizeof(Sensor_val));//发送传感器数据消息队列
        rt_thread_mdelay(1000);
    }
}


static void OneNet(void* parameter)
{
    float Onenet_temp, Onenet_humi;
    char buffer1[2];
    char buffer2[2];
    onenet_mqtt_init();
    while (1)
    {
//        rt_sem_take(upload_flag, RT_WAITING_FOREVER);
//        rt_sem_release(upload_flag);
        Onenet_temp = Sensor_val.AHT10_temp_val;
        Onenet_humi = Sensor_val.AHT10_humi_val;
        sprintf(buffer1, "%.2f", Onenet_temp);
        onenet_mqtt_upload_string("AHT10_temp", buffer1);//上传数据
        rt_thread_delay(rt_tick_from_millisecond(5 * 1000));

        sprintf(buffer2, "%.2f", Onenet_humi);
        onenet_mqtt_upload_string("AHT10_humi", buffer2);
        rt_thread_delay(rt_tick_from_millisecond(5 * 1000));
//        rt_sem_delete(upload_flag);

        sprintf(buf3, "%d", pitch1);
        onenet_mqtt_upload_string("pitch", buf3);
        rt_thread_delay(rt_tick_from_millisecond(5 * 1000));
//        rt_sem_delete(upload_flag);
    }
}

static void Oled(void* parameter)
{
    u8g2_t u8g2;
    u8g2_Setup_ssd1306_i2c_128x64_noname_f( &u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_rtthread);
    u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_I2C_CLOCK, OLED_I2C_PIN_SCL);
    u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_I2C_DATA, OLED_I2C_PIN_SDA);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    while(1)
    {
        u8g2_ClearBuffer(&u8g2);

        u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
        u8g2_DrawStr(&u8g2, 10, 15, "temp:");
        u8g2_DrawStr(&u8g2, 10, 30, "humi:");
        u8g2_DrawStr(&u8g2, 10, 45, "pitch:");
        u8g2_SendBuffer(&u8g2);

        sprintf(buf1, "%.2f", temp);
        sprintf(buf2, "%.2f", humi);
        sprintf(buf3, "%d", pitch1);
        u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
        u8g2_DrawStr(&u8g2, 82, 15, buf1);
        u8g2_SendBuffer(&u8g2);

        u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
        u8g2_DrawStr(&u8g2, 82, 30, buf2);
        u8g2_SendBuffer(&u8g2);

        u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
        u8g2_DrawStr(&u8g2, 82, 45, buf3);
        u8g2_SendBuffer(&u8g2);
    }
}

void led_entry(void *parameter)
{
    /* set LED2 pin mode to output */
    rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);
    /* set LED2 pin mode to output */
    rt_pin_mode(LED3_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(KEY1_PIN, PIN_MODE_INPUT);
    while (1)
    {
        rt_pin_write(LED2_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED2_PIN, PIN_LOW);
        rt_thread_mdelay(500);
        if(rt_pin_read(KEY1_PIN) == PIN_HIGH)
        {
            rt_pin_write(LED3_PIN, PIN_HIGH);
        }
        else
        {
            rt_pin_write(LED3_PIN, PIN_LOW);
        }
    }
}
