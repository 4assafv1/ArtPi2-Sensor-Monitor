/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-xx-xx     RT-Thread    first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdlib.h> // for abs()
#include "drivers/sensor.h"

/* ================= Art-Pi 专用配置区 ================= */

#define DHT_NAME        "temp_dht11"
#define JY_UART_NAME    "uart1"
#define JY_BAUDRATE     BAUD_RATE_9600
#define MQ2_ADC_NAME    "adc1"
#define MQ2_ADC_CH      5

/* 定义事件标志位 */
#define EVENT_SMOKE_ALARM  (1 << 0)

/* ================= 全局对象管理 ================= */

/* 数据结构体 */
struct MySensorData {
    int temp;
    int humi;
    float roll;
    float pitch;
    float yaw;
    int mq2_raw;
    float mq2_vol;
} global_data = {0};

/* RTT 核心对象句柄 */
static rt_device_t jy_serial = RT_NULL;
static rt_sem_t jy_rx_sem = RT_NULL;
static rt_mutex_t data_mutex = RT_NULL; // 【新增】互斥量，保护 global_data
static rt_event_t alarm_event = RT_NULL; // 【新增】事件集，用于报警

/* ================= JY61P 驱动逻辑 ================= */

// 串口接收中断回调
static rt_err_t jy_uart_input(rt_device_t dev, rt_size_t size)
{
    if (jy_rx_sem) rt_sem_release(jy_rx_sem);
    return RT_EOK;
}

// JY61P 数据处理线程
static void thread_jy61p_entry(void *parameter)
{
    char ch;
    unsigned char buf[11];
    int count = 0;

    while (1)
    {
        // 阻塞等待信号量，减少 CPU 占用
        while (rt_device_read(jy_serial, -1, &ch, 1) != 1)
        {
            rt_sem_take(jy_rx_sem, RT_WAITING_FOREVER);
        }

        if (count == 0 && ch != 0x55) { count = 0; continue; }
        buf[count++] = ch;

        if (count == 11)
        {
            count = 0;
            unsigned char sum = 0;
            for(int i=0; i<10; i++) sum += buf[i];

            if (sum == buf[10] && buf[1] == 0x53)
            {
                short acc[3];
                acc[0] = (short)(buf[3] << 8 | buf[2]);
                acc[1] = (short)(buf[5] << 8 | buf[4]);
                acc[2] = (short)(buf[7] << 8 | buf[6]);

                /* 【关键修改】进入临界区，保护全局变量写入 */
                if (rt_mutex_take(data_mutex, RT_WAITING_FOREVER) == RT_EOK)
                {
                    global_data.roll  = (float)acc[0] / 32768.0f * 180.0f;
                    global_data.pitch = (float)acc[1] / 32768.0f * 180.0f;
                    global_data.yaw   = (float)acc[2] / 32768.0f * 180.0f;

                    rt_mutex_release(data_mutex); // 退出临界区
                }
            }
        }
    }
}

/* ================= 综合采集与打印线程 ================= */
static void thread_collect_entry(void *parameter)
{
    /* 初始化传感器 */
    rt_device_t dht_dev = rt_device_find(DHT_NAME);
    if (dht_dev) rt_device_open(dht_dev, RT_DEVICE_FLAG_RDONLY);

    rt_adc_device_t adc_dev = (rt_adc_device_t)rt_device_find(MQ2_ADC_NAME);
    if (adc_dev) rt_adc_enable(adc_dev, MQ2_ADC_CH);

    struct rt_sensor_data sensor_data;
    struct MySensorData local_snap; // 用于打印的本地快照

    while (1)
    {
        /* 1. 获取互斥锁，开始更新数据 */
        rt_mutex_take(data_mutex, RT_WAITING_FOREVER);

        /* A. 读取温湿度 */
        if (dht_dev && rt_device_read(dht_dev, 0, &sensor_data, 1) == 1)
        {
            rt_int32_t raw = sensor_data.data.temp;
            global_data.temp = raw & 0xFFFF;
            global_data.humi = (raw >> 16) & 0xFFFF;
        }

        /* B. 读取 MQ-2 */
        if (adc_dev)
        {
            global_data.mq2_raw = rt_adc_read(adc_dev, MQ2_ADC_CH);
            global_data.mq2_vol = (float)global_data.mq2_raw * 3.3f / 65536.0f;
        }

        /* 创建数据的本地快照，以便尽快释放互斥锁，减少对 IMU 线程的阻塞 */
        local_snap = global_data;

        rt_mutex_release(data_mutex);
        /* 释放锁 */

        /* 2. 逻辑判断与事件发送 */
        if (local_snap.mq2_vol > 1.0f)
        {
            // 发送烟雾报警事件
            rt_event_send(alarm_event, EVENT_SMOKE_ALARM);
        }

        /* 3. 打印逻辑 (使用快照数据) */
        // 为了代码整洁，小数处理可以在打印时内联或辅助函数处理，这里保持你的风格
        int p_i = (int)local_snap.pitch;
        int p_d = abs((int)(local_snap.pitch * 100) % 100);
        int r_i = (int)local_snap.roll;
        int r_d = abs((int)(local_snap.roll * 100) % 100);
        int y_i = (int)local_snap.yaw;
        int y_d = abs((int)(local_snap.yaw * 100) % 100);
        int v_i = (int)local_snap.mq2_vol;
        int v_d = abs((int)(local_snap.mq2_vol * 100) % 100);

        rt_kprintf("\033[2J\033[H"); // 可选：清屏 ANSI 码，让显示更像仪表盘
        rt_kprintf("================ SENSOR MONITOR ================\n");
        rt_kprintf("[Env] Temp: %d C     | Humi: %d %%\n", local_snap.temp, local_snap.humi);
        rt_kprintf("[IMU] Pitch: %d.%02d   | Roll: %d.%02d   | Yaw: %d.%02d\n", p_i, p_d, r_i, r_d, y_i, y_d);
        rt_kprintf("[Gas] Raw: %-5d     | Volt: %d.%02d V\n", local_snap.mq2_raw, v_i, v_d);
        rt_kprintf("================================================\n");

        /* 检查是否有报警事件 (非阻塞检查) */
        rt_uint32_t e;
        if (rt_event_recv(alarm_event, EVENT_SMOKE_ALARM,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_NO, &e) == RT_EOK)
        {
            rt_kprintf("\n!!! DANGER: SMOKE DETECTED !!!\n\n");
            // 这里可以添加蜂鸣器控制代码
        }

        rt_thread_mdelay(1000);
    }
}

/* ================= MSH 命令导出 ================= */
/*
 * 这是一个 RTT 特性。
 * 你可以在串口终端输入 "sensor_dump" 直接查看当前数据，无需等待循环打印。
 * 这对于调试非常有用。
 */
void sensor_dump(void)
{
    struct MySensorData snap;

    if (rt_mutex_take(data_mutex, 100) != RT_EOK)
    {
        rt_kprintf("System busy, cannot access data.\n");
        return;
    }
    snap = global_data;
    rt_mutex_release(data_mutex);

    rt_kprintf("--- One Shot Dump ---\n");
    rt_kprintf("Temp: %d, Humi: %d\n", snap.temp, snap.humi);
    rt_kprintf("Pitch: %d, Roll: %d, Yaw: %d\n", (int)snap.pitch, (int)snap.roll, (int)snap.yaw);
    rt_kprintf("Gas Vol: %d mv\n", (int)(snap.mq2_vol * 1000));
}
/* 导出到 MSH (FinSH) */
MSH_CMD_EXPORT(sensor_dump, Dump current sensor values);


/* ================= 启动逻辑 ================= */
/*
 * 使用 INIT_APP_EXPORT 实现自动初始化。
 * 这样不再需要去 main.c 里调用 app_start()。
 * 系统启动到 main 线程之前会自动调用它。
 */
int app_start(void)
{
    /* 1. 初始化 IPC 对象 */
    data_mutex = rt_mutex_create("d_mutex", RT_IPC_FLAG_PRIO);
    alarm_event = rt_event_create("alarm", RT_IPC_FLAG_FIFO);
    jy_rx_sem = rt_sem_create("jy_sem", 0, RT_IPC_FLAG_FIFO);

    if (!data_mutex || !alarm_event || !jy_rx_sem)
    {
        rt_kprintf("IPC Init Failed!\n");
        return -1;
    }

    /* 2. 初始化 JY61P 串口 */
    jy_serial = rt_device_find(JY_UART_NAME);
    if (jy_serial)
    {
        struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
        config.baud_rate = JY_BAUDRATE;
        rt_device_control(jy_serial, RT_DEVICE_CTRL_CONFIG, &config);

        rt_device_open(jy_serial, RT_DEVICE_FLAG_INT_RX);
        rt_device_set_rx_indicate(jy_serial, jy_uart_input);

        rt_thread_t t1 = rt_thread_create("jy_th", thread_jy61p_entry, RT_NULL, 2048, 15, 10);
        if (t1) rt_thread_startup(t1);
    }
    else
    {
        rt_kprintf("Error: UART %s not found.\n", JY_UART_NAME);
    }

    /* 3. 创建采集线程 */
    rt_thread_t t2 = rt_thread_create("col_th", thread_collect_entry, RT_NULL, 2048, 20, 10);
    if (t2) rt_thread_startup(t2);

    return 0;
}
/* 自动导出到自动初始化链表 */
INIT_APP_EXPORT(app_start);

/* ================= 主函数 ================= */
/*
 * 由于使用了 INIT_APP_EXPORT，main 函数现在非常干净。
 * 仅仅作为一个空闲循环或保留给其他用途。
 */
int main(void)
{
    // 系统 LED 闪烁或其他保活逻辑
    return RT_EOK;
}
