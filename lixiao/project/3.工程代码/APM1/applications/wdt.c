/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-03     Esperanza       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>

#define WDT_DEVICE_NAME    "wdt"    /* ���Ź��豸���� */
static rt_device_t wdg_dev;         /* ���Ź��豸��� */

static int wdt_sample(int argc, char *argv[])
{
    rt_err_t ret = RT_EOK;
    rt_uint32_t timeout = 5;        /* ���ʱ�䣬��λ���� */

    /* �����豸���Ʋ��ҿ��Ź��豸����ȡ�豸��� */
    wdg_dev = rt_device_find("wdt");
    rt_device_init(wdg_dev);

    /* ���ÿ��Ź����ʱ�� */
    ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &timeout);

    /* �������Ź� */
    ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_START, RT_NULL);

    return ret;
}
/* ������ msh �����б��� */
MSH_CMD_EXPORT(wdt_sample, wdt sample);


