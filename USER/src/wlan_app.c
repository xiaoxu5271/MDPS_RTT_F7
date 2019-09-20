#include <rthw.h>
#include <rtthread.h>

#include <wlan_mgnt.h>
#include <wlan_prot.h>
#include <wlan_cfg.h>
#include <arpa/inet.h>
#include <netdev.h> /* 当需要网卡操作时，需要包含这两个头文件 */

#include "wlan_app.h"

#define WLAN_SSID "SmartWiFi-3704"
#define WLAN_PASSWORD "12345678"
#define NET_READY_TIME_OUT (rt_tick_from_millisecond(15 * 1000))

static rt_sem_t net_ready = RT_NULL;

static void
wifi_ready_callback(int event, struct rt_wlan_buff *buff, void *parameter)
{
    rt_kprintf("%s\n", __FUNCTION__);
    rt_sem_release(net_ready);
}

static void
wifi_connect_callback(int event, struct rt_wlan_buff *buff, void *parameter)
{
    rt_kprintf("%s\n", __FUNCTION__);
    if ((buff != RT_NULL) && (buff->len == sizeof(struct rt_wlan_info)))
    {
        rt_kprintf("ssid : %s \n", ((struct rt_wlan_info *)buff->data)->ssid.val);
    }
}

static void
wifi_disconnect_callback(int event, struct rt_wlan_buff *buff, void *parameter)
{
    rt_kprintf("%s\n", __FUNCTION__);
    if ((buff != RT_NULL) && (buff->len == sizeof(struct rt_wlan_info)))
    {
        rt_kprintf("ssid : %s \n", ((struct rt_wlan_info *)buff->data)->ssid.val);
    }
}

static void
wifi_connect_fail_callback(int event, struct rt_wlan_buff *buff, void *parameter)
{
    rt_kprintf("%s\n", __FUNCTION__);
    if ((buff != RT_NULL) && (buff->len == sizeof(struct rt_wlan_info)))
    {
        rt_kprintf("ssid : %s \n", ((struct rt_wlan_info *)buff->data)->ssid.val);
    }
}

rt_err_t wifi_connect(void)
{
    rt_err_t result = RT_EOK;

    /* Configuring WLAN device working mode */
    rt_wlan_set_mode(RT_WLAN_DEVICE_STA_NAME, RT_WLAN_STATION);
    /* station connect */
    rt_kprintf("start to connect ap ...\n");
    net_ready = rt_sem_create("net_ready", 0, RT_IPC_FLAG_FIFO);
    rt_wlan_register_event_handler(RT_WLAN_EVT_READY,
                                   wifi_ready_callback, RT_NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_CONNECTED,
                                   wifi_connect_callback, RT_NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_DISCONNECTED,
                                   wifi_disconnect_callback, RT_NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_CONNECTED_FAIL,
                                   wifi_connect_fail_callback, RT_NULL);

    //rt_wlan_config_autoreconnect(RT_TRUE); //开启自动重连
    // netdev_set_dns_server("w0", 0, "114.114.114.114");

    /* connect wifi */
    result = rt_wlan_connect(WLAN_SSID, WLAN_PASSWORD);

    if (result == RT_EOK)
    {
        /* waiting for IP to be got successfully  */
        result = rt_sem_take(net_ready, NET_READY_TIME_OUT);
        if (result == RT_EOK)
        {
            netdev_set_default_user("w0");
            rt_kprintf("networking ready!\n");
        }
        else
        {
            rt_kprintf("wait ip got timeout!\n");
        }
        // rt_wlan_unregister_event_handler(RT_WLAN_EVT_READY);
        // rt_sem_delete(net_ready);

        // rt_thread_delay(rt_tick_from_millisecond(5 * 1000));
        // rt_kprintf("wifi disconnect test!\n");
        // /* disconnect */
        // result = rt_wlan_disconnect();
        // if (result != RT_EOK)
        // {
        //     rt_kprintf("disconnect failed\n");
        //     return result;
        // }
        // rt_kprintf("disconnect success\n");
    }
    else
    {
        rt_kprintf("connect failed!(NOT ENVEN BACK)\n");
        if (rt_wlan_get_autoreconnect_mode())
        {
            // TIME_START();
        }
    }
    return result;
}

int user_connect(int argc, char *argv[])
{
    wifi_connect();
    return 0;
}
MSH_CMD_EXPORT(user_connect, connect test.);

//设置默认网卡
static int netdev_set_default_user(const char *name)
{
    struct netdev *netdev = RT_NULL;

    /* 通过网卡名称获取网卡对象，名称可以通过 ifconfig 命令查看 */
    netdev = netdev_get_by_name(name);
    if (netdev == RT_NULL)
    {
        rt_kprintf("not find network interface device name(%s).\n", name);
        return -1;
    }

    /* 设置默认网卡对象 */
    netdev_set_default(netdev);

    rt_kprintf("set default network interface device(%s) success.\n", name);
    return 0;
}

static int netdev_set_default_test(int argc, char **argv)
{
    struct netdev *netdev = RT_NULL;

    if (argc != 2)
    {
        rt_kprintf("netdev_set_default [netdev_name]   --set default network interface device.\n");
        return -1;
    }

    /* 通过网卡名称获取网卡对象，名称可以通过 ifconfig 命令查看 */
    netdev = netdev_get_by_name(argv[1]);
    if (netdev == RT_NULL)
    {
        rt_kprintf("not find network interface device name(%s).\n", argv[1]);
        return -1;
    }

    /* 设置默认网卡对象 */
    netdev_set_default(netdev);

    rt_kprintf("set default network interface device(%s) success.\n", argv[1]);
    return 0;
}

#ifdef FINSH_USING_MSH
#include <finsh.h>
/* 导出命令到 FinSH 控制台 */
MSH_CMD_EXPORT_ALIAS(netdev_set_default_test, netdev_set_default, set default network interface device);
#endif /* FINSH_USING_MSH */
