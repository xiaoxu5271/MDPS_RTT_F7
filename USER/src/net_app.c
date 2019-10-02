#include <rthw.h>
#include <rtthread.h>

#include <wlan_mgnt.h>
#include <wlan_prot.h>
#include <wlan_cfg.h>
#include <arpa/inet.h>
#include <netdev.h> /* 当需要网卡操作时，需要包含这两个头文件 */

#include "net_app.h"

#define WLAN_SSID "work-GN"
#define WLAN_PASSWORD "work12345678"
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

void get_default_mac(char *mac_buf)
{

#define NETDEV_IFCONFIG_MAC_MAX_LEN 6

    rt_ubase_t index;
    rt_slist_t *node = RT_NULL;
    struct netdev *netdev = RT_NULL;
    struct netdev *cur_netdev_list = netdev_list;

    if (cur_netdev_list == RT_NULL)
    {
        rt_kprintf("ifconfig: network interface device list error.\n");
        return;
    }

    for (node = &(cur_netdev_list->list); node; node = rt_slist_next(node))
    {
        netdev = rt_list_entry(node, struct netdev, list);

        // rt_kprintf("network interface device: %.*s%s\n",
        //            RT_NAME_MAX, netdev->name,
        //            (netdev == netdev_default) ? " (Default)" : "");
        // rt_kprintf("MTU: %d\n", netdev->mtu);
        if (netdev == netdev_default)
        {
            /* 6 - MAC address */
            if (netdev->hwaddr_len == NETDEV_IFCONFIG_MAC_MAX_LEN)
            {
                rt_sprintf(mac_buf, "%02x:%02x:%02x:%02x:%02x:%02x",
                           netdev->hwaddr[0],
                           netdev->hwaddr[1],
                           netdev->hwaddr[2],
                           netdev->hwaddr[3],
                           netdev->hwaddr[4],
                           netdev->hwaddr[5]);
                rt_kprintf("MAC: %s\n", mac_buf);
                // for (index = 0; index < netdev->hwaddr_len; index++)
                // {
                //     rt_kprintf("%02x ", netdev->hwaddr[index]);
                // }
            }
        }
    }
}

#ifdef FINSH_USING_MSH
#include <finsh.h>
/* 导出命令到 FinSH 控制台 */
MSH_CMD_EXPORT_ALIAS(netdev_set_default_test, netdev_set_default, set default network interface device);
#endif /* FINSH_USING_MSH */
