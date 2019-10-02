#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <rtthread.h>
#include <sys/socket.h> /* 使用BSD socket，需要包含sockets.h头文件 */
#include "netdb.h"
#include <finsh.h>
#include <webclient.h>
#include "string.h"

#include <dw1000_usr.h>
#include <cJSON.h>
#include "net_app.h"
#include <rtdbg.h>

#define GET_HEADER_BUFSZ 1024
#define GET_REV_BUF_SIZE 1024

//http用参数
uint8_t http_ip[16] = "192.168.1.8";
uint8_t http_port[5] = "8080";
uint8_t http_heart_uri[64]; //"http://192.168.1.8:8080/netlog?data="
uint8_t http_noresp_uri[64];
#define equip_model "Ubitrack_A_v1.0.0"

//心跳包用定时
static rt_timer_t timer1;
static rt_sem_t dynamic_sem = RT_NULL; //心跳定时用信号量
uint32_t heart_tick = (60 * 1000);     //心跳时间 单位：ms

void anchor_tick_str(char *pdst);
void cmd_return_str(char *pdst);
void cmd_devid_str(char *pdst);
char buf[512];

char url_cmd_return[256];
char url_devid[256];

static int send_http_get(const char *uri, unsigned char *buf, int *len)
{
    int ret = 0, resp_status;
    struct webclient_session *session = RT_NULL;
    int file_size = 0, length, total_length = 0;
    rt_uint8_t *buffer_read = RT_NULL;

    /* create webclient session and set header response size */
    session = webclient_session_create(GET_HEADER_BUFSZ);
    if (!session)
    {
        LOG_E("open uri failed.");
        ret = -RT_ERROR;
        goto __exit;
    }

    /* send GET request by default header */
    if ((resp_status = webclient_get(session, uri)) != 200)
    {
        LOG_E("webclient GET request failed, response(%d) error.", resp_status);
        ret = -RT_ERROR;
        goto __exit;
    }
    file_size = webclient_content_length_get(session);
    rt_kprintf("http file_size:%d\n", file_size);

    if (file_size == 0)
    {
        LOG_E("Request file size is 0!");
        ret = -RT_ERROR;
        goto __exit;
    }
    else if (file_size < 0)
    {
        LOG_E("webclient GET request type is chunked.");
        ret = -RT_ERROR;
        goto __exit;
    }
    buffer_read = web_malloc(GET_REV_BUF_SIZE);
    if (buffer_read == RT_NULL)
    {
        LOG_E("No memory for http get rev!");
        ret = -RT_ERROR;
        goto __exit;
    }
    memset(buffer_read, 0x00, GET_REV_BUF_SIZE);
    do
    {
        length = webclient_read(session, buffer_read, file_size - total_length > GET_REV_BUF_SIZE ? GET_REV_BUF_SIZE : file_size - total_length);
        if (length > 0)
        {
            memcpy(buf + total_length, buffer_read, length);
            total_length += length;
            *len = total_length;
            LOG_I("get length:(%d)!", length);
        }
        else
        {
            LOG_E("Exit: server return err (%d)!", length);
            ret = -RT_ERROR;
            goto __exit;
        }

    } while (total_length != file_size);

    ret = RT_EOK;

    if (total_length == file_size)
    {
        LOG_I("get data ok!\r\n");
        //memcpy(buf,buffer_read,total_length);
        if (session != RT_NULL)
            webclient_close(session);
        if (buffer_read != RT_NULL)
            web_free(buffer_read);
    }
    return ret;

__exit:
    if (session != RT_NULL)
        webclient_close(session);
    if (buffer_read != RT_NULL)
        web_free(buffer_read);
    return ret;
}
//-----------------------------------------------------

void http_get_task(void *arg)
{
    int cnt = 0;
    int ret = 0;
    int len;
    char data[512];
    char url_tick[600];
    static rt_err_t result;

    rt_snprintf(http_heart_uri, sizeof(http_heart_uri), "http://%s:%s/netlog?data=", http_ip, http_port);
    // cmd_return_str(data);
    // sprintf(url_cmd_return, "%s%s", test_uri, data);
    // cmd_devid_str(data);
    // sprintf(url_devid, "%s%s", test_uri, data);

    //rt_kprintf("url%s\r\n",url);
    //rt_kprintf("url");
    while (1)
    {
        result = rt_sem_take(dynamic_sem, RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            rt_kprintf("Datanum = %d\n", Datanum);
            anchor_tick_str(data);
            rt_snprintf(url_tick, sizeof(url_tick), "%s%s", http_heart_uri, data);
            rt_kprintf("url_tick:\n%s\n", url_tick);
            ret = send_http_get(url_tick, buf, &len);
            if (ret == RT_EOK)
            {
                rt_kprintf("get data:%s", buf);
            }
        }
        else
        {
            rt_kprintf("rt_sem_take fail!\n");
        }
        Datanum = 0;
    }
}

/* 定时器 1 超时函数 */
static void timeout1(void *parameter)
{
    rt_sem_release(dynamic_sem);
    // rt_kprintf("periodic timer is timeout!\n");
}

/*获取 忽略标签列表 */
void noresp_get_task(void *arg)
{
    uint16_t sleep_time = 400; //延时时间 ms
    int ret = 0;
    int len;
    rt_snprintf(http_noresp_uri, sizeof(http_noresp_uri), "http://%s:%s/nolist?serial=%s", http_ip, http_port, dev_address_str);
    rt_kprintf("http_noresp_uri:\n%s\n", http_noresp_uri);
    while (1)
    {
        ret = send_http_get(http_noresp_uri, buf, &len);
        if (ret == RT_EOK)
        {
            // rt_kprintf("get data:%s", buf);
        }
        rt_thread_mdelay(sleep_time);
    }
}

/*启动 http 相关任务 */
void start_http_get(void)
{
    dynamic_sem = rt_sem_create("dsem", 0, RT_IPC_FLAG_FIFO); //创建信号量
    rt_thread_t tid;
    tid = rt_thread_create("http_get_task",
                           http_get_task,
                           RT_NULL,
                           4096,
                           RT_THREAD_PRIORITY_MAX / 2,
                           20);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }

    timer1 = rt_timer_create("timer1",
                             timeout1,
                             RT_NULL,
                             heart_tick,
                             RT_TIMER_FLAG_PERIODIC);

    tid = rt_thread_create("noresp_get_task",
                           noresp_get_task,
                           RT_NULL,
                           2048,
                           20,
                           20);
    if (tid != RT_NULL)
    {
        // rt_thread_startup(tid);
    }

    return;
}

void anchor_tick_str(char *pdst)
{
    char default_mac[18];
    get_default_mac(default_mac);

    cJSON *root = RT_NULL,
          *data = RT_NULL;
    //   *data_obj = RT_NULL,
    //   *data_obj_data1 = RT_NULL,
    //   *data_obj_data2 = RT_NULL,
    //   *data_obj_data3 = RT_NULL,
    //   *data_obj_data4 = RT_NULL;

    root = cJSON_CreateObject();
    data = cJSON_CreateObject();
    // data_obj = cJSON_CreateObject();
    // data_obj_data1 = cJSON_CreateObject();
    // data_obj_data2 = cJSON_CreateObject();
    // data_obj_data3 = cJSON_CreateObject();
    // data_obj_data4 = cJSON_CreateObject();

    cJSON_AddNumberToObject(data, "p", 66);
    cJSON_AddStringToObject(data, "exceptstr", "");
    cJSON_AddNumberToObject(data, "exceptnum", 0);
    cJSON_AddNumberToObject(data, "datanum", Datanum);
    cJSON_AddStringToObject(data, "selfaddr", dev_address_str);

    // cJSON_AddNumberToObject(data_obj_data1, "a258", 112);
    // cJSON_AddItemToObject(data_obj, "1", data_obj_data1);

    // cJSON_AddNumberToObject(data_obj_data2, "0308", 463);
    // cJSON_AddNumberToObject(data_obj_data2, "0303", 463);
    // cJSON_AddNumberToObject(data_obj_data2, "a258", 58);
    // cJSON_AddNumberToObject(data_obj_data2, "0304", 467);
    // cJSON_AddNumberToObject(data_obj_data2, "0306", 421);
    // cJSON_AddItemToObject(data_obj, "2", data_obj_data2);

    // cJSON_AddNumberToObject(data_obj_data3, "0308", 463);
    // cJSON_AddNumberToObject(data_obj_data3, "0303", 463);
    // cJSON_AddNumberToObject(data_obj_data3, "a258", 58);
    // cJSON_AddItemToObject(data_obj, "3", data_obj_data3);

    // cJSON_AddNumberToObject(data_obj_data4, "0304", 467);
    // cJSON_AddItemToObject(data_obj, "4", data_obj_data4);

    // cJSON_AddItemToObject(data, "data_obj", data_obj);
    cJSON_AddItemToObject(root, "data", data);

    cJSON_AddNumberToObject(root, "nowtime", 0);
    cJSON_AddStringToObject(root, "mac", (const char *)default_mac);
    cJSON_AddStringToObject(root, "eqm", equip_model);

    char *datas = cJSON_PrintUnformatted(root);
    // rt_kprintf("\n%s\n", datas);
    // strcpy(pdst, datas);
    rt_sprintf(pdst, "%s", datas);
    // rt_kprintf("%s\n", pdst);
    rt_free(datas);
    datas = RT_NULL;
    cJSON_Delete(root);
}
/*
{"p":60,"src_addr":"a001","result":"success",mac:"xxx",address:"a001"}
*/
void cmd_return_str(char *pdst)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "p", 60);
    cJSON_AddStringToObject(root, "src_addr", "a001");
    cJSON_AddStringToObject(root, "result", "success");
    cJSON_AddStringToObject(root, "mac", "aabbccddeeff");
    cJSON_AddStringToObject(root, "address", "a001");
    //char* datas = cJSON_Print(root);
    //printf("%s\n", datas);
    char *datas = cJSON_PrintUnformatted(root);
    strcpy(pdst, datas);
    printf("%s\n", pdst);
    cJSON_Delete(root);
    rt_free(datas);
}
/*
{"p":9,"Host_Anchor":0,"PanId":"0411","Address":"1004","Channel":2,"Speed":1,"R	espSlot":4,
"SyncPeriod":500, "range_period":5000,"cm_led":1,mac:"xxx"}
*/
void cmd_devid_str(char *pdst)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "p", 9);
    cJSON_AddNumberToObject(root, "Host_Anchor", 0);

    cJSON_AddStringToObject(root, "PanId", "0411");
    cJSON_AddStringToObject(root, "Address", "1004");

    cJSON_AddNumberToObject(root, "Channel", 2);

    cJSON_AddNumberToObject(root, "Speed", 1);
    cJSON_AddNumberToObject(root, "RespSlot", 4);
    cJSON_AddNumberToObject(root, "SyncPeriod", 4);
    cJSON_AddNumberToObject(root, "range_period", 5000);
    cJSON_AddNumberToObject(root, "cm_led", 1);
    cJSON_AddStringToObject(root, "mac", "aabbccddeeff");

    //char* datas = cJSON_Print(root);
    //printf("%s\n", datas);
    char *datas = cJSON_PrintUnformatted(root);
    strcpy(pdst, datas);
    printf("%s\n", pdst);
    cJSON_Delete(root);
    rt_free(datas);
}

// MSH_CMD_EXPORT(start_http_get, test send http get);