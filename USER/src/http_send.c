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

#include <rtdbg.h>

#define GET_HEADER_BUFSZ 1024
#define GET_REV_BUF_SIZE 1024

const char *test_uri = "http://192.168.1.8:8080/netlog?data=";

void anchor_tick_str(char *pdst);
void cmd_return_str(char *pdst);
void cmd_devid_str(char *pdst);
char buf[512];
//test
char url_tick[512];
char url_cmd_return[256];
char url_devid[256];
char data[1024];

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

void send_http_get_test(void *arg)
{
    int cnt = 0;
    int ret = 0;
    int len;
    anchor_tick_str(data);
    sprintf(url_tick, "%s%s", test_uri, data);

    // cmd_return_str(data);
    // sprintf(url_cmd_return, "%s%s", test_uri, data);
    // cmd_devid_str(data);
    // sprintf(url_devid, "%s%s", test_uri, data);

    //rt_kprintf("url%s\r\n",url);
    //rt_kprintf("url");
    while (1)
    {

        // switch (cnt)
        // {
        // case 0:
        // 	ret = send_http_get(url_tick, buf, &len);
        // 	cnt++;
        // 	break;
        // case 1:
        // 	ret = send_http_get(url_cmd_return, buf, &len);
        // 	cnt++;
        // 	break;
        // case 2:
        // 	ret = send_http_get(url_devid, buf, &len);
        // 	cnt++;
        // 	break;
        // default:
        // 	break;
        // }

        ret = send_http_get(url_tick, buf, &len);
        if (ret == RT_EOK)
        {
            rt_kprintf("get data:%s", buf);
        }
        // if (cnt >= 3)
        // 	cnt = 0;

        rt_thread_mdelay(10 * 1000);
    }
}

static void start_http_get(int argc, char **argv)
{
    rt_thread_t tid;
    tid = rt_thread_create("send_http_get_test",
                           send_http_get_test, RT_NULL,
                           2048, RT_THREAD_PRIORITY_MAX / 3, 20);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    return;
}

void anchor_tick_str(char *pdst)
{
    cJSON *root = RT_NULL,
          *data = RT_NULL,
          *data_obj = RT_NULL,
          *data_obj_data1 = RT_NULL,
          *data_obj_data2 = RT_NULL,
          *data_obj_data3 = RT_NULL,
          *data_obj_data4 = RT_NULL;

    root = cJSON_CreateObject();
    data = cJSON_CreateObject();
    data_obj = cJSON_CreateObject();
    data_obj_data1 = cJSON_CreateObject();
    data_obj_data2 = cJSON_CreateObject();
    data_obj_data3 = cJSON_CreateObject();
    data_obj_data4 = cJSON_CreateObject();

    cJSON_AddNumberToObject(data, "p", 66);
    cJSON_AddStringToObject(data, "exceptstr", "");
    cJSON_AddNumberToObject(data, "exceptnum", 0);
    cJSON_AddNumberToObject(data, "datanum", 3229);
    cJSON_AddStringToObject(data, "selfaddr", "a666");

    cJSON_AddNumberToObject(data_obj_data1, "a258", 112);
    cJSON_AddItemToObject(data_obj, "1", data_obj_data1);

    cJSON_AddNumberToObject(data_obj_data2, "0308", 463);
    cJSON_AddNumberToObject(data_obj_data2, "0303", 463);
    cJSON_AddNumberToObject(data_obj_data2, "a258", 58);
    cJSON_AddNumberToObject(data_obj_data2, "0304", 467);
    cJSON_AddNumberToObject(data_obj_data2, "0306", 421);
    cJSON_AddItemToObject(data_obj, "2", data_obj_data2);

    cJSON_AddNumberToObject(data_obj_data3, "0308", 463);
    cJSON_AddNumberToObject(data_obj_data3, "0303", 463);
    cJSON_AddNumberToObject(data_obj_data3, "a258", 58);
    cJSON_AddItemToObject(data_obj, "3", data_obj_data3);

    cJSON_AddNumberToObject(data_obj_data4, "0304", 467);
    cJSON_AddItemToObject(data_obj, "4", data_obj_data4);

    cJSON_AddItemToObject(data, "data_obj", data_obj);
    cJSON_AddItemToObject(root, "data", data);

    cJSON_AddNumberToObject(root, "nowtime", 15);
    cJSON_AddStringToObject(root, "mac", "aabbccddeeff");

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

MSH_CMD_EXPORT(start_http_get, test send http get);