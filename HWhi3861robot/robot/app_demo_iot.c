/*
 * Copyright (c) 2022 HiSilicon (Shanghai) Technologies CO., LIMITED.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <hi_task.h>
#include <string.h>
#include <hi_wifi_api.h>
#include <hi_mux.h>
#include <hi_io.h>
#include <hi_gpio.h>
#include "iot_config.h"
#include "iot_log.h"
#include "iot_main.h"
#include "iot_profile.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "iot_gpio.h"
#include "iot_uart.h"
#include "cJSON.h"
#include "rader.h"
/* attribute initiative to report */
#define TAKE_THE_INITIATIVE_TO_REPORT
#define ONE_SECOND                          (8192)
/* oc request id */
#define CN_COMMAND_INDEX                    "commands/request_id="
#define WECHAT_SUBSCRIBE_LIGHT              "light"
#define WECHAT_SUBSCRIBE_LIGHT_ON_STATE     "1"
#define WECHAT_SUBSCRIBE_LIGHT_OFF_STATE    "0"
#define IOT_UART_IDX_2  (2)
int g_lightStatus = -1;

typedef void (*FnMsgCallBack)(hi_gpio_value val);

typedef struct FunctionCallback {
    hi_bool  stop;
    hi_u32 conLost;
    hi_u32 queueID;
    hi_u32 iotTaskID;
    FnMsgCallBack    msgCallBack;
}FunctionCallback;
FunctionCallback g_functionCallback;

/* CPU Sleep time Set */
unsigned int TaskMsleep(unsigned int ms)
{
    if (ms <= 0) {
        return HI_ERR_FAILURE;
    }
    return hi_sleep((hi_u32)ms);
}

static void DeviceConfigInit(hi_gpio_value val)
{
    hi_io_set_func(HI_IO_NAME_GPIO_9, HI_IO_FUNC_GPIO_9_GPIO);
    hi_gpio_set_dir(HI_GPIO_IDX_9, HI_GPIO_DIR_OUT);
    hi_gpio_set_ouput_val(HI_GPIO_IDX_9, val);
}

static int  DeviceMsgCallback(FnMsgCallBack msgCallBack)
{
    g_functionCallback.msgCallBack = msgCallBack;
    return 0;
}

static void wechatControlDeviceMsg(hi_gpio_value val)
{
    DeviceConfigInit(val);
}

// < this is the callback function, set to the mqtt, and if any messages come, it will be called
// < The payload here is the json string
static void DemoMsgRcvCallBack(int qos, const char *topic, const char *payload)
{
    IOT_LOG_DEBUG("RCVMSG:QOS:%d TOPIC:%s PAYLOAD:%s\r\n", qos, topic, payload);
    /* 云端下发命令后，板端的操作处理 */
    if (strstr(payload, WECHAT_SUBSCRIBE_LIGHT) != NULL) {
        if (strstr(payload, WECHAT_SUBSCRIBE_LIGHT_OFF_STATE) != NULL) {
            wechatControlDeviceMsg(HI_GPIO_VALUE1);
            g_lightStatus = HI_FALSE;
        } else {
            wechatControlDeviceMsg(HI_GPIO_VALUE0);
            g_lightStatus = HI_TRUE;
        }
    }
    return HI_NULL;
}

/* publish sample */
hi_void IotPublishSample(float dis, int duty, int i,int key)
{
    /* reported attribute */
    //printf("\ntemp:%s\n",temp);
    WeChatProfile weChatProfile = {
        .subscribeType = "type",
        .status.subState = "state",
        .status.subReport = "reported",
        .status.reportVersion = "version",
        .status.Token = "clientToken",
        /* report motor */
        .reportAction.subDeviceActionMotor = "height",
        .reportAction.motorActionStatus = duty, /* 0 : motor off */
        /* report temperature */
        .reportAction.subDeviceActionTemperature = "angle",
        .reportAction.temperatureData = i, /* 30 :temperature data */
        /* report humidity */
        .reportAction.subDeviceActionHumidity = "distance",
        .reportAction.humidityActionData = dis, /* humidity data */
        /* report light_intensity */
        .reportAction.subDeviceActionLightIntensity = "key",
        .reportAction.lightIntensityActionData = key, /* 60 : light_intensity */
    };

    /* report light */
    if (g_lightStatus == HI_TRUE) {
        weChatProfile.reportAction.subDeviceActionLight = "light";
        weChatProfile.reportAction.lightActionStatus = 1; /* 1: light on */
    } else if (g_lightStatus == HI_FALSE) {
        weChatProfile.reportAction.subDeviceActionLight = "light";
        weChatProfile.reportAction.lightActionStatus = 0; /* 0: light off */
    } else {
        weChatProfile.reportAction.subDeviceActionLight = "light";
        weChatProfile.reportAction.lightActionStatus = 0; /* 0: light off */
    }
    /* profile report */
    IoTProfilePropertyReport(CONFIG_USER_ID, &weChatProfile);
}
int usr_uart_config(void)
{
    //IotUartAttribute g_uart_cfg = {115200, 8, 1, IOT_UART_PARITY_NONE, 500, 500, 0};
    //38400
    IotUartAttribute g_uart_cfg = {38400, 8, 1,0};
    int ret = IoTUartInit(IOT_UART_IDX_2, &g_uart_cfg);
    if (ret != 0) {
        printf("uart init fail\r\n");
    }
    else
    {
        printf("uart init success\r\n");
    }

    return ret;
}
// < this is the demo main task entry,here we will set the wifi/cjson/mqtt ready and
// < wait if any work to do in the while
static hi_void *DemoEntry(const char *arg)
{
    WifiStaReadyWait();
    func_init();
    cJsonInit();
    IoTMain();
    /* 云端下发回调 */
    IoTSetMsgCallback(DemoMsgRcvCallBack);
    float a;
    int key=0;
    int duty=500;
    /* 主动上报 */
    turnhight(500);
    turnangle(500);
    //IoTGpioInit(11);
    //IoTGpioInit(12);
    //usr_uart_config();
#ifdef TAKE_THE_INITIATIVE_TO_REPORT
    while (1) {
        /* 用户可以在这调用发布函数进行发布，需要用户自己写调用函数 */
        
            //strcat(uarttest,uartreadbuff);
            //printf("\n---\n");
            //.printf("%s",uartreadbuff);
            //printf("\n---\n");
        turnhight(duty);
        TaskMsleep(10);
        
        for (int i = 500; i <= 2500; i+=20) {
            turnangle(i);
            a=GetDistance();
            key++;
            IotPublishSample(a,(duty-500)*180/2000,(i-500)*180/2000,key);
            TaskMsleep(10);
            //osDelay(20);
            //printf("hight=%d\r\n",duty*180/2500);
            //printf("angle=%d\r\n",i*180/2500);
            //printf("%f\r\n",a);
        }
        duty+=50;
        if(duty>1000)
            duty=500;
        //TaskMsleep(1000);
        
        
        //for (int i = 2500; i >= 500; i-=20) {
        //    turnangle(i);
        //    a=GetDistance();
        //    key++;
        //    IotPublishSample(a,(duty-500)*180/2000,(i-500)*180/2000,key);
        //}
         // 发布例程
#endif
        
    }
    return NULL;
}

// < This is the demo entry, we create a task here,
// and all the works has been done in the demo_entry
#define CN_IOT_TASK_STACKSIZE  0x1000
#define CN_IOT_TASK_PRIOR 25
#define CN_IOT_TASK_NAME "IOTDEMO"

static void AppDemoIot(void)
{
    osThreadAttr_t attr;
    IoTWatchDogDisable();

    attr.name = "IOTDEMO";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = CN_IOT_TASK_STACKSIZE;
    attr.priority = CN_IOT_TASK_PRIOR;

    if (osThreadNew((osThreadFunc_t)DemoEntry, NULL, &attr) == NULL) {
        printf("[mqtt] Failed to create IOTDEMO!\n");
    }
}

SYS_RUN(AppDemoIot);