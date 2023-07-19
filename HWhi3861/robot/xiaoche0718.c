#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"
#include "robot_control.h"
#include "hi_adc.h"
#include "iot_errno.h"
//#include "mpu6050.h"
#include "config.h"
#include "hcsr04.c"
#include "encoder.h"
#include "sin.c"
#include <hi_task.h>
#include <string.h>
#include <hi_wifi_api.h>
#include <hi_mux.h>
#include <hi_gpio.h>
#include "iot_config.h"
#include "iot_log.h"
#include "iot_main.h"
#include "iot_profile.h"
#include <unistd.h>
#include "iot_uart.h"
#include "cJSON.h"




#define STACK_SIZE (4096)
//#define GPIO5 5
#define FUNC_GPIO 0
#define     ADC_TEST_LENGTH             (20)
#define     VLT_MIN                     (100)
#define     OLED_FALG_ON                ((unsigned char)0x01)
#define     OLED_FALG_OFF               ((unsigned char)0x00)
#define GPIO0 0 //left
#define GPIO1 1 //left
#define GPIO9 9 //right
#define GPIO10 10 //right
#define GPIOFUNC 0
#define pi 3.14159
#define C 20.42
#define D 20





#define PWM_FREQ_DIVITION 64000
#define cos10   0.98481
#define sin10   0.17365
#define TAKE_THE_INITIATIVE_TO_REPORT
#define ONE_SECOND                          (1000)
/* oc request id */
#define CN_COMMAND_INDEX                    "commands/request_id="
#define WECHAT_SUBSCRIBE_LIGHT              "light"
#define WECHAT_SUBSCRIBE_LIGHT_ON_STATE     "1"
#define WECHAT_SUBSCRIBE_LIGHT_OFF_STATE    "0"
#define IOT_UART_IDX_2  (2)



unsigned short  g_adc_buf[ADC_TEST_LENGTH] = { 0 };
//unsigned short  g_gpio5_adc_buf[ADC_TEST_LENGTH] = { 0 };
//unsigned int  g_gpio5_tick = 0;
unsigned char   g_car_control_mode = 0;
unsigned char   g_car_speed_control = 0;
unsigned int  g_car_control_demo_task_id = 0;
unsigned char   g_car_status = CAR_STOP_STATUS;
double x;
double y;
float distance_middle;//2
float distance_Left;//1
float distance_Right;//0
int a;
float angle;
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

hi_double datas[16];
extern float GetDistance0(void);
extern float GetDistance1(void);
extern float GetDistance2(void);
extern void hcsr04_init();
extern float _sin(float);
extern float _cos(float);

//////////////////////////
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
hi_void IotPublishSample(double xx,double yy,float aangle)
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
        .reportAction.subDeviceActionMotor = "x",
        .reportAction.motorActionStatus = xx, /* 0 : motor off */
        /* report temperature */
        .reportAction.subDeviceActionTemperature = "y",
        .reportAction.temperatureData = yy, /* 30 :temperature data */
        /* report humidity */
        .reportAction.subDeviceActionHumidity = "angle",
        .reportAction.humidityActionData = aangle, /* humidity data */
        /* report light_intensity */
        .reportAction.subDeviceActionLightIntensity = "light_intensity",
        .reportAction.lightIntensityActionData = 60, /* 60 : light_intensity */
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
////////////////////

void motor_init(void)//0 1左边 9 10 右边 0 9:M+ ;1 10:-
{
    IoTGpioInit(0);
    IoTGpioInit(1);
    IoTGpioInit(9);
    IoTGpioInit(10);
    hi_io_set_func(HI_IO_NAME_GPIO_0, 0);
    hi_io_set_func(HI_IO_NAME_GPIO_1, 0);
    hi_io_set_func(HI_IO_NAME_GPIO_9, 0);
    hi_io_set_func(HI_IO_NAME_GPIO_10, 0);
    //IoTPwmInit(3);IoTPwmInit(4);
    //IoTPwmInit(1);
    //IoTPwmInit(0);
}

void GoForward(void)
{

    hi_io_set_func(0, GPIOFUNC);
    IoTGpioSetDir(0, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(0, IOT_GPIO_VALUE1);
    hi_io_set_func(1, GPIOFUNC);
    IoTGpioSetDir(1, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(1, IOT_GPIO_VALUE0);
    hi_io_set_func(9, GPIOFUNC);
    IoTGpioSetDir(9, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(9, IOT_GPIO_VALUE1);
    hi_io_set_func(10, GPIOFUNC);
    IoTGpioSetDir(10, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(10, IOT_GPIO_VALUE0);
}
void TurnLeft(void)
{  
    hi_io_set_func(0, GPIOFUNC);
    IoTGpioSetDir(0, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(0, IOT_GPIO_VALUE0);
    hi_io_set_func(1, GPIOFUNC);
    IoTGpioSetDir(1, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(1, IOT_GPIO_VALUE1);
    hi_io_set_func(9, GPIOFUNC);
    IoTGpioSetDir(9, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(9, IOT_GPIO_VALUE1);
    hi_io_set_func(10, GPIOFUNC);
    IoTGpioSetDir(10, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(10, IOT_GPIO_VALUE0);
}
void TurnRight(void)
{  
    hi_io_set_func(0, GPIOFUNC);
    IoTGpioSetDir(0, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(0, IOT_GPIO_VALUE1);
    hi_io_set_func(1, GPIOFUNC);
    IoTGpioSetDir(1, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(1, IOT_GPIO_VALUE0);
    hi_io_set_func(9, GPIOFUNC);
    IoTGpioSetDir(9, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(9, IOT_GPIO_VALUE0);
    hi_io_set_func(10, GPIOFUNC);
    IoTGpioSetDir(10, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(10, IOT_GPIO_VALUE1);
}
void GoBackward(void)
{  
    hi_io_set_func(0, GPIOFUNC);
    IoTGpioSetDir(0, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(0, IOT_GPIO_VALUE0);
    hi_io_set_func(1, GPIOFUNC);
    IoTGpioSetDir(1, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(1, IOT_GPIO_VALUE1);
    hi_io_set_func(9, GPIOFUNC);
    IoTGpioSetDir(9, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(9, IOT_GPIO_VALUE0);
    hi_io_set_func(10, GPIOFUNC);
    IoTGpioSetDir(10, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(10, IOT_GPIO_VALUE1);
}
float mymax(float a,float b,float c)
{
    if(a>=b && a>=c)
        return a;
    else if(b>=a && b>=c)
        return b;
    else if (c>=a && c>=b )
        return c;
}
void stop0710(void){
    hi_io_set_func(0, GPIOFUNC);
    IoTGpioSetDir(0, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(0, IOT_GPIO_VALUE0);
    hi_io_set_func(1, GPIOFUNC);
    IoTGpioSetDir(1, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(1, IOT_GPIO_VALUE0);
    hi_io_set_func(9, GPIOFUNC);
    IoTGpioSetDir(9, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(9, IOT_GPIO_VALUE0);
    hi_io_set_func(10, GPIOFUNC);
    IoTGpioSetDir(10, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(10, IOT_GPIO_VALUE0);
}

void Gowhere(void)
{   
    
    
   
    float  distance_Left = GetDistance1();
 
    float distance_Right = GetDistance0();

    float distance_middle = GetDistance2();

    if(distance_middle<20 || distance_Left<15 || distance_Right<15)
    {
        if(distance_middle<8 || distance_Left<5 || distance_Right<5)
        {
            GoBackward();

            //while(GetDistance2()<15);
            hi_udelay(200000);
            x=x-cnt*C/1320*_sin(angle*pi/180);
            y=y-cnt*C/1320*_cos(angle*pi/180);
        }
        else if(distance_Left<10)
        {
            L_angle(35);
            angle-=10;//右转为减
        }
        else if(distance_Right<10)
        {
            R_angle(35);//10度
            angle+=10;
        }
        else{
            GoForward();
            hi_udelay(40000);
        }
    }
    else if(distance_middle>30 && distance_Left>120 && distance_Right>50 && distance_middle<40)
    {
        
        angle+=90;
        R_angle(315);
        GoForward();
        hi_udelay(40000);
        x=x+cnt*C/1320*_sin(angle*pi/180);
        y=y+cnt*C/1320*_cos(angle*pi/180);
    }
    else   {
        GoForward();
        hi_udelay(40000);
        x=x+cnt*C/1320*_sin(angle*pi/180);
        y=y+cnt*C/1320*_cos(angle*pi/180);
    }
   
    
}


void L_angle(int dy)
{
    TurnLeft();
    osDelay(dy);
    stop0710();
}

void R_angle(int dy)
{
    TurnRight();
    osDelay(dy);
    stop0710();
}

static void Test(void)
{
    x = 0;
    y = 0;
    hcsr04_init();
    motor_init();
    encoder_init();
    //WifiStaReadyWait();
    //cJsonInit();
    //IoTMain();
    angle=90;
    //static unsigned long start_time0 = 0, time0 = 0;
    //start_time0 = hi_get_seconds();
    //IoTSetMsgCallback(DemoMsgRcvCallBack);
//#ifdef TAKE_THE_INITIATIVE_TO_REPORT
    while(1){
        cnt=0;
        //time0= hi_get_seconds();
        Gowhere();
        //if(time0-start_time0>1){
        //    IotPublishSample(x,y,angle);
        //    start_time0 = hi_get_seconds();
        //}
//#endif
        //printf("angle = %f\r\n",angle);
        //printf("x=%f\r\n",x);
        //printf("y=%f\r\n",y);
    }
}


static void PWMLedDemo(void)
{
    osThreadAttr_t attr;
    IoTWatchDogDisable();
    // set Red LED pin to GPIO function
    //IoTGpioInit(RED_LED_PIN_NAME);      

    attr.name = "Test";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = osPriorityNormal;

    if (osThreadNew(Test, NULL, &attr) == NULL) {
        printf("[ColorfulLightDemo] Falied to create Test!\n");
    }
}
APP_FEATURE_INIT(PWMLedDemo);