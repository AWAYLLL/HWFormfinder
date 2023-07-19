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
#include "mpu6050.h"
#include "inv_mpu.h"
#include "dmpKey.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpmap.h"
#include "config.h"


#include "hcsr04.c"

#include "encoder.h"
#include "sin.c"



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


hi_double datas[16];
extern float GetDistance0(void);
extern float GetDistance1(void);
extern float GetDistance2(void);
extern void hcsr04_init();
extern float _sin(float);
extern float _cos(float);





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
            hi_udelay(40000);
            x=x-cnt*C/1320*_sin(angle*pi/180);
            y=y-cnt*C/1320*_cos(angle*pi/180);
            L_angle();
            angle-=10;
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
            hi_udelay(10000);
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
    MPU_Init();
    encoder_init();
    while(mpu_dmp_init());
    
    angle=90;
    while(1){
        cnt=0;
        
        
        Gowhere();


        printf("angle = %f\r\n",angle);
        printf("x=%f\r\n",x);
        printf("y=%f\r\n",y);
    }
}


static void PWMLedDemo(void)
{
    osThreadAttr_t attr;

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