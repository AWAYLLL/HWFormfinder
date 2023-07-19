#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"

#define GPIO_EC0 8//echo
#define GPIO_TG0 7//trigger
#define GPIO_TG1 13//trigger 
#define GPIO_TG3 14//trigger
#define GPIO_EC1 2
#define GPIO_EC3 6
#define GPIO_FUNC 0

void hcsr04_init(void)
{
    IoTGpioInit(GPIO_EC1);
    IoTGpioInit(GPIO_EC3);
    IoTGpioInit(GPIO_EC0);
    IoTGpioInit(GPIO_TG0);
    IoTGpioInit(GPIO_TG1);
    IoTGpioInit(GPIO_TG3);
    hi_io_set_func(GPIO_EC1, GPIO_FUNC);
    hi_io_set_func(GPIO_EC3, GPIO_FUNC);
    hi_io_set_func(GPIO_EC0, GPIO_FUNC);
    hi_io_set_func(GPIO_TG0, HI_IO_FUNC_GPIO_7_GPIO);
    hi_io_set_func(GPIO_TG1, HI_IO_FUNC_GPIO_13_GPIO);
    hi_io_set_func(GPIO_TG3, HI_IO_FUNC_GPIO_14_GPIO);
    IoTGpioSetDir(GPIO_EC1, IOT_GPIO_DIR_IN);
    IoTGpioSetDir(GPIO_EC3, IOT_GPIO_DIR_IN);
    IoTGpioSetDir(GPIO_EC0, IOT_GPIO_DIR_IN);
    IoTGpioSetDir(GPIO_TG0, IOT_GPIO_DIR_OUT);
    IoTGpioSetDir(GPIO_TG1, IOT_GPIO_DIR_OUT);
    IoTGpioSetDir(GPIO_TG3, IOT_GPIO_DIR_OUT);
}

float GetDistance0  (void) {
    static unsigned long start_time0 = 0, time0 = 0;
    float distance = 0.0;
    IotGpioValue value = IOT_GPIO_VALUE0;
    unsigned int flag = 0;

    

    IoTGpioSetOutputVal(GPIO_TG0, IOT_GPIO_VALUE1);
    hi_udelay(20);
    IoTGpioSetOutputVal(GPIO_TG0, IOT_GPIO_VALUE0);

    while (1) {
        IoTGpioGetInputVal(GPIO_EC0, &value);
        if ( value == IOT_GPIO_VALUE1 && flag == 0) {
            start_time0 = hi_get_us();
            flag = 1;
        }
        if (value == IOT_GPIO_VALUE0 && flag == 1) {
            time0 = hi_get_us() - start_time0;
            start_time0 = 0;
            break;
        }
    }
    distance = time0 * 0.034 / 2;
    return distance;
}
float GetDistance1  (void) {
    static unsigned long start_time = 0, time = 0;
    float distance = 0.0;
    IotGpioValue value = IOT_GPIO_VALUE0;
    unsigned int flag = 0;

    

    IoTGpioSetOutputVal(GPIO_TG1, IOT_GPIO_VALUE1);
    hi_udelay(20);
    IoTGpioSetOutputVal(GPIO_TG1, IOT_GPIO_VALUE0);

    while (1) {
       
        IoTGpioGetInputVal(GPIO_EC1, &value);
        if ( value == IOT_GPIO_VALUE1 && flag == 0) {
            start_time = hi_get_us();
            flag = 1;
        }
        if (value == IOT_GPIO_VALUE0 && flag == 1) {
            time = hi_get_us() - start_time;
            start_time = 0;
            break;
        }
    }
    distance = time * 0.034 / 2;
    return distance;
}
float GetDistance2  (void) {
    static unsigned long start_time3 = 0, time3 = 0;
    float distance = 0.0;
    IotGpioValue value = IOT_GPIO_VALUE0;
    unsigned int flag = 0;

    

    IoTGpioSetOutputVal(GPIO_TG3, IOT_GPIO_VALUE1);
    hi_udelay(20);
    IoTGpioSetOutputVal(GPIO_TG3, IOT_GPIO_VALUE0);

    while (1) {
        IoTGpioGetInputVal(GPIO_EC3, &value);
        if ( value == IOT_GPIO_VALUE1 && flag == 0) {
            start_time3 = hi_get_us();
            flag = 1;
        }
        if (value == IOT_GPIO_VALUE0 && flag == 1) {
            time3 = hi_get_us() - start_time3;
            start_time3 = 0;
            break;
        }
    }
    distance = time3 * 0.034 / 2;
    return distance;
}

