#include "rader.h"


void func_init(void)
{
    IoTGpioInit(gpiohight);
    IoTGpioInit(gpiodir);
    hi_io_set_func(gpiodir,HI_IO_FUNC_GPIO_2_GPIO);
    hi_io_set_func(gpiohight,HI_IO_FUNC_GPIO_1_GPIO);
    IoTGpioSetDir(gpiodir,IOT_GPIO_DIR_OUT);

    IoTGpioSetDir(gpiohight,IOT_GPIO_DIR_OUT);
    

    IoTGpioInit(gpioec);
    IoTGpioInit(gpiotg);
    hi_io_set_func(gpioec,HI_IO_FUNC_GPIO_7_GPIO);

    hi_io_set_func(gpiotg,HI_IO_FUNC_GPIO_8_GPIO);
    IoTGpioSetDir(gpioec,IOT_GPIO_DIR_IN);
    IoTGpioSetDir(gpiotg,IOT_GPIO_DIR_OUT);
}
void turnangle1(int duty)
{
    for (int i = 0; i <10; i++) 
    {
        set_angle(duty);
    }
}
void set_angle( unsigned int duty) {
    //IoTGpioSetDir(gpiodir, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(gpiodir, IOT_GPIO_VALUE1);
    hi_udelay(duty);
    //IoTGpioSetOutputVal(gpiodir, IOT_GPIO_VALUE0);
    hi_udelay(20000 - duty);
}
void turnangle(int duty)
{
    
    IoTGpioSetOutputVal(gpiodir,IOT_GPIO_VALUE1);
    hi_udelay(duty);
    IoTGpioSetOutputVal(gpiodir,IOT_GPIO_VALUE0);
    hi_udelay(20000-duty);
    
    
}

void turnhight(int duty)
{
    IoTGpioSetOutputVal(gpiohight,IOT_GPIO_VALUE1);
    hi_udelay(duty);
    IoTGpioSetOutputVal(gpiohight,IOT_GPIO_VALUE0);
    hi_udelay(20000-duty);
}

float GetDistance(void) {
    static unsigned long start_time = 0, time = 0;
    float distance = 0.0;
    IotGpioValue value = IOT_GPIO_VALUE0;
    unsigned int flag = 0;

    IoTGpioSetOutputVal(gpiotg, IOT_GPIO_VALUE1);
    hi_udelay(20);
    IoTGpioSetOutputVal(gpiotg, IOT_GPIO_VALUE0);

    while (1) {
        IoTGpioGetInputVal(gpioec, &value);
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