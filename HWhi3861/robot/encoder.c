#include "config.h"


#define mygpio     5
#define LED_TASK_STACK_SIZE 4096
float cnt;


static void encoder_itrp(void)//zhongduan
{
    cnt++;
}
void encoder_init(void)
{
    IoTGpioInit(mygpio);
    hi_io_set_func(mygpio, 0);
    IoTGpioSetDir(mygpio, IOT_GPIO_DIR_IN);
    hi_io_set_pull(mygpio, HI_IO_PULL_DOWN);

    IoTGpioRegisterIsrFunc(mygpio, IOT_INT_TYPE_EDGE,
        IOT_GPIO_EDGE_RISE_LEVEL_HIGH,
        encoder_itrp, NULL);
}