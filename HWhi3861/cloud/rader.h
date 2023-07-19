#ifndef __RADAR_H__
#define __RADAR_H__

#include <stdio.h>
#include <unistd.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "iot_pwm.h"
#include "iot_errno.h"
#include "hi_io.h"
#include "hi_time.h"
#define gpiodir 2
#define PWM_FREQ_DIVITION 64000
#define DELAY_US 1000
#define STACK_SIZE (4096)
#define PWM_PORT_NUM  (2)

#define gpiohight 1

#define gpioec 8
#define gpiotg 7

void turnangle1(int duty);
void set_angle( unsigned int duty);
void func_init(void);

void turnangle(int duty);

void turnhight(int duty);
float GetDistance(void) ;

#endif