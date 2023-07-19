#include <stdio.h>


#define _PI_2  1.57079632679
#define _PI    3.14159265359
#define _3PI_2 4.71238898038
#define _2PI   6.28318530718

#define _round(x) ((x) >= 0 ? (long)((x) + 0.5) : (long)((x) - 0.5))

const int sine_array[200] = {
    0, 79, 158, 237, 316, 395, 473, 552, 631, 710,
    789, 867, 946, 1024, 1103, 1181, 1260, 1338, 1416, 1494,
    1572, 1650, 1728, 1806, 1883, 1961, 2038, 2115, 2192, 2269,
    2346, 2423, 2499, 2575, 2652, 2728, 2804, 2879, 2955, 3030,
    3105, 3180, 3255, 3329, 3404, 3478, 3552, 3625, 3699, 3772,
    3845, 3918, 3990, 4063, 4135, 4206, 4278, 4349, 4420, 4491,
    4561, 4631, 4701, 4770, 4840, 4909, 4977, 5046, 5113, 5181,
    5249, 5316, 5382, 5449, 5515, 5580, 5646, 5711, 5775, 5839,
    5903, 5967, 6030, 6093, 6155, 6217, 6279, 6340, 6401, 6461,
    6521, 6581, 6640, 6699, 6758, 6815, 6873, 6930, 6987, 7043,
    7099, 7154, 7209, 7264, 7318, 7371, 7424, 7477, 7529, 7581,
    7632, 7683, 7733, 7783, 7832, 7881, 7930, 7977, 8025, 8072,
    8118, 8164, 8209, 8254, 8298, 8342, 8385, 8428, 8470, 8512,
    8553, 8594, 8634, 8673, 8712, 8751, 8789, 8826, 8863, 8899,
    8935, 8970, 9005, 9039, 9072, 9105, 9138, 9169, 9201, 9231,
    9261, 9291, 9320, 9348, 9376, 9403, 9429, 9455, 9481, 9506,
    9530, 9554, 9577, 9599, 9621, 9642, 9663, 9683, 9702, 9721,
    9739, 9757, 9774, 9790, 9806, 9821, 9836, 9850, 9863, 9876,
    9888, 9899, 9910, 9920, 9930, 9939, 9947, 9955, 9962, 9969,
    9975, 9980, 9985, 9989, 9992, 9995, 9997, 9999, 10000, 10000
};

float _sin ( float a ) {
    if ( a < _PI_2 ) {
        return 0.0001 * sine_array[_round ( 126.6873 * a )];
    } else if ( a < _PI ) {
        return 0.0001 * sine_array[398 - _round ( 126.6873 * a )];
    } else if ( a < _3PI_2 ) {
        return -0.0001 * sine_array[-398 + _round ( 126.6873 * a )];
    } else {
        return -0.0001 * sine_array[796 - _round ( 126.6873 * a )];
    }
}

float _cos ( float a ) {
    float a_sin = a + _PI_2;
    a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
    return _sin ( a_sin );
}





 

/*static void Test(void)
{
float test =  3.1415926 /3;
    //float x = _sin(test);
    //float y = _cos(test);

    printf ( "_sin is %f\n", _sin ( test ) );
    printf ( "_cos is %f\n", _cos ( test ) );


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
APP_FEATURE_INIT(PWMLedDemo);  */