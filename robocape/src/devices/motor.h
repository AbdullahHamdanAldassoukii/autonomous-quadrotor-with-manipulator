/*******************************************************************************
* motor.h
*
* Peter Gaskell 2016
*  
*******************************************************************************/

#include "../robocape.h"
#include "../usefulincludes.h"
#include "../simple/simple_pwm.h"
#include "../simple/simple_gpio.h"
#include "../mmap/mmap_pwmss.h"
#include "../mmap/mmap_gpio_adc.h"
#include "../mmap/mmap_gpio_adc_defs.h"


#define MOTOR_PWM_FREQ 2000

typedef struct{
    int pwm_num;
    char pwm_chan;
    int dir_pin;
    int nEn_pin;
    float speed;
} motor_t;

motor_t init_motor(int pwm_num, char pwm_chan, int dir_pin, int nEn_pin);
int set_motor_speed(motor_t* motor, float speed);
int set_motor_on(motor_t motor);
int set_motor_off(motor_t motor);
int uninit_motor(motor_t motor);