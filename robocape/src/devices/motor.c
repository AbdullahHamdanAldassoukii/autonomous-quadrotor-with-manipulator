/*******************************************************************************
* motor.c
*
* Peter Gaskell 2016
*
*******************************************************************************/
#include "motor.h"

motor_t init_motor(int pwm_num, char pwm_chan, int dir_pin, int nEn_pin){
    // Initialize your motor by setting up GPIO and PWM
    motor_t motor;

    // n_enable pin (nEn_pin), disables with high value.
    motor.nEn_pin = nEn_pin;

    // export the direction pin and n_enable pin for GPIO
    gpio_export(dir_pin);
    gpio_export(nEn_pin);

    // Set the GPIO pins to either input or output
    // INPUT: read, write
    // OUTPUT: read

    // PWM
    // use simple_init_pwm

    return motor;
}

int set_motor_speed(motor_t* motor, float speed){
    // Adjust motor speed via PWM and direction via the dir pins
    // SV1, SV3: are the actual pin locations on the board correspond to
    //           A0, A1. 
    int check = 0;

    return check;
}

/* CODE BELOW IS ALREADY COMPLETE */
int set_motor_on(motor_t motor){
    // Enable motor via GPIO
    int check = mmap_gpio_write(motor.nEn_pin, 0);
    return check;
}

int set_motor_off(motor_t motor){
    // Disable motor via GPIO
    int check = mmap_gpio_write(motor.nEn_pin, 1);
    return check;
}

int uninit_motor(motor_t motor){
    // Unitialize motor by disableing the motor, and uninitializing PWMs

    // GPIO
    set_motor_off(motor);
    gpio_unexport(motor.dir_pin);
    gpio_unexport(motor.nEn_pin);

    // PWM
    simple_uninit_pwm(motor.pwm_num);

    return 0;
}
