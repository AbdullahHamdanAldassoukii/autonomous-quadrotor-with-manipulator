// C library for interfacing with the TI PWM userspace driver
// This relies on a device tree overlay enabling hrpwm 


#ifndef SIMPLEPWM_H_
#define SIMPLEPWM_H_

int simple_init_pwm(int ss, int frequency);
int simple_uninit_pwm(int ss);
int simple_set_pwm_duty(int ss, char ch, float duty);
int simple_set_pwm_duty_ns(int ss, char ch, int duty_ns);
int simple_get_pwm_duty_ns(int ss, char ch);

#endif