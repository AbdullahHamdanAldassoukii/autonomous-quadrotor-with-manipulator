// ti_pwm_userspace_defs.h
//
// this is a list of the userspace directories created by the ti pwm driver
// all directories derived from 4.4.9-bone10

#ifndef TIPWMUSR_H_
#define TIPWMUSR_H_

// export directories
const char* pwm_export_path[2][3] = {{ \
"/sys/class/pwm/pwmchip1/export", \
"/sys/class/pwm/pwmchip3/export", \
"/sys/class/pwm/pwmchip0/export"},{ \
"/sys/class/pwm/pwmchip1/export", \
"/sys/class/pwm/pwmchip3/export", \
"/sys/class/pwm/pwmchip0/export"}};\


const char* pwm_unexport_path[2][3] = {{ \
"/sys/class/pwm/pwmchip1/unexport", \
"/sys/class/pwm/pwmchip3/unexport", \
"/sys/class/pwm/pwmchip0/unexport"},{ \
"/sys/class/pwm/pwmchip1/unexport", \
"/sys/class/pwm/pwmchip3/unexport", \
"/sys/class/pwm/pwmchip0/unexport"}};\


// channel enable
const char* pwm_chA_enable_path[2][3] = {{ \
"/sys/class/pwm/pwmchip1/pwm0/enable", \
"/sys/class/pwm/pwmchip3/pwm0/enable", \
"/sys/class/pwm/pwmchip0/pwm0/enable"},{
"/sys/class/pwm/pwmchip1/pwm0/enable", \
"/sys/class/pwm/pwmchip3/pwm0/enable", \
"/sys/class/pwm/pwmchip0/pwm0/enable"}};

const char* pwm_chB_enable_path[2][3] = {{ \
"/sys/class/pwm/pwmchip1/pwm1/enable", \
"/sys/class/pwm/pwmchip3/pwm1/enable", \
"/sys/class/pwm/pwmchip0/pwm1/enable"},{
"/sys/class/pwm/pwmchip1/pwm1/enable", \
"/sys/class/pwm/pwmchip3/pwm1/enable", \
"/sys/class/pwm/pwmchip0/pwm1/enable"}};

// channel polarity
const char* pwm_chA_polarity_path[2][3] = {{ \
"/sys/class/pwm/pwmchip1/pwm0/polarity", \
"/sys/class/pwm/pwmchip3/pwm0/polarity", \
"/sys/class/pwm/pwmchip0/pwm0/polarity"},{
"/sys/class/pwm/pwmchip1/pwm0/polarity", \
"/sys/class/pwm/pwmchip3/pwm0/polarity", \
"/sys/class/pwm/pwmchip0/pwm0/polarity"}};

const char* pwm_chB_polarity_path[2][3] = {{ \
"/sys/class/pwm/pwmchip1/pwm1/polarity", \
"/sys/class/pwm/pwmchip3/pwm1/polarity", \
"/sys/class/pwm/pwmchip0/pwm1/polarity"},{
"/sys/class/pwm/pwmchip1/pwm1/polarity", \
"/sys/class/pwm/pwmchip3/pwm1/polarity", \
"/sys/class/pwm/pwmchip0/pwm1/polarity"}};


// channel period
const char* pwm_chA_period_path[2][3] = {{ \
"/sys/class/pwm/pwmchip1/pwm0/period", \
"/sys/class/pwm/pwmchip3/pwm0/period", \
"/sys/class/pwm/pwmchip0/pwm0/period"},{
"/sys/class/pwm/pwmchip1/pwm0/period", \
"/sys/class/pwm/pwmchip3/pwm0/period", \
"/sys/class/pwm/pwmchip0/pwm0/period"}};

const char* pwm_chB_period_path[2][3] = {{ \
"/sys/class/pwm/pwmchip1/pwm1/period", \
"/sys/class/pwm/pwmchip3/pwm1/period", \
"/sys/class/pwm/pwmchip0/pwm1/period"},{
"/sys/class/pwm/pwmchip1/pwm1/period", \
"/sys/class/pwm/pwmchip3/pwm1/period", \
"/sys/class/pwm/pwmchip0/pwm1/period"}};

// channel duty cycle
const char* pwm_chA_duty_path[2][3] = {{ \
"/sys/class/pwm/pwmchip1/pwm0/duty_cycle", \
"/sys/class/pwm/pwmchip3/pwm0/duty_cycle", \
"/sys/class/pwm/pwmchip0/pwm0/duty_cycle"},{
"/sys/class/pwm/pwmchip1/pwm0/duty_cycle", \
"/sys/class/pwm/pwmchip3/pwm0/duty_cycle", \
"/sys/class/pwm/pwmchip0/pwm0/duty_cycle"}};

const char* pwm_chB_duty_path[2][3] = {{ \
"/sys/class/pwm/pwmchip1/pwm1/duty_cycle", \
"/sys/class/pwm/pwmchip3/pwm1/duty_cycle", \
"/sys/class/pwm/pwmchip0/pwm1/duty_cycle"},{
"/sys/class/pwm/pwmchip1/pwm1/duty_cycle", \
"/sys/class/pwm/pwmchip3/pwm1/duty_cycle", \
"/sys/class/pwm/pwmchip0/pwm1/duty_cycle"}};

#endif