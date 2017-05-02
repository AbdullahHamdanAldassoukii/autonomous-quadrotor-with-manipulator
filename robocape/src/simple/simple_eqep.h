// C Library for TI userspace eQEP

#ifndef SIMPLEEQEP_H_
#define SIMPLEEQEP_H_

int simple_init_eqep(int eQEPnum);
int simple_read_eqep(int eQEPnum);
int simple_write_eqep(int eQEPnum, int position);
int simple_set_period_eqep(int eQEPnum, uint64_t period);
int simple_set_mode_eqep(int eQEPnum, int mode);
uint64_t simple_get_period_eqep(int eQEPnum);
int simple_get_mode_eqep(int eQEPnum);

#endif