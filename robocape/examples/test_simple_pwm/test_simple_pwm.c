/*******************************************************************************
* simple_test_pwm.c
*
*	Works with SV1-SV5 on robotics breakout cape
*
* Peter Gaskell 2016
* 
*******************************************************************************/

#include "../../src/usefulincludes.h"
#include "../../src/robocape.h"
#include "../../src/simple/simple_pwm.h"
#include "../../src/mmap/mmap_pwmss.h"

int main(){
	// always initialize cape library first
	initialize_cape();
	simple_init_pwm(0,1000);
	simple_init_pwm(1,1000);
	simple_init_pwm(2,1000);

	set_state(RUNNING);
	// Keep Running until state changes to EXITING
	while(get_state()!=EXITING){
		// handle other states
		if(get_state()==RUNNING){
			simple_set_pwm_duty(0, 'A', 0.1);
			simple_set_pwm_duty(0, 'B', 0.3);
			simple_set_pwm_duty(1, 'A', 0.5);
			simple_set_pwm_duty(1, 'B', 0.7);
			simple_set_pwm_duty(2, 'A', 0.9);
		}
		// always sleep at some point
		usleep(100000);
	}
	
	cleanup_cape(); // exit cleanly
	simple_uninit_pwm(0);
	simple_uninit_pwm(1);
	simple_uninit_pwm(2);
	return 0;
}