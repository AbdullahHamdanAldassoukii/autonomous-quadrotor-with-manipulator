/*******************************************************************************
* simple_mmap_pwm.c
* Only works with SV1-SV4 on breakout cape
*
*
* Peter Gaskell 2016
* 
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
	simple_init_pwm(1,1000);;

	set_state(RUNNING);
	// Keep Running until state changes to EXITING
	while(get_state()!=EXITING){
		// handle other states
		if(get_state()==RUNNING){
			set_pwm_duty(0, 'A', 0.1);
			set_pwm_duty(0, 'B', 0.3);
			set_pwm_duty(1, 'A', 0.5);
			set_pwm_duty(1, 'B', 0.7);
		}
		// always sleep at some point
		usleep(100000);
	}
	
	cleanup_cape(); // exit cleanly
	simple_uninit_pwm(0);
	simple_uninit_pwm(1);
	return 0;
}