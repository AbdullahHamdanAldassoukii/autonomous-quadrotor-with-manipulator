/*******************************************************************************
* test_motor.c
*******************************************************************************/

#include "../../src/devices/motor.h"

int main(){
	if(initialize_cape()<0){
		printf("ERROR: failed to initialize cape");
	}
	
	motor_t left_motor = init_motor(0, 'A', 66, 67);
	motor_t right_motor = init_motor(0, 'B', 68, 69);
	set_motor_on(left_motor);
	set_motor_on(right_motor);
	
	set_motor_speed(&left_motor, 0.8);
	set_motor_speed(&right_motor, 0.1);
	usleep(2000000);
	set_motor_speed(&left_motor, 0);
	set_motor_speed(&right_motor, 0);
	usleep(2000000);
	set_motor_speed(&left_motor, -0.1);
	set_motor_speed(&right_motor, -0.8);
	usleep(2000000);
	set_motor_off(left_motor);
	set_motor_off(right_motor);
	usleep(2000000);
	set_motor_speed(&left_motor, 0);
	set_motor_speed(&right_motor, 0);
	
	//set_motor_speed(right_motor, 0.5);
	
	while(get_state() != EXITING){
		usleep(1000000);
	}
	
	cleanup_cape();
	uninit_motor(left_motor);
	uninit_motor(right_motor);
	return 0;
}

