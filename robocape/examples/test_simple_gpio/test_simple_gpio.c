/*******************************************************************************
* bare_minimum.c
*
* James Strawson 2016
* This is meant to be a skeleton program for robotics cape projects. 
*******************************************************************************/

#include "../../src/usefulincludes.h"
#include "../../src/robocape.h"
#include "../../src/simple/simple_gpio.h"

int main(int argc, char *argv[]){
	// always initialize cape library first
	int gpio_pin = 69;
	if(argc > 1){
		 gpio_pin = atoi(argv[1]);
	}
	// always initialize cape library first
	initialize_cape();
	gpio_export(gpio_pin);
	gpio_set_dir(gpio_pin, OUTPUT_PIN);


	set_state(RUNNING);
	// Keep Running until state changes to EXITING
	while(get_state()!=EXITING){
		// handle other states
		if(get_state()==RUNNING){
			if(!gpio_get_value(gpio_pin)){
				gpio_set_value(gpio_pin, 1);
			}
			else {
				gpio_set_value(gpio_pin, 0);
			}
		}
		// always sleep at some point
		usleep(10000);
	}
	
	gpio_unexport(gpio_pin);
	cleanup_cape(); // exit cleanly

	return 0;
}
