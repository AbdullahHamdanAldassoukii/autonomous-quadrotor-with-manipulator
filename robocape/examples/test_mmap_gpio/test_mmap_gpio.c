/*******************************************************************************
* test_mmap_gpio.c
*
* Peter Gaskell 2016
*  
*******************************************************************************/

#include "../../src/usefulincludes.h"
#include "../../src/robocape.h"
#include "../../src/mmap/mmap_gpio_adc.h"
#include "../../src/simple/simple_gpio.h"

int main(int argc, char *argv[]){
	// always initialize cape library first
	int gpio_pin = 69;
	if(argc > 1){
		 gpio_pin = atoi(argv[1]);
	}
	
	int i;
	initialize_cape();
	initialize_mmap_gpio();

	gpio_export(gpio_pin);
	gpio_set_dir(gpio_pin, OUTPUT_PIN);

	set_state(RUNNING);
	// Keep Running until state changes to EXITING
	while(get_state()!=EXITING){
		// handle other states
		if(get_state()==RUNNING){

			for(i = 0; i < 100; i++){
				if(!mmap_gpio_read(gpio_pin)){
					mmap_gpio_write(gpio_pin, 1);
				}
				else {
					mmap_gpio_write(gpio_pin, 0);
				}
			}
		}
		// always sleep at some point
		usleep(100);
	}
	
	gpio_unexport(gpio_pin);
	cleanup_cape(); // exit cleanly

	return 0;
}
