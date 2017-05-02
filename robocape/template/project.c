/*******************************************************************************
* project_template.c
*
* This is meant to be a skeleton program for robotics cape projects. 
* Change this description and file name 
*******************************************************************************/

#include "../src/usefulincludes.h"
#include "../src/robocape.h"

/*******************************************************************************
* int main() 
*	
* This template main function contains these critical components
* - call to initialize_cape
* - main while loop that checks for EXITING condition
* - cleanup_cape() at the end
*******************************************************************************/
int main(){
	// always initialize cape library first
	initialize_cape();

	// do your own initialization here
	printf("\nHello BeagleBone\n");

	// done initializing so set state to RUNNING
	set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(get_state()!=EXITING){
		// handle other states
		if(get_state()==RUNNING){
			// do things
		}
		else if(get_state()==PAUSED){
			// do other things
		}
		// always sleep at some point
		usleep(100000);
	}
	
	// exit cleanly
	cleanup_cape(); 
	return 0;
}