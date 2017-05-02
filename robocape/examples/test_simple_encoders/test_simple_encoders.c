/*******************************************************************************
* test_encoders.c
*
* James Strawson 2016
* Prints out current encoder ticks for 2channels
* channels 1-3 are counted using eQEP 0-2
*******************************************************************************/

#include "../../src/usefulincludes.h"
#include "../../src/robocape.h"


int main(){
	int i;

	if(initialize_cape()<0){
		printf("ERROR: failed to initialize cape");
	}

	printf("\nRaw encoder positions\n");
	printf("   E1   |");
	printf("   E2   |");
	printf(" \n");
	while(get_state() != EXITING){
		printf("\r");
		for(i=1;i<=2;i++){
			printf("%6d  |", simple_get_encoder_pos(i));
		}			
		fflush(stdout);
		usleep(50000);
	}
	
	cleanup_cape();
	return 0;
}

