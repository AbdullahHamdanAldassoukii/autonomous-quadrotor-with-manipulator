/*******************************************************************************
* test_adc.c
*
* James Strawson 2016
* prints voltages read by all adc channels
*******************************************************************************/

#include "../../src/usefulincludes.h"
#include "../../src/robocape.h"

int main(){
	int i;
		
	initialize_cape();
	
	printf(" adc_0 |");
	printf(" adc_1 |");
	printf(" adc_2 |");
	printf(" adc_3 |");
	printf(" adc_4 |");
	printf(" adc_5 |");
	printf(" adc_6 |");
	
	printf("\n");
	

	while(get_state()!=EXITING){
		printf("\r");
		//print all channels
		for(i=0;i<7;i++){
			printf("%6.2f |", get_adc_volt(i));
		}
		fflush(stdout);
		usleep(100000);
	}
	cleanup_cape();
	return 0;
}