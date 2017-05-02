#define EXTERN  // Needed for global data declarations in bbb.h
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <lcm/lcm.h>

#include "io/util.h"
#include "../lcmtypes/channels_t.h"

static int verbose = 0;
int16_t rxdata[8];
int16_t txdata[8];
double map(double input, double iMin, double iMax, double oMin, double oMax);
void handler_channels_lcm (const lcm_recv_buf_t *rbuf, const char *channel, const channels_t *msg, void *userdata);
lcm_t *lcm;


int main() 
{
    lcm = lcm_create(NULL);
    channels_t tx_msg;
    channels_t_subscribe(lcm, "CHANNELS_.*_RX", handler_channels_lcm, rxdata);
    
    //init tx msg
	tx_msg.utime = utime_now();
    tx_msg.num_channels = 8;
    tx_msg.channels = (int16_t*) malloc(sizeof(int16_t) * tx_msg.num_channels);
    
    for(;;) {
        lcm_handle(lcm);
	
	//do transformation of incoming data here
	for(int i=0; i<8; i++){
		txdata[i] = rxdata[i]-500;
	}
    	
	//make tx msg
	tx_msg.utime = utime_now();
	for(int i=0; i < tx_msg.num_channels; i++){
		tx_msg.channels[i] = txdata[i];
	}
	//pblish transmint msg
    	channels_t_publish(lcm, "CHANNELS_1_TX", &tx_msg);

    }
    lcm_destroy(lcm);
    free(tx_msg.channels);
    return 0;
}

void handler_channels_lcm (const lcm_recv_buf_t *rbuf, const char *channel, const channels_t *msg, void *userdata)
{
    if(verbose){
        printf("Received message on channel %s, timestamp %" PRId64 "\n",channel, msg->utime);
        printf("we read %d channels\n", msg->num_channels);
    }
    for(int i=0; i<(msg->num_channels); i++){
	if(verbose){printf("CH %d:\t%d", i, msg->channels[i]);}
        rxdata[i] = msg->channels[i];
    }
}


double map(double x, double in_min, double in_max,
            double out_min, double out_max)
{   
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;   
}




