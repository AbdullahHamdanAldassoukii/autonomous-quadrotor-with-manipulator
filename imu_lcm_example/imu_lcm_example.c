/*******************************************************************************
*
*	imu_lcm_example.c
*   Publishes IMU data to "BALANCEBOT_IMU_DATA" as an example of how to publish 
*   to LCM 
*
*	pgaskell@umich.edu
*******************************************************************************/

#include <lcm/lcm.h>

#include "../robocape/src/usefulincludes.h"
#include "../robocape/src/robocape.h"
#include "../lcmtypes/mpu9250_imu_t.h"

#define 	SAMPLE_RATE_HZ 			100	  // imu read speed
#define 	DT 						0.01  // 1/sample_rate
#define		LCM_HZ				    20

// Global Variables
imu_data_t imu_data;
mpu9250_imu_t mpu9250_imu_data;

// LCM Global Variables
lcm_t * lcm = NULL;
static const char IMU_DATA[] = "MPU9250_IMU_DATA";

// IMU interrupt routine
int read_imu();

//threads
void* lcm_publish_loop(void* ptr);


int main(int argc, char *argv[]){
	//initialize robocape
	initialize_cape();
	set_cpu_frequency(FREQ_1000MHZ);
    lcm = lcm_create(NULL);

    printf("Starting LCM Thread\n");
	//start lcm publish thread
	pthread_t  lcm_publish_thread;
	pthread_create(&lcm_publish_thread, NULL, lcm_publish_loop, (void*) NULL);

	// set up IMU configuration
	imu_config_t imu_config = get_default_imu_config();
	imu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	imu_config.orientation = ORIENTATION_Z_UP;

    printf("Starting IMU Thread\n");
	// start imu
	if(initialize_imu_dmp(&imu_data, imu_config)){
		printf("ERROR: can't talk to IMU\n");
		return -1;
	}

	//attach control routine to imu interrupt
	set_imu_interrupt_func(&read_imu);
	
	// start in the RUNNING state
	set_state(RUNNING);

	// Keep Running until state changes to EXITING
	while(get_state()!=EXITING){
		// always sleep at some point
		usleep(10000);
	}
	
	cleanup_cape(); // exit cleanly
	set_cpu_frequency(FREQ_ONDEMAND);
	return 0;
}


/*******************************************************************************
* read_imu() IMU interrupt routine to populate lcm message
* Called at SAMPLE_RATE_HZ
*******************************************************************************/
int read_imu(){

	/******************************************************************
	* STATE_ESTIMATION
	* read IMU and fill lcm message
	******************************************************************/

    mpu9250_imu_data.timestamp = micros_since_epoch();

    mpu9250_imu_data.accel[0] = imu_data.accel[0];
    mpu9250_imu_data.accel[1] = imu_data.accel[1];
    mpu9250_imu_data.accel[2] = imu_data.accel[2];

    mpu9250_imu_data.gyro[0] = imu_data.gyro[0];
    mpu9250_imu_data.gyro[1] = imu_data.gyro[1];
    mpu9250_imu_data.gyro[2] = imu_data.gyro[2];

    mpu9250_imu_data.mag[0] = imu_data.mag[0];
    mpu9250_imu_data.mag[1] = imu_data.mag[1];
    mpu9250_imu_data.mag[2] = imu_data.mag[2];

    mpu9250_imu_data.temp = imu_data.temp;

	mpu9250_imu_data.TB_angles[0] = imu_data.dmp_TaitBryan[TB_PITCH_X];
    mpu9250_imu_data.TB_angles[1] = imu_data.dmp_TaitBryan[TB_ROLL_Y];
    mpu9250_imu_data.TB_angles[2] = imu_data.dmp_TaitBryan[TB_YAW_Z];

    mpu9250_imu_data.quat[0] = imu_data.dmp_quat[0];
    mpu9250_imu_data.quat[1] = imu_data.dmp_quat[1];
    mpu9250_imu_data.quat[2] = imu_data.dmp_quat[2];
    mpu9250_imu_data.quat[3] = imu_data.dmp_quat[3];


	/*************************************************************
	* check for various exit conditions AFTER state estimate
	***************************************************************/
	if(get_state() == EXITING){
		return 0;
	}
	return 1;
}

/*******************************************************************************
* lcm publish_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*******************************************************************************/

void* lcm_publish_loop(void* ptr){
	while(get_state()!=EXITING){
		//publish lcm messages here, always publishes the latest data
        mpu9250_imu_t_publish(lcm, IMU_DATA, &mpu9250_imu_data);
		usleep(1000000 / LCM_HZ);
	}
	return NULL;
}

