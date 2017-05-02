/*******************************************************************************
* robocape.c
* 
* This is one of many c-files that are used to build libroboticscape.so
* however it contains the majority of the core components.
*******************************************************************************/

#define DEBUG

#include "usefulincludes.h"
#include "robocape.h"

#include "mmap/mmap_gpio_adc.h"     // used for fast gpio and adc functions
#include "mmap/mmap_pwmss.h"        // used for fast pwm and eQEP functions

#include "simple/simple_gpio.h"// used for setting interrupt input pin
#include "simple/simple_pwm.h"  // for configuring pwm
#include "simple/simple_eqep.h"     // for configuring eQEP


// defines
#define CAPE_NAME 	"RoboCape"

/*******************************************************************************
* Global Variables
*******************************************************************************/
enum state_t rc_state = UNINITIALIZED;



/*******************************************************************************
* local function declarations
*******************************************************************************/
int is_cape_loaded();
void shutdown_signal_handler(int signo);

/*******************************************************************************
* local thread function declarations
*******************************************************************************/
void* example_handler(void* ptr);
;

/*******************************************************************************
* local thread structs
*******************************************************************************/
pthread_t example_thread;


/*******************************************************************************
* int initialize_cape()
* sets up necessary hardware and software
* should be the first thing your program calls
*******************************************************************************/
int initialize_cape(){
	FILE *fd; 
	//printf("test change\n");
	// check if another project was using resources
	// kill that process cleanly with sigint if so
	#ifdef DEBUG
		printf("checking for existing PID_FILE\n");
	#endif
	kill_robot();
	
	// Start Signal Handler
	#ifdef DEBUG
	printf("Initializing exit signal handler\n");
	#endif
	signal(SIGINT, shutdown_signal_handler);	
	signal(SIGTERM, shutdown_signal_handler);	

	// initialize mmap io libs
	#ifdef DEBUG
	printf("Initializing: GPIO\n");
	#endif

	if(initialize_mmap_gpio()){
		printf("mmap_gpio_adc.c failed to initialize gpio\n");
		return -1;
	}
	
	#ifdef DEBUG
	printf("Initializing: ADC\n");
	#endif
	if(initialize_mmap_adc()){
		printf("mmap_gpio_adc.c failed to initialize adc\n");
		return -1;
	}

	#ifdef DEBUG
	printf("Initializing: eQEP\n");
	#endif
//	if(init_eqep(0)){
//		printf("ERROR: failed to initialize eQEP0\n");
//		// return -1;
//	}
	if(init_eqep(1)){
		printf("ERROR: failed to initialize eQEP1\n");
		// return -1;
	}
	if(init_eqep(2)){
		printf("ERROR: failed to initialize eQEP2\n");
		// return -1;
	}
	
	// setup pwm driver
	#ifdef DEBUG
	printf("Initializing: PWM\n");
	#endif
	if(simple_init_pwm(1,PWM_FREQ)){
		printf("ERROR: failed to initialize hrpwm1\n");
//		return -1;
	}
	if(simple_init_pwm(2,PWM_FREQ)){
		printf("ERROR: failed to initialize PWMSS 2\n");
		// return -1;
	}
	
	// start some gpio pins at defaults
	
	// create new pid file with process id
	#ifdef DEBUG
		printf("opening PID_FILE\n");
	#endif
	fd = fopen(PID_FILE, "ab+");
	if (fd < 0) {
		printf("\n error opening PID_FILE for writing\n");
		return -1;
	}
	pid_t current_pid = getpid();
	fprintf(fd,"%d",(int)current_pid);
	fflush(fd);
	fclose(fd);

	// Print current PID
	#ifdef DEBUG
	printf("Process ID: %d\n", (int)current_pid); 
 	#endif

	// all done
	set_state(PAUSED);

	return 0;
}

/*******************************************************************************
*	int cleanup_cape()
*	shuts down library and hardware functions cleanly
*	you should call this before your main() function returns
*******************************************************************************/
int cleanup_cape(){
	// just in case the user forgot, set state to exiting
	set_state(EXITING);
	
	// announce we are starting cleanup process
	printf("\nExiting Cleanly\n");
	
	//allow up to 3 seconds for thread cleanup
	struct timespec thread_timeout;
	clock_gettime(CLOCK_REALTIME, &thread_timeout);
	thread_timeout.tv_sec += 3;
	
	#ifdef DEBUG
	printf("deleting PID file\n");
	#endif
	FILE* fd;
	// clean up the pid_file if it still exists
	fd = fopen(PID_FILE, "r");
	if (fd != NULL) {
		// close and delete the old file
		fclose(fd);
		remove(PID_FILE);
	}
	return 0;
}


/*******************************************************************************
* @ state_t get_state()
*
* returns the high-level robot state variable
* use this for managing how your threads start and stop
*******************************************************************************/
state_t get_state(){
	return rc_state;
}


/*******************************************************************************
* @ int set_state(state_t new_state)
*
* sets the high-level robot state variable
* use this for managing how your threads start and stop
*******************************************************************************/
int set_state(state_t new_state){
	rc_state = new_state;
	return 0;
}

/*******************************************************************************
* @ int print_state()
* 
* Prints the textual name of the state to the screen.
*******************************************************************************/
int print_state(){
	switch(rc_state){
	case UNINITIALIZED:
		printf("UNINITIALIZED");
		break;
	case PAUSED:
		printf("PAUSED");
		break;
	case RUNNING:
		printf("RUNNING");
		break;
	case EXITING:
		printf("EXITING");
		break;
	default:
		printf("ERROR: invalid state\n");
		return -1;
	}
	return 0;
}


/*******************************************************************************
* int get_encoder_pos(int ch)
* 
* returns the encoder counter position
*******************************************************************************/
int get_encoder_pos(int ch){
	if(ch<1 || ch>2){
		printf("Encoder Channel must be from 1 or 2\n");
		return -1;
	}
	// first 3 channels counted by eQEP
	return  read_eqep(ch);
}



/*******************************************************************************
* int set_encoder_pos(int ch, int val)
* 
* sets the encoder counter position
*******************************************************************************/
int set_encoder_pos(int ch, int val){
	if(ch<1 || ch>2){
		printf("Encoder Channel must be from 1 or 2\n");
		return -1;
	}
	// else write to eQEP
	return write_eqep(ch, val);
}


/*******************************************************************************
* int simple_get_encoder_pos(int ch)
* 
* returns the encoder counter position using file i/o
*******************************************************************************/
int simple_get_encoder_pos(int ch){
	if(ch<1 || ch>2){
		printf("Encoder Channel must be from 1 or 2\n");
		return -1;
	}
	// first 3 channels counted by eQEP
	return  simple_read_eqep(ch);
}

/*******************************************************************************
* int set_encoder_pos(int ch, int val)
* 
* sets the encoder counter position using file i/o
*******************************************************************************/
int simple_set_encoder_pos(int ch, int val){
	if(ch<1 || ch>2){
		printf("Encoder Channel must be from 1 or 2\n");
		return -1;
	}
	// else write to eQEP
	return simple_write_eqep(ch, val);
}

/*******************************************************************************
* int get_adc_raw(int ch)
*
* returns the raw adc reading
*******************************************************************************/
int get_adc_raw(int ch){
	if(ch<0 || ch>6){
		printf("analog pin must be in 0-6\n");
		return -1;
	}
	return mmap_adc_read_raw((uint8_t)ch);
}

/*******************************************************************************
* float get_adc_volt(int ch)
* 
* returns an actual voltage for an adc channel
*******************************************************************************/
float get_adc_volt(int ch){
	if(ch<0 || ch>6){
		printf("analog pin must be in 0-6\n");
		return -1;
	}
	int raw_adc = mmap_adc_read_raw((uint8_t)ch);
	return raw_adc * 1.8 / 4095.0;
}


/*******************************************************************************
* shutdown_signal_handler(int signo)
*
* catch Ctrl-C signal and change system state to EXITING
* all threads should watch for get_state()==EXITING and shut down cleanly
*******************************************************************************/
void shutdown_signal_handler(int signo){
	if (signo == SIGINT){
		set_state(EXITING);
		printf("\nreceived SIGINT Ctrl-C\n");
 	}else if (signo == SIGTERM){
		set_state(EXITING);
		printf("\nreceived SIGTERM\n");
 	}
}


/*******************************************************************************
*	is_cape_loaded()
*
*	check to make sure robotics cape overlay is loaded
*	return 1 if cape is loaded
*	return -1 if cape_mgr is missing
* 	return 0 if mape_mgr is present but cape is missing
*******************************************************************************/
int is_cape_loaded(){
	int ret;
	
	if(system("ls /sys/devices/platform/ | grep -q \"bone_capemgr\"")==0){
		#ifdef DEBUG
		printf("checking /sys/devices/platform/bone_capemgr*/slots\n");
		#endif
		ret = system("grep -q "CAPE_NAME" /sys/devices/platform/bone_capemgr*/slots");
	}
	else{
		printf("Cannot find bone_capemgr*/slots\n");
		return -1;
	}
	
	if(ret == 0){
		#ifdef DEBUG
		printf("Cape Loaded\n");
		#endif
		return 1;
	} 
	
	#ifdef DEBUG
	printf("Cape NOT Loaded\n");
	printf("grep returned %d\n", ret);
	#endif
	
	return 0;
}


/*******************************************************************************
* @ int kill_robot()
*
* This function is used by initialize_cape to make sure any existing program
* using the robotics cape lib is stopped. The user doesn't need to integrate
* this in their own program as initialize_cape calls it. However, the user may
* call the kill_robot example program from the command line to close whatever
* program is running in the background.
*
* return values: 
* -2 : invalid contents in PID_FILE
* -1 : existing project failed to close cleanly and had to be killed
*  0 : No existing program is running
*  1 : An existing program was running but it shut down cleanly.
*******************************************************************************/
int kill_robot(){
	FILE* fd;
	int old_pid, i;

	// start by checking if a pid file exists
	if(access(PID_FILE, F_OK ) != 0){
		// PID file missing
		return 0;
	}

	// attempt to open PID file
	// if the file didn't open, no project is runnning in the background
	// so return 0
	fd = fopen(PID_FILE, "r");
	if (fd == NULL) return 0;
	
	// try to read the current process ID
	fscanf(fd,"%d", &old_pid);
	fclose(fd);
	
	// if the file didn't contain a PID number, remove it and 
	// return -1 indicating weird behavior
	if(old_pid == 0){
		remove(PID_FILE);
		return -2;
	}

	// check if it's our own pid, if so return 0
	if(old_pid == (int)getpid()) return 0;
	
	// now see if the process for the read pid is still running
	if(getpgid(old_pid) < 0){
		// process not running, remove the pid file
		remove(PID_FILE);
		return 0;
	}

	// process must be running, attempt a clean shutdown
	kill((pid_t)old_pid, SIGINT);
	
	// check every 0.1 seconds to see if it closed 
	for(i=0; i<30; i++){
		if(getpgid(old_pid) >= 0) usleep(100000);
		else{ // succcess, it shut down properly
			remove(PID_FILE);
			return 1; 
		}
	}
	
	// otherwise force kill the program if the PID file never got cleaned up
	kill((pid_t)old_pid, SIGKILL);
	usleep(500000);

	// delete the old PID file if it was left over
	remove(PID_FILE);

	// return -1 indicating the program had to be killed
	return -1;
}


