/*******************************************************************************
*
*	quadrotor.c
*   Template for ROB550 FlightLab W2017
*
*	pgaskell@umich.edu
*	wzih@umich.edu
*******************************************************************************/
#define EXTERN
#include "quadrotor_main.h"

const double f = 240;     // base reference triangle side length
const double e = 61;     // end effector reference triangle side length
const double rf = 119;    // Top link length
const double re = 195;    // Bottom link length
 
// // trigonometric constants
const double pi = 3.141592653;    // PI
const double sqrt3 = 1.732051;    // sqrt(3)
const double sin120 = 0.866025;   // sqrt(3)/2
const double cos120 = -0.5;        
const double tan60 = 1.732051;    // sqrt(3)
const double sin30 = 0.5;
const double tan30 = 0.57735;     // 1/sqrt(3) 

int main(int argc, char *argv[]){
	

    //initialize robocape
	initialize_cape();
	set_cpu_frequency(FREQ_1000MHZ);


    //initialization
    initialize_parameters();

  if (argc == 2){
    //hover at switch point
    if(atof(argv[1]) == 0){//task 1: hover
          task_num = 0;
          printf("----------------------------------------\n");
          printf("Hover mode.\n");
          printf("----------------------------------------\n");
    }
    if(atof(argv[1]) == 1){//task 2
          task_num = 1;

          // final target location

          task1_final_x =1.5;
          task1_final_y=-1.5;
          task1_final_z=-1;

          printf("----------------------------------------\n");
          printf("Go-to-a-point-and-back mode.\n");
          printf("----------------------------------------\n");
    }
    if(atof(argv[1]) == 2){//task 3
          task_num = 2;
          printf("----------------------------------------\n");
          printf("One time pick-drop mode.\n");
          printf("----------------------------------------\n");
    }  
    if(atof(argv[1]) == 3){//task 4
          task_num = 3;
          printf("----------------------------------------\n");
          printf("Multi time pick-drop mode.\n");
          printf("----------------------------------------\n");
    }  
    if(atof(argv[1]) == 4){//task 5
          task_num = 4;
          printf("----------------------------------------\n");
          printf("Chase pick-drop mode.\n");
          printf("----------------------------------------\n");
    } 

    
    if(atof(argv[1]) == 7){
          task_num = 7;
          printf("----------------------------------------\n");
          printf("hand cary gripper mode.\n");
          printf("----------------------------------------\n");
    }  
        if(atof(argv[1]) == 8){
          task_num = 8;
          printf("----------------------------------------\n");
          printf("wander around mode.\n");
          printf("----------------------------------------\n");
    }
            if(atof(argv[1]) == 9){
          task_num = 9;
          printf("----------------------------------------\n");
          printf("wander around 2 two mode.\n");
          printf("----------------------------------------\n");
    }
  }



// we don't need IMU data and IMU clock here
/*    // set up IMU configuration
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
    set_imu_interrupt_func(&interrupt_func);*/

    //initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);

    //initialize LCM
    lcm = lcm_create(NULL);
    printf("Starting LCM Threads\n");
    //start lcm publish thread
    pthread_t  lcm_publish_thread;
    pthread_t  lcm_subscribe_thread;
    
    pthread_create(&lcm_publish_thread, NULL, lcm_publish_loop, (void*) NULL);
    pthread_create(&lcm_subscribe_thread, NULL, lcm_subscribe_loop, (void*) NULL);



    // start printf_thread if running from a terminal
    // if it was started as a background process then don't bother
    if(isatty(fileno(stdout))){
        printf("Starting Printf Thread\n");
        pthread_t  printf_thread;
        pthread_create(&printf_thread, NULL, printf_loop, (void*) NULL);
    }

	// start in the RUNNING state
    state.autonomous_mode = 0;
	set_state(RUNNING);

	// Keep Running until state changes to EXITING
	while(get_state()!=EXITING){
		// always sleep at some point
		usleep(10000);
	}
	  dynamixel_command_list_t_destroy(&command_list);

	cleanup_cape(); // exit cleanly
	set_cpu_frequency(FREQ_ONDEMAND);
	return 0;
}


/*******************************************************************************
* initialize_parameters() initialize all the parameters we need
* Called in the beginning of Main()
*******************************************************************************/
void initialize_parameters(){

gripper_loc_init();
time_step_flag = 0;
time_step_flag_tor = 0;
    drone_motion_state = IDLE;
test1 = 0;
test2 = 0;
test3 = 0;


gripper_pick_pos = -0.9;


count_check_for_pick = 0;
    gripper_pick_torque = 1;
    
    step_finished = 0;
    step_num = 1;
    check_waypoint_count = 0;

gripper_fb_pos = 0;
    theta_gripper[0] = 0;
	theta_gripper[1] = 0;
	theta_gripper[2] = 0;

    auto_first_time = 0;

    dist_acc_config = 0.1;
    vel_ref_config = 0.3;

    dist_acc_z_config = 0.1;
    vel_ref_z_config = 0.4;

    //this just go back to original point.
    state.Q_target_x = 0;
    state.Q_target_y = 0;
    state.Q_target_z = -1;

    //x PID, used in trajectory loop
    xPID.time_stamp = 0;
    xPID.kp = 200;                          //xPID.kp = 40;
    xPID.ki = 0;                          //xPID.ki = 0;
    xPID.kd = 50;                        //xPID.kd = 0.2 ;
    xPID.prev_err = 0;
    xPID.integral_err = 0;
    xPID.integral_limit_max = 0.01;
    xPID.integral_limit_min = -0.01;

    //y PID, used in trajectory loop
    yPID.time_stamp = 0;
    yPID.kp = 200;                        //yPID.kp = 40;
    yPID.ki = 0;                        //yPID.ki = 0;
    yPID.kd = 50;                        //yPID.kd = 0.2;
    yPID.prev_err = 0;
    yPID.integral_err = 0;
    yPID.integral_limit_max = 0.01;
    yPID.integral_limit_min = -0.01;

    //z PID, used in trajectory loop
    zPID.time_stamp = 0;
    zPID.kp = 200;                    //zPID.kp = 40;
    zPID.ki = 0;                     //zPID.ki = 0;
    zPID.kd = 50;                      //zPID.kd = 1;
    zPID.prev_err = 0;
    zPID.integral_err = 0;
    zPID.integral_limit_max = 0.01;
    zPID.integral_limit_min = -0.01;

    //yaw PID, used in trajectory loop
    yawPID.time_stamp = 0;
    yawPID.kp = 200;                     //yawPID.kp = 40;
    yawPID.ki = 0;                      //yawPID.ki = 0;
    yawPID.kd = 50;                      //yawPID.kd = 1;
    yawPID.prev_err = 0;
    yawPID.integral_err = 0;
    yawPID.integral_limit_max = 0.01;
    yawPID.integral_limit_min = -0.01;

    //used in potentiameter - ADC - beaglebone tuning process
    adc_PID.p = 0;
    adc_PID.i = 0;
    adc_PID.d = 0;
    adc_PID.p_scale = 1;
    adc_PID.i_scale = 1;
    adc_PID.d_scale = 1;

    float vel_times = 3;
    float dist_times = 1;

    filter_x = create_first_order_lowpass(DT, dist_times*SAMPLE_DELTA_T);
    filter_y = create_first_order_lowpass(DT, dist_times*SAMPLE_DELTA_T);
    filter_z = create_first_order_lowpass(DT, dist_times*SAMPLE_DELTA_T);
    
    filter_xdot = create_first_order_lowpass(DT, vel_times*SAMPLE_DELTA_T);
    filter_ydot = create_first_order_lowpass(DT, vel_times*SAMPLE_DELTA_T);
    filter_zdot = create_first_order_lowpass(DT, vel_times*SAMPLE_DELTA_T);

    filter_yaw = create_first_order_lowpass(DT, dist_times*SAMPLE_DELTA_T);
    filter_yawdot = create_first_order_lowpass(DT, vel_times*SAMPLE_DELTA_T);

    command_list.len = 4;
    command_list.commands = calloc(command_list.len, sizeof(dynamixel_command_t));
}


void check_torque(){
if(gripper_fb_torque>-0.1){
  if(clock_tm2_tor(2)){
      if(gripper_fb_torque>-0.1){
        gripper_pick_pos = -0.88;
        printf("open gripper\n");
      }
  }
}
else{
  time_step_flag_tor =0;
}
}


float theta_regulate(float theta){
    
    while ((theta<-PI_val)||(theta>PI_val)){
        if (theta < -PI_val)
            theta = theta + PI_val*2;
        if (theta > PI_val)
            theta = theta - PI_val*2;
    }
    return theta;
}

void dynamixel_handler(const lcm_recv_buf_t *rbuf, const char *channel,
              const dynamixel_status_list_t *msg, void *userdata) {
									//inverse kinematics
								//	loc_ADC_tune();
									delta_arm_grab();

    dynamixel_status_t* status = msg -> statuses;
    int32_t num_servos = msg -> len;
    // printf("\n Received motor status message: %" PRIu32 " servos \n", num_servos);

    int i;
    for (i = 0; i < num_servos; i++) {
    	if(i==3){
        	gripper_fb_torque = status->load;
          gripper_fb_pos = status->position_radians;
        }
        servo_positions[i] = status->position_radians;
        status += 1;
    }

    // test delta arm forward kinematics
    double x;
    double y;
    double z;
    delta_calcForward(servo_positions[1], servo_positions[2], servo_positions[0], &x, &y, &z);
    gripper_position[0] = -y;
    gripper_position[1] = -x;
    gripper_position[2] = -z; 
        dynamixel_cmd();

/*    if(clock_tm2(0.8)){
      printf("\nee\n");
    }*/
        //check_torque();

}
/*******************************************************************************
* loc_ADC_tune() read otentialmeter values through ADC to simulate location
*******************************************************************************/
void loc_ADC_tune(){//float *pose_temp
/*    pose_temp[0] = get_adc_volt(0);
    pose_temp[1] = get_adc_volt(1);
    pose_temp[2] = get_adc_volt(2);*/

/*    test1 = get_adc_volt(0)*2-1.7;
    test2 = get_adc_volt(1)*2-1.7;
    test3 = -get_adc_volt(2)*2+1.7;*/

  // gripper_x_mm = (get_adc_volt(0)*2-1.7)*40;
  //  gripper_y_mm = (get_adc_volt(1)*2-1.7)*40;
  //  gripper_z_mm = 150;
  //gripper_z_mm = 200;
	//gripper_z_mm = 150;
   // gripper_pick_pos = -get_adc_volt(2)/1+0.4;

}

int clock_tm2(float value){
          if(time_step_flag==0){
          count_time_stamp = micros_since_epoch();
          time_step_flag = 1;
        }
        if(time_step_flag==1){
          if((micros_since_epoch()-count_time_stamp)>value*1000000){
            time_step_flag=0;
            return 1;
          }
        }
        return 0;
}


int clock_tm2_tor(float value){
          if(time_step_flag_tor==0){
          count_time_stamp_tor = micros_since_epoch();
          time_step_flag_tor = 1;
        }
        if(time_step_flag_tor==1){
          if((micros_since_epoch()-count_time_stamp_tor)>value*1000000){
            time_step_flag_tor=0;
            return 1;
          }
        }
        return 0;
}



void dynamixel_cmd(){
    dynamixel_command_t command;
    for (int j = 0; j < 3; j++) {//num_servos-1

      //command.position_radians = pi/8;
      command.position_radians = theta_gripper[j];
      command.utime = micros_since_epoch();
      command.speed = 1;
      command.max_torque = 1;

      command_list.commands[j] = command;

    }
      command.position_radians = gripper_pick_pos;//global
      command.utime = micros_since_epoch();
      command.speed = 1;
      command.max_torque = gripper_pick_torque;//global

      command_list.commands[3] = command;
      delta_arm_trim();
      dynamixel_command_list_t_publish (lcm, "ARM_COMMAND", &command_list);



}

void delta_arm_trim(){
  if(gripper_x_mm<-40){
    gripper_x_mm = -40;
  }
    if(gripper_x_mm>40){
    gripper_x_mm = 40;
  }
    if(gripper_y_mm<-40){
    gripper_y_mm = -40;
  }
    if(gripper_y_mm>40){
    gripper_y_mm = 40;
  }
    if(gripper_z_mm<65){
    gripper_z_mm = 65;
  }
    if(gripper_z_mm>260){
    gripper_z_mm = 260;
  }
}

void delta_arm_grab(){
    delta_calcInverse(-gripper_y_mm, -gripper_x_mm, -gripper_z_mm, &theta_gripper[1], &theta_gripper[2], &theta_gripper[0]);
}

/*******************************************************************************
* interrupt_func() IMU interrupt routine to state variables
*******************************************************************************/
int interrupt_func(){
    //USB2arduino();
    //tune_PID(&pathPID);
  //  read_imu();
    //test_value();
    return 1;
}


/*******************************************************************************
* read_imu() IMU interrupt routine to state variables
* Called at SAMPLE_RATE_HZ
*******************************************************************************/
int read_imu(){

	/******************************************************************
	* STATE_ESTIMATION
	* read IMU and fill lcm message
	******************************************************************/
    pthread_mutex_lock(&state_mutex);
    state.IMU_timestamp = micros_since_epoch();
	  state.IMU_pitch = imu_data.dmp_TaitBryan[TB_PITCH_X];
    state.IMU_roll = imu_data.dmp_TaitBryan[TB_ROLL_Y];
    state.IMU_yaw = imu_data.dmp_TaitBryan[TB_YAW_Z];
    pthread_mutex_unlock(&state_mutex);
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
		//publish you own lcm messages here
        //use this to record what you want in lcm logger
        //always publishes the latest data
		
		usleep(1000000 / LCM_PUB_HZ);
	}
	return NULL;
}


/*******************************************************************************
* lcm subscribe_loop() 
*
* subscribe to all handler functions
* run lcm_handle_timeout continously to handle income lcm messages
* 
*******************************************************************************/
void *lcm_subscribe_loop(void *data){
    //Set data frequency of blocks to 100Hz
    cfg_data_frequency_t cfg_data_frequency;
    cfg_data_frequency.hz = (uint8_t) 100;
    cfg_data_frequency_t_publish(lcm, "CFG_DATA_FREQUENCY_1_TX", &cfg_data_frequency);
    cfg_data_frequency_t_publish(lcm, "CFG_DATA_FREQUENCY_1_RX", &cfg_data_frequency);
    //Subscribe to lcm channels here
    channels_t_subscribe(lcm, BLOCKS_RX_CHANNEL, channels_handler, lcm);
    pose_list_t_subscribe(lcm, OPTI_CHANNEL, optitrack_handler, lcm);

    dynamixel_status_list_t_subscribe (lcm, "ARM_STATUS", dynamixel_handler, lcm);
    
    while(1){
      lcm_handle_timeout(lcm, 1);
      usleep(1000000 / LCM_HZ);
    }   
    lcm_destroy(lcm);
    return 0;
}


/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*******************************************************************************/
void* printf_loop(void* ptr){
    //channels_t msg ; 
    state_t last_state, new_state; // keep track of last state 
    while(get_state()!=EXITING){
        new_state = get_state();
        // check if this is the first time since being paused
        if(new_state==RUNNING && last_state!=RUNNING){
            printf("\nRUNNING\n");
            printf("\n       |");

            printf("       OPTITRACK       |");
            printf("          IMU          |");
            printf("        BLOCKS          |");
            printf("\n       |");
            printf("   X   |");
            printf("   Y   |");
            printf("   Z   |");
            printf("  ROLL |");
            printf(" PITCH |");
            printf("  YAW  |");
            printf("THST|");
            printf("ROLL|");
            printf("PTCH|");
            printf("YAW |");
            printf("AUTO|");
            printf("count|");

            printf("\n");
        }
        else if(new_state==PAUSED && last_state!=PAUSED){
            printf("\nPAUSED\n");
        }
        last_state = new_state;
        
        if(new_state == RUNNING){   
            printf("\r");
            if(state.autonomous_mode == 1){
                printf(" AUTO  |");
            }
            else{
                printf("MANUAL |");
            }
            //printf("qq %s",new_state);
            //Add Print stattements here, do not follow with /n
                pthread_mutex_lock(&state_mutex);
            printf("x:%6.4f |", state.Q_xpos);
            printf("%6.4f |", state.Q_ypos);
            printf("%6.4f |", state.Q_zpos);

/*
           printf("%d|",state.RC_cmds[0]);
           printf("%d|",state.RC_cmds[1]);
           printf("%d|",state.RC_cmds[2]);
           printf("%d|",state.RC_cmds[3]);*/
           
           //location
           printf("PU x:%6.2f|",state.Q_PU_xpos);//gripper_x_mm
           printf("y:%6.2f|",state.Q_PU_xpos);
           printf("z:%6.2f|",state.Q_PU_xpos);
                      printf("PU x:%6.2f|",state.Q_DO_xpos);//gripper_x_mm
           printf("y:%6.2f|",state.Q_DO_xpos);
           printf("z:%6.2f|",state.Q_DO_xpos);

           
/*           printf("the x:%6.2f|",theta_gripper[0]);//gripper_x_mm
           printf("y:%6.2f|",theta_gripper[1]);
           printf("z:%6.2f|",theta_gripper[2]);*/

        /*   printf("gri cm x:%6.2f|",gripper_x_mm);//gripper_x_mm
           printf("y:%6.2f|",gripper_y_mm);
           printf("z:%6.4f|",gripper_z_mm);*/


           printf("pos:%6.2f|",gripper_pick_pos);
           printf("pos_fb:%6.2f|",gripper_fb_pos);
           printf("tor_fb:%6.2f|",gripper_fb_torque);

           //  printf("swi%d|",state.RC_cmds[7]);
           // printf("%d|",check_waypoint_count);
        //  printf("Gri pos: %f | %f | %f ", gripper_position[0], gripper_position[1], gripper_position[2]);
            // printf("Servo positions: %0.2f | %0.2f | %0.2f | %0.2f", servo_positions[0], servo_positions[1], servo_positions[2], servo_positions[3]);
    pthread_mutex_unlock(&state_mutex);
            fflush(stdout);
        }
        usleep(1000000 / PRINTF_HZ);
    }
    return NULL;
} 


//forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
//returned status: 0=OK, -1=non-existing position
int delta_calcForward(double theta1, double theta2, double theta3, double *x0, double *y0, double *z0) 
{

  double t = (f-e)*tan30/2;
 
  double y1 = -(t + rf*cos(theta1));
  double z1 = -rf*sin(theta1);
 
  double y2 = (t + rf*cos(theta2))*sin30;
  double x2 = y2*tan60;
  double z2 = -rf*sin(theta2);
 
  double y3 = (t + rf*cos(theta3))*sin30;
  double x3 = -y3*tan60;
  double z3 = -rf*sin(theta3);
 
  double dnm = (y2-y1)*x3-(y3-y1)*x2;
 
  double w1 = y1*y1 + z1*z1;
  double w2 = x2*x2 + y2*y2 + z2*z2;
  double w3 = x3*x3 + y3*y3 + z3*z3;
     
  // x = (a1*z + b1)/dnm
  double a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
  double b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
 
  // y = (a2*z + b2)/dnm;
  double a2 = -(z2-z1)*x3+(z3-z1)*x2;
  double b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
 
  // a*z^2 + b*z + c = 0
  double a = a1*a1 + a2*a2 + dnm*dnm;
  double b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
  double c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);

  
  // discriminant
  double d = b*b - (double)4.0*a*c;
  if (d < 0) return -1; // non-existing point
 
  *z0 = -(double)0.5*(b+sqrt(d))/a;
  *x0 = (a1*(*z0) + b1)/dnm;
  *y0 = (a2*(*z0) + b2)/dnm;

  return 0;
}
 
// inverse kinematics
// helper functions, calculates angle theta1 (for YZ-plane)
int delta_calcAngleYZ(double x0, double y0, double z0, double *theta) {
  double y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
  y0 -= 0.5 * 0.57735    * e;    // shift center to edge
  double a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
  double b = (y1-y0)/z0;
  // discriminant
  double d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
  if (d < 0) {
    printf("Non-existing point \n");
    return -1; // non-existing point
  }
  double yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
  double zj = a + b*yj;

  *theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
  *theta = (*theta)*pi/(double)180.0; 
  return 0;
}
 
// inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
// returned status: 0=OK, -1=non-existing position
int delta_calcInverse(double x0, double y0, double z0, double *theta1, double *theta2, double *theta3) {
  *theta1 = *theta2 = *theta3 = 0;
  int status = delta_calcAngleYZ(x0, y0, z0, theta1);
  if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
  if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
  return status;
}
