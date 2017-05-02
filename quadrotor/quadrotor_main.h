#include <lcm/lcm.h>

#include "../robocape/src/usefulincludes.h"
#include "../robocape/src/robocape.h"
#include "../lcmtypes/pose_xyzrpy_t.h"
#include "../lcmtypes/pose_list_t.h"
#include "../lcmtypes/channels_t.h"
#include "../lcmtypes/cfg_data_frequency_t.h"
//TODO: include other lcm types as needed
#include "../lcmtypes/dynamixel_status_list_t.h"
#include "../lcmtypes/dynamixel_status_t.h"
#include "../lcmtypes/dynamixel_command_list_t.h"
#include "../lcmtypes/dynamixel_command_t.h"


#define     SAMPLE_RATE_HZ          100   // imu read speed
#define     DT                      0.02  // 1/sample_rate
#define     LCM_HZ                  1000  // check for lcm messages
#define     LCM_PUB_HZ              30    // publish user messages
#define     PRINTF_HZ                10    // print to terminal
#define     PI_val                  3.1415926 //pi
#define     SAMPLE_DELTA_T          0.02

static const char BLOCKS_RX_CHANNEL[] = "CHANNELS_1_RX";
static const char BLOCKS_TX_CHANNEL[] = "CHANNELS_1_TX";
static const char OPTI_CHANNEL[] = "QUADROTOR_POSE_CHANNEL";

typedef struct current_state_t{

/*******************************************************************************
* updated in  optitrack_handler()
*******************************************************************************/
    float Opti_dt;          //seconds from last message
    uint64_t Q_timestamp;   //timestamp of optitrack message
    uint64_t channel_timestamp; //timestamp for channnel

    float Q_xpos;           // X position of Quadrotor
    float Q_ypos;           // Y position of Quadrotor
    float Q_zpos;           // Z position of Quadrotor

    float Q_xdot;           // X velocity of Quadrotor
    float Q_ydot;           // Y velocity of Quadrotor
    float Q_zdot;           // Z velocity of Quadrotor

    float Q_roll;           // roll of Quadrotor
    float Q_pitch;          // pitch of Quadrotor

    float Q_yaw;            // yaw of Quadrotor
    float Q_yawdot;         // rotational velocity of Quadrotor
/*******************************************************************************
* updated in  channels_handler()
*******************************************************************************/
    int RC_cmds[8];         //RC stick commands, only use 0~3 and 7 in this project
    int autonomous_mode;    //Stores current mode, 1 for autonomous


/*******************************************************************************
* updated in  quadrotor_main(), not neccessary ????
*******************************************************************************/
    uint64_t IMU_timestamp; //IMU Data
    float IMU_roll;
    float IMU_pitch;
    float IMU_yaw;
/*******************************************************************************
* PU and DO pick and drop location
*******************************************************************************/
    float Q_PU_xpos;           // X position of Quadrotor pick
    float Q_PU_ypos;           // Y position of Quadrotor pick
    float Q_PU_zpos;           // Z position of Quadrotor pick

    float Q_DO_xpos;           // X position of Quadrotor drop
    float Q_DO_ypos;           // Y position of Quadrotor drop
    float Q_DO_zpos;           // Z position of Quadrotor drop

    float Q_target_x;
    float Q_target_y;
    float Q_target_z;

 /*******************************************************************************
* updated DIY
*******************************************************************************/   
/*        float x1;
            int x2;
                int x3;
                    long double x4;
                    long double x5;
                    long double x6;*/
    //TODO: Add other state variables as needed
} current_state_t;

//new defined in autonomous mode
typedef enum drone_motion_state_t {
    IDLE,
    WAYPOINT,
    PICK,
    DROP,
} drone_motion_state_t;



typedef struct PID//this is for PID
{
    uint64_t time_stamp;
    float kp;
    float ki;
    float kd;
    float prev_err;
    float integral_err; //for I: accu = accu + ki*err*DT
    float integral_limit_max;
    float integral_limit_min;

} PID;

// potentiameter used to tune the PID parameters
typedef struct ADC_val//this is for PID reading from ADC port
{
    float p;
    float i;
    float d;
    float p_scale;
    float i_scale;
    float d_scale;
}ADC_val;

// Global data
// #define EXTERN in your main() .c file
// #define EXTERN extern in all other .c files
// Global Variables
EXTERN imu_data_t imu_data;
// LCM Global Variables
EXTERN lcm_t * lcm;
EXTERN pthread_mutex_t state_mutex;
EXTERN current_state_t state;

EXTERN drone_motion_state_t drone_motion_state;

//x,y
EXTERN float dist_acc_config;
EXTERN float vel_ref_config;
//z
EXTERN float dist_acc_z_config;
EXTERN float vel_ref_z_config;

//for task 1: Go-to-a-point-and-back mode.
EXTERN float task1_origin_x;
EXTERN float task1_origin_y;
EXTERN float task1_origin_z;
EXTERN float gripper_pick_pos;
EXTERN float gripper_pick_torque;
EXTERN float gripper_fb_torque;
EXTERN double theta_gripper[3];
EXTERN double gripper_fb_pos;

EXTERN double gripper_x_mm;
EXTERN double gripper_y_mm;
EXTERN double gripper_z_mm;

EXTERN int check_waypoint_count;
EXTERN int auto_first_time;
EXTERN int step_finished;
EXTERN int task_num;
EXTERN int step_num;
EXTERN int count_check_for_pick;

EXTERN float task1_final_x;
EXTERN float task1_final_y;
EXTERN float task1_final_z;

// need to tune
EXTERN PID turnPID;
EXTERN PID xPID;// used for z path
EXTERN PID yPID;// used for z path
EXTERN PID zPID;// used for z path
EXTERN PID yawPID;// used for z path
EXTERN PID pathPID;// used for x,y path
EXTERN PID velPID;
EXTERN ADC_val adc_PID;

EXTERN enum quad_states{
    holdWithoutBlock,
    holdWithBlock,
    transitWithoutBlock,
    transitWithBlock,
    pickBlock,
    placeBlock,

} quad_state ; 

EXTERN enum events{
    autoON,
    moveToPick, 
    waypointReached,
    attemptPicking, 
    pickSuccessful, 
    attemptPlace,
    placeSuccessful,
    placeWaypoint
}quad_events;


EXTERN uint64_t count_time_stamp;
EXTERN int time_step_flag;
EXTERN uint64_t count_time_stamp_tor;
EXTERN int time_step_flag_tor;
//ERIC
double servo_positions[4];
double gripper_position[3];
// dynamixel_command_t dynam_commands;
dynamixel_command_list_t command_list;

// IMU interrupt routine
int read_imu();



EXTERN d_filter_t filter_x, filter_y, filter_z, filter_yaw, filter_xdot, filter_ydot, filter_zdot, filter_yawdot;
//threads
void* printf_loop(void* ptr);
void* lcm_publish_loop(void* ptr);
void* lcm_subscribe_loop(void* ptr);

void auto_control(float *pose, float *set_points, int16_t *channels_ptr);//add: pass time stamp
void channels_handler(const lcm_recv_buf_t *rbuf, const char *channel,
              const channels_t *msg, void *userdata);
void optitrack_handler(const lcm_recv_buf_t *rbuf, const char *channel,
              const pose_list_t *msg, void *userdata);

void dynamixel_handler(const lcm_recv_buf_t *rbuf, const char *channel,
              const dynamixel_status_list_t *msg, void *userdata);


void initialize_parameters();
void PID_ADC_read();
float theta_regulate(float theta);
void tune_PID(PID *p);
int interrupt_func();

// in autonomous_control.c
float PID_func(PID* pid, float err_input);
float PWM_regulator(float PWM, int PWM_type);
void integral_regulator(PID* pid);
float dist_vel_map(float *acc_dist, float *vel_ref_max, float dist_err);

// // Delat arm function declarations
int delta_calcForward(double theta1, double theta2, double theta3, double *x0, double *y0, double *z0);
int delta_calcAngleYZ(double x0, double y0, double z0, double *theta);
int delta_calcInverse(double x0, double y0, double z0, double *theta1, double *theta2, double *theta3);

int check_waypoint(float *pose, float *set_points, int type);
int round_trip(float *pose, float *set_points);
int pick_drop_one_time(float *pose, float *set_points);
int pick_drop_multi_time(float *pose, float *set_points);
int chase_pick_drop(float *pose, float *set_points);
int wander_around(float *pose, float *set_points);
int wander_around_2(float *pose, float *set_points);
int task2(float *pose, float *set_points);
void gripper_loc_drop_for_bucket();

int check_waypoint_fast(float *pose, float *set_points);


int check_pick();
int check_drop();
void gripper_loc_init();
void gripper_loc_pick_cal();
void gripper_loc_drop_cal();
void delta_arm_grab();
int pick_block(float *pose, float *set_points);
int drop_block(float *pose, float *set_points);
int check_loc(float *pose, float *set_points);
void dynamixel_cmd();
int gripper_hand_carry(float *pose, float *set_points);
int clock_tm2(float value);
int clock_tm2_tor(float value);
void check_torque();
void check_tor_simple();
void loc_ADC_tune();//float *pose_temp
void delta_arm_trim();

EXTERN float test1;
EXTERN float test2;
EXTERN float test3;

