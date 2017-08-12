// Handler function to parse Optitrack lcm messages
#define EXTERN extern
#include "quadrotor_main.h"

////////////////////////////////////////////////////////////////////////

void optitrack_handler(const lcm_recv_buf_t *rbuf, const char *channel,
              const pose_list_t *msg, void *userdata){

    //lock mutex state while editing
    // pass all data from msg received to state space
    pthread_mutex_lock(&state_mutex);
    state.Opti_dt = (msg->Q_pose.utime - state.Q_timestamp)/1000000.0;
    state.Q_timestamp = msg->Q_pose.utime;
    state.Q_xdot = (-state.Q_xpos + msg->Q_pose.x)/state.Opti_dt;//msg->Q_pose.x
    state.Q_ydot = (-state.Q_ypos + msg->Q_pose.y)/state.Opti_dt;//msg->Q_pose.y
    state.Q_zdot = (-state.Q_zpos + msg->Q_pose.z)/state.Opti_dt;//msg->Q_pose.z

/*
    state.Q_xdot = 0;//msg->Q_pose.x
    state.Q_ydot = 0;//msg->Q_pose.y
    state.Q_zdot = 0;//msg->Q_pose.z*/

/*
    state.Q_xdot = march_filter(&filter_xdot, (msg->Q_pose.x - state.Q_xpos)/state.Opti_dt);
    state.Q_ydot = march_filter(&filter_ydot, (msg->Q_pose.y - state.Q_ypos)/state.Opti_dt);
    state.Q_zdot = march_filter(&filter_zdot, (msg->Q_pose.z - state.Q_zpos)/state.Opti_dt);
*/
    state.Q_xpos = msg->Q_pose.x;
    state.Q_ypos = msg->Q_pose.y;
    state.Q_zpos = msg->Q_pose.z;

/*    state.Q_xpos = march_filter(&filter_x, msg->Q_pose.x);
    state.Q_ypos = march_filter(&filter_y, msg->Q_pose.y);
    state.Q_zpos = march_filter(&filter_z, msg->Q_pose.z);*/


    state.Q_roll = msg->Q_pose.roll;
    state.Q_pitch = msg->Q_pose.pitch;
    state.Q_yawdot = (-state.Q_yaw + msg->Q_pose.yaw)/state.Opti_dt;
    //state.Q_yawdot = march_filter(&filter_yawdot, (msg->Q_pose.yaw - state.Q_yaw)/state.Opti_dt);

    state.Q_yaw = msg->Q_pose.yaw;
    //state.Q_yaw = march_filter(&filter_yaw, msg->Q_pose.yaw);


    state.Q_PU_xpos = msg->PU_pose.x;//
    state.Q_PU_ypos = msg->PU_pose.y;
    state.Q_PU_zpos = msg->PU_pose.z;
    state.Q_DO_xpos = msg->DO_pose.x;
    state.Q_DO_ypos = msg->DO_pose.y;
    state.Q_DO_zpos = msg->DO_pose.z;
    pthread_mutex_unlock(&state_mutex);
}