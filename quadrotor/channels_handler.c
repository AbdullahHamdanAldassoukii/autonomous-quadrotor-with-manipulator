// Handler function to either pass through RX commands to Naza or else
// copy computer (autonomous control) commands through to Naza.
#define EXTERN extern
#include "quadrotor_main.h"

////////////////////////////////////////////////////////////////////////

void channels_handler(const lcm_recv_buf_t *rbuf, const char *channel,
              const channels_t *msg, void *userdata)
{
  // create a copy of the received message
  channels_t new_msg;
  new_msg.utime = msg->utime;

  state.channel_timestamp = new_msg.utime;

  new_msg.num_channels = msg->num_channels;
  new_msg.channels = (int16_t*) malloc(msg->num_channels*sizeof(int16_t));

  //copy stick commands into new message and to curent state
  pthread_mutex_lock(&state_mutex);
  for(int i = 0; i < msg->num_channels; i++){
    //state.RC_cmds[i] = msg->channels[i]; //in current_state_t state
    new_msg.channels[i] = msg->channels[i];
  }
  if(msg->channels[7]>=1500){
    state.autonomous_mode = 1;//in current_state_t state
  }

  else{
    state.autonomous_mode = 0;
  }

  // Copy state to local state struct to minimize mutex lock time
  struct current_state_t localstate;
  localstate = state;
  pthread_mutex_unlock(&state_mutex);


  // Decide whether or not to edit the motor message prior to sending it...
  float pose[8], set_points[8];
  // Grab pose form current state here
  if(state.autonomous_mode == 1){
    
    //get current quadrotor position, orientation, and relevant velocities
    pose[0] = march_filter(&filter_x, localstate.Q_xpos);
    pose[1] = march_filter(&filter_y, localstate.Q_ypos);
    pose[2] = march_filter(&filter_z, localstate.Q_zpos);
    pose[3] = march_filter(&filter_yaw,localstate.Q_yaw);

    pose[4] = march_filter(&filter_xdot,localstate.Q_xdot);
    pose[5] = march_filter(&filter_ydot,localstate.Q_ydot);
    pose[6] = march_filter(&filter_zdot,localstate.Q_zdot);
    pose[7] = march_filter(&filter_yawdot, localstate.Q_yawdot);

    if(task_num == 0){
          //auto_first_time++;
          if(auto_first_time == 0){
                pthread_mutex_lock(&state_mutex);
                state.Q_target_x = localstate.Q_xpos;
                state.Q_target_y = localstate.Q_ypos;
                state.Q_target_z = localstate.Q_zpos;
                pthread_mutex_unlock(&state_mutex);
                auto_first_time = 1;
          }
    }
    if(task_num == 1){
      if(auto_first_time == 0){
                pthread_mutex_lock(&state_mutex);
                task1_origin_x = localstate.Q_xpos;
                task1_origin_y = localstate.Q_ypos;
                task1_origin_z = localstate.Q_zpos;
                pthread_mutex_unlock(&state_mutex);
                auto_first_time = 1;
              }
    }

/*******************************************************************************
* target location
*******************************************************************************/
    set_points[0] = state.Q_target_x;
    set_points[1] = state.Q_target_y;
    set_points[2] = state.Q_target_z;
    set_points[3] = 0.0;//Yaw shouldn't be changed

    set_points[4] = 0.0;// This will be changed in autonomous_control.c   x_vel_ref
    set_points[5] = 0.0;//y_vel_ref
    set_points[6] = 0.0;//z_vel_ref
    set_points[7] = 0.0;//yaw_vel_ref

    auto_control(pose, set_points, new_msg.channels);
    pthread_mutex_lock(&state_mutex);
    for(int i = 0; i < msg->num_channels; i++){
      state.RC_cmds[i] = new_msg.channels[i]; //in current_state_t state
    }
    pthread_mutex_unlock(&state_mutex);

  } else{

    // pass user commands through without modifying
    if((task_num == 0)&&(auto_first_time == 1)){
       auto_first_time = 0;

     }
    if((task_num == 1)&&(auto_first_time == 1)){
       auto_first_time = 0;

     }
  }

  // send lcm message to motors
  channels_t_publish((lcm_t *) userdata, BLOCKS_TX_CHANNEL, &new_msg);

    // Save received (msg) and modified (new_msg) command data to file.
  // NOTE:  Customize as needed (set_points[] is for geofencing)
/*  fprintf(block_txt,"%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f\n",
   // (long int) msg->utime,msg->channels[0],msg->channels[1],msg->channels[2],
    (long int) utime_now(),msg->channels[0],msg->channels[1],msg->channels[2],
    msg->channels[3], msg->channels[7],
    new_msg.channels[0],new_msg.channels[1],new_msg.channels[2], 
    new_msg.channels[3],new_msg.channels[7],
    set_points[0],set_points[1],set_points[2],
    set_points[3],set_points[4],set_points[5],set_points[6],
    set_points[7]);
  fflush(block_txt);*/
}