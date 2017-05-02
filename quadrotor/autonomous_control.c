//
// auto_control:  Function to generate autonomous control PWM outputs.
// 
#define EXTERN extern
#include "quadrotor_main.h"

// Define outer loop controller 
// PWM signal limits and neutral (baseline) settings

// THRUST
#define thrust_PWM_up 1575 // Upper saturation PWM limit.
#define thrust_PWM_base 1500 // Zero z_vela PWM base value. 
#define thrust_PWM_down 1425 // Lower saturation PWM limit. 
  
// ROLL
#define roll_PWM_left 1620  // Left saturation PWM limit.
#define roll_PWM_base 1500  // Zero roll_dot PWM base value. 
#define roll_PWM_right 1380 //Right saturation PWM limit. 

// PITCH
#define pitch_PWM_forward 1620  // Forward direction saturation PWM limit.
#define pitch_PWM_base 1500 // Zero pitch_dot PWM base value. 
#define pitch_PWM_backward 1380 // Backward direction saturation PWM limit. 

// YAW
#define yaw_PWM_ccw 1575 // Counter-Clockwise saturation PWM limit (ccw = yaw left).
#define yaw_PWM_base 1500 // Zero yaw_dot PWM base value. 
#define yaw_PWM_cw 1425 // Clockwise saturation PWM limit (cw = yaw right). 

#define PWM_trivial_change_threshold 50

//pick threshold
#define x_err 0.05//0.1
#define y_err 0.05//0.1
#define z_err 0.05
#define yaw_err 1
#define x_dot_err 0.15
#define y_dot_err 0.15
#define z_dot_err 0.15
#define yaw_dot_err 3

//moving threshold
#define x_err_0 0.15
#define y_err_0 0.15
#define z_err_0 0.15
#define yaw_err_0 2
#define x_dot_err_0 0.2
#define y_dot_err_0 0.2
#define z_dot_err_0 0.2
#define yaw_dot_err_0 3


//moving threshold
#define x_err_2 0.15
#define y_err_2 0.15
#define z_err_2 0.4
#define yaw_err_2 2
#define x_dot_err_2 0.2
#define y_dot_err_2 0.2
#define z_dot_err_2 0.2
#define yaw_dot_err_2 3


#define GRIPPER_CLOSE 0.22//0.22
#define GRIPPER_OPEN -1//-0.88
#define GRIPPER_CLOSE_THRE -0.06
#define GRIPPER_OPEN_THRE -0.5
#define Z_BIAS_MM 133
#define ERR_GRIPPER_BLOCK 100

#define H1_mm 567//pick block-to-surface high
#define H2_mm 130//drone_upper_flat-to-gripper_upper_flat
#define H3_mm 500//drop block-to-surface high

#define check_waypoint_count_threshold 20

// Outer loop controller to generate PWM signals for the Naza-M autopilot
void auto_control(float *pose, float *set_points, int16_t* channels_ptr)
{  
  // pose (size 8):  actual {x, y , alt, yaw, xdot, ydot, altdot, yawdot}
  // set_points (size 8):  reference state (you need to set this!) 
  //                       {x, y, alt, yaw, xdot, ydot, altdot, yawdot} 

  // channels_ptr (8-element array of PWM commands to generate in this function)
  // Channels for you to set:
  // [0] = thrust
  // [1] = roll
  // [2] = pitch
  // [3] = yaw

/*******************************************************************************
* path loop, only for x,y
*******************************************************************************/
	float x_vel_ref = dist_vel_map(&dist_acc_config, &vel_ref_config, set_points[0] - pose[0]);
	set_points[4] = x_vel_ref;
	float x_spd = pitch_PWM_base + PID_func(&xPID, x_vel_ref - pose[4]);//roll

	float y_vel_ref = dist_vel_map(&dist_acc_config, &vel_ref_config, set_points[1] - pose[1]);
	set_points[5] = y_vel_ref;
	float y_spd = roll_PWM_base - PID_func(&yPID, y_vel_ref - pose[5]);//pitch

/*******************************************************************************
* path loop, only for z,  always running
*******************************************************************************/
	float z_vel_ref = dist_vel_map(&dist_acc_config, &vel_ref_config, set_points[2] - pose[2]);
	set_points[6] = z_vel_ref;
	float z_spd =  thrust_PWM_base - PID_func(&zPID, z_vel_ref - pose[6]);//thrust 

/*******************************************************************************
* yaw stable loop, always running
*******************************************************************************/
	float yaw_vel_ref = dist_vel_map(&dist_acc_config, &vel_ref_config, set_points[3] - pose[3]);//set_points[3] = 0;
	set_points[7] = yaw_vel_ref;
	float yaw_spd = yaw_PWM_base - PID_func(&yawPID, yaw_vel_ref - pose[7]);//yaw

/*******************************************************************************
* velocity loop
*******************************************************************************/

  channels_ptr[0] = PWM_regulator(z_spd,0);//thrust
  channels_ptr[1] = PWM_regulator(y_spd,1);//roll
  channels_ptr[2] = PWM_regulator(x_spd,2);//pitch
  //channels_ptr[3] = PWM_regulator(yaw_spd,3);//yaw

  //channels_ptr[7] = 1500;   

/*  test4 = 1000*(set_points[0]-pose[0]);
  test5 = 1000*(set_points[1]-pose[1]);
  test6 = 1000*(set_points[2]-pose[2])-Z_BIAS_MM;*/

//loc_ADC_tune();

  if(task_num == 1){
	task2(pose, set_points);//round_trip(pose, set_points);
  }

  if(task_num == 2){
    pick_drop_one_time(pose, set_points);
  }

  if(task_num == 3){
    pick_drop_multi_time(pose, set_points);
  }
    if(task_num == 4){
    chase_pick_drop(pose, set_points);
  }





  if(task_num == 7){
    gripper_hand_carry(pose, set_points);
  }
  if(task_num == 8){
    wander_around(pose, set_points);
  }
   if(task_num == 9){
    wander_around_2(pose, set_points);
  }
  return;

}


int round_trip(float *pose, float *set_points){
  if(step_finished == 0){

      // above pick up block
      if (step_num == 1){
        state.Q_target_x = 0;//task1_origin_x;
        state.Q_target_y = 0;//task1_origin_y;
        state.Q_target_z = -1;//task1_origin_z;


        if(check_waypoint(pose, set_points, 0)){
          if(clock_tm2(4)){

                    printf("----------------------------------------\n");
        printf("step:%d.\n",step_num);
        printf("----------------------------------------\n");
                    step_num++;
          }
          
        }
      }
      if (step_num == 2){
        state.Q_target_x = task1_final_x;
        state.Q_target_y = task1_final_y;
        state.Q_target_z = task1_final_z;


        if(check_waypoint(pose, set_points, 0)){
          if(clock_tm2(4)){
            
                    printf("----------------------------------------\n");
        printf("step:%d.\n",step_num);
        printf("----------------------------------------\n");
        step_num++;
          }
          
        }
      }
      // arrive pick up block
      if (step_num == 3){

        state.Q_target_x = 0;//task1_origin_x;
        state.Q_target_y = 0;//task1_origin_y;
        state.Q_target_z = -1;//task1_origin_z;

        if(check_waypoint(pose, set_points, 0)){
          step_finished = 1;
                  printf("----------------------------------------\n");
        printf("step:%d.\n",step_num);
        printf("----------------------------------------\n");
        }
      }
    }
    else if(step_finished == 1){
      return 1;
    }
  return 0;
}

int pick_drop_one_time(float *pose, float *set_points){
	if(step_finished == 0){


     // if()

      // above pick up block
  		if (step_num == 1){

  			state.Q_target_x = state.Q_PU_xpos;
  			state.Q_target_y = state.Q_PU_ypos;
  			state.Q_target_z = -1.2;//-1.1;

  			if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
  				step_num++;
  			}
  		}
      // arrive pick up block
  		if (step_num == 2){

  			state.Q_target_x = state.Q_PU_xpos;
  			state.Q_target_y = state.Q_PU_ypos;
  			state.Q_target_z = -1;

  			if(check_waypoint(pose, set_points, 1)){//require precice
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
  				step_num++;
  			}
  		}
      //pick up
      if (step_num == 3){

//set gripper close

        if(pick_block(pose,set_points)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
      //above pick up block
  		if (step_num == 4){
      //  check_tor_simple();
  			state.Q_target_x = state.Q_PU_xpos;
  			state.Q_target_y = state.Q_PU_ypos;
  			state.Q_target_z = -1.1;

  			if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
  				step_num++;
  			}
  		}
      //above drop off bucket
  		if (step_num == 5){
//check_tor_simple();
  			state.Q_target_x = state.Q_DO_xpos;
  			state.Q_target_y = state.Q_DO_ypos;
  			state.Q_target_z = -1.2;//-1.1;

  			if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
  				step_num++;
  			}
  		}
      //arrive drop off bucket
  		if (step_num == 6){
  			state.Q_target_x = state.Q_DO_xpos;
  			state.Q_target_y = state.Q_DO_ypos;
  			state.Q_target_z = -1;

  			if(check_waypoint(pose, set_points, 1)){// requre precise
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
  				step_num++;
  			}
  		}
      //drop off block
      if (step_num == 7){
//drop gripper

        if(drop_block(pose,set_points)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
      //above drop off bucket
  		if (step_num == 8){

  			state.Q_target_x = state.Q_DO_xpos;
  			state.Q_target_y = state.Q_DO_ypos;
  			state.Q_target_z = -1.1;

  			if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
  				step_num++;
  				//step_finished = 1;
  			}
  		}
      // back to 0 point
      if (step_num == 9){
        state.Q_target_x = 0;
        state.Q_target_y = 0;
        state.Q_target_z = -1;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num = 1;
      //    step_finished = 1;
        }
      }

  	}
  	else if(step_finished == 1){
  		return 1;
  	}
	return 0;
}




int task2(float *pose, float *set_points){
	if(step_finished == 0){


     // if()
      // arrive pick up block
  		if (step_num == 1){

  			state.Q_target_x = 1.5;
  			state.Q_target_y = -1.5;
  			state.Q_target_z = -1.2;

/*  			if(check_waypoint(pose, set_points, 1)){//require precice
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
  				step_num++;
  			}*/
  			        if(check_waypoint(pose, set_points, 0)){
          if(clock_tm2(6)){

                    printf("----------------------------------------\n");
        printf("step:%d.\n",step_num);
        printf("----------------------------------------\n");
                    step_num++;
          }
          
        }
  		}

      //above pick up block
  		if (step_num == 2){
      //  check_tor_simple();
  			state.Q_target_x = 0;
  			state.Q_target_y = 0;
  			state.Q_target_z = -1.2;

/*  			if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
  				step_num++;
  				step_finished = 1;
  			}*/
  			        if(check_waypoint(pose, set_points, 0)){
          if(clock_tm2(4)){

                    printf("----------------------------------------\n");
        printf("step:%d.\n",step_num);
        printf("----------------------------------------\n");
                    step_num++;
                    step_finished = 1;
          }
          
        }
  		}
   

  	}
  	else if(step_finished == 1){
  		return 1;
  	}
	return 0;
}


int pick_drop_multi_time(float *pose, float *set_points){
	if(step_finished == 0){

      // above pick up block
  		if (step_num == 1){

  			state.Q_target_x = state.Q_PU_xpos;
  			state.Q_target_y = state.Q_PU_ypos;
  			state.Q_target_z = -1.3;

  			if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
  				step_num++;
  			}
  		}
      // arrive pick up block
  		if (step_num == 2){

  			state.Q_target_x = state.Q_PU_xpos;
  			state.Q_target_y = state.Q_PU_ypos;
  			state.Q_target_z = -1;

  			if(check_waypoint(pose, set_points, 1)){//require precice
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
  				step_num++;
  			}
  		}
      //pick up
      if (step_num == 3){

//set gripper close

        if(pick_block(pose,set_points)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
      //above pick up block
  		if (step_num == 4){
      //  check_tor_simple();
  			state.Q_target_x = state.Q_PU_xpos;
  			state.Q_target_y = state.Q_PU_ypos;
  			state.Q_target_z = -1.2;

  			if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
  				step_num++;
  			}
  		}
      //above drop off bucket
  		if (step_num == 5){
//check_tor_simple();
  			state.Q_target_x = state.Q_DO_xpos;
  			state.Q_target_y = state.Q_DO_ypos;
  			state.Q_target_z = -1.2;//-1.3;

  			if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
  				step_num++;
  			}
  		}
      //drop off block
      if (step_num == 6){
//drop gripper

        if(drop_block(pose,set_points)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num = 1;
        }
      }

  	}
  	else if(step_finished == 1){
  		return 1;
  	}
	return 0;
}



int chase_pick_drop(float *pose, float *set_points){
	if(step_finished == 0){

      // above pick up block
  		if (step_num == 1){

  			state.Q_target_x = state.Q_PU_xpos;
  			state.Q_target_y = state.Q_PU_ypos;
  			state.Q_target_z = -1.3;

  			if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
  				step_num++;
  			}
  		}
      // arrive pick up block
  		if (step_num == 2){

  			state.Q_target_x = state.Q_PU_xpos;
  			state.Q_target_y = state.Q_PU_ypos;
  			state.Q_target_z = -1;

  			if(check_waypoint(pose, set_points, 1)){//require precice
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
  				step_num++;
  			}
  		}
      //pick up
      if (step_num == 3){

//set gripper close

        if(pick_block(pose,set_points)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }

            // arrive pick up block
  		if (step_num == 4){

  			state.Q_target_x = state.Q_PU_xpos;
  			state.Q_target_y = state.Q_PU_ypos;
  			state.Q_target_z = -1.2;

  			if(check_waypoint(pose, set_points, 0)){//require precice
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
  				step_num++;
  			}
  		}

      //above pick up block
  		if (step_num == 5){
      //  check_tor_simple();
  			state.Q_target_x = state.Q_DO_xpos;
  			state.Q_target_y = state.Q_DO_ypos;
  			state.Q_target_z = -0.9;
  			

  			if(check_waypoint(pose, set_points, 0)){//require precice
  				gripper_loc_drop_for_bucket();
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
  				step_num++;
  			}
  		}


      //above pick up block
  		if (step_num == 6){
      //  check_tor_simple();
  			state.Q_target_x = state.Q_DO_xpos;
  			state.Q_target_y = state.Q_DO_ypos;
  			state.Q_target_z = -0.9;

  			if(check_waypoint_fast(pose, set_points)){
  				gripper_pick_pos = GRIPPER_OPEN;
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
  				step_num++;

  			}
  		}

  		      //above pick up block
  		if (step_num == 7){
      //  check_tor_simple();
  			state.Q_target_x = state.Q_DO_xpos;
  			state.Q_target_y = state.Q_DO_ypos;
  			state.Q_target_z = -1.3;

  			if(check_waypoint(pose, set_points,0)){
  				gripper_loc_init();
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
  				step_num=1;

  			}
  		}

  	}
  	else if(step_finished == 1){
  		return 1;
  	}
	return 0;
}

int gripper_hand_carry(float *pose, float *set_points){

      // arrive pick up block
      if (step_num == 1){
        state.Q_target_x = state.Q_PU_xpos;
        state.Q_target_y = state.Q_PU_ypos;
        state.Q_target_z = -1.1;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
      //pick up
      if (step_num == 2){

//set gripper close

        if(pick_block(pose,set_points)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
      //arrive drop off bucket
      if (step_num == 3){

        state.Q_target_x = state.Q_DO_xpos;
        state.Q_target_y = state.Q_DO_ypos;
        state.Q_target_z = -1.1;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
      //drop off block
      if (step_num == 4){
//drop gripper

        if(drop_block(pose,set_points)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num = 1;
          step_finished = 1;
          return 1;
        }
      }
  return 0;
}



int wander_around(float *pose, float *set_points){
  if(step_finished == 0){
      // above pick up block
      if (step_num == 1){

        state.Q_target_x = 0;
        state.Q_target_y = 0;
        state.Q_target_z = -1.5;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
      // arrive pick up block
      if (step_num == 2){

        state.Q_target_x = 0;
        state.Q_target_y = 0;
        state.Q_target_z = -1;

        if(check_waypoint(pose, set_points, 1)){//require precice
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }

      //above pick up block
      if (step_num == 3){

        state.Q_target_x = 1;
        state.Q_target_y = 0;
        state.Q_target_z = -1.5;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
      //above drop off bucket
      if (step_num == 4){

        state.Q_target_x = 1;
        state.Q_target_y = 0;
        state.Q_target_z = -1;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
     
      //above pick up block
      if (step_num == 5){

        state.Q_target_x = 1;
        state.Q_target_y = 1;
        state.Q_target_z = -1.5;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
      //above drop off bucket
      if (step_num == 6){

        state.Q_target_x = 1;
        state.Q_target_y = 1;
        state.Q_target_z = -1;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }

            //above pick up block
      if (step_num == 7){

        state.Q_target_x = -1;
        state.Q_target_y = -1;
        state.Q_target_z = -1.5;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
      //above drop off bucket
      if (step_num == 8){

        state.Q_target_x = -0.5;
        state.Q_target_y = -0.5;
        state.Q_target_z = -1;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }

                  //above pick up block
      if (step_num == 9){

        state.Q_target_x = 1;
        state.Q_target_y = -0.5;
        state.Q_target_z = -1.5;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
      //above drop off bucket
      if (step_num == 10){

        state.Q_target_x = 1;
        state.Q_target_y = -1;
        state.Q_target_z = -1;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
      // back to 0 point
      if (step_num == 11){
        state.Q_target_x = 0;
        state.Q_target_y = 0;
        state.Q_target_z = -1;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num = 1;
          step_finished = 1;
        }
      }

    }
    else if(step_finished == 1){
      return 1;
    }
  return 0;
}

int wander_around_2(float *pose, float *set_points){
  if(step_finished == 0){
      // above pick up block
      if (step_num == 1){

        state.Q_target_x = 0;
        state.Q_target_y = 0;
        state.Q_target_z = -1.5;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
      // arrive pick up block
      if (step_num == 2){

        state.Q_target_x = 0;
        state.Q_target_y = 0;
        state.Q_target_z = -1;

        if(check_waypoint(pose, set_points, 0)){//require precice
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }

      //above pick up block
      if (step_num == 3){

        state.Q_target_x = state.Q_PU_xpos;
        state.Q_target_y = state.Q_PU_ypos;
        state.Q_target_z = -1.5;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
      //above drop off bucket
      if (step_num == 4){

        state.Q_target_x = state.Q_PU_xpos;
        state.Q_target_y = state.Q_PU_ypos;
        state.Q_target_z = -1;

        if(check_waypoint(pose, set_points, 1)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
     
      //above pick up block
      if (step_num == 5){

        state.Q_target_x = 1;
        state.Q_target_y = 1;
        state.Q_target_z = -1.5;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
      //above drop off bucket
      if (step_num == 6){

        state.Q_target_x = 1;
        state.Q_target_y = 1;
        state.Q_target_z = -1;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }

            //above pick up block
      if (step_num == 7){

        state.Q_target_x = state.Q_DO_xpos;
        state.Q_target_y = state.Q_DO_ypos;
        state.Q_target_z = -1.5;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
      //above drop off bucket
      if (step_num == 8){

        state.Q_target_x = state.Q_DO_xpos;
        state.Q_target_y = state.Q_DO_ypos;
        state.Q_target_z = -1;

        if(check_waypoint(pose, set_points, 1)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }

                  //above pick up block
      if (step_num == 9){

        state.Q_target_x = -1;
        state.Q_target_y = 0;
        state.Q_target_z = -1.5;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
      //above drop off bucket
      if (step_num == 10){

        state.Q_target_x = -1;
        state.Q_target_y = 0;
        state.Q_target_z = -1;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num++;
        }
      }
      // back to 0 point
      if (step_num == 11){
        state.Q_target_x = 0;
        state.Q_target_y = 0;
        state.Q_target_z = -1;

        if(check_waypoint(pose, set_points, 0)){
          printf("----------------------------------------\n");
          printf("step:%d finished.\n",step_num);
          printf("----------------------------------------\n");
          step_num = 1;
          step_finished = 1;
        }
      }

    }
    else if(step_finished == 1){
      return 1;
    }
  return 0;
}


float dist_vel_map(float *acc_dist, float *vel_ref_max, float dist_err){
	if ((dist_err < *acc_dist) && (dist_err > -*acc_dist)){
		return dist_err * (*vel_ref_max)/(*acc_dist);
	}
	else if (dist_err >= *acc_dist){
		return *vel_ref_max;
	}
	else if (dist_err <= -*acc_dist){
		return -*vel_ref_max;
	}
	return 0;
}


float PID_func(PID* pid, float err_input){
	float delt_t = (state.channel_timestamp - pid->time_stamp)/1000000.00;
	//float delt_t = 0.02;
	//pid->integral_err += err_input*delt_t;
	//integral_regulator(pid);
	float response = pid->kp * err_input +  pid->kd * (err_input-pid->prev_err)/delt_t;// pid->ki*pid->integral_err*delt_t +
	pid->prev_err = err_input;
	pid->time_stamp = state.channel_timestamp;
	return response;
}




/*******************************************************************************
* PID_ADC_read() read otentialmeter values through ADC
*******************************************************************************/
void PID_ADC_read(){
    adc_PID.p = get_adc_volt(0)*adc_PID.p_scale;
    adc_PID.i = get_adc_volt(1)*adc_PID.i_scale;
    adc_PID.d = get_adc_volt(2)*adc_PID.d_scale;
}
/*******************************************************************************
* tune_PID() IMU interrupt routine to state variables
*******************************************************************************/
void tune_PID(PID *p){
    PID_ADC_read();
    p->kp = adc_PID.p;
    p->ki = adc_PID.i;
    p->kd = adc_PID.d;
}



float PWM_regulator(float PWM, int PWM_type){
	//thrust
	if (PWM_type == 0){
		if (PWM > thrust_PWM_up)
			PWM = thrust_PWM_up;
		if (PWM < thrust_PWM_down)
			PWM = thrust_PWM_down;
	}
	//roll
	if (PWM_type == 1){
		if (PWM > roll_PWM_left)
			PWM = roll_PWM_left;
		if (PWM < roll_PWM_right)
			PWM = roll_PWM_right;
	}
	//pitch
	if (PWM_type == 2){
		if (PWM > pitch_PWM_forward)
			PWM = pitch_PWM_forward;
		if (PWM < pitch_PWM_backward)
			PWM = pitch_PWM_backward;
	}
	//yaw
	if (PWM_type == 3){
		if (PWM > yaw_PWM_ccw)
			PWM = yaw_PWM_ccw;
		if (PWM < yaw_PWM_cw)
			PWM = yaw_PWM_cw;
	}
	return PWM;
	
}

void integral_regulator(PID* pid){
	if(pid->integral_err > pid->integral_limit_max)
		pid->integral_err = pid->integral_limit_max;
	else if (pid->integral_err < pid->integral_limit_min)
		pid->integral_err = pid->integral_limit_min;
}



int pick_block(float *pose, float *set_points){
  gripper_loc_pick_cal(pose, set_points);
  if( clock_tm2(1)){
  printf("\nstart pick\n");
    gripper_pick_pos = GRIPPER_CLOSE;
  }


/*  if(check_loc(pose,set_points)){
      gripper_pick_pos = GRIPPER_CLOSE;
  }*/
 // some problem here
  return check_pick();
}

int drop_block(float *pose, float *set_points){
  gripper_loc_drop_cal(pose, set_points);

  if( clock_tm2(1)){
  printf("\nstart drop\n");
    gripper_pick_pos = GRIPPER_OPEN;
  }
  //if(check_loc(pose,set_points)){

      
 // }

  return check_drop();
}

int check_pick(){

 // gripper_fb_pos;
  if((gripper_fb_pos>GRIPPER_CLOSE_THRE)){//&&(gripper_fb_torque<-0.2)
    count_check_for_pick++;
    if(count_check_for_pick > 10){
      count_check_for_pick = 0;
      printf("\npick checked\n");
      gripper_loc_init();
      time_step_flag=0;//initial timer
      return 1;
    }
  }
  else{
    count_check_for_pick = 0;
  }
  return 0;
}

int check_drop(){
  if(gripper_fb_pos<GRIPPER_OPEN_THRE){
    count_check_for_pick++;
    if(count_check_for_pick > 4){
      count_check_for_pick = 0;
      printf("\ndrop checked\n");
      gripper_loc_init();
      time_step_flag=0;//initial timer
      return 1;
    }
  }
  else{
    count_check_for_pick = 0;
  }
  return 0;
}

int check_loc(float *pose, float *set_points){

  if((fabs(gripper_position[0]+1000*pose[0]-1000*set_points[0])<ERR_GRIPPER_BLOCK) &&
  (fabs(gripper_position[1]+1000*pose[1]-1000*set_points[1])<ERR_GRIPPER_BLOCK)  &&
  (fabs(gripper_position[2]+1000*pose[2]+H2_mm+H1_mm-1000*set_points[2])<ERR_GRIPPER_BLOCK)){
    //printf("\nloc checked.\n");
    return 1;
  }
  return 0;
}


void check_tor_simple(){
  if(gripper_fb_torque>-0.1){
    printf("torque loose.\n");
    step_num = 1;
    gripper_loc_init();
  }
}

//type == 1: accurate, used for pick and drop; type == 0: inaccurate, used for middle point, threshold is larger.
int check_waypoint(float *pose, float *set_points, int type){
	if(type == 1){
       // printf("qq1-");
			if((fabs(pose[0]-set_points[0])<x_err) && (fabs(pose[1]-set_points[1])<y_err) && (fabs(pose[2]-set_points[2])<z_err) && (fabs(pose[3]-set_points[3])<yaw_err)){
				//if((fabs(pose[4]-set_points[4])<x_dot_err) && (fabs(pose[5]-set_points[5])<y_dot_err) && (fabs(pose[6]-set_points[6])<z_dot_err) && (fabs(pose[7]-set_points[7])<yaw_dot_err)){
					check_waypoint_count ++;
				//}
			}
			else{
				check_waypoint_count = 0;
			}
	}
	else if(type == 0){
    //printf("qq0-");
			if((fabs(pose[0]-set_points[0])<x_err_0) && (fabs(pose[1]-set_points[1])<y_err_0) && (fabs(pose[2]-set_points[2])<z_err_0) && (fabs(pose[3]-set_points[3])<yaw_err_0)){
			//	if((fabs(pose[4]-set_points[4])<x_dot_err_0) && (fabs(pose[5]-set_points[5])<y_dot_err_0) && (fabs(pose[6]-set_points[6])<z_dot_err_0) && (fabs(pose[7]-set_points[7])<yaw_dot_err_0)){
          check_waypoint_count ++;
			//	}
			}
			else{
				check_waypoint_count = 0;
			}
	}
		else if(type == 2){
    //printf("qq0-");
			if((fabs(pose[0]-set_points[0])<x_err_2) && (fabs(pose[1]-set_points[1])<y_err_2) && (fabs(pose[2]-set_points[2])<z_err_2) && (fabs(pose[3]-set_points[3])<yaw_err_2)){
			//	if((fabs(pose[4]-set_points[4])<x_dot_err_0) && (fabs(pose[5]-set_points[5])<y_dot_err_0) && (fabs(pose[6]-set_points[6])<z_dot_err_0) && (fabs(pose[7]-set_points[7])<yaw_dot_err_0)){
          check_waypoint_count ++;
			//	}
			}
			else{
				check_waypoint_count = 0;
			}
	}

	if (check_waypoint_count > check_waypoint_count_threshold){
		check_waypoint_count = 0;
		return 1;
	}
	return 0;
}


int check_waypoint_fast(float *pose, float *set_points){
			if((fabs(pose[0]-set_points[0])<0.05) && (fabs(pose[1]-set_points[1])<0.05) && (fabs(pose[2]-set_points[2])<0.1)){
				//if((fabs(pose[4]-set_points[4])<x_dot_err) && (fabs(pose[5]-set_points[5])<y_dot_err) && (fabs(pose[6]-set_points[6])<z_dot_err) && (fabs(pose[7]-set_points[7])<yaw_dot_err)){
				return 1;
			}
	return 0;
}

void gripper_loc_init(){
  gripper_x_mm = 0;
  gripper_y_mm = 0;
  gripper_z_mm = 66;
  //gripper_pick_pos = -0.8;
  printf("init gripper loc\n");
}


void gripper_loc_drop_for_bucket(){
  gripper_x_mm = 0;
  gripper_y_mm = 0;
  gripper_z_mm = 240;
  //gripper_pick_pos = -0.8;
  //printf("init gripper loc\n");
}

void gripper_loc_pick_cal(float *pose, float *set_points){
  gripper_x_mm = 1000*(set_points[0]-pose[0]);
  gripper_y_mm = 1000*(set_points[1]-pose[1]);
  gripper_z_mm = 150;//150-pose[2]*1000+1047;//1000*(set_points[2]-pose[2])-H1_mm-H2_mm+900;//1000*((set_points[2]-h1)-(pose[2]+h2));1047
}

void gripper_loc_drop_cal(float *pose, float *set_points){
  gripper_x_mm = 1000*(set_points[0]-pose[0]);
  gripper_y_mm = 1000*(set_points[1]-pose[1]);
  gripper_z_mm = 150;//150-pose[2]*1000+1047;//1000*(set_points[2]-pose[2])-H3_mm-H2_mm+900;//1000*((set_points[2]-h1)-(pose[2]+h2));
}