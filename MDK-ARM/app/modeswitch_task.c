/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file modeswitch_task.c
 *  @version 2.0
 *  @date Jun 2019
 *
 *  @brief infantry control mode switch
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */
 
#include "modeswitch_task.h"
#include "info_get_task.h"
#include "remote_ctrl.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "comm_task.h"
#include "shoot_task.h"
#include "detect_task.h"
#include "infantry_info.h"
#include "bsp_uart.h"
#include "keyboard.h"
#include "sys_config.h"
#include "math.h"
#include "stdlib.h"
#include "cmsis_os.h"

/* stack usage monitor */
UBaseType_t mode_stack_surplus;

/* mode switch task static parameter */
infantry_mode_e last_glb_ctrl_mode;
infantry_mode_e glb_ctrl_mode;

extern TaskHandle_t info_get_task_t;
extern osTimerId chassis_timer_id;
extern osTimerId gimbal_timer_id;
void mode_switch_task(void const *argu)
{
  gimbal_self_check();
  
  osTimerStart(gimbal_timer_id, GIMBAL_PERIOD);
  osTimerStart(chassis_timer_id, CHASSIS_PERIOD);
  
  uint32_t mode_wake_time = osKernelSysTick();
  while (1)
  {
    taskENTER_CRITICAL(); //关键任务，禁用中断
    
    get_main_ctrl_mode();
    
    get_chassis_mode();
    get_gimbal_mode();
    get_shoot_mode();
    
    get_global_last_mode(); //存储上次云台模式
    
    taskEXIT_CRITICAL();
    
    osSignalSet(info_get_task_t, INFO_GET_EXE_SIGNAL);
    
    mode_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(&mode_wake_time, INFO_GET_PERIOD);
  }

}

static void kb_enable_hook(void) //鼠标键盘控制还是遥控器控制。
{
  if (rc.sw2 == RC_UP)
    km.kb_enable = 1;
  else
    km.kb_enable = 0;

}

/**
 * Edited by Y.H Liu
 * @Jun 4th, 2019
 *  
 * Change the state transferring logic, and simplify the control
 * After modification, the SW2 switch the robot from 
 *     Chassis-Follow-Gimbal, a.k.a. Manual               [UP]
 *     Relaxed, a.k.a. Auto: Disabled                     [MID]
 *     Gimbal-Follow-Chassis, a.k.a. SemiAuto: Debugging  [DOWN]
*/ 
void get_main_ctrl_mode(void)
{
  //host PC has been connected
  if (1) //(!g_err.list[PC_SYS_OFFLINE].err_exist)
  {
    switch (rc.sw2)
    {
      case RC_UP:
      {
        glb_ctrl_mode = MANUAL_CTRL_MODE;
      }break;
      
#ifdef AUTO_NAVIGATION
      case RC_MI:
      {
        glb_ctrl_mode = AUTO_CTRL_MODE; //Only the auto: disabled is in used
      }break;
      
      case RC_DN:
      {
        glb_ctrl_mode = SEMI_AUTO_MODE; //Only the semi-auto: gimbal follow chassis is in used
      }break;
#endif
      
      default:
      {
        glb_ctrl_mode = SAFETY_MODE;
      }break;
    }
  }
  //host PC offline
  else
  {
    switch (rc.sw2)
    {
      case RC_UP:
      {
        glb_ctrl_mode = MANUAL_CTRL_MODE;
      }break;
      
      default:
      {
        glb_ctrl_mode = SAFETY_MODE;
      }break;
    }
  }
  
  
  if (rc.sw2 == RC_MI)
    glb_ctrl_mode = SAFETY_MODE;
  
  kb_enable_hook();
  
}


static action_mode_e remote_is_action(void)
{
  if ((abs(rc.ch1) >= 10) //遥控器混乱值过滤
   || (abs(rc.ch2) >= 10)
   || (abs(rc.ch3) >= 10)
   || (abs(rc.ch4) >= 10)
   || (abs(rc.mouse.x) >= 10)
   || (abs(rc.mouse.y) >= 10)
	 || rc.kb.bit.Q
	 || rc.kb.bit.E)
  {
    return IS_ACTION;
  }
  else
  {
    return NO_ACTION;
  }
}

/**
 * Edited by Y.H. Liu
 * @Jun 4th, 2019
 * 
 * To match the modification mentioned above.  
*/ 
static void gimbal_mode_handler(void)
{
  switch (glb_ctrl_mode)
  {
    case MANUAL_CTRL_MODE:
    {
      if (last_glb_ctrl_mode == SEMI_AUTO_MODE)
        gim.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
      
      /* no input control signal gimbal mode handle */
      if (gim.input.ac_mode == NO_ACTION)
      {
        if (gim.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)
        {
          //if (fabs(chassis.vw) <= gim.input.action_angle)
          if (fabs(gim.sensor.yaw_relative_angle) <= gim.input.action_angle)
          {
            //begin no action handle
            gim.ctrl_mode = GIMBAL_NO_ARTI_INPUT;
            
            gim.input.no_action_flag = 1;
            gim.input.no_action_time = HAL_GetTick();
          }
        }
      }
      else  //IS_ACTION mode
      {
        chassis.follow_gimbal = 1;
        if (gim.ctrl_mode == GIMBAL_NO_ARTI_INPUT)
        {
          gim.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
          gim.input.no_action_flag = 0;
          
          gim.pid.yaw_angle_ref = 0;
          gim.yaw_offset_angle = gim.sensor.yaw_gyro_angle;
        }
      }
      
      /* manual trigger chassis twist */
      if (km.twist_ctrl)
        gim.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
      

      /* manual trigger track armor */
      if (km.track_ctrl)
        gim.ctrl_mode = GIMBAL_TRACK_ARMOR;
//      
//      /* manual trigger big buff */
//      if (km.buff_ctrl && km.kb_enable)
//      {
//        gim.ctrl_mode = GIMBAL_SHOOT_BUFF;
//        chassis.follow_gimbal = 0;
//        
//        if (gim.last_ctrl_mode != GIMBAL_SHOOT_BUFF)
//        {
//          gim.auto_ctrl_cmd = CMD_CALI_FIVE;
//        }
//      }
//      else
//        chassis.follow_gimbal = 1;

      
      if (gim.last_ctrl_mode == GIMBAL_RELAX || 
          gim.last_ctrl_mode == GIMBAL_TRACK_ARMOR && !km.track_ctrl ||
          chassis.last_ctrl_mode == DODGE_MODE && !km.twist_ctrl)
        gim.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
    }break;
    
    case SEMI_AUTO_MODE:
    {
      gim.ctrl_mode = GIMBAL_RELATIVE_MODE;
    }break;
    
    case AUTO_CTRL_MODE:
    {
      gim.ctrl_mode = GIMBAL_RELAX;
    }break;
    
    default:
    {
      gim.ctrl_mode = GIMBAL_RELAX;
    }break;
  }
}

void get_gimbal_mode(void)
{
  gim.input.ac_mode = remote_is_action();
  
  if (gim.ctrl_mode != GIMBAL_INIT)
  {
    gimbal_mode_handler();
  }
  
  /* gimbal back to center */
  if (gim.last_ctrl_mode == GIMBAL_RELAX && gim.ctrl_mode != GIMBAL_RELAX && glb_ctrl_mode != AUTO_CTRL_MODE)
  {
    /* set gimbal init mode */
    gim.ctrl_mode = GIMBAL_INIT;
    /* record yaw angle initial offset */
    gim.ecd_offset_angle = gim.sensor.yaw_relative_angle;
    /* set initial parameters */
    gimbal_back_param();
  }
}

static void get_global_last_mode(void)
{
  last_glb_ctrl_mode = glb_ctrl_mode;
  gim.last_ctrl_mode = gim.ctrl_mode;
  chassis.last_ctrl_mode = chassis.ctrl_mode;
}

/**
 * Edited by Y.H. Liu
 * @Jun 4th, 2019
 * 
 * To match the modification mentioned above.  
*/ 
static void chassis_mode_handler(void)
{
  switch (glb_ctrl_mode)
  {
    case MANUAL_CTRL_MODE:
    {
      chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
      
      /* keyboard trigger chassis twist mode */
      if (km.twist_ctrl)
        chassis.ctrl_mode = DODGE_MODE;

    }break;
    
    case SEMI_AUTO_MODE:
    {
			chassis.ctrl_mode = MANUAL_SEPARATE_GIMBAL;
      //chassis.follow_gimbal = 1;
      //chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
      uint8_t back_to_netural = gim.sensor.yaw_relative_angle < 5 && gim.sensor.yaw_relative_angle > -5 ? 1 : 0;
      if (km.twist_ctrl || chassis.ctrl_mode==DODGE_MODE && !back_to_netural) 
      { //two situation for dodging: the key is pressed, 
        //               or (the key is not pressed but) the chassis is still dodging and the gimbal hasn't come back to netural
        chassis.ctrl_mode = DODGE_MODE;
      }
      
    }break;
    
    case AUTO_CTRL_MODE:
    {
      chassis.ctrl_mode = CHASSIS_STOP;
    }break;
    
    default:
    {
      chassis.ctrl_mode = CHASSIS_STOP;
    }break;
    
  }
  
}
extern int8_t   twist_side;
extern int8_t   twist_sign;
extern uint32_t twist_count;

extern int16_t twist_period;
extern int16_t twist_angle;
void get_chassis_mode(void)
{
  if (gim.ctrl_mode == GIMBAL_INIT)
  {
    chassis.ctrl_mode = CHASSIS_STOP;
  }
  else
  {
    chassis_mode_handler();
  }
  
  /* chassis just enter dodge mode */
  if (chassis.last_ctrl_mode != DODGE_MODE && chassis.ctrl_mode == DODGE_MODE)
  {
    #ifndef ROTATING
    if (gim.sensor.yaw_relative_angle > 0)
      twist_side = 1;
    else
      twist_side = -1;
    if ((gim.sensor.yaw_relative_angle < twist_angle) && (gim.sensor.yaw_relative_angle > -twist_angle))
      twist_count = acos((gim.sensor.yaw_relative_angle - twist_side*twist_angle)/(-twist_sign*40.0)) * twist_period / (2*PI);
    #endif
    //TWIST: twist count is calculated based on the current relative angle and is used to generate the position ref
    //ROTATION: however, the rotating is successive, thus there is no need to count.  
  }
  
}

uint8_t gimbal_is_controllable(void)
{
  if (gim.ctrl_mode == GIMBAL_RELAX
   || g_err.list[REMOTE_CTRL_OFFLINE].err_exist
   || g_err.list[GIMBAL_YAW_OFFLINE].err_exist
   || g_err.list[GIMBAL_PIT_OFFLINE].err_exist)
    return 0;
  else
    return 1;
}

uint8_t chassis_is_controllable(void)
{
  if (chassis.ctrl_mode == CHASSIS_RELAX 
   || g_err.list[REMOTE_CTRL_OFFLINE].err_exist)
    return 0;
  else
    return 1;
}

/**
 * Edited by Y.H. Liu
 * @Jun 4th, 2019
 * 
 * To match the modification mentioned above.  
*/ 
void get_shoot_mode(void)
{
  switch (glb_ctrl_mode)
  {
    case MANUAL_CTRL_MODE:
    {
      if (km.kb_enable)
        shoot.ctrl_mode = KEYBOARD_CTRL_SHOT;
      else
        shoot.ctrl_mode = REMOTE_CTRL_SHOT;
    }break;
    
    case SEMI_AUTO_MODE:
    {
      if (km.kb_enable)
        shoot.ctrl_mode = KEYBOARD_CTRL_SHOT;
      else
        shoot.ctrl_mode = REMOTE_CTRL_SHOT;
    }break;
    
    case AUTO_CTRL_MODE:
    {
      shoot.ctrl_mode = SHOT_DISABLE;
    }break;
    
    default:
    {
      shoot.ctrl_mode = SHOT_DISABLE;
    }break;
    
  }

  if (gim.ctrl_mode == GIMBAL_RELAX)
    shoot.ctrl_mode = SHOT_DISABLE;
}
