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
/** @file shoot_task.c
 *  @version 1.1
 *  @date June 2017
 *
 *  @brief shoot bullet task
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "shoot_task.h"
#include "gimbal_task.h"
#include "detect_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "remote_ctrl.h"
#include "bsp_io.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_io.h"
#include "keyboard.h"
#include "pid.h"
#include "sys_config.h"
#include "cmsis_os.h"
#include "string.h"
#include "math.h"

#define INFANTRY_NUM INFANTRY_3

#define CAMERA_ON_GIMBAL

#ifndef INFANTRY_NUM
  #error "INFANTRY_NUM must be define!"
#endif

/* stack usage monitor */
UBaseType_t shoot_stack_surplus;

/* shoot task global parameter */
shoot_t   shoot;
trigger_t trig;
maga_t maga = {0,41};


uint32_t shoot_time_last;
int shoot_time_ms;
void shoot_task(void const *argu)
{
  osEvent event;
  
  while (1)
  {
    event = osSignalWait(SHOT_TASK_EXE_SIGNAL, osWaitForever);
    
    if (event.status == osEventSignal)
    {
      if (event.value.signals & SHOT_TASK_EXE_SIGNAL)
      {
        shoot_time_ms = HAL_GetTick() - shoot_time_last;
        shoot_time_last = HAL_GetTick();
        
        fric_wheel_ctrl();
        
        if (!shoot.fric_wheel_run)
        {
          shoot.shoot_cmd   = 0;
          shoot.c_shoot_cmd = 0;
        }
        
#ifdef OLD_TRIGGER  
        /*
        if (shoot.fric_wheel_run)
        {
          if (glb_sw.last_sw1 == RC_DN)
            trig.pos_ref = moto_trigger.total_ecd;
          if (shoot.shoot_cmd)
          {
            trig.pos_ref = moto_trigger.total_ecd;
            trig.pos_ref += 130922 * trig.dir;
            shoot.shoot_cmd = 0;
          }
        
          pid_calc(&pid_trigger, moto_trigger.total_ecd / 100, trig.pos_ref / 100);
          
          if (shoot.c_shoot_cmd)
            trig.spd_ref = -4000;
          else
            trig.spd_ref = pid_trigger.out;
          
          block_bullet_handler();
          pid_calc(&pid_trigger_spd, moto_trigger.speed_rpm, trig.spd_ref);
        }
        else
        {
          pid_trigger_spd.out = 0;
        }
        */
#else
        
        trig.key = get_trigger_key_state_bypass();
        
        if (shoot.fric_wheel_run)
        {
          shoot_bullet_handler();
        }
        else
        {
          pid_trigger_spd.out = 0;
        }
        
        trig.key_last = trig.key;
#endif
      }
    }
    
    shoot_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
  }
}



/**
  * This function is to replace the get_trigger_key_state when the bullet_shot_detector is not fixed;
  * return either 0 or 1
	* return 1 if the bullet is sending into the chamber
	* return 0 otherwise
*/
float trigger_motor_rotation = 0.0f;
float trigger_motor_rot_last = 0.0f;
int32_t total_angle_last		 = 0;
uint8_t get_trigger_key_state_bypass(void)
{
	trigger_motor_rot_last = trigger_motor_rotation;
	trigger_motor_rotation += (moto_trigger.total_angle-total_angle_last)/36.0f;
	total_angle_last = moto_trigger.total_angle;
	trigger_motor_rotation = fmodf(trigger_motor_rotation,360);
	float bullet_passing_offset  = fmodf(trigger_motor_rotation, 45);//45 = 360/8
	if(bullet_passing_offset>=15 && bullet_passing_offset<30 && trigger_motor_rot_last != trigger_motor_rotation)
		/*bullet_passing_offset in between 15 and 30
			and the trigger motor is rotating now. 
		*/
		return 1;
	else
		return 0;
}
/*
 * This function deals with the situation when the trigger motor is blocked. 
 * When the output of trigger motor pid (i.e. the currency) is too high for 0.25s
 * which illustrates the motor is blocked or rotating too slow
 * the trigger motor will rotate backward for 0.25s. 
*/
void block_bullet_handler(void)
{
  static uint32_t stall_count = 0;
  static uint32_t stall_inv_count = 0;
  static uint8_t  stall_f = 0;
  
  if (pid_trigger_spd.out >= 5000)
  {
    if (stall_f == 0)
      stall_count ++;
  }
  else
    stall_count = 0;
  
  if (stall_count >= 250)         //0.25s
  {
    stall_f = 1;
    stall_count = 0;
  }
  
  if (stall_f == 1)
  {
    stall_inv_count++;
    
    if (stall_inv_count >= 250)  //0.25s
    {
      stall_f = 0;
      stall_inv_count = 0;
    }
    else
      trig.spd_ref = -2000;
  }
}

//Eric Edited
//Modified by Y H Liu @Apr 3rd, 2019
//not only for the friction wheels but magazine lid as well. Both are using PWM
static void fric_wheel_ctrl(void)
{
  if (shoot.fric_wheel_run)
  {
    turn_on_friction_wheel(shoot.fric_wheel_spd);
    turn_on_laser();
  }
  else
  {
    turn_off_friction_wheel();
    turn_off_laser();
  }
	//For magazine lid control
	if(maga.funct)
	{
		turn_on_magalid(maga.servo_debug);
	}
	else
	{
		turn_off_magalid();
	}
}


#if (INFANTRY_NUM == INFANTRY_1)
  int speed_debug = 200;

#elif (INFANTRY_NUM == INFANTRY_2)
  int speed_debug = 200;

#elif (INFANTRY_NUM == INFANTRY_3)
  int speed_debug = 120; //17.5

#elif (INFANTRY_NUM == INFANTRY_4)
  int speed_debug = 200;//16.5
  
#elif (INFANTRY_NUM == INFANTRY_5)
  int speed_debug = 200;//16.5
  
#elif (INFANTRY_NUM == INFANTRY_6)
  int speed_debug = 200;//15.5
  
#else
  #error "INFANTRY_NUM define error!"
  
#endif

int debug_tri_spd = 2000;
int debug_c_spd   = 2000;//2300;//2300//10

int shoot_cmd;
static void shoot_bullet_handler(void)
{
  shoot_cmd = shoot.shoot_cmd;
  if (shoot.shoot_cmd)
  {
    if (trig.one_sta == TRIG_INIT)
    {
      if (trig.key == 0)
      {
        trig.one_sta = TRIG_PRESS_DOWN;
        trig.one_time = HAL_GetTick();
      }
    }
    else if (trig.one_sta == TRIG_PRESS_DOWN)
    {
      if (HAL_GetTick() - trig.one_time >= 2000) //before the rising
      {
        trig.one_sta = TRIG_ONE_DONE;
      }

      if ((trig.key_last == 0) && (trig.key))    //Rising edge trigger button bounce
      {
        trig.one_sta = TRIG_BOUNCE_UP;
        trig.one_time = HAL_GetTick();
      }
    }
    else if (trig.one_sta == TRIG_BOUNCE_UP)
    {
      if (HAL_GetTick() - trig.one_time >= 2000)
      {
        trig.one_sta = TRIG_ONE_DONE;
      }
      
      if ((trig.key_last) && (trig.key == 0))    //Falling edge trigger button be press
      {
        trig.one_sta = TRIG_ONE_DONE;
      }
    }
    else
    {
    }
    
    if (trig.one_sta == TRIG_ONE_DONE)
    {
      trig.spd_ref = 0;
      trig.one_sta = TRIG_INIT;
      
      shoot.shoot_cmd = 0;
      shoot.shoot_bullets++;
    }
    else
      trig.spd_ref = debug_tri_spd;//trig.feed_bullet_spd;
    
  }
  else if (shoot.c_shoot_cmd)
  {
    trig.one_sta = TRIG_INIT;
    trig.spd_ref = debug_c_spd;//trig.c_shoot_spd;
    
    if ((trig.key_last == 0) && (trig.key == 1))
      shoot.shoot_bullets++;
    
    block_bullet_handler();
  }
  else
  {
    if (trig.key)       //not trigger
      trig.spd_ref = debug_tri_spd;//trig.feed_bullet_spd;
    else
      trig.spd_ref = 0;
  }
  
  pid_calc(&pid_trigger_spd, moto_trigger.speed_rpm, trig.spd_ref);
}

void shoot_param_init(void)
{
  memset(&shoot, 0, sizeof(shoot_t));
  
  shoot.ctrl_mode      = SHOT_DISABLE;
  shoot.fric_wheel_spd = DEFAULT_FRIC_WHEEL_SPEED;
  //shoot.remain_bullets = 0;
  
  memset(&trig, 0, sizeof(trigger_t));
  
  trig.dir             = 1;
  trig.feed_bullet_spd = 2000;
  trig.c_shoot_spd     = 4000;
  trig.one_sta         = TRIG_INIT;
	trig.key						 = 0;
  
}

