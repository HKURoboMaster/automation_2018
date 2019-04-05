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
/** @file keyboard.c
 *  @version 1.1
 *  @date June 2017
 *
 *  @brief keyboard message handle
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */
 
#include "keyboard.h"
#include "bsp_uart.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "info_get_task.h"
#include "modeswitch_task.h"
#include "ramp.h"
#include "remote_ctrl.h"
#include "cmsis_os.h"
#include "sys_config.h"

/* mouse button long press time */
#define LONG_PRESS_TIME  1000  //ms
/* key acceleration time */
#define KEY_ACC_TIME     1500  //ms

kb_ctrl_t km;

ramp_t fb_ramp = RAMP_GEN_DAFAULT;
ramp_t lr_ramp = RAMP_GEN_DAFAULT;


void key_fsm(kb_state_e *sta, uint8_t key)
{
  switch (*sta)
  {
    case KEY_RELEASE:
    {
      if (key)
        *sta = KEY_WAIT_EFFECTIVE;
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_WAIT_EFFECTIVE:
    {
      if (key)
        *sta = KEY_PRESS_ONCE;
      else
        *sta = KEY_RELEASE;
    }break;
    
    
    case KEY_PRESS_ONCE:
    {
      if (key)
      {
        *sta = KEY_PRESS_DOWN;
        if (sta == &km.lk_sta)
          km.lk_cnt = 0;
        else
          km.rk_cnt = 0;
      }
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_PRESS_DOWN:
    {
      if (key)
      {
        if (sta == &km.lk_sta)
        {
          if (km.lk_cnt++ > LONG_PRESS_TIME/INFO_GET_PERIOD)
            *sta = KEY_PRESS_LONG;
        }
        else
        {
          if (km.rk_cnt++ > LONG_PRESS_TIME/INFO_GET_PERIOD)
            *sta = KEY_PRESS_LONG;
        }
      }
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_PRESS_LONG:
    {
      if (!key)
      {
        *sta = KEY_RELEASE;
      }
    }break;
    
    default:
    break;
      
  }
}
extern hero_frame frame_ctrl;
static void move_spd_ctrl(uint8_t fast, uint8_t slow)
{
  if ((frame_ctrl.status==BOTTOM_STAY || frame_ctrl.status==OFF) && fast)
  {
		/*In hero, once the framework is raised, high speed mode is prohibited*/
    km.move = FAST_MODE;
    km.x_spd_limit = CHASSIS_KB_MAX_SPEED_X;
    km.y_spd_limit = CHASSIS_KB_MAX_SPEED_Y;
  }
  else if (slow)
  {
    km.move = SLOW_MODE;
    km.x_spd_limit = 0.5f * CHASSIS_KB_MAX_SPEED_X;
    km.y_spd_limit = 0.5f * CHASSIS_KB_MAX_SPEED_Y;
  }
  else
  {
    km.move = NORMAL_MODE;
    km.x_spd_limit = 0.7f * CHASSIS_KB_MAX_SPEED_X;
    km.y_spd_limit = 0.7f * CHASSIS_KB_MAX_SPEED_Y;
  }
}

static void move_direction_ctrl(uint8_t forward, uint8_t back,
                                uint8_t left,    uint8_t right)
{
  //add ramp
  if (forward)
  {
    km.vx = km.x_spd_limit * ramp_calc(&fb_ramp);
  }
  else if (back)
  {
    km.vx = -km.x_spd_limit * ramp_calc(&fb_ramp);
  }
  else
  {
    km.vx = 0;
    ramp_init(&fb_ramp, KEY_ACC_TIME/INFO_GET_PERIOD);
  }

  if (left)
  {
    km.vy = km.y_spd_limit * ramp_calc(&lr_ramp);
  }
  else if (right)
  {
    km.vy = -km.y_spd_limit * ramp_calc(&lr_ramp);
  }
  else
  {
    km.vy = 0;
    ramp_init(&lr_ramp, KEY_ACC_TIME/INFO_GET_PERIOD);
  }
  
  //if (forward || back || left || right)
    //km.twist_ctrl = 0;
	/*
	delete the above two statements because the moving while dodging
	has been implemented in chassis_task.c already
	*/
}

static void chassis_operation_func(uint8_t twist_chassis)
{
  if (twist_chassis)
    km.twist_ctrl = km.twist_ctrl?0:1;
}


static void kb_fric_ctrl(uint8_t trig_fric)
{
  if (trig_fric)
    shoot.fric_wheel_run = shoot.fric_wheel_run?0:1;
}

static void kb_shoot_cmd(uint8_t single_fir, uint8_t cont_fir)
{
  if (single_fir)
  {
    shoot.shoot_cmd   = 1;
    shoot.c_shoot_cmd = 0;
  }
  
  if (cont_fir)
  {
    shoot.shoot_cmd   = 0;
    shoot.c_shoot_cmd = 1;
  }
  else
    shoot.c_shoot_cmd = 0;

}
static void gimbal_operation_func(int16_t pit_ref_spd, int16_t yaw_ref_spd,
                                  uint8_t shoot_buff,  uint8_t track_armor)
{
  km.pit_v = -pit_ref_spd * 0.01f;
  km.yaw_v = -yaw_ref_spd * 0.01f;
  
  
  
  if (shoot_buff)
    km.buff_ctrl = 1;
  
  if (track_armor)
    km.track_ctrl = 1;
  else
    km.track_ctrl = 0;

}

static void exit_buff_hook(uint8_t forward, uint8_t back,
                           uint8_t left,    uint8_t right)
{
  if (forward || back || left || right)
    km.buff_ctrl = 0;
}

void keyboard_global_hook(void)
{
  if (km.kb_enable)
  {
    key_fsm(&km.lk_sta, rc.mouse.l);
    key_fsm(&km.rk_sta, rc.mouse.r);
  }
}


void keyboard_chassis_hook(void)
{
  if (km.kb_enable)
  {
    move_spd_ctrl(FAST_SPD, SLOW_SPD);
    
    move_direction_ctrl(FORWARD, BACK, LEFT, RIGHT);
    
    chassis_operation_func(TWIST_CTRL);
  }
  else
  {
    km.vx = 0;
    km.vy = 0;
    km.twist_ctrl = 0;
  }
}

void keyboard_gimbal_hook(void)
{
  if (km.kb_enable)
  {
		/*Added by Y. H. Liu*/
		/*For keyboard controlling rotation*/
		int16_t mouse_key_yaw_ref = rc.mouse.x + (CW-CCW)*GIMBAL_KB_MOVE_CONST_YAW;
    gimbal_operation_func(rc.mouse.y, mouse_key_yaw_ref, BUFF_CTRL, TRACK_CTRL);
    /*End of modification*/
    exit_buff_hook(FORWARD, BACK, LEFT, RIGHT);
  }
  else
  {
    km.pit_v = 0;
    km.yaw_v = 0;
    km.buff_ctrl = 0;
    km.track_ctrl = 0;
  }
}

static void kb_magalid_ctrl(uint8_t open_mega)
{
	if(open_mega)
		maga.funct = maga.funct?0:1;
}

void keyboard_maga_hook(void)
{
	kb_magalid_ctrl(KB_TURN_ON_MAGALID);
}

void keyboard_shoot_hook(void)
{
  //friction wheel control
  kb_fric_ctrl(KB_TRIG_FRIC_WHEEL);
  //single or continuous trigger bullet control
  kb_shoot_cmd(KB_SINGLE_SHOOT, KB_CONTINUE_SHOOT);
}
void keyboard_hero_frame()
{
	/*signal[0][1] dealler*/
	if(KB_HERO_FRAME_CONT)
	{
		if(frame_ctrl.status == BOTTOM_STAY)
		{
			frame_ctrl.signal[TOTOP] = 1;
			frame_ctrl.signal[TOBOTTOM] = 0;
		}
		else
		{
			frame_ctrl.signal[TOBOTTOM] = 1;
			frame_ctrl.signal[TOTOP] = 0;
		}
	}
	else
	{
		frame_ctrl.signal[TOBOTTOM] = 0;
		frame_ctrl.signal[TOTOP] = 0;
	}
	/*signal[2][3] dealler*/
	if(KB_HERO_FRAME_DOWN)
	{
		frame_ctrl.signal[UP] = 0;
		frame_ctrl.signal[DOWN] = 1;
	}
	else if(KB_HERO_FRAME_UP)
	{
		frame_ctrl.signal[UP] = 1;
		frame_ctrl.signal[DOWN] = 0;
	}
	else
	{
		frame_ctrl.signal[DOWN] = 0;
		frame_ctrl.signal[UP] = 0;
	}
	/*Make the signal[0][1]: mutually exclusive*/
	/*Make the signal[2][3]: mutually exclusive*/
	if(frame_ctrl.signal[UP] && frame_ctrl.signal[DOWN])
	{
		frame_ctrl.signal[UP] = 0;
	}
	if(frame_ctrl.signal[TOTOP] && frame_ctrl.signal[TOBOTTOM])
	{
		frame_ctrl.signal[TOTOP] = 0;
	}
}
