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
/** @file info_get_task.c
 *  @version 1.0
 *  @date Oct 2017
 *
 *  @brief get infantry sensor and control information
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "info_get_task.h"
#include "comm_task.h"
#include "info_interactive.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "cmsis_os.h"

#define HERO_FRAME_JSDEBUG
/* stack usage monitor */
UBaseType_t info_stack_surplus;

/* information get task global parameter */
infantry_structure_t glb_struct;
#ifdef HERO
extern hero_frame frame_ctrl;
#ifdef HERO_FRAME_JSDEBUG
int hero_frame_ctrl_output_js = 0;
int hero_frame_ctrl_statusUP_js = 0;
int hero_frame_ctrl_statusDOWN_js = 0;
int hero_frame_ctrl_statusTOTOP_js = 0;
int hero_frame_ctrl_statusTOBOTTOM_js = 0;
int hero_frame_ctrl_statusFSM_js = 0;
int debug_la_cmd;//Put this in debug  watch window
#endif
#endif
uint32_t info_time_last;
int info_time_ms;
void info_get_task(void const *argu)
{
  osEvent event;
  
  while (1)
  {
    event = osSignalWait(INFO_GET_EXE_SIGNAL, osWaitForever);
    
    if (event.status == osEventSignal)
    {
      if (event.value.signals & INFO_GET_EXE_SIGNAL)
      {
        info_time_ms = HAL_GetTick() - info_time_last;
        info_time_last = HAL_GetTick();
        
        taskENTER_CRITICAL();
        
        keyboard_global_hook();
        
        get_chassis_info();
        get_gimbal_info();
        get_shoot_info();
				//Eric edited For hard ware testing.
				LA_Debug(debug_la_cmd);
				#ifdef HERO
					get_frame_info();
					send_linear_actuator_mesg(1);
					#ifdef HERO_FRAME_JSDEBUG
						hero_frame_ctrl_output_js = frame_ctrl.output;
						hero_frame_ctrl_statusUP_js = frame_ctrl.signal[UP];
						hero_frame_ctrl_statusDOWN_js = frame_ctrl.signal[DOWN];
						hero_frame_ctrl_statusTOTOP_js = frame_ctrl.signal[TOTOP];
						hero_frame_ctrl_statusTOBOTTOM_js = frame_ctrl.signal[TOBOTTOM];
						hero_frame_ctrl_statusFSM_js = frame_ctrl.status;
					#endif
				#endif
        get_global_last_info();
        
        taskEXIT_CRITICAL();

        chassis_position_measure();
      }
    }
    
    info_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
  }

//  uint32_t info_time = osKernelSysTick();
//  while (1)
//  {
//    info_time_ms = HAL_GetTick() - info_time_last;
//    info_time_last = HAL_GetTick();
//    
//    taskENTER_CRITICAL();
//    
//    keyboard_global_hook();
//    
//    get_chassis_info();
//    get_gimbal_info();
//    get_shoot_info();
//    
//    taskEXIT_CRITICAL();

//    chassis_position_measure();
//    
//    osDelayUntil(&info_time, INFO_GET_PERIOD);
//    
//  }
}

static void get_global_last_info(void)
{
  glb_sw.last_sw1 = rc.sw1;
  glb_sw.last_sw2 = rc.sw2;
  glb_sw.last_wh  = rc.ch7;
}

void LA_Debug(int cmd) //Bottom control test
{
	if(cmd==0) //Stop Motion
	{
		HAL_GPIO_WritePin(GPIOH, LA12_1_Pin|LA34_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOH, LA12_0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LA34_0_GPIO_Port, LA34_0_Pin, GPIO_PIN_SET);
	}
	else if(cmd==1) //Raise
	{
		HAL_GPIO_WritePin(GPIOH, LA12_1_Pin|LA34_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOH, LA12_0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LA34_0_GPIO_Port, LA34_0_Pin, GPIO_PIN_SET);
	}
	else if(cmd == 2) //Retraction
  {
		HAL_GPIO_WritePin(GPIOH, LA12_1_Pin|LA34_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOH, LA12_0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LA34_0_GPIO_Port, LA34_0_Pin, GPIO_PIN_RESET);
	}		
}

