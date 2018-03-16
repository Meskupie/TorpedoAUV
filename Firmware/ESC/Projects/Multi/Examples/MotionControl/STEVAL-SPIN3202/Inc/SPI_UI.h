/**
 ******************************************************************************
 * @file    SPI_UI.h
 * @author  IPC Rennes
 * @version V1.0.0
 * @date    April 24th, 2017
 * @brief   This file provides a set of functions needed to manage the UART com.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include "6Step_Lib.h"
#include "stdlib.h"
#include "stdio.h"
#include <string.h>






/** @addtogroup MIDDLEWARES     MIDDLEWARES 
  * @brief  Middlewares Layer
  * @{ 
  */


/** @addtogroup UART_UI  UART UI
  * @brief  Serial communication through PC serial terminal
  * @{ 
  */

/** @defgroup Exported_function_Uart  Exported_function_Uart
* @{
*/
void SPI_Communication_Task(void);
/** 
  * @brief  UART function  
  */

/**
  * @} 
  */
typedef enum
{
    ESC_CMD_Stop,
    ESC_CMD_Start,
    ESC_CMD_GetTemperature,
    ESC_CMD_GetRunState,
    ESC_CMD_GetCurrent,
    ESC_CMD_GetSpeed,
    ESC_CMD_SetSpeed,
    ESC_CMD_SetCurrent,
    ESC_CMD_SetDirection,
    ESC_CMD_SetAcceleration,
    ESC_CMD_SetKP,
    ESC_CMD_SetKI,
    ESC_CMD_SetKD,
		ESC_CMD_RESET,
    ESC_CMD_GetStatusStruct
} ESC_COMMAND;
typedef struct
{
    int32_t speedSetPoint : 32;
    int32_t speedMeasured : 32;
    int16_t currentMeasured :16;
    uint8_t temperature:8;
    SIXSTEP_Base_SystStatus_t runState : 7;
    uint8_t direction:1;
}ESC_StatusStruct;

typedef union
{
    ESC_StatusStruct statusStruct;
    uint8_t stuctRaw[sizeof(ESC_StatusStruct)];
} ESC_StatusStructUnion;


/**
  * @} 
  */

/**
  * @} 
  */
