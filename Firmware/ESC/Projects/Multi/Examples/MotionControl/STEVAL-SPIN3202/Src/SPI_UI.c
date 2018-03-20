/**
 ******************************************************************************
 * @file    SPI_UI.c
 * @author  Alex Bondarenko
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

/* Includes ------------------------------------------------------------------*/
#include "SPI_UI.h"


#define FASTCOMM
#define WATCHDOG

#ifdef FASTCOMM
#define COMM_KEY  (0x5f49fbf9)
#define RX_BUFFER_SIZE 96
#else
#define RX_BUFFER_SIZE 3
#endif
#ifdef ROV

extern SPI_HandleTypeDef hspi1;
extern IWDG_HandleTypeDef hiwdg;

extern SIXSTEP_Base_InitTypeDef SIXSTEP_parameters;      /*!< Main SixStep structure*/
extern SIXSTEP_PI_PARAM_InitTypeDef_t PI_parameters;     /*!< SixStep PI regulator structure*/
extern void CMD_Parser(char* pCommandString);
extern void MC_Set_PI_param(SIXSTEP_PI_PARAM_InitTypeDef_t *);
extern void MC_StartMotor(void);
extern void MC_StopMotor(void);


uint8_t SPI_Receive_Buffer[RX_BUFFER_SIZE];
uint16_t new16BitValue;
uint16_t current_TEST = 101;
uint16_t tempSpeed;
ESC_StatusStructUnion statusStructUnion;
ESC_CommandStructUnion commandStructUnion;
uint8_t currentlyRunning = 0;

uint8_t SPI_Recived_DUMP[20];

int sendError;

/** @defgroup UART_Communication_Task    UART_Communication_Task
 *  @{
 * @brief UART start receive function
 */
void SPI_Send_16BIT(uint16_t data)
{
    uint8_t txData[2];
    txData[0] = (uint8_t)(data>>8);
    txData[1] = (uint8_t)data& 0x0f;
    HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)txData,SPI_Recived_DUMP,2,100);
}

int16_t speedToThrust(int16_t speed)
{
	int16_t thrust;
	double speedDub = speed;
	speedDub *= 0.00096416;
	speedDub *= speedDub;
	thrust =  (int16_t)(0.0928 * (speedDub));
	return thrust;
}

#ifdef FASTCOMM
void SPI_Communication_Task()
{
    statusStructUnion.statusStruct.speedSetPoint = PI_parameters.Reference;
    statusStructUnion.statusStruct.speedMeasured = SIXSTEP_parameters.speed_fdbk_filtered;
    statusStructUnion.statusStruct.runState = SIXSTEP_parameters.STATUS;
    statusStructUnion.statusStruct.direction = SIXSTEP_parameters.CW_CCW;
	#ifndef VOLTAGE_MODE
		statusStructUnion.statusStruct.currentMeasured = SIXSTEP_parameters.current_reference;
	#endif
    HAL_SPI_TransmitReceive_DMA(&hspi1,statusStructUnion.stuctRaw,commandStructUnion.stuctRaw,sizeof(statusStructUnion));
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX_RX);
    
    if (commandStructUnion.commandStruct.comms_key == COMM_KEY)
    {
#ifdef WATCHDOG
        if(HAL_IWDG_GetState(&hiwdg) == HAL_IWDG_STATE_READY)
        {
            HAL_IWDG_Start(&hiwdg);
            if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)==1)
            {
                SIXSTEP_parameters.STATUS = WD_RESET;
            }
        }
        else
        {
            HAL_IWDG_Refresh(&hiwdg);
        }
#endif
        if(currentlyRunning != commandStructUnion.commandStruct.state)
        {
            if(commandStructUnion.commandStruct.state ==1)
            {
                MC_StartMotor();
                currentlyRunning = 1;
                MC_Set_Thrust(commandStructUnion.commandStruct.thrust_mN);
            }
            else if (commandStructUnion.commandStruct.state ==0)
            {
                MC_StopMotor();
                currentlyRunning = 0;
            }
        }
        else
        {
            MC_Set_Thrust(commandStructUnion.commandStruct.thrust_mN);
        }
    }
    else
    {
        HAL_SPI_FlushRxFifo(&hspi1);
    }
}
#else
void SPI_Communication_Task()
{

   while(__HAL_SPI_GET_FLAG(&hspi1,SPI_FLAG_RXNE)==1) // wait for new data
	 {
		 if(__HAL_SPI_GET_FLAG(&hspi1,SPI_FLAG_OVR) == 1)
		 {
			 __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
		 }
	 }
    switch(HAL_SPI_Receive(&hspi1,(uint8_t*)SPI_Receive_Buffer,RX_BUFFER_SIZE,500))
    {
        case HAL_OK:
  
            switch (SPI_Receive_Buffer[0]) {
								case ESC_CMD_COM_SYNC_ERROR:
									while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
									{
									}
									//HAL_Delay(20);
									HAL_SPI_FlushRxFifo(&hspi1);
								break;
                case ESC_CMD_Stop:
                    MC_StopMotor();
                    break;
                case ESC_CMD_Start:
                    MC_StartMotor();
                    break;
								
                    //                case ESC_CMD_GetTemperature:
                    //                    //HAL_SPI_Transmit(&hspi1, (uint8_t*)temperature,1,1000);
                    
                    //                    break;
                case ESC_CMD_GetRunState:
                    HAL_SPI_TransmitReceive(&hspi1,&SIXSTEP_parameters.STATUS,SPI_Recived_DUMP,1,100);
                    
                    break;
                case ESC_CMD_GetCurrent:
                    //SPI_Send_16BIT(SIXSTEP_parameters.ADC_SEQ_Buffer[ADC_CH_2]);
                    break;
                case ESC_CMD_GetSpeed:
                    SPI_Send_16BIT(SIXSTEP_parameters.speed_fdbk_filtered);
                    break;
                case ESC_CMD_SetSpeed:
                    new16BitValue = SPI_Receive_Buffer[1];
                    new16BitValue = new16BitValue<<8;
                    new16BitValue = new16BitValue | SPI_Receive_Buffer[2];
                    MC_Set_Speed(new16BitValue);
                    break;
							  case ESC_CMD_SetThrust:
                    new16BitValue = SPI_Receive_Buffer[1];
                    new16BitValue = new16BitValue<<8;
                    new16BitValue = new16BitValue | SPI_Receive_Buffer[2];
                    MC_Set_Thrust(new16BitValue);
                    break;
								case ESC_CMD_GetThrust:
                    SPI_Send_16BIT(speedToThrust(SIXSTEP_parameters.speed_fdbk_filtered));
                    break;
                case ESC_CMD_SetCurrent:
                    new16BitValue = SPI_Receive_Buffer[1];
                    new16BitValue = new16BitValue<<8;
                    new16BitValue = new16BitValue | SPI_Receive_Buffer[2]; 
                    break;
								
                case ESC_CMD_SetDirection:
                    SIXSTEP_parameters.CW_CCW = SPI_Receive_Buffer[1];
                    
                    MC_Set_PI_param(&PI_parameters);
                    
                    break;
                case ESC_CMD_SetAcceleration:
                    new16BitValue = SPI_Receive_Buffer[1];
                    new16BitValue = new16BitValue<<8;
                    new16BitValue = new16BitValue | SPI_Receive_Buffer[2];
                    SIXSTEP_parameters.ACCEL = new16BitValue;
                    
                    break;
                case ESC_CMD_SetKP:
                    new16BitValue = SPI_Receive_Buffer[1];
                    new16BitValue = new16BitValue<<8;
                    new16BitValue = new16BitValue | SPI_Receive_Buffer[2];
                    PI_parameters.Kp_Gain = new16BitValue;
                    
                    break;
                case ESC_CMD_SetKI:
                    new16BitValue = SPI_Receive_Buffer[1];
                    new16BitValue = new16BitValue<<8;
                    new16BitValue = new16BitValue | SPI_Receive_Buffer[2];
                    PI_parameters.Ki_Gain = new16BitValue;
                    
                    break;
                    //                case ESC_CMD_SetKD:
                    //                    new16BitValue = SPI_Receive_Buffer[1];
                    //                    new16BitValue = new16BitValue<<8;
                    //                    new16BitValue = new16BitValue | SPI_Receive_Buffer[2];
                    
                    //                    break;
                case ESC_CMD_RESET:
                    MC_SixStep_RESET();
                    
                    break;
                    
                case ESC_CMD_GetStatusStruct:
//										if(HAL_IWDG_GetState(&hiwdg) == HAL_IWDG_STATE_READY)
//										{
//											HAL_IWDG_Start(&hiwdg);
//											
//											if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)==1)
//												{
//													SIXSTEP_parameters.STATUS = WD_RESET;
//												}
//										}
//										else
//										{
//											HAL_IWDG_Refresh(&hiwdg);
//										}
                    statusStructUnion.statusStruct.speedSetPoint = PI_parameters.Reference;
                    statusStructUnion.statusStruct.speedMeasured = SIXSTEP_parameters.speed_fdbk_filtered;
                    statusStructUnion.statusStruct.runState = SIXSTEP_parameters.STATUS;
                    statusStructUnion.statusStruct.direction = SIXSTEP_parameters.CW_CCW;
								
                    //statusStructUnion.statusStruct.currentMeasured = SIXSTEP_parameters.ADC_SEQ_Buffer[ADC_CH_2];
										//statusStructUnion.statusStruct.currentMeasured = SIXSTEP_parameters.current_reference;
                    //                                        statusStructUnion.statusStruct.speedSetPoint = sizeof(statusStructUnion.statusStruct);
                    //                                        statusStructUnion.statusStruct.speedMeasured = sizeof(statusStructUnion);
                    //                                        statusStructUnion.statusStruct.runState = SIXSTEP_parameters.STATUS;
                    //                                        statusStructUnion.statusStruct.direction = 1;
                    
                   //HAL_SPI_Transmit(&hspi1,statusStructUnion.stuctRaw,sizeof(statusStructUnion),10);
									 
										
									 //HAL_SPI_Transmit_DMA(&hspi1,statusStructUnion.stuctRaw,sizeof(statusStructUnion));
									 //HAL_SPI_TransmitReceive_DMA(&hspi1,statusStructUnion.stuctRaw,SPI_Recived_DUMP,sizeof(statusStructUnion));
									 //HAL_SPI_TransmitReceive_DMA(&hspi1,statusStructUnion.stuctRaw,SPI_Recived_DUMP,sizeof(statusStructUnion));
									 HAL_SPI_TransmitReceive(&hspi1,statusStructUnion.stuctRaw,SPI_Recived_DUMP,sizeof(statusStructUnion),50);
									 //HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)statusStructUnion.stuctRaw,(uint8_t *)SPI_Recived_DUMP,sizeof(statusStructUnion),5);
									 while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
									{
									}
									HAL_SPI_FlushRxFifo(&hspi1);
                    break;
                    
                default:
                    
                    break;
            }
            //HAL_UART_Transmit(&huart2, (uint8_t*)aRxBuffer, RX_BUFFER_SIZE, 0xffffff); // send message via UART
            //my_printf("\n");
            break;
        case HAL_TIMEOUT:
					HAL_SPI_FlushRxFifo(&hspi1);
				
            break;
            
        case HAL_ERROR:
            /* Call Timeout Handler */
            
            //Error_Handler();
            break;
        default:
            break;
    }
    
}
#endif

#endif

