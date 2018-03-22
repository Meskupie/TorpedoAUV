///
/// @file		ESC.h
/// @brief		Library header
/// @details	ST spin driver
/// @n
/// @n @b		Project SMC
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Alex Bondarenko
/// @author		Alex Bondarenko
///
/// @date		2018-02-28 5:57 PM
/// @version	<#version#>
///
/// @copyright	(c) Alex Bondarenko, 2018
/// @copyright	<#licence#>
///
/// @see		ReadMe.txt for references
///


// Core library for code-sense - IDE-based
// Help: http://bit.ly/2AdU7cu
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"
#elif defined(ROBOTIS) // Robotis specific
#include "libpandora_types.h"
#include "pandora.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
#include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
#include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#include "Arduino.h"
#elif defined(RFDUINO) // RFduino specific
#include "Arduino.h"
#elif defined(SPARK) // Spark specific
#include "application.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#else // error
#error Platform not defined
#endif // end IDE

#ifndef ESC____FILEEXTENSION___
///
/// @brief	Release
///
#define ESC____FILEEXTENSION___





#include "SPI.h"
#define FASTCOMM

#define COMM_KEY  (0x5f49fbf9)

#define ESC_PIN_FR A5
#define ESC_PIN_RR A4
#define ESC_PIN_FC A3
#define ESC_PIN_RC A2
#define ESC_PIN_FL A1
#define ESC_PIN_RL A0

#define ESC_INDEX_FR 0
#define ESC_INDEX_RR 1
#define ESC_INDEX_FC 2
#define ESC_INDEX_RC 3
#define ESC_INDEX_FL 4
#define ESC_INDEX_RL 5

#define ESC_DIRECTION_FR (-1) //1 for forward -1 for reverse
#define ESC_DIRECTION_RR (-1)
#define ESC_DIRECTION_FC (-1)
#define ESC_DIRECTION_RC (-1)
#define ESC_DIRECTION_FL (-1)
#define ESC_DIRECTION_RL (-1)

#define ESC_DEADBAND_FR (250) //1 for forward -1 for reverse
#define ESC_DEADBAND_RR (250)
#define ESC_DEADBAND_FC (250)
#define ESC_DEADBAND_RC (250)
#define ESC_DEADBAND_FL (250)
#define ESC_DEADBAND_RL (250)

#define ESC_DEADBAND_U (250)
#define ESC_DEADBAND_L (50)






typedef enum
{
    ESC_CMD_COM_SYNC_ERROR,
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
    ESC_CMD_GetStatusStruct,
    ESC_CMD_GetThrust,
    ESC_CMD_SetThrust
} ESC_COMMAND;

typedef enum
{
    COMM_FAILURE,                       /* 0 */
    IDLE,                               /* 1 */
    STARTUP,                            /* 2 */
    VALIDATION,                         /* 3 */
    STOP,                               /* 4 */
    START,                              /* 5 */
    RUN,                                /* 6 */
    ALIGNMENT,                          /* 7 */
    SPEEDFBKERROR,                      /* 8 */
    OVERCURRENT,                        /* 9 */
    STARTUP_FAILURE,                    /* 10 */
    STARTUP_BEMF_FAILURE,               /* 11 */
    LF_TIMER_FAILURE,                   /* 12 */
    WD_RESET                            /* 13 */
} ESC_RUN_STATE;

typedef struct
{
    uint8_t pin;
    uint16_t deadband;
    int16_t speedSetPoint;
    ESC_RUN_STATE runState;
    int16_t thrustSetPoint_mN;
    int16_t thrustMeasured_mN;
    int32_t speedMeasured;
    int16_t currentMeasured;
    uint8_t temperature;
    uint8_t direction;
    int8_t flipMotor;
}ESC_Struct;

//typedef struct
//{
//    int32_t speedSetPoint : 32;
//    int32_t speedMeasured : 32;
//    int16_t currentMeasured :16;
//    uint8_t temperature:8;
//    ESC_RUN_STATE runState : 7;
//    uint8_t direction:1;
//}ESC_StatusStruct;

typedef struct
{
    int16_t speedSetPoint_rpm : 16;
    int16_t speedMeasured_rpm : 16;
    int16_t thrustMeasured_mN : 16;
    int16_t currentMeasured_mA :16;
    uint8_t temperature:8;
    ESC_RUN_STATE runState : 7;
    uint8_t direction:1;
}ESC_StatusStruct;
typedef struct
{
    uint16_t thrust_mN:16;
    uint8_t state:8;
    uint8_t thrusterType :8;
    uint32_t comms_key :32;
    uint32_t filler2 :32;
}ESC_CommandStruct;
typedef union
{
    ESC_CommandStruct commandStruct;
    uint8_t stuctRaw[sizeof(ESC_CommandStruct)];
} ESC_CommandStructUnion;




typedef union
{
    ESC_StatusStruct statusStruct;
    uint8_t stuctRaw[sizeof(ESC_StatusStruct)];
} ESC_StatusStructUnion;



ESC_StatusStruct ESC_Fast_COMM(ESC_Struct* ESC_hande);

unsigned int ESC_update_all();
unsigned int ESC_init(ESC_Struct* ESC_hande,int pin,int8_t flip,int16_t deadband);
unsigned int ESC_init_all(void);
unsigned int ESC_check_all();
unsigned int ESCStart(ESC_Struct* ESC_hande);
unsigned int ESCStop(ESC_Struct* ESC_hande);
unsigned int ESC_Stop_all();
unsigned int ESCSetSpeed(ESC_Struct* ESC_hande,int16_t speed);
unsigned int ESCSetDirection(ESC_Struct* ESC_hande,uint8_t direction);
unsigned int ESCSetAcceleration(ESC_Struct* ESC_hande,int16_t acceleration);
unsigned int ESCGetStatus(ESC_Struct* ESC_hande);
int16_t ESCGetSpeed(ESC_Struct* ESC_hande);
uint16_t ESCGetCurrent(ESC_Struct* ESC_hande);
unsigned int ESCReset(ESC_Struct* ESC_hande);
ESC_StatusStruct ESCGetStatusStruct(ESC_Struct* ESC_hande);

int16_t ESCGetThrust(ESC_Struct* ESC_hande);
unsigned int ESCSetThrust(ESC_Struct* ESC_hande,int16_t thrustSetPoint_mN);


#endif // ESC____FILEEXTENSION___



