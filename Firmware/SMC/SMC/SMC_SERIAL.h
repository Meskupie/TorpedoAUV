///
/// @file		SMC_SERIAL.h
/// @brief		Library header
/// @details	Description of the file or library
/// @n
/// @n @b		Project SMC
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Alex Bondarenko
/// @author		Alex Bondarenko
///
/// @date		2018-03-01 9:25 PM
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

#include "ESC.h"
#include "BQ34110.h"
#include "Adafruit_BNO055.h"
#include "debounce.h"
#ifndef SMC_SERIAL____FILEEXTENSION___
///
/// @brief	Release
///
#define SMC_SERIAL____FILEEXTENSION___

extern BQ34110 gasGauge;
extern Adafruit_BNO055 IMU;
extern debounce swFront;
extern debounce swCenter;
extern debounce swRear;

typedef enum
{
    System_Idle,                               /* 0 */
    System_Startup,                            /* 1 */
    System_Fault
} SystemRunState;

typedef struct
{
    uint16_t battVoltage_mV:16;
    uint16_t depth_m : 16;
    int16_t imu_x : 16;
    int16_t imu_y : 16;
    int16_t imu_z : 16;
    int16_t imu_w : 16;
    int16_t motorThrust0_mN : 16;
    int16_t motorThrust1_mN : 16;
    int16_t motorThrust2_mN : 16;
    int16_t motorThrust3_mN : 16;
    int16_t motorThrust4_mN : 16;
    int16_t motorThrust5_mN : 16;
    uint8_t battSOC :8;
    uint8_t battSOP :8;
    int16_t battCurrent_mA :16;
    uint8_t ambientTemperature_C:8;
    ESC_RUN_STATE motorStatus0 : 4;
    ESC_RUN_STATE motorStatus1 : 4;
    ESC_RUN_STATE motorStatus2 : 4;
    ESC_RUN_STATE motorStatus3 : 4;
    ESC_RUN_STATE motorStatus4 : 4;
    ESC_RUN_STATE motorStatus5 : 4;
    SystemRunState SMC_Status : 3;
    uint8_t swStateFront : 1;
    uint8_t swStateCenter : 1;
    uint8_t swStateRear : 1;
    uint8_t _filler:2;
}hostUpdateStruct_transmit_t; //length 33 bytes
typedef union
{
    hostUpdateStruct_transmit_t statusStruct;
    uint8_t stuctRaw[sizeof(statusStruct)];
} hostUpdateStruct_transmit_union;

typedef struct
{
    int16_t motorThrust0_mN : 16;
    int16_t motorThrust1_mN : 16;
    int16_t motorThrust2_mN : 16;
    int16_t motorThrust3_mN : 16;
    int16_t motorThrust4_mN : 16;
    int16_t motorThrust5_mN : 16;
    uint8_t count            : 4;
    uint8_t checkSum         : 4;
} hostUpdateStruct_receive_t; //length 13 bytes
typedef union
{
    hostUpdateStruct_receive_t motorStruct;
    uint8_t stuctRaw[sizeof(motorStruct)];
} hostUpdateStruct_receive_union;


bool rxCheckSum(hostUpdateStruct_receive_union input);
int16_t castQuatToUint16(float quatVector);
void printESCState(int state);
void printStatusStruct(ESC_StatusStruct printStruct);
long readIntegerSerial(void);
void readSerialCommand(void);


#endif // SMC_SERIAL____FILEEXTENSION___
