///
/// @file		BQ34110.h
/// @brief		Library header
/// @details	TI BQ34110 Driver
/// @n
/// @n @b		Project SMC
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Alex Bondarenko
/// @author		Alex Bondarenko
///
/// @date		2018-03-07 1:03 AM
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

#ifndef BQ34110____FILEEXTENSION___
///
/// @brief	Release
///
#define BQ34110____FILEEXTENSION___

#define REG_PIN_CONT
#define BIT_VEN_EN 4

#include "Wire.h"

class BQ34110
{
public:
    typedef enum
    {
        /* Page id register definition */
        BQ34110_Voltage          = 0X06,
        
    } BQ34110_reg_t;
    BQ34110();
    uint16_t getVoltage();
    int16_t getCurrent();
    uint16_t getChipTemp();
    uint16_t testDataWriteToFlash();
    bool begin(void);
    
    
private:
    byte  read8( BQ34110_reg_t reg);
    uint16_t read16(BQ34110_reg_t reg);
//    bool  readLen ( BQ34110_reg_t, byte* buffer, uint8_t len );
    bool  write8  ( BQ34110_reg_t, byte value );
    
    uint8_t _address;

};



#endif // BQ34110____FILEEXTENSION___
