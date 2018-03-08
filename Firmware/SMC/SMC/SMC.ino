///
/// @mainpage	SMC
///
/// @details	Description of the project
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Alex Bondarenko
/// @author		Alex Bondarenko
/// @date		2018-02-28 5:47 PM
/// @version	<#version#>
///
/// @copyright	(c) Alex Bondarenko, 2018
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
///


///
/// @file		SMC.ino
/// @brief		Main sketch
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Alex Bondarenko
/// @author		Alex Bondarenko
/// @date		2018-02-28 5:47 PM
/// @version	<#version#>
///
/// @copyright	(c) Alex Bondarenko, 2018
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
/// @n
///


// Core library for code-sense - IDE-based
// !!! Help: http://bit.ly/2AdU7cu
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
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
#include "application.h"
#elif defined(ESP8266) // ESP8266 specific
#include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#else // error
#error Platform not defined
#endif // end IDE

// Set parameters


// Include application, user and local libraries
#include "ESC.h"
#include "SMC_SERIAL.h"
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BNO055.h"
#include <utility/imumaths.h>
#include "BQ34110.h"



extern ESC_Struct ESC[];

// Define structures and classes

#define BAUD_RATE 115200

#define MIN_SPEED 500

#define TIMEOUT (1000)

#define MANUAL_CONTROL

#define BNO055_SAMPLERATE_DELAY_MS (100)


Adafruit_BNO055 bno = Adafruit_BNO055();
BQ34110 gasGauge = BQ34110();

// Define variables and constants
int currChar = 0;

String constantlySend = "";


// Prototypes
// !!! Help: http://bit.ly/2l0ZhTa


// Utilities


// Functions



// Add setup code

void setup()
{
    ESC_init_all();
    bno.begin();
    bno.setExtCrystalUse(true);
//    bno.setMode()
    Serial.begin(BAUD_RATE);
    Serial.println("Hello!");
    gasGauge.begin();
 
    
}
ESC_RUN_STATE lastState = STOP;
ESC_RUN_STATE newState =STOP;
ESC_Struct* testEsc = &ESC[0];
// Add loop code

int loopCount = 0;
char frontDelimiter = '<';
char endDelimiter = '>';
char jsonData[255];

int16_t thrusterSpeed[6];

unsigned long lastTime = 0;

void loop()
{
    


//    printESCState(ESCGetStatus(&ESC[0]));
    printStatusStruct(ESCGetStatusStruct(&ESC[3]));
    
//    Serial.print("ver: ");
//    Serial.println(gasGauge.testDataWriteToFlash());

//    int8_t temp = bno.getTemp();
//    Serial.print("Current Temperature: ");
//    Serial.print(temp);
//    Serial.println(" C");
//    Serial.println("");
//
//   imu::Quaternion quat = bno.getQuat();
////
//    Serial.print("quat Size: ");
//    Serial.println(sizeof(quat));
//
//    Serial.print("qW: ");
//    Serial.print(quat.w(), 4);
//    Serial.print(" qX: ");
//    Serial.print(quat.y(), 4);
//    Serial.print(" qY: ");
//    Serial.print(quat.x(), 4);
//    Serial.print(" qZ: ");
//    Serial.print(quat.z(), 4);
//    Serial.print("\n");
//
#ifdef MANUAL_CONTROL
    if(Serial.available())
    {
        readSerialCommand();
    }

    if(loopCount%500==0)
    {
        newState = ESCGetStatus(testEsc);
        if (newState!=lastState)
        {
            lastState = newState;
            Serial.print("new State = ");
            printESCState(newState);
        }
    }
#else
    if(Serial.available() > 0) {
        currChar = Serial.read();

        if(currChar == frontDelimiter) {
            byte length = Serial.readBytesUntil(endDelimiter, jsonData, sizeof(jsonData));

            char sub[length+1];
            memcpy( sub, &jsonData[0], length);
            sub[length] = '\0';
            StaticJsonBuffer<200> jsonBuffer;
            JsonObject& root = jsonBuffer.parseObject(jsonData);
            if (!root.success()) {
                return;
            }
            thrusterSpeed[0]= root["motorValues"]["FL"];
            thrusterSpeed[1]= root["motorValues"]["FR"];
            thrusterSpeed[2]= root["motorValues"]["FV"];
            thrusterSpeed[3]= root["motorValues"]["BL"];
            thrusterSpeed[4]= root["motorValues"]["BR"];
            thrusterSpeed[5]= root["motorValues"]["BV"];
            lastTime = millis();
        }
    }
    
    
    for (int i = 0; i<5;i++)
    {
        if (TIMEOUT>(lastTime-millis()) {
            ESCSetSpeed(&ESC[i], thrusterSpeed[i]);
        }
        else
        {
            ESCStop(&ESC[i]);
        }
        
    }
            
            
    
#endif
    loopCount++;
    loopCount = loopCount%1000;
    delay(100);
    
}
