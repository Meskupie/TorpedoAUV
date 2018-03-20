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
#include "debounce.h"



extern ESC_Struct ESC[];

// Define structures and classes

#define BAUD_RATE 115200

#define MIN_SPEED 500

#define TIMEOUT (1000)

#define MANUAL_CONTROL

#define BNO055_SAMPLERATE_DELAY_MS (100)





Adafruit_BNO055 IMU = Adafruit_BNO055();
BQ34110 gasGauge = BQ34110();

// Define variables and constants
int currChar = 0;



// Prototypes
// !!! Help: http://bit.ly/2l0ZhTa

#define PIN_REED_SW_FRONT   9
#define PIN_REED_SW_CENTER  10
#define PIN_REED_SW_REAR    5





// Utilities
debounce swFront;
debounce swCenter;
debounce swRear;





// Functions
void reedSwitchInit()
{
    swFront.initButton(PIN_REED_SW_FRONT,INPUT_PULLUP);
    swCenter.initButton(PIN_REED_SW_CENTER,INPUT_PULLUP);
    swRear.initButton(PIN_REED_SW_REAR,INPUT_PULLUP);
}


// Add setup code

void setup()
{
    // ESC Init
    ESC_init_all();
    // IMU Init
    IMU.begin();
    IMU.setExtCrystalUse(true);
    // Gas Guage init
    gasGauge.begin();
    
    // serial init
    Serial.begin(BAUD_RATE);
    Serial.println("Hello!");
    // reed swich Init
    reedSwitchInit();
    
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
#ifdef MANUAL_CONTROL
    printStatusStruct(ESC_Fast_COMM(&ESC[5]));
    delay(20);
    
#else
#endif


////    printESCState(ESCGetStatus(&ESC[0]));
    
    

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

    if(Serial.available())
    {
        readSerialCommand();
    }

    loopCount++;
    loopCount = loopCount%1000;
            
}
            
