//
// ESC.cpp
// Library C++ code
// ----------------------------------
// Developed with embedXcode+
// http://embedXcode.weebly.com
//
// Details	    ST spin driver
// Project 		SMC
//
// Created by 	Alex Bondarenko, 2018-02-28 5:57 PM
// 				Alex Bondarenko
//
// Copyright 	(c) Alex Bondarenko, 2018
// Licence		<#licence#>
//
// See 			ESC.h and ReadMe.txt for references
//

#define CW 0
#define CCW 1
#define SPI_COMMS_DELAY_MICROSECONDS 1000

#define MINIMUM_SPEED (500)
#define MINIMUM_THRUST (50)

// Library header
#include "ESC.h"
#include "SPI.h"



ESC_Struct ESC[6];

unsigned int ESC_init(ESC_Struct* ESC_hande,int pin);
unsigned int ESC_init_all(void);
unsigned int ESCStart(ESC_Struct* ESC_hande);
unsigned int ESCStop(ESC_Struct* ESC_hande);
unsigned int ESCSetSpeed(ESC_Struct* ESC_hande,int16_t speed);
unsigned int ESCSetDirection(ESC_Struct* ESC_hande,uint8_t direction);
unsigned int ESCSetAcceleration(ESC_Struct* ESC_hande,int16_t acceleration);


unsigned int ESC_init_all()
{
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    ESC_init(&ESC[ESC_INDEX_FL],ESC_PIN_FL);
    ESC_init(&ESC[ESC_INDEX_FR],ESC_PIN_FR);
    ESC_init(&ESC[ESC_INDEX_FC],ESC_PIN_FC);
    ESC_init(&ESC[ESC_INDEX_RL],ESC_PIN_RL);
    ESC_init(&ESC[ESC_INDEX_RR],ESC_PIN_RR);
    ESC_init(&ESC[ESC_INDEX_RC],ESC_PIN_RC);
}

unsigned int ESC_update_all()
{
    for(int i = 0; i< 6; i++)
    {
        ESC_Fast_COMM(&ESC[i]);
    }
}

unsigned int ESC_init(ESC_Struct* ESC_hande,int pin)
{
    ESC_hande->pin = pin;
    ESC_hande->thrustSetPoint_mN = 0;
    pinMode(pin, OUTPUT);
    digitalWrite(ESC_hande->pin, HIGH);
    
    return 0;
}

unsigned int ESCStart(ESC_Struct* ESC_hande)
{
#ifndef FASTCOMM
    digitalWrite(ESC_hande->pin, LOW);
    SPI.transfer(ESC_CMD_Start);
    SPI.transfer(0);
    SPI.transfer(0);
    digitalWrite(ESC_hande->pin, HIGH);
#endif
    ESC_hande->thrustSetPoint_mN = 100;
    return 0;
}

unsigned int ESCStop(ESC_Struct* ESC_hande)
{
#ifndef FASTCOMM
    digitalWrite(ESC_hande->pin, LOW);
    SPI.transfer(ESC_CMD_Stop);
    SPI.transfer(0);
    SPI.transfer(0);
    digitalWrite(ESC_hande->pin, HIGH);
#endif
    ESC_hande->thrustSetPoint_mN = 0;
    return 0;
}

//unsigned int ESCSetSpeed(ESC_Struct* ESC_hande,int16_t newSpeedSetPoint)
//{
//    #ifndef FASTCOM
//    uint8_t direction;
//    if(newSpeedSetPoint>0)
//    {
//        direction = 1;
//    }
//    else
//    {
//        direction = 0;
//    }
//    if ((direction && newSpeedSetPoint>MINIMUM_SPEED) || (!direction && newSpeedSetPoint<(-1*MINIMUM_SPEED)))
//    {
//        if (direction !=ESC_hande->direction)
//        {
//            ESCSetDirection(ESC_hande, direction);
//        }
//        if(newSpeedSetPoint!=ESC_hande->speedSetPoint)
//        {
//            uint16_t newSpeed;
//            if(direction)
//            {
//                newSpeed = newSpeedSetPoint;
//            }
//            else
//            {
//                newSpeed = - newSpeedSetPoint;
//            }
//            digitalWrite(ESC_hande->pin, LOW);
//            SPI.transfer(ESC_CMD_SetSpeed);
//            SPI.transfer(newSpeed>>8);
//            SPI.transfer(newSpeed & 0xff);
//            digitalWrite(ESC_hande->pin, HIGH);
//            ESC_hande->speedSetPoint =newSpeedSetPoint;
//        }
//        if(ESCGetStatus(ESC_hande)==STOP)
//        {
//            ESCStart(ESC_hande);
//        }
//    }
//    else
//    {
//        ESCStop(ESC_hande);
//    }
//    #endif
//    return 0;
//}
unsigned int ESCSetThrust(ESC_Struct* ESC_hande,int16_t thrustSetPoint_mN)
{
    #ifndef FASTCOMM
    digitalWrite(ESC_hande->pin, LOW);
    SPI.transfer(ESC_CMD_SetThrust);
    SPI.transfer(thrustSetPoint_mN>>8);
    SPI.transfer(thrustSetPoint_mN & 0xff);
    digitalWrite(ESC_hande->pin, HIGH);
    #endif
    ESC_hande->thrustSetPoint_mN =thrustSetPoint_mN;
    return 0;
}
int16_t ESCGetThrust(ESC_Struct* ESC_hande)
{
    #ifndef FASTCOMM
    int16_t thrustMeasured_mN = 0;
    digitalWrite(ESC_hande->pin, LOW);
    SPI.transfer(ESC_CMD_GetThrust);
    SPI.transfer(0);
    SPI.transfer(0);
    digitalWrite(ESC_hande->pin, HIGH);
    delayMicroseconds(SPI_COMMS_DELAY_MICROSECONDS);
    digitalWrite(ESC_hande->pin, LOW);
    thrustMeasured_mN = SPI.transfer(0);
    thrustMeasured_mN = thrustMeasured_mN<<8;
    thrustMeasured_mN = thrustMeasured_mN | SPI.transfer(0);
    digitalWrite(ESC_hande->pin, HIGH);
    ESC_hande->thrustMeasured_mN = thrustMeasured_mN;
    return thrustMeasured_mN;
    #else
    return ESC_hande->thrustMeasured_mN;
    #endif
    
}
uint16_t ESCGetSpeed(ESC_Struct* ESC_hande)
{
    #ifndef FASTCOMM
    uint16_t speed = 0;
    digitalWrite(ESC_hande->pin, LOW);
    SPI.transfer(ESC_CMD_GetSpeed);
    SPI.transfer(0);
    SPI.transfer(0);
    digitalWrite(ESC_hande->pin, HIGH);
    delayMicroseconds(SPI_COMMS_DELAY_MICROSECONDS);
    digitalWrite(ESC_hande->pin, LOW);
    speed = SPI.transfer(0);
    speed = speed<<8;
    speed = speed | SPI.transfer(0);
    digitalWrite(ESC_hande->pin, HIGH);
    ESC_hande->speedMeasured =speed;
    return speed;
    #else
    return ESC_hande->speedMeasured;
    #endif

}
uint16_t ESCGetCurrent(ESC_Struct* ESC_hande)
{
    #ifndef FASTCOMM
    uint16_t current = 0;
    digitalWrite(ESC_hande->pin, LOW);
    SPI.transfer(ESC_CMD_GetCurrent);
    SPI.transfer(0);
    SPI.transfer(0);
    digitalWrite(ESC_hande->pin, HIGH);
    delayMicroseconds(SPI_COMMS_DELAY_MICROSECONDS);
    digitalWrite(ESC_hande->pin, LOW);
    current = SPI.transfer(0);
    current = current<<8;
    current = current | SPI.transfer(0);
    digitalWrite(ESC_hande->pin, HIGH);
    ESC_hande->currentMeasured =current;
        return current;
    #else
    return ESC_hande->currentMeasured;
    #endif

}

unsigned int ESCSetAcceleration(ESC_Struct* ESC_hande,int16_t acceleration)
{
    #ifndef FASTCOMM
    digitalWrite(ESC_hande->pin, LOW);
    SPI.transfer(ESC_CMD_SetAcceleration);
    SPI.transfer(acceleration>>8);
    SPI.transfer(acceleration & 0xff);
    digitalWrite(ESC_hande->pin, HIGH);
    return 0;
    #else
    return 1;
    #endif
}

unsigned int ESCGetStatus(ESC_Struct* ESC_hande )
{
    #ifndef FASTCOMM
    digitalWrite(ESC_hande->pin, LOW);
    SPI.transfer(ESC_CMD_GetRunState);
    SPI.transfer(0);
    SPI.transfer(0);
    digitalWrite(ESC_hande->pin, HIGH);
    delayMicroseconds(SPI_COMMS_DELAY_MICROSECONDS);
    digitalWrite(ESC_hande->pin, LOW);
    int id = SPI.transfer(0);
    digitalWrite(ESC_hande->pin, HIGH);
    return id;
    #else
    return ESC_hande->runState;
    #endif
}

ESC_StatusStruct ESCGetStatusStruct(ESC_Struct* ESC_hande )
{
    ESC_StatusStructUnion newData;
    digitalWrite(ESC_hande->pin, LOW);
    SPI.transfer(ESC_CMD_GetStatusStruct);
    SPI.transfer(0);
    SPI.transfer(0);
    digitalWrite(ESC_hande->pin, HIGH);
    delayMicroseconds(SPI_COMMS_DELAY_MICROSECONDS);
    digitalWrite(ESC_hande->pin, LOW);
    for (int i = 0;i<sizeof(newData); i++)
    {
        newData.stuctRaw[i] = SPI.transfer(0);
    }
    digitalWrite(ESC_hande->pin, HIGH);
    ESC_hande->currentMeasured = newData.statusStruct.currentMeasured;
    ESC_hande->runState = newData.statusStruct.runState;
    ESC_hande->speedSetPoint = newData.statusStruct.speedSetPoint;
    ESC_hande->speedMeasured = newData.statusStruct.speedMeasured;
    return newData.statusStruct;
}

unsigned int ESCReset(ESC_Struct* ESC_hande)
{
    digitalWrite(ESC_hande->pin, LOW);
    SPI.transfer(ESC_CMD_RESET);
    SPI.transfer(0);
    SPI.transfer(0);
    digitalWrite(ESC_hande->pin, HIGH);
    return 0;
}

ESC_StatusStruct ESC_Fast_COMM(ESC_Struct* ESC_hande)
{
    ESC_StatusStructUnion newStatusStruct;
    ESC_CommandStructUnion newCommandStruct;
    newCommandStruct.commandStruct.comms_key = COMM_KEY;
    if ((ESC_hande->thrustSetPoint_mN>MINIMUM_THRUST) || ESC_hande->thrustSetPoint_mN <(-MINIMUM_THRUST))
    {
        newCommandStruct.commandStruct.state = 1;
        newCommandStruct.commandStruct.thrust_mN = ESC_hande->thrustSetPoint_mN;
    }
    else
    {
        newCommandStruct.commandStruct.state = 0;
        newCommandStruct.commandStruct.thrust_mN = 0;
    }
    Serial.print("setting thrust to: ");
    
    Serial.println(newCommandStruct.commandStruct.thrust_mN);
    digitalWrite(ESC_hande->pin, LOW);
    for (int i = 0;i<sizeof(newStatusStruct); i++)
    {
        newStatusStruct.stuctRaw[i] = SPI.transfer(newCommandStruct.stuctRaw[i]);
    }
    digitalWrite(ESC_hande->pin, HIGH);
    ESC_hande->currentMeasured = newStatusStruct.statusStruct.currentMeasured;
    ESC_hande->runState = newStatusStruct.statusStruct.runState;
    ESC_hande->speedSetPoint = newStatusStruct.statusStruct.speedSetPoint;
    ESC_hande->speedMeasured = newStatusStruct.statusStruct.speedMeasured;
    ESC_hande->temperature = newStatusStruct.statusStruct.temperature;
    ESC_hande->direction =newStatusStruct.statusStruct.direction;
    return newStatusStruct.statusStruct;
}




// Code




