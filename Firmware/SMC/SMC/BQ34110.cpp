//
// BQ34110.cpp
// Library C++ code
// ----------------------------------
// Developed with embedXcode+
// http://embedXcode.weebly.com
//
// Details	    TI BQ34110 Driver
// Project 		SMC
//
// Created by 	Alex Bondarenko, 2018-03-07 1:03 AM
// 				Alex Bondarenko
//
// Copyright 	(c) Alex Bondarenko, 2018
// Licence		<#licence#>
//
// See 			BQ34110.h and ReadMe.txt for references
//


// Library header
#include "BQ34110.h"
BQ34110::BQ34110()
{
    _address = 0b1010101;
}

bool BQ34110::begin()
{
    /* Enable I2C */
    Wire.begin();
}

uint16_t BQ34110::getVoltage()
{
    uint16_t data;
    Wire.beginTransmission(_address);
    Wire.write(0x08);
    Wire.endTransmission();
    Wire.beginTransmission(_address);
    Wire.endTransmission();
    Wire.requestFrom(_address, 2);
        data =  Wire.read()<<8;
        data |=  Wire.read();
    if (data>9000)
    {
        data = data*0.064093;
        data = data + 8387;
    }
    else
    {
        data = data* 0.065104;
        data = data + 12580;
    }
    return data;
}
uint16_t BQ34110::getChipTemp()
{
    uint16_t data;
    Wire.beginTransmission(_address);
    Wire.write(0x28);
    Wire.endTransmission();

    Wire.requestFrom(_address, 2);
        data =  Wire.read()<<8;
        data |=  Wire.read();
//    data = data*0.064093;
//    data = data + 8387;
    return data;
}

int16_t BQ34110::getCurrent()
{
    int16_t data;
    Wire.beginTransmission(_address);
    Wire.write(0x7A);
    Wire.endTransmission();
    Wire.requestFrom(_address, 2);
    data =  Wire.read()<<8;
    data |=  Wire.read();
//    data = data*0.064093;
//    data = data + 8387;
    return data;
}
   uint16_t BQ34110::testDataWriteToFlash()
{
    int16_t data;
    Wire.beginTransmission(_address);
    Wire.write(0x00);
    Wire.write(0x02);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(1);
    Wire.beginTransmission(_address);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom(_address, 2);
    data =  Wire.read()<<8;
    data |=  Wire.read();
    //    data = data*0.064093;
    //    data = data + 8387;
    return data;
}
byte BQ34110::read8(BQ34110_reg_t reg)
{
    byte value = 0;
    
    Wire.beginTransmission(_address);
#if ARDUINO >= 100
    Wire.write((uint8_t)reg);
#else
    Wire.send(reg);
#endif
    Wire.endTransmission();
    Wire.requestFrom(_address, (byte)1);
#if ARDUINO >= 100
    value = Wire.read();
#else
    value = Wire.receive();
#endif
    
    return value;
}

uint16_t BQ34110::read16(BQ34110_reg_t reg)
{
    Wire.beginTransmission(_address);
    uint16_t data;
#if ARDUINO >= 100
    Wire.write((uint8_t)reg);
#else
    Wire.send(reg);
#endif
    Wire.endTransmission();
    Wire.requestFrom(_address, 2);
    

#if ARDUINO >= 100
        data =  Wire.read()<<8;
        data |=  Wire.read();
#else
        data =  Wire.receive()<<8;
      data |=  Wire.receive();
#endif
    
    
    /* ToDo: Check for errors! */
    return true;
}
// Code
