//
// debounce.cpp
// Library C++ code
// ----------------------------------
// Developed with embedXcode+
// http://embedXcode.weebly.com
//
// Details	    Description of the file or library
// Project 		SMC
//
// Created by 	Alex Bondarenko, 2018-03-08 7:18 PM
// 				Alex Bondarenko
//
// Copyright 	(c) Alex Bondarenko, 2018
// Licence		<#licence#>
//
// See 			debounce.h and ReadMe.txt for references
//


// Library header
#include "debounce.h"

// Code
debounce::debounce()
{
    
}

debounce::initButton(uint32_t pin,uint32_t mode)
{
    _buttonPin = pin;
    pinMode(pin, mode);
}

boolean debounce::updateButton()
{
    update(digitalRead(_buttonPin));
    if (_lastEstimate>_threshold) return false;
    return true;
}

float debounce::update(float input)
{
    _lastEstimate = _lambda*input + (1.0 - _lambda)*_lastEstimate;
}
