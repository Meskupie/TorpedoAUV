//
// SMC_SERIAL.cpp
// Library C++ code
// ----------------------------------
// Developed with embedXcode+
// http://embedXcode.weebly.com
//
// Details	    Description of the file or library
// Project 		SMC
//
// Created by 	Alex Bondarenko, 2018-03-01 9:25 PM
// 				Alex Bondarenko
//
// Copyright 	(c) Alex Bondarenko, 2018
// Licence		<#licence#>
//
// See 			SMC_SERIAL.h and ReadMe.txt for references
//


// Library header
#include "SMC_SERIAL.h"


#define SERIAL_READ Serial.read
#define SERIAL_AVAILABLE Serial.available
#define STOP_ALL_MOTORS for (int i = 0; i<6;i++) ESCStop(&ESC[i]);

hostUpdateStruct_transmit_union txUnion;
hostUpdateStruct_receive_union rxUnion;
imu::Quaternion quat;


char queryType = 'X';
int motor = 0;
long speed = 0;
uint16_t current = 0;
int8_t direction = 0;

extern ESC_Struct ESC[];

void printESCState(int state)
{
    switch (state) {
        case IDLE:
            Serial.print("IDLE\n");
            break;
            
        case STARTUP:
            Serial.print("STARTUP\n");
            break;
            
        case VALIDATION:
            Serial.print("VALIDATION\n");
            break;
            
        case STOP:
            Serial.print("STOP\n");
            break;
            
        case START:
            Serial.print("START\n");
            break;
            
        case RUN:
            Serial.print("RUN\n");
            break;
            
        case ALIGNMENT:
            Serial.print("ALIGNMENT\n");
            break;
            
        case SPEEDFBKERROR:
            Serial.print("SPEEDFBKERROR\n");
            break;
            
        case OVERCURRENT:
            Serial.print("OVERCURRENT\n");
            break;
            
        case STARTUP_FAILURE:
            Serial.print("STARTUP_FAILURE\n");
            break;
            
        case STARTUP_BEMF_FAILURE:
            Serial.print("STARTUP_BEMF_FAILURE\n");
            break;
            
        case LF_TIMER_FAILURE:
            Serial.print("LF_TIMER_FAILURE\n");
            break;
    }
}

void printStatusStruct(ESC_StatusStruct printStruct)
{

    Serial.print("speed Set Point: ");
    Serial.println(printStruct.speedSetPoint);
    Serial.print("speed Measured: ");
    Serial.println(printStruct.speedMeasured);
    Serial.print("current Measured: ");
    Serial.println(printStruct.currentMeasured);
    Serial.print("temp: ");
    Serial.println(printStruct.temperature);
    Serial.print("run state: ");
    printESCState(printStruct.runState);
    Serial.print("direction: ");
    Serial.println(printStruct.direction);
    
}

// Code
void readSerialCommand() {
    // Check for serial message
    if (SERIAL_AVAILABLE()) {
        queryType = SERIAL_READ();
        switch (queryType) {
            case 'G': // StartMotor
                motor  = readIntegerSerial();
                if( motor>=0&& motor<6)
                {
                    Serial.print("STARTING MOTOR: ");
                    Serial.println(motor);
                    ESCStart(&ESC[motor]);
                }
                else
                {
                    Serial.print("Invalid index");
                }
                break;
            case 'H': // STOP
                motor  = readIntegerSerial();
                if( motor>=0&& motor<6)
                {
                    ESCStop(&ESC[motor]);
                }
                else
                {
                    Serial.print("Invalid index");
                }
                break;
            case 'M': // TEST
                motor  = readIntegerSerial();
                if( motor>=0&& motor<6)
                {
                    Serial.println(motor);
                }
                else
                {
                    Serial.println("Invalid index");
                }
                break;
            case 'D': // SET Direction
                motor  = readIntegerSerial();
                direction = readIntegerSerial();
                if( motor>=0&& motor<6)
                {
                    ESCSetDirection(&ESC[motor], direction);
                    Serial.print("motor: ");
                    Serial.print(motor);
                    Serial.print(" direction: ");
                    Serial.println(direction);
                }
                else
                {
                    Serial.println("Invalid Motor index");
                }
            case 'S': // SET SPEED
                motor  = readIntegerSerial();
                speed = readIntegerSerial();
                if( motor>=0&& motor<6)
                {
                    ESCSetSpeed(&ESC[motor], speed);
                    Serial.print("motor: ");
                    Serial.print(motor);
                    Serial.print(" speed: ");
                    Serial.println(speed);
                }
                else
                {
                    Serial.println("Invalid Motor index");
                }
                break;
            case 's': // get SPEED
                motor = readIntegerSerial();
                if( motor>=0&& motor<6)
                {
                    speed = ESCGetSpeed(&ESC[motor]);
                    Serial.print("motor: ");
                    Serial.print(motor);
                    Serial.print(" speed: ");
                    Serial.println(speed);
                }
                else
                {
                    Serial.println("Invalid Motor index");
                }
                break;
            case 'C': // SET Current
                motor  = readIntegerSerial();
                speed = readIntegerSerial();
                if( motor>=0&& motor<6)
                {
//                (&ESC[motor], speed);
//                    Serial.print("motor: ");
//                    Serial.print(motor);
//                    Serial.print(" speed: ");
//                    Serial.println(speed);
                }
                else
                {
                    Serial.println("Invalid Motor index");
                }
                break;
            case 'c': // get Current
                motor  = readIntegerSerial();
                if( motor>=0&& motor<6)
                {
                    ESCGetCurrent(&ESC[motor]);
                    Serial.print("motor: ");
                    Serial.print(motor);
                    Serial.print(" speed: ");
                    Serial.println(speed);
                }
                else
                {
                    Serial.println("Invalid Motor index");
                }
                break;
            case 'i': // Get Status
                motor  = readIntegerSerial();
                if( motor>=0&& motor<6)
                {
                    Serial.print("state: ");
                    ESC_RUN_STATE state = ESCGetStatus(&ESC[motor]);
                    printESCState(state);
                }
                else
                {
                    Serial.println("Invalid Motor index");
                }
                break;
                
            case ' ': // STOP ALL
                for (int i = 0; i<6;i++) ESCStop(&ESC[i]);
                break;
                
                case 'R':
                motor  = readIntegerSerial();
                if( motor>=0&& motor<6)
                {
                    ESCReset(&ESC[motor]);
                    Serial.print("motor: ");
                    Serial.print(motor);
                    Serial.println(" reset: ");
                }
                else
                {
                    Serial.println("Invalid Motor index");
                }
                
                break;
            case 'w': // get struct
                motor  = readIntegerSerial();
                if( motor>=0&& motor<6)
                {
                    printStatusStruct(ESCGetStatusStruct(&ESC[motor]));
                }
                else
                {
                    Serial.println("Invalid Motor index");
                }
                break;
                
            case 'b':
                Serial.print("Voltage: ");
                Serial.println(gasGauge.getVoltage());
                Serial.print("Current: ");
                Serial.println(gasGauge.getCurrent());
                break;
                
            case 'q': // get system Update Struct
                txUnion.statusStruct.battVoltage_mV = gasGauge.getVoltage();
                txUnion.statusStruct.battCurrent_mA = gasGauge.getCurrent();
                txUnion.statusStruct.ambientTemperature_C = IMU.getTemp();
                quat = IMU.getQuat();
                txUnion.statusStruct.imu_x = castQuatToUint16(quat.x());
                txUnion.statusStruct.imu_y = castQuatToUint16(quat.y());
                txUnion.statusStruct.imu_z = castQuatToUint16(quat.z());
                txUnion.statusStruct.imu_w = castQuatToUint16(quat.w());
                txUnion.statusStruct.swStateFront = swFront.updateButton();
                txUnion.statusStruct.swStateCenter = swCenter.updateButton();
                txUnion.statusStruct.swStateRear = swRear.updateButton();
                ESC_Status_update_all();
                txUnion.statusStruct.motorStatus0 = ESC[0].runState;
                txUnion.statusStruct.motorStatus1 = ESC[1].runState;
                txUnion.statusStruct.motorStatus2 = ESC[2].runState;
                txUnion.statusStruct.motorStatus3 = ESC[3].runState;
                txUnion.statusStruct.motorStatus4 = ESC[4].runState;
                txUnion.statusStruct.motorStatus5 = ESC[5].runState;
                 //TODO: find how to map current to thrust
                txUnion.statusStruct.motorThrust0_mN = ESC[0].currentMeasured;
                txUnion.statusStruct.motorThrust1_mN = ESC[1].currentMeasured;
                txUnion.statusStruct.motorThrust2_mN = ESC[2].currentMeasured;
                txUnion.statusStruct.motorThrust3_mN = ESC[3].currentMeasured;
                txUnion.statusStruct.motorThrust4_mN = ESC[4].currentMeasured;
                txUnion.statusStruct.motorThrust5_mN = ESC[5].currentMeasured;
                Serial.write(txUnion.stuctRaw, sizeof(txUnion));
                break;
                
            case 'Q': // Set Motor
                Serial.readBytes(rxUnion.stuctRaw, sizeof(rxUnion));
                if (rxCheckSum(rxUnion))
                {
                    ESCSetSpeed(&ESC[0], rxUnion.motorStruct.motorThrust0_mN);
                    ESCSetSpeed(&ESC[1], rxUnion.motorStruct.motorThrust1_mN);
                    ESCSetSpeed(&ESC[2], rxUnion.motorStruct.motorThrust2_mN);
                    ESCSetSpeed(&ESC[3], rxUnion.motorStruct.motorThrust3_mN);
                    ESCSetSpeed(&ESC[4], rxUnion.motorStruct.motorThrust4_mN);
                    ESCSetSpeed(&ESC[5], rxUnion.motorStruct.motorThrust5_mN);
                }
                break;

            default:
                Serial.println("Invalid Command");
                break;
                
        }
    }
}
//TODO: implement checksum
bool rxCheckSum(hostUpdateStruct_receive_union input)
{
    int16_t sum =0;
    for (int i = 0; i<(sizeof(input)-1); i++) {
        sum = sum+input.stuctRaw[i];
    }
    sum = sum + input.motorStruct.count;
    if ((sum%16) == input.motorStruct.checkSum) return true;
    return false;
}
int16_t castQuatToUint16(float quatVector)
{
    return (int16_t) (quatVector*__INT16_MAX__);
}
void readValueSerial(char *data, byte size) {
    byte index = 0;
    byte timeout = 0;
    data[0] = '\0';
    
    do {
        if (SERIAL_AVAILABLE() == 0) {
            delay(1);
            timeout++;
        } else {
            data[index] = SERIAL_READ();
            timeout = 0;
            index++;
        }
    } while ((index == 0 || data[index-1] != ';') && (timeout < 10) && (index < size-1));
    
    data[index] = '\0';
}

// Used to read integer values from the serial port
long readIntegerSerial() {
    char data[16] = "";
    
    readValueSerial(data, sizeof(data));
    return atol(data);
}

