/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef _DXL_HPP_
#define _DXL_HPP_

#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "dynamixel_sdk.h"              // Uses Dynamixel SDK library

#define STDIN_FILENO 0

#define MX12W                           0 //for MX-12W
#define XC430W150                       1 //for XC430-W150
#define XL430W250                       2 //for XL430-W250

#define DXL_MODEL   MX12W
//#define DXL_MODEL   XL430W250
//#define DXL_MODEL   XC430W150

// Control table address for MX-12W
#define ADDR_MX_TORQUE_ENABLE           24                  
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_MOVING_SPEED            32

// Control table address for XL and XC model
#define ADDR_XL_TORQUE_ENABLE           64                 
#define ADDR_XL_OPERATING_MODE          11                 
#define ADDR_XL_GOAL_POSITION           116
#define ADDR_XL_PRESENT_POSITION        132
#define ADDR_XL_GOAL_VELOCITY           104
#define ADDR_XL_PRESENT_VELOCITY	    128

// Data Byte Length for MX-12W
#define LEN_MX_GOAL_POSITION		    2
#define LEN_MX_PRESENT_POSITION         2
#define LEN_MX_MOVING_SPEED             2

// Data Byte Length for XL and XC model
#define LEN_XL_GOAL_POSITION		    4
#define LEN_XL_PRESENT_POSITION        	4
#define LEN_XL_GOAL_VELOCITY           	4
#define LEN_XL_PRESENT_VELOCITY        	4

// Protocol version for MX-12W
#define MX_PROTOCOL_VERSION                1.0                 
// Protocol version for XL and XC model
#define XL_PROTOCOL_VERSION                2.0                 

// Default setting
#define DXL1_ID                         1		        // Dynamixel#1 ID: 1
#define DXL2_ID                         2               // Dynamixel#2 ID: 2
#define MX_BAUDRATE                     2000000         // for MX-12W model
#define XL_BAUDRATE                     4000000         // for XL and XC model
#define DEVICENAME                      "/dev/ttyUSB0"  // device file name
                                                        
#define TORQUE_ENABLE                   1               // Value for enabling the torque
#define TORQUE_DISABLE                  0               // Value for disabling the torque
#define OPMODE_XL_VELOCITY              1               
#define OPMODE_XL_POSITION              3               
#define OPMODE_XL_PWM	                16              
#define ESC_ASCII_VALUE                 0x1b

class Dxl
{
private:
    int port_num;
    int group_num;
    int dxl_comm_result;             // Communication result
    uint8_t dxl_addparam_result;     // AddParam result
    uint8_t dxl_error;               // Dynamixel error
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;

public:
    Dxl(void);
    bool open(void);
    void close(void);
    bool setVelocity(int goal_rpm1, int goal_rpm2);
    unsigned int velConvert(int speed);
    int getch(void);
    bool kbhit(void);
};

#endif //_DXL_HPP_