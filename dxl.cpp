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

#include "dxl.hpp"

int Dxl::getch(void)
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

bool Dxl::kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return true;
    }
    return false;
}

#if DXL_MODEL == MX12W

Dxl::Dxl(void)
{
    port_num = 0;
    group_num = 0;
    dxl_comm_result = COMM_TX_FAIL;             // Communication result
    dxl_addparam_result = false;            // AddParam result
    dxl_error = 0;                          // Dynamixel error
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = dynamixel::PacketHandler::getPacketHandler(MX_PROTOCOL_VERSION);
    
}

bool Dxl::open(void)
{    
    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return false;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(MX_BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return false;
    }

    // Enable Dynamixel#1 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
    }

    // Enable Dynamixel#2 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
    }
    return true;
}

void Dxl::close(void)
{
    // stop motor
    setVelocity(0, 0);
    
    // Disable Dynamixel#1 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    // Disable Dynamixel#2 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    // Close port
    portHandler->closePort();
}
//goal_rpm1,2 : -470~470
bool Dxl::setVelocity(int goal_rpm1, int goal_rpm2)
{

    int goal_velocity1, goal_velocity2;
    
    goal_velocity1 = goal_rpm1/0.916; //goal velocity : 0.916rpm
    goal_velocity2 = goal_rpm2/0.916;

    if(goal_velocity1 < -470) goal_velocity1 = -470;
    else if(goal_velocity1 > 470) goal_velocity1 = 470;
    if(goal_velocity2 < -470) goal_velocity2 = -470;
    else if(goal_velocity2 > 470) goal_velocity2 = 470;    

    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_MOVING_SPEED, LEN_MX_MOVING_SPEED);
    
    //int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };  // Goal position
    uint8_t param_goal_position[2];
    
    // Allocate goal position value into byte array
    param_goal_position[0] = DXL_LOBYTE(velConvert(goal_velocity1));
    param_goal_position[1] = DXL_HIBYTE(velConvert(goal_velocity1));
    
    // Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
        return false;
    }
   
    param_goal_position[0] = DXL_LOBYTE(velConvert(goal_velocity2));
    param_goal_position[1] = DXL_HIBYTE(velConvert(goal_velocity2));
   
    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
        return false;
    }    
    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();
    return true;
}

//unit : 0.9rpm, range : -1023 <= speed <= 1023 -> +:CCW(0~1023), -:CW(1024~2047)
unsigned int Dxl::velConvert(int speed)
{
    unsigned int temp;
    if (speed > 1023) speed = 1023;
    else if (speed < -1023) speed = -1023;

    if (speed >= 0) temp = (unsigned int)speed;
    else temp = (unsigned int)(-speed + 1023);

    return temp;
}

#elif DXL_MODEL == XC430W150

Dxl::Dxl(void)
{
    port_num = 0;
    group_num = 0;
    dxl_comm_result = COMM_TX_FAIL;             // Communication result
    dxl_addparam_result = false;            // AddParam result
    dxl_error = 0;                          // Dynamixel error
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = dynamixel::PacketHandler::getPacketHandler(XL_PROTOCOL_VERSION);
    
}

bool Dxl::open(void)
{    
    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return false;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(XL_BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return false;
    }

    // Enable Dynamixel#1 Torque
    //dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_XL_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
    }

    // Enable Dynamixel#2 Torque
    //dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_XL_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
    }
    return true;
}

void Dxl::close(void)
{
    // stop motor
    setVelocity(0, 0);
    
    // Disable Dynamixel#1 Torque
    //dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_XL_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    // Disable Dynamixel#2 Torque
    //dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_XL_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    // Close port
    portHandler->closePort();
}

// XC430-W150 goal_rpm1,2 : -105~105rpm(105=460*0.229rpm)
bool Dxl::setVelocity(int goal_rpm1, int goal_rpm2)
{
    int goal_velocity1, goal_velocity2;
    
    goal_velocity1 = goal_rpm1/0.229; //goal velocity : 0.229rpm
    goal_velocity2 = goal_rpm2/0.229;

    //XC430-W150 : vel limit = 460
    if(goal_velocity1 < -460) goal_velocity1 = -460;
    else if(goal_velocity1 > 460) goal_velocity1 = 460;
    if(goal_velocity2 < -460) goal_velocity2 = -460;
    else if(goal_velocity2 > 460) goal_velocity2 = 460;    

    //printf("vel1:%d,vel2:%d\n", goal_velocity1, goal_velocity2);
        
    //dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_MOVING_SPEED, LEN_MX_MOVING_SPEED);
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_XL_GOAL_VELOCITY, LEN_XL_GOAL_VELOCITY);
    
    //int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };  // Goal position
    uint8_t param_goal_velocity[4];
    
    // Allocate goal position value into byte array
    //param_goal_position[0] = DXL_LOBYTE(vel_convert(goal_velocity1));
    //param_goal_position[1] = DXL_HIBYTE(vel_convert(goal_velocity1));
    param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(goal_velocity1));
    param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(goal_velocity1));
    param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(goal_velocity1));
    param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(goal_velocity1));    
    
    // Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_velocity);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
        return false;
    }
   
    //param_goal_position[0] = DXL_LOBYTE(vel_convert(goal_velocity2));
    //param_goal_position[1] = DXL_HIBYTE(vel_convert(goal_velocity2));
    param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(goal_velocity2));
    param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(goal_velocity2));
    param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(goal_velocity2));
    param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(goal_velocity2));
   
    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_velocity);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
        return false;
    }    
    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();
    return true;
}

#elif DXL_MODEL == XL430W250

Dxl::Dxl(void)
{
    port_num = 0;
    group_num = 0;
    dxl_comm_result = COMM_TX_FAIL;             // Communication result
    dxl_addparam_result = false;            // AddParam result
    dxl_error = 0;                          // Dynamixel error
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = dynamixel::PacketHandler::getPacketHandler(XL_PROTOCOL_VERSION);
    
}

bool Dxl::open(void)
{    
    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return false;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(XL_BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return false;
    }

    // Enable Dynamixel#1 Torque
    //dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_XL_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
    }

    // Enable Dynamixel#2 Torque
    //dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_XL_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
    }
    return true;
}

void Dxl::close(void)
{
    // stop motor
    setVelocity(0, 0);
    
    // Disable Dynamixel#1 Torque
    //dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_XL_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    // Disable Dynamixel#2 Torque
    //dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_XL_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    // Close port
    portHandler->closePort();
}

// XL430-W250 goal_rpm1,2 : -60~60rpm(60=265*0.229rpm)
bool Dxl::setVelocity(int goal_rpm1, int goal_rpm2)
{
    int goal_velocity1, goal_velocity2;
    
    goal_velocity1 = goal_rpm1/0.229; //goal velocity : 0.229rpm
    goal_velocity2 = goal_rpm2/0.229;

    //XL430-W250 : vel limit = 265
    if(goal_velocity1 < -265) goal_velocity1 = -265;
    else if(goal_velocity1 > 265) goal_velocity1 = 265;
    if(goal_velocity2 < -265) goal_velocity2 = -265;
    else if(goal_velocity2 > 265) goal_velocity2 = 265;    

    //printf("vel1:%d,vel2:%d\n", goal_velocity1, goal_velocity2);
        
    //dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_MOVING_SPEED, LEN_MX_MOVING_SPEED);
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_XL_GOAL_VELOCITY, LEN_XL_GOAL_VELOCITY);
    
    //int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };  // Goal position
    uint8_t param_goal_velocity[4];
    
    // Allocate goal position value into byte array
    //param_goal_position[0] = DXL_LOBYTE(vel_convert(goal_velocity1));
    //param_goal_position[1] = DXL_HIBYTE(vel_convert(goal_velocity1));
    param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(goal_velocity1));
    param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(goal_velocity1));
    param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(goal_velocity1));
    param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(goal_velocity1));    
    
    // Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_velocity);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
        return false;
    }
   
    //param_goal_position[0] = DXL_LOBYTE(vel_convert(goal_velocity2));
    //param_goal_position[1] = DXL_HIBYTE(vel_convert(goal_velocity2));
    param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(goal_velocity2));
    param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(goal_velocity2));
    param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(goal_velocity2));
    param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(goal_velocity2));
   
    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_velocity);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
        return false;
    }    
    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();
    return true;
}

#endif
