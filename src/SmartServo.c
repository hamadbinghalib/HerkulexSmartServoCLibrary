#include "SmartServo.h"


void ServoClear(SmartServo *motor) {
    uint8_t packet_size = 11;
    motor->servo_packet_size = packet_size;
    uint8_t txBuf[11];
    
    txBuf[0] = SERVO_HEADER;              // Packet Header (0xFF)
    txBuf[1] = SERVO_HEADER;              // Packet Header (0xFF)
    txBuf[2] = SERVO_MIN_PACKET_SIZE + 4; // Packet Size
    txBuf[3] = motor->servo_id;                  // Servo ID
    txBuf[4] = SERVO_CMD_RAM_WRITE;       // Command Ram Write (0x03)
    txBuf[5] = 0;                   // Checksum1
    txBuf[6] = 0;                   // Checksum2
    txBuf[7] = SERVO_RAM_STATUS_ERROR;    // Address 48
    txBuf[8] = SERVO_BYTE2;               // Length
    txBuf[9] = 0;                   // Clear SERVO_RAM_STATUS_ERROR
    txBuf[10]= 0;                   // Clear RAM_STATUS_DETAIL
    
    // Checksum1 = (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n]) & 0xFE
    // Checksum2 = (~Checksum1)&0xFE
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]^txBuf[9]^txBuf[10]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;


    for (int ii = 0; ii<=packet_size-1; ii++)
    {
        motor->servo_tx_packet[ii] = txBuf[ii];
    }

    // servoTxUART(motor);

}

//------------------------------------------------------------------------------
void ServoSetTorque(SmartServo *motor, uint8_t cmdTorque) {
    uint8_t txBuf[10];
    uint8_t packet_size = 10;
    motor->servo_packet_size = packet_size;
    
    txBuf[0] = SERVO_HEADER;              // Packet Header (0xFF)
    txBuf[1] = SERVO_HEADER;              // Packet Header (0xFF)
    txBuf[2] = SERVO_MIN_PACKET_SIZE + 3; // Packet Size
    txBuf[3] = motor->servo_id;                  // Servo ID
    txBuf[4] = SERVO_CMD_RAM_WRITE;       // Command Ram Write (0x03)
    txBuf[5] = 0;                   // Checksum1
    txBuf[6] = 0;                   // Checksum2
    txBuf[7] = SERVO_RAM_TORQUE_CONTROL;  // Address 52
    txBuf[8] = SERVO_BYTE1;               // Length
    txBuf[9] = cmdTorque;            // Torque ON

    // Checksum1 = (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n]) & 0xFE
    // Checksum2 = (~Checksum1)&0xFE
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]^txBuf[9]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;


    for (int ii = 0; ii<=packet_size-1; ii++) {
        motor->servo_tx_packet[ii] = txBuf[ii];
    }

    // servoTxUART(motor);
}

// move servo to specific position
void ServoPositionControl(SmartServo *motor, uint16_t position, uint8_t playtime) {
    if (position > 1023) return;
    if (playtime > 255) return;

    uint8_t goalposition_msb = position >> 8;
    uint8_t goalposition_lsb = position & 0xff;

    uint8_t packet_size = 12;
    motor->servo_packet_size = packet_size;

    
    uint8_t txBuf[12];
    
    txBuf[0]  = SERVO_HEADER;                 // Packet Header (0xFF)
    txBuf[1]  = SERVO_HEADER;                 // Packet Header (0xFF)
    txBuf[2]  = SERVO_MIN_PACKET_SIZE + 5;    // Packet Size
    txBuf[3]  = motor->servo_id;               
    txBuf[4]  = SERVO_CMD_I_JOG;              // Command I JOG (0x06)
    txBuf[5]  = 0;                      // Checksum1
    txBuf[6]  = 0;                      // Checksum2
    txBuf[7]  = goalposition_lsb;              
    txBuf[8]  = goalposition_msb;  
    txBuf[9]  = 4;
    txBuf[10] = motor->servo_id;      // Pos Mode and LED on/off
    txBuf[11] = playtime;                     // Servo ID
    
    // Checksum1 = (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n]) & 0xFE
    // Checksum2 = (~Checksum1)&0xFE
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]^txBuf[9]^txBuf[10]^txBuf[11]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;


     for (int ii = 0; ii<=packet_size-1; ii++) {
        motor->servo_tx_packet[ii] = txBuf[ii];
    }

    // servoTxUART(motor);
}

// move servo at a continous rotation
void ServoVelocityControl(SmartServo *motor, uint16_t speed, uint8_t setLED) {
    if (speed > 1023 || speed < -1023) return;
    
    uint8_t txBuf[12];
    
    txBuf[0]  = SERVO_HEADER;                 // Packet Header (0xFF)
    txBuf[1]  = SERVO_HEADER;                 // Packet Header (0xFF)
    txBuf[2]  = SERVO_MIN_PACKET_SIZE + 5;    // Packet Size
    txBuf[3]  = SERVO_MAX_PID;                // pID is total number of servos in the network (0 ~ 253)
    txBuf[4]  = SERVO_CMD_S_JOG;              // Command S JOG (0x06)
    txBuf[5]  = 0;                      // Checksum1
    txBuf[6]  = 0;                      // Checksum2
    txBuf[7]  = 0;                      // Playtime, unmeaningful in turn mode
    txBuf[8]  = speed & 0x00FF;         // Speed (LSB, Least Significant Bit)
    txBuf[9]  =(speed & 0xFF00) >> 8;   // Speed (MSB, Most Significanct Bit)
    txBuf[10] = SERVO_TURN_MODE | setLED;     // Turn Mode and LED on/off
    txBuf[11] = motor->servo_id;                     // Servo ID
    
    // Checksum1 = (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n]) & 0xFE
    // Checksum2 = (~Checksum1)&0xFE
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]^txBuf[9]^txBuf[10]^txBuf[11]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;

    // servoTxUART(motor);
}

// reboot servo
void ServoReboot(SmartServo *motor) {

    uint8_t packet_size = SERVO_MIN_PACKET_SIZE;
    motor->servo_packet_size = packet_size;
    uint8_t txBuf[SERVO_MIN_PACKET_SIZE];

    txBuf[0] = SERVO_HEADER;
    txBuf[1] = SERVO_HEADER;

    txBuf[2] = SERVO_MIN_PACKET_SIZE;

    txBuf[3] = motor->servo_id;

    txBuf[4] = SERVO_CMD_REBOOT;

    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;


    for (int ii = 0; ii<=packet_size-1; ii++) {
        motor->servo_tx_packet[ii] = txBuf[ii];
    }

    // servoTxUART(motor);
}

void ServoSetLED(SmartServo *motor, uint8_t led) {
    uint8_t packet_size = SERVO_MIN_PACKET_SIZE + 2 + 1; //length of data = 1
    uint8_t txBuf[packet_size];
    motor->servo_packet_size = packet_size;
    
    txBuf[0] = SERVO_HEADER;
    txBuf[1] = SERVO_HEADER;

    txBuf[2] = packet_size;

    txBuf[3] = motor->servo_id;

    txBuf[4] = SERVO_CMD_RAM_WRITE;

    txBuf[5] = 0;
    txBuf[6] = 0;

    txBuf[7] = SERVO_RAM_LED_CONTROL;

    txBuf[8] = 1;

    txBuf[9] = 1;

    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]^txBuf[9]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;


    for (int ii = 0; ii<=(packet_size-1); ii++) {
        motor->servo_tx_packet[ii] = txBuf[ii];
    }

    // servoTxUART(motor);
}

void ServoStat(SmartServo *motor) {
    uint8_t packet_size = 7;
    uint8_t txBuf[packet_size];
    motor->servo_packet_size = packet_size;

    txBuf[0] = SERVO_HEADER;
    txBuf[1] = SERVO_HEADER;

    txBuf[2] = packet_size;

    txBuf[3] = motor->servo_id;

    txBuf[4] = SERVO_CMD_STAT;

    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;


    for (int ii = 0; ii<=(packet_size-1); ii++) {
        motor->servo_tx_packet[ii] = txBuf[ii];
    }

    // servoTxUART(motor);
}

// Returns actual servo position
void ServoPositionFeedback(SmartServo *motor) {

    uint8_t position = 0;
    
    uint8_t txBuf[9];
    
    txBuf[0] = SERVO_HEADER;                  // Packet Header (0xFF)
    txBuf[1] = SERVO_HEADER;                  // Packet Header (0xFF)
    txBuf[2] = SERVO_MIN_PACKET_SIZE + 2;     // Packet Size
    txBuf[3] = motor->servo_id;                      // Servo ID
    txBuf[4] = SERVO_CMD_RAM_READ;            // Status Error, Status Detail request
    txBuf[5] = 0;                       // Checksum1
    txBuf[6] = 0;                       // Checksum2    
    txBuf[7] = SERVO_RAM_CALIBRATED_POSITION; // Address 52
    txBuf[8] = SERVO_BYTE2;                   // Address 52 and 53      

    // Check Sum1 and Check Sum2
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;

    // send packet
    // HAL_UART_Transmit(&huart1, txBuf, 9, 100);
    
    //receive packet
    uint8_t rxBuf[13];
    // HAL_UART_Receive(&huart1, rxBuf, 13, 100);

    // Checksum1
    uint8_t chksum1 = (rxBuf[2]^rxBuf[3]^rxBuf[4]^rxBuf[7]^rxBuf[8]^rxBuf[9]^rxBuf[10]^rxBuf[11]^rxBuf[12]) & 0xFE;    
    // if (chksum1 != rxBuf[5])
    // {
    //     #ifdef HERKULEX_DEBUG
    //         pc->printf("Checksum1 fault\n");
    //     #endif
        
    //     return -1;
    // }
    
    // // Checksum2
    uint8_t chksum2 = (~rxBuf[5]&0xFE);
    // if (chksum2 != rxBuf[6])
    // {
    //     #ifdef HERKULEX_DEBUG
    //         pc->printf("Checksum2 fault\n");
    //     #endif
        
    //     return -1;
    // }

    position = ((rxBuf[10]&0x03)<<8) | rxBuf[9];
    
    // #ifdef HERKULEX_DEBUG
    //     pc->printf("position = %04X(%d)\n", position, position);
    // #endif
    
    motor->servo_pos_deg = position;
}