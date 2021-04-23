/*
 * File:   MCP4551.c
 * Author: user
 *
 * Created on 2021年4月23日, 下午 4:47
 */


#include "Common.h"

uint16_t MCP4551_Command(pot_memoryaddress MemoryAddress, pot_operationbits OperationBits, uint16_t Data) {
    I2C2_MESSAGE_STATUS I2C2MessageStatus;
    mcp4551cmd_t Command = {
        .Data8 = (Data >> 8) & 0x01,
        .OperationBits = OperationBits,
        .MemoryAddress = MemoryAddress,

        .Data7_0 = Data & 0xFF
    };
    uint16_t i2c_PendingTimeout = 0, readData = 0;
    uint8_t RcvData[2];
    I2C2_TRANSACTION_REQUEST_BLOCK readTRB[2];
    if (OperationBits != ReadData) {
        if (MemoryAddress == Volatile_Wiper_1) MemoryAddress = Volatile_Wiper_0;
        I2C2_MasterWrite(Command.bData, 2, SLAVE_I2C2_MCP4551_ADDRESS, &I2C2MessageStatus);
        /*while (I2C2MessageStatus == I2C2_MESSAGE_PENDING) {
            if (i2c_PendingTimeout == SLAVE_I2C2_DEVICE_TIMEOUT) {
                break;
            } else i2c_PendingTimeout++;
        }*/
    } else {
        I2C2_MasterWriteTRBBuild(readTRB, Command.bData, 1, SLAVE_I2C2_MCP4551_ADDRESS);
        I2C2_MasterReadTRBBuild(&readTRB[1], RcvData, 2, SLAVE_I2C2_MCP4551_ADDRESS);
        I2C2_MasterTRBInsert(2, readTRB, &I2C2MessageStatus);
        i2c_PendingTimeout = 0;
        while (I2C2MessageStatus == I2C2_MESSAGE_PENDING) {
            if (i2c_PendingTimeout == SLAVE_I2C2_DEVICE_TIMEOUT) {
                break;
            } else i2c_PendingTimeout++;
        }
        if (I2C2MessageStatus == I2C2_MESSAGE_COMPLETE) readData = RcvData[1] + ((RcvData[0]&0x01) << 8);
    }
    return readData;
}