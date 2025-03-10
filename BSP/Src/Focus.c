/*
 * Focus.c
 *
 *  Created on: Nov 23, 2024
 *      Author: Fei
 */

#include "Focus.h"

SendFocusFrameTypedef SendFocusFrame = {
    .head = {0xaa, 0x01},
    .len = 0x00,
    .func = 0x00,
    .data = {0},
    .check = 0x00
};

f32_u8_t setFocusPos = {0.f};
f32_u8_t getFocusPos = {0};
uint8_t setFocusFreq = 0;




uint8_t checkData(uint8_t* data, uint8_t length) {
    uint8_t check = 0x00;
    for (int i = 0; i < length; i++) {
        check ^= data[i];
    }
    return check;
}

void SendFocusData(SendFocusFrameTypedef *SendType)
{
    SendType->data[0] = SendType->head[0];
    SendType->data[1] = SendType->head[1];
    SendType->data[4] = 0x03; // 设备ID
    SendType->data[5] = 0x00; //通道
    switch (SendType->func)
    {
        case SetPos:{
            SendType->len = 0x0c;
            SendType->data[2] = SendType->len;
            SendType->data[3] = SetPos;
            
            // if(setFocusPos.f > 2000.f)
            // {
            //     setFocusPos.f = 2000.f;
            // }else if(setFocusPos.f < -2000.f)
            // {
            //     setFocusPos.f = -2000.f;
            // }
            SendType->data[6] = setFocusPos.u8t[3];
            SendType->data[7] = setFocusPos.u8t[2];
            SendType->data[8] = setFocusPos.u8t[1];
            SendType->data[9] = setFocusPos.u8t[0];
            SendType->data[10] = 0x01;
            SendType->data[11] = checkData(SendType->data, 11);
        }break;
        case GetCurPos:{
            SendType->len = 0x08;
            SendType->data[2] = SendType->len;
            SendType->data[3] = GetCurPos;
            SendType->data[6] = 0x01;
            SendType->data[7] = checkData(SendType->data, 7);
        }break;
        case StopRun:{
            SendType->len = 0x08;
            SendType->data[2] = SendType->len;
            SendType->data[3] = StopRun;
            SendType->data[6] = 0x00;
            SendType->data[7] = checkData(SendType->data, 7);
        }break;
        case MultiGetCurPos:{
            SendType->len = 0x08;
            SendType->data[2] = SendType->len;
            SendType->data[3] = MultiGetCurPos;
            SendType->data[6] = setFocusFreq;
            SendType->data[7] = checkData(SendType->data, 7);
        }break;
        case StopGetPos:{
            SendType->len = 0x06;
            SendType->data[2] = SendType->len;
            SendType->data[3] = StopGetPos;
            SendType->data[5] = checkData(SendType->data, 5);
        }break;
        case ClearPos:{
            SendType->len = 0x06;
            SendType->data[2] = SendType->len;
            SendType->data[3] = ClearPos;
            SendType->data[5] = checkData(SendType->data, 5);
        }break;
        default:
            break;
    }
    for(int i = 0;i < SendType->len;i++)
    {
        // SendType->data[SendType->len] = SendType->check;
        uart_printf("%02x ", SendType->data[i]);
    }
    HAL_UART_Transmit(&huart6, SendType->data, SendType->len, HAL_MAX_DELAY);
}