/*
 * RangefinderModule.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Fei
 */

#include "main.h"
#include "RangefinderModule.h"

RangefinderDataFrame_t RangefinderDataFrameSend = {
    .head = {0xEE,0x16},
    // .len = 0x00,
    .DeviceID = 0x03,
    .Param = {0x00},
    .FuncCode = DeviceCheck,
    .Data = {0x00},
    .crc = 0x00
};

RangeFinderRevData_t RangeFinderRevData = {
    .FPGAState = 0,
    .RayState = 0,
    .ZhuBoState = 0,
    .HuiBoState = 0,
    .TempState = 0,
    .RayOn = 0,
    .PowerState = 0,
    .Distance = 0.0f
};

uint8_t cSetTarget      = 0;
uint8_t cSetMultiFreq   = 0;
u16_u8_t cSetMinRange   = {0};
u16_u8_t cSetMaxRange   = {0};
float RangefinderDistance = 0.0f;



// 校验和是将DeviceID、FuncCode、Param[4]相加取低8位
uint8_t CheckSumRangefinder(uint8_t *ptr,uint8_t len)
{
    uint8_t sum = 0;
    for(int i = 0;i < len;i++)
    {
        sum += ptr[i];
    }
    return sum;
}

void PackRangefinerData(RangefinderDataFrame_t *SendType)
{
    switch (SendType->FuncCode)
    {
        case DeviceCheck:
        {
            SendType->len = 0x02;
            break;
        }
        case SingleMeasure:
        {
            SendType->len = 0x02;
            break;
        }
        case SetTarget:
        {
            SendType->len = 0x03;
            SendType->Data[5] = cSetTarget;
            break;
        }
        case MultiMeasure:
        {
            SendType->len = 0x02;
            break;
        }
        case StopMeasure:
        {
            SendType->len = 0x02;
            break;
        }
        case SetBaudRate:
        {
            SendType->len = 0x06;
            break;
        }
        case SetMultiFreq:
        {
            SendType->len = 0x04;
            SendType->Data[5] = cSetMultiFreq;
            SendType->Data[6] = 0x00;
            break;
        }
        case SetMinRange:
        {
            SendType->len = 0x04;
            SendType->Data[5] = cSetMinRange.data_u8[1];
            SendType->Data[6] = cSetMinRange.data_u8[0];
            break;
        }
        case GetMinRange:
        {
            SendType->len = 0x02;
            break;
        }
        case SetMaxRange:
        {
            SendType->len = 0x04;
            SendType->Data[5] = cSetMaxRange.data_u8[1];
            SendType->Data[6] = cSetMaxRange.data_u8[0];
            break;
        }
        case GetMaxRange:
        {
            SendType->len = 0x02;
            break;
        }
        default:
            break;
    }
    SendType->Data[0] = SendType->head[0];
    SendType->Data[1] = SendType->head[1];
    SendType->Data[2] = SendType->len;
    SendType->Data[3] = SendType->DeviceID; 
    SendType->Data[4] = SendType->FuncCode;

    if(SendType->len == 0x02)
    {
        SendType->Data[5] = CheckSumRangefinder(SendType->Data+3,2);
    }
    else if(SendType->len == 0x03)
    {
        SendType->Data[6] = CheckSumRangefinder(SendType->Data+3,3);
    }
    else if(SendType->len == 0x04)
    {
        SendType->Data[7] = CheckSumRangefinder(SendType->Data+3,4);
    }
    
}


void Send2RangefinderModule(RangefinderDataFrame_t *SendType)
{
    PackRangefinerData(SendType);
    if(SendType->len == 0x02)
    {
        HAL_UART_Transmit(&huart7, SendType->Data, 6, 0xffff);
    }
    else if(SendType->len == 0x03)
    {
        HAL_UART_Transmit(&huart7, SendType->Data, 7, 0xffff);
    }
    else if(SendType->len == 0x04)
    {
        HAL_UART_Transmit(&huart7, SendType->Data, 8, 0xffff);
    }
    // for(int i = 0;i < 8;i++)
    // {
    //     uart_printf("%02x ", SendType->Data[i]);
    // }
    memset(SendType->Data, 0, 16);
}

void RevRangefinderData(uint8_t *RevData)
{
    switch(RevData[4])
    {
        case DeviceCheck:
        {
            // TODO: 待完善
            break;
        }
        case SingleMeasure:
        {
            RangefinderDistance = RevData[6] * 256 + RevData[7] + RevData[8] * 0.1;
            break;
        }
        case SetTarget:
        {
            break;
        }
        case MultiMeasure:
        {
            RangefinderDistance = RevData[6] * 256 + RevData[7] + RevData[8] * 0.1;
            break;
        }


        default:
            break;
    }



}
