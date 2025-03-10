/*
 * RangefinderModule.h
 *
 *  Created on: Nov 19, 2024
 *      Author: Fei
 */

#ifndef INC_RANGEFINDERMODULE_H_
#define INC_RANGEFINDERMODULE_H_

typedef struct RangefinderDataFrame
{
    uint8_t head[2];
    uint8_t len;
    uint8_t DeviceID;
    uint8_t FuncCode;
    uint8_t Param[4];
    uint8_t Data[16];
    uint8_t crc;
}RangefinderDataFrame_t;


typedef struct _RangefinderRevData
{
    uint8_t HuiBoValue; //回波值
    uint8_t FPGAState; //FPGA状态
    uint8_t RayState; //激光状态
    uint8_t ZhuBoState; //主波状态
    uint8_t HuiBoState; //回波状态
    uint8_t TempState; //温度状态
    uint8_t RayOn; //激光开关
    uint8_t PowerState; //电源状态
    float   Distance; //测距结果
}RangeFinderRevData_t;

typedef enum
{
    DeviceCheck     = 0x01,
    SingleMeasure   = 0x02,
    SetTarget       = 0x03,
    MultiMeasure    = 0x04,
    StopMeasure     = 0x05,
    DeviceError     = 0x06,
    SetBaudRate     = 0xA0,
    SetMultiFreq    = 0xA1,
    SetMinRange     = 0xA2,
    GetMinRange     = 0xA3,
    SetMaxRange     = 0xA4,
    GetMaxRange     = 0xA5
}RangeFinderFuncCode_t;


extern uint8_t cSetTarget      ;
extern uint8_t cSetMultiFreq   ;
extern u16_u8_t cSetMinRange   ;
extern u16_u8_t cSetMaxRange   ;
extern RangefinderDataFrame_t RangefinderDataFrameSend;
extern RangeFinderRevData_t RangeFinderRevData;
uint8_t CheckSumRangefinder(uint8_t *ptr,uint8_t len);
void PackRangefinerData(RangefinderDataFrame_t *SendType);
void Send2RangefinderModule(RangefinderDataFrame_t *SendType);

#endif /* INC_RANGEFINDERMODULE_H_ */
