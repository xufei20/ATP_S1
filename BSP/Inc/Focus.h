/*
 * Focus.h
 *
 *  Created on: Nov 23, 2024
 *      Author: Fei
 */

#ifndef INC_FOCUS_H_
#define INC_FOCUS_H_

#include "main.h"

enum FocusFunc
{
    SetPos              = 0x01, // 设置位置
    GetCurPos           = 0x03, // 获取当前位置
    MultiGetCurPos      = 0x0f, // 实时获取当前位置
    StopGetPos          = 0x10, // 停止获取当前位置
    ClearPos            = 0x1b, // 清除位置
    StopRun             = 0x05, // 停止运动
};

typedef struct _SendFocusFrameTypedef
{
    uint8_t     head[2]     ; //帧头
    uint8_t     len         ; //数据长度
    uint8_t     func        ; //功能码
    uint8_t     data[16]	; //数据
    uint8_t     check       ; //校验和
}SendFocusFrameTypedef;

extern SendFocusFrameTypedef SendFocusFrame;
extern uint8_t setFocusFreq;
extern f32_u8_t setFocusPos;
extern f32_u8_t getFocusPos;

void SendFocusData(SendFocusFrameTypedef *SendType);
uint8_t checkData(uint8_t* data, uint8_t length);


#endif /* INC_FOCUS_H_ */
