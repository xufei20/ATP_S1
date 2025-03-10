#include "main.h"
#include "ImageModule.h"
#include "usart.h"

ImgSendTypedef_t ImgSendTypedef_CU = {
		.Head1 = SEND_HEAD1,
		.Head2 = SEND_HEAD2,
//		.DataLen = 64,
		.DeviceID = TRACKING_CU,
		.FuncCode = 0x06,
		.FuncSubCode = 0x03,
		.FrameMark = 0xCC,
		.FrameCount = 0x00,
		.Tail = SEND_TAIL
};

ImgSendDataTypedef ImgSendDataTypedef_CU = {
        .focusAdjust = 0, // 调焦
        .zoomAdjust = 0, // 变焦
        .exposure = 100, // 曝光
        .offsetX = 0, // 光学中心偏移量X
        .offsetY = 0, // 光学中心偏移量Y
        .AutoTracking = {0,0x01,0,0}, // 自动跟踪设置
        .ManualTracking = {0,0,0,0}, // 手动跟踪设置
        .ImgTrans = {0} // 图像传输设置
};


ImgSendTypedef_t ImgSendTypedef_JING = {
		.Head1 = SEND_HEAD1,
		.Head2 = SEND_HEAD2,
//		.DataLen = 64,
		.DeviceID = TRACKING_JING,
		.FuncCode = 0x06,
		.FuncSubCode = 0x03,
		.FrameMark = 0xCC,
		.FrameCount = 0x00,
		.Tail = SEND_TAIL
};

ImgSendDataTypedef ImgSendDataTypedef_JING = {
        .focusAdjust = 576, // 调焦
        .zoomAdjust = 0, // 变焦
        .exposure = 100, // 曝光
        .offsetX = 0, // 光学中心偏移量X
        .offsetY = 0, // 光学中心偏移量Y
        .AutoTracking = {0,0x01,0,0}, // 自动跟踪设置
        .ManualTracking = {0}, // 手动跟踪设置
        .ImgTrans = {0} // 图像传输设置
};

ImgRecvDataTypedef ImgRecvDataTypedef_CU = {
        .code1 = {0},
        .code2 = {0},
        .code4 = {0},
        .WorkState = 0,
        .ImgCount = 0,
        .DataCount = 0,
        .Offset_X = 0,
        .Offset_Y = 0
};

ImgRecvDataTypedef ImgRecvDataTypedef_JING = {
        .code1 = {0},
        .code2 = {0},
        .code4 = {0},
        .WorkState = 0,
        .ImgCount = 0,
        .DataCount = 0,
        .Offset_X = 0,
        .Offset_Y = 0
};


void packData(ImgSendTypedef_t *SendType,ImgSendDataTypedef *SendData)
{
	uint8_t i = 0;
    switch (SendType->FuncCode) // 功能码
    {
        case SETTING_DEVICEPARAM: // 0x06
        {
            switch (SendType->FuncSubCode) // 功能码子项
            {
                case FOCUS_ADJUST: // 0x03
                {
                	for(i = 0;i < 4;i++)
                	{
                		SendType->Data[10+i] = SendData->focusAdjust >> 8*(i);
                	}
                    SendType->DataLen = 6;
                    break;
                }
                case ZOOM_ADJUST: // 0x04
                {
                    for(i = 0;i < 4;i++)
                	{
                		SendType->Data[10+i] = SendData->zoomAdjust >> 8*(i);
                	}
                    SendType->DataLen = 6;
                    break;
                }
                case EXPOSURE: // 0x05
                {
                    for(i = 0;i < 4;i++)
                	{
                		SendType->Data[10+i] = SendData->exposure >> 8*(i);
                	}
                    SendType->DataLen = 6;
                    break;
                }
                case OPTICAL_CENTER: // 0x06
                {
                    for(i = 0;i < 4;i++)
                    {
                        SendType->Data[10+i] = SendData->offsetX >> 8*(3-i);
                        SendType->Data[14+i] = SendData->offsetY >> 8*(3-i);
                    }
                    SendType->DataLen = 10;
                    break;
                }
                case SERVER_SETTING: // 0x07
                {
                    // SendType->Data[10] = ServerIPAddr.u8t[0];
                    // SendType->Data[11] = ServerIPAddr.u8t[1];
                    // SendType->Data[12] = ServerIPAddr.u8t[2];
                    // SendType->Data[13] = ServerIPAddr.u8t[3];
                    // SendType->Data[14] = ServerPort >> 8;
                    // SendType->Data[15] = ServerPort;
                    // SendType->Data[16] = DeviceIPAddr.u8t[0];
                    // SendType->Data[17] = DeviceIPAddr.u8t[1];
                    // SendType->Data[18] = DeviceIPAddr.u8t[2];
                    // SendType->Data[19] = DeviceIPAddr.u8t[3];
                    // SendType->Data[20] = DevicePort >> 8;
                    // SendType->Data[21] = DevicePort;
                    SendType->DataLen = 14;
                    break;
                }
                default:
                {
                    break;
                }
            }
        }break;
        case SETTING_TRACK: // 0x07
        {
            switch (SendType->FuncSubCode) // 功能码子项
            {
                case STANDBY_MODE: // 0x01
                {
                    SendType->DataLen = 2;
                    break;
                }
                case AUTO_TRACK: // 0x02
                {
                    SendType->Data[10] = SendData->AutoTracking.TrackingType;
                    SendType->Data[11] = SendData->AutoTracking.CaptureStrategy;
                    SendType->Data[12] = SendData->AutoTracking.OffsetX;
                    SendType->Data[13] = SendData->AutoTracking.OffsetX >> 8;
                    SendType->Data[14] = SendData->AutoTracking.OffsetY;
                    SendType->Data[15] = SendData->AutoTracking.OffsetY >> 8;
                    SendType->DataLen = 8;
                    break;
                }
                case MANUAL_TRACK: // 0x03
                {
                    for(i = 0;i < 2;i++)
                    {
                        SendType->Data[10+i] = SendData->ManualTracking.TrackingCenterX >> 8*(i);
                        SendType->Data[12+i] = SendData->ManualTracking.TrackingCenterY >> 8*(i);
                        SendType->Data[14+i] = SendData->ManualTracking.TrackingWidth >> 8*(i);
                        SendType->Data[16+i] = SendData->ManualTracking.TrackingHeight >> 8*(i);
                    }
                    SendType->DataLen = 10;
                    break;
                }
                default:
                {
                    break;
                }
            }
        }break;
        case SETTING_DETECTPARAM: // 0x12
        {

        }break;
        case SETTING_TRACKPARAM: // 0x13
        {


        }break;
        case SETTING_IMGTRANS: // 0x14
        {
            switch (SendType->FuncSubCode) // 功能码子项
            {
                case 0x02: // 停止传输图像
                {
                    break;
                }
                case 0x01: // 开始传输图像
                {
                    SendType->Data[10] = SendData->ImgTrans.ResolutionX;
                    SendType->Data[11] = SendData->ImgTrans.ResolutionX >> 8;
                    SendType->Data[12] = SendData->ImgTrans.ResolutionY;
                    SendType->Data[13] = SendData->ImgTrans.ResolutionY >> 8;
                    SendType->Data[14] = SendData->ImgTrans.OSDType;
                    SendType->DataLen = 7;
                    break;
                }
            }
        }break;
    }


    SendType->Data[0] = SendType->Head1;
    SendType->Data[1] = SendType->Head2;
    SendType->Data[2] = SendType->DataLen >> 8; //小端
    SendType->Data[3] = SendType->DataLen;
    SendType->Data[4] = SendType->FrameMark;
    SendType->Data[5] = SendType->DeviceID;
    SendType->Data[6] = SendType->FuncCode;
    SendType->Data[7] = SendType->FuncSubCode;
    SendType->Data[8] = 0x00;
    SendType->Data[9] = 0x00;
    SendType->Data[TXDATA_LEN - 1] = SendType->Tail;
}

void Send2ImgModule(ImgSendTypedef_t *SendType,ImgSendDataTypedef *SendData)
{

		packData(SendType,SendData);
//		uart_printf("%x",SendType->Data);
// 		for(int i = 0;i < TXDATA_LEN;i++)
// 		{
// 			// uart_printf("%02x ",SendType->Data[i]);
// //            HAL_UART_Transmit(&huart4,&SendType->Data[i],1,0xffff);
            
// 		}
		// memset(SendType->Data,0xff,64);
        // for(int i = 0;i < 64;i++)
        // {
        //     uart_printf("%02x ",SendType->Data[i]);
        // }
	switch(SendType->DeviceID)
	{
		case TRACKING_CU:{
			// 串口3发送
            HAL_UART_Transmit(&huart3,SendType->Data,64, HAL_MAX_DELAY);
            memset(SendType->Data,0xff,64);
		}break;
		case TRACKING_JING:{
			// 串口4发送
            HAL_UART_Transmit(&huart4,SendType->Data,64, HAL_MAX_DELAY);
            memset(SendType->Data,0xff,64);
		}break;
		default:break;
	}

}

