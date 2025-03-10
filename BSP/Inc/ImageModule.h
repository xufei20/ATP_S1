/*
 * ImageModule.h
 *
 *  Created on: Nov 15, 2024
 *      Author: Fei
 */

#ifndef INC_IMAGEMODULE_H_
#define INC_IMAGEMODULE_H_


#define SEND_HEAD1 0xAA		// 发送帧头
#define SEND_HEAD2 0xC0
#define SEND_TAIL  0xAA		// 发送帧尾

#define TXDATA_LEN 64

typedef struct ImgSendTypedef
{
	uint8_t		Head1;		// 帧头1
	uint8_t		Head2;		// 帧头2
	uint16_t 	DataLen;	// 数据长度
	uint8_t		FrameMark;	// 帧标识
	uint8_t		DeviceID;	// 设备ID
	uint8_t		FuncCode;	// 功能码
	uint8_t		FuncSubCode;// 功能码子项
	uint16_t	FrameCount;	// 帧计数
	uint8_t		Data[TXDATA_LEN];	// 数据
	uint8_t		Tail;		// 帧尾
}ImgSendTypedef_t;




// 设备ID
typedef enum DeviceID
{
    TRACKING_CU     = 0x01,     // 粗跟踪
    TRACKING_JING   = 0x02,     // 精跟踪
}DeviceID_t;

// 功能码
typedef enum FuncCode
{
    SETTING_DEVICEPARAM     = 0x06,     // 设备参数设置
    SETTING_TRACK           = 0x08,     // 跟踪设置
    SETTING_DETECTPARAM     = 0x12,     // 检测参数设置
    SETTING_TRACKPARAM      = 0x13,     // 跟踪参数设置
    SETTING_IMGTRANS        = 0x14,     // 图像传输设置
}FuncCode_t;

// 设备参数设置 (功能码子项)
typedef enum SettingDeviceParam
{
    FOCUS_ADJUST     = 0x03,     // 调焦 uint32_t 0-65535
    ZOOM_ADJUST      = 0x04,     // 变焦 uint32_t 0-65535
    EXPOSURE         = 0x05,     // 曝光 uint32_t 0-65535
    OPTICAL_CENTER   = 0x06,     // 光学中心设置
    SERVER_SETTING   = 0x07,     // 服务器参数设置
}SettingDeviceParam_t;

// 服务器参数设置
typedef enum ServerSetting
{
    DEVICE_IP       = 0x01,     // 设备IP设置
    DEVICE_PORT     = 0x02,     // 设备端口设置
    SERVER_IP       = 0x01,     // 服务器IP设置
    SERVER_PORT     = 0x02,     // 服务器端口设置
    SERVER_ACCOUNT  = 0x03,     // 服务器账号设置
    SERVER_PASSWORD = 0x04,     // 服务器密码设置
}ServerSetting_t;

// 跟踪设置 (功能码子项)
typedef enum Tracking_Mode
{
    STANDBY_MODE    = 0x01,     // 待机模式设置
    AUTO_TRACK      = 0x02,     // 自动跟踪设置
    MANUAL_TRACK    = 0x03,     // 手动跟踪设置
}Tracking_Mode_t;

// 自动跟踪设置
typedef struct AutoTracking
{
    uint8_t     TrackingType;   // 跟踪类别 0-127
    uint8_t     CaptureStrategy; // 捕获策略 0x01 距离中心最近目标 0x02 视野中最大目标
    // 跟踪点偏移比例x
    uint16_t   OffsetX;        // 跟踪点偏移比例x 1代表0.1%
    uint16_t   OffsetY;        // 跟踪点偏移比例y
}AutoTracking_t;

// 手动跟踪设置
typedef struct ManualTracking
{
    uint16_t TrackingCenterX;    // 跟踪中心x 1代表1像素
    uint16_t TrackingCenterY;    // 跟踪中心y
    uint16_t TrackingWidth;      // 跟踪宽度
    uint16_t TrackingHeight;     // 跟踪高度
}ManualTracking_t;

// 跟踪参数设置 (功能码子项)


// 检测参数设置 (功能码子项)

// 图像传输设置 (功能码子项)
typedef struct ImgTrans
{
    // 传输分辨率设置x
    uint16_t ResolutionX;   // 传输分辨率设置x
    uint16_t ResolutionY;   // 传输分辨率设置y
    uint8_t OSDType;        // OSD叠加设置 0x01 叠加 0x02 不叠加
}ImgTrans_t;

typedef struct ImgSendDataTypedef_t
{
    uint32_t focusAdjust; // 调焦
    uint32_t zoomAdjust; // 变焦
    uint32_t exposure; // 曝光
    uint32_t offsetX; // 光学中心偏移量X
    uint32_t offsetY; // 光学中心偏移量Y
    AutoTracking_t AutoTracking; // 自动跟踪设置
    ManualTracking_t ManualTracking; // 手动跟踪设置
    ImgTrans_t ImgTrans; // 图像传输设置
}ImgSendDataTypedef;


// 接收数据帧
#define REV_HEAD1 0xCC
#define REV_HEAD2 0xC0
#define REV_TAIL  0xBB

// typedef struct RevFrame_t
// {
//     uint8_t     Head1;      // 帧头1
//     uint8_t     Head2;      // 帧头2
//     uint8_t     DeviceID;   // 设备ID
//     uint8_t     State1;     // 状态1
//     uint8_t     State2;     // 状态2
//     uint8_t     State3;     // 状态3
//     uint8_t     State4;     // 状态4
//     u16_u8_t    ImgCount;   // 图像计数 低字节在前 bit0-bit6用于计数 bit7为0
//     u16_u8_t    DataCount;  // 数据包计数 低字节在前 bit0-bit6用于计数 bit7为0
//     float       Offset_X;   // 偏移量X
//     float       Offset_Y;   // 偏移量Y
//                             // 预留位 占三个字节
//     uint8_t     d1;         // 预留位1
//     uint8_t     d2;         // 预留位2
//     uint8_t     d3;         // 预留位3
//     uint8_t     Tail;       // 帧尾
// }RevFrame_t;

// 定义一个 union，包含 struct 作为成员
//typedef union RevFrameUnion{
//    RevFrame_t frame;
//    uint8_t rawData[25];
//} RevFrameUnion_t;


typedef struct StateCode1_t
{
    uint8_t SelfCheck;      // 自检
    uint8_t Standby;        // 待机
    uint8_t AutoTrack;      // 自动跟踪
    uint8_t ManualTrack;    // 手动跟踪
}StateCode1;

typedef struct StateCode2_t
{
    uint8_t CheckState;     // 检测状态
    uint8_t InitState;      // 初始化状态
    uint8_t NormalTrack;    // 正常跟踪
    uint8_t RememberTrack;  // 记忆跟踪
    uint8_t TrackLose;      // 跟踪丢失
}StateCode2;

extern uint8_t CuWorkState;
extern uint8_t JingWorkState;

typedef struct StateCode4_t
{
    uint8_t VersionSoft;    // 软件版本
    uint8_t VersionHard;    // 硬件版本
}StateCode4;


typedef struct ImgRecvDataTypedef_t
{
    StateCode1 code1;
    StateCode2 code2;
    StateCode4 code4;
    uint8_t WorkState;
    uint8_t SoftVersion;
    uint8_t HardVersion;
    uint16_t ImgCount;
    uint16_t DataCount;
    float Offset_X;
    float Offset_Y;
}ImgRecvDataTypedef;


// extern uint16_t CuImgCount;
// extern uint16_t JingImgCount;

// extern uint16_t CuDataCount;
// extern uint16_t JingDataCount;

// extern float CuOffset_X;
// extern float CuOffset_Y;

// extern float JingOffset_X;
// extern float JingOffset_Y;

extern ImgSendTypedef_t ImgSendTypedef_CU;
extern ImgSendTypedef_t ImgSendTypedef_JING;
extern ImgSendDataTypedef ImgSendDataTypedef_CU;
extern ImgSendDataTypedef ImgSendDataTypedef_JING;

extern ImgRecvDataTypedef ImgRecvDataTypedef_CU;
extern ImgRecvDataTypedef ImgRecvDataTypedef_JING;





void packData(ImgSendTypedef_t *SendType,ImgSendDataTypedef *SendData);
void Send2ImgModule(ImgSendTypedef_t *SendType,ImgSendDataTypedef *SendData);

#endif /* INC_IMAGEMODULE_H_ */
