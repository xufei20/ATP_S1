/*
 * ServoModule.h
 *
 *  Created on: Nov 18, 2024
 *      Author: Fei
 */

#ifndef INC_SERVOMODULE_H_
#define INC_SERVOMODULE_H_

#define ServoHead1 0xEB
#define ServoHead2 0x90
#define ServoTail  0xFE



typedef struct _{
    uint8_t     head[2]     ; //帧头
    uint8_t     len         ; //数据长度
    uint8_t     data[50]	; //数据
    uint8_t     crc         ; //循环码
    uint8_t     check       ; //校验和
    uint8_t     end         ; //帧尾
}SendCmd_Typedef;

// 转台工作模式(电机开启时才有效)
typedef enum{
    TURNTABLE_CLOSE_LOOP    = 0xB0, //单杆闭环
    TURNTABLE_OPEN_LOOP     = 0xB1, //单杆开环
    TURNTABLE_GUIDE         = 0xB2, //外引导
    TURNTABLE_CU_TRACKING   = 0xB3, //粗电视跟踪
    TURNTABLE_SINGAL_DETEC  = 0xB4,  //单检测
    TURNTABLE_SCAN          = 0xB5  //扫描
}Turntable_Mode;

// 转台电机使能
typedef enum{
    MOTOR_ENBALE            = 0x66, //开启
    MOTOR_DISABLE           = 0x33  //关闭
}Motor_Enable;

// 引导数据有效
typedef enum{
    DATA_VALID              = 0x33, //数据有效
    DATA_INVALID            = 0x11, //数据无效
}Tracking_Data_Valid;

// 粗跟踪故障清除
typedef enum{
	CUTRACKING_FAULT_CLEAN    = 0x66, //清除
    CUTRACKING_FAULT_NO_CLEAN = 0x55,  //不清除
    JINGTRACKING_FAULT_CLEAN    = 0x33, //清除
    JINGTRACKING_FAULT_NO_CLEAN = 0x55  //不清除
}Tracking_Fault_Clean;



// 快反镜工作模式(默认发0xC0)
typedef enum{
    FSM_ZERO                = 0xC0, //回零
    FSM_DIRECTION           = 0xC1, //指向
    FSM_TRACKING            = 0xC2, //跟踪
    FSM_ZERO_CORRECTION     = 0xC3  //零点修正
}FSM_Mode;

// 精跟踪控制使能
typedef enum{
    ENABLE_JING             = 0x66, //开启
    DISABLE_JING            = 0x33  //关闭
}Control_Enable_Jing;

// 精电视鼠标点偏有效
typedef enum{
    MOUSE_POINT_VALID       = 0x00, //点偏无效
    MOUSE_POINT_INVALID     = 0x01  //点偏有效
}Mouse_Point_Jing;

// 转台跟踪状态
typedef enum
{
    TurntableTrackingState_CLOSE_LOOP = 0xB0, //单杆闭环
    TurntableTrackingState_OPEN_LOOP = 0xB1, //单杆开环
    TurntableTrackingState_GUIDE = 0xB2, //外引导
    TurntableTrackingState_CU_TRACKING = 0xB3, //粗电视跟踪
    TurntableTrackingState_SINGAL_DETEC = 0xB4, //单检测
}TurntableTrackingState;

// 快反镜跟踪状态
typedef enum
{
    FSMTrackingState_ZERO = 0xC0, //回零
    FSMTrackingState_DIRECTION = 0xC1, //指向
    FSMTrackingState_TRACKING = 0xC2, //跟踪
    FSMTrackingState_ZERO_CORRECTION = 0xC3, //零点修正
}FSMTrackingState;

// 电限位状态
typedef enum
{   
    
    PositionNormal = 0x00,// 正常
    PositionOverLimit = 0x01, //位置超上限
    PositionUnderLimit = 0x02, //位置超下限
}PositionState;

// 转台电机电源状态
typedef enum
{
    MotorPowerOn = 0x66, //电机电源开启
    MotorPowerOff = 0x33, //电机电源关闭
}MotorPowerState;

// 精跟踪控制使能反馈状态
typedef enum
{
    ControlEnable = 0x66, //控制使能
    ControlDisable = 0x33, //控制关闭
}ControlEnableState;



typedef struct _TrackingFaultCu_Typedef
{
    uint8_t errorCode[2];
    uint8_t CommunicationFault; //总控通信故障
    uint8_t EncoderFault; //编码器故障
    uint8_t DriverFaultA; //驱动器A故障
    uint8_t DriverFaultE; //驱动器E故障
    uint8_t OverSpeedA; //机架超速A轴
    uint8_t OverSpeedE; //机架超速E轴
    uint8_t ControlOverLimitA;// 机架控制量超限A轴
    uint8_t ControlOverLimitE;// 机架控制量超限E轴
    uint8_t PositionOverLimitE;// 转台E轴位置超上限
    uint8_t PositionUnderLimitE;// 转台E轴位置超下限
    uint8_t TrackingFaultCuByte1;// 粗电视故障
    uint8_t TrackingFaultCuByte2;
    uint8_t TrackingFaultCuByte3;
    uint8_t TrackingFaultCuByte4;
}TrackingFaultCu_Typedef;

typedef struct _TrackingFaultJing_Typedef
{
    uint8_t errorCode[2];
    uint8_t FSMControlOverLimitX;//快反镜控制量X轴超限
    uint8_t FSMControlOverLimitY;//快反镜控制量Y轴超限
    uint8_t FSMPositionOverLimitX;//快反镜位置X轴超限
    uint8_t FSMPositionOverLimitY;//快反镜位置Y轴超限
    uint8_t TrackingFaultJing;//精电视故障
}TrackingFaultJing_Typedef;

typedef struct _ServoRevTypedef
{
    uint8_t cCuOffsetState              ;
    uint8_t cJingOffsetState            ;
    uint8_t cJingISOffsetState          ;
    s16_u8_t CuOffset_X                 ;
    s16_u8_t CuOffset_Y                 ;
    s16_u8_t JingOffset_X               ;
    s16_u8_t JingOffset_Y               ;
    uint8_t cTurnTableTrackState        ;
    uint8_t cFSMTrackState              ;
    uint8_t cElectPositionState         ;
    uint8_t cTurnTableMotorPowerState   ;
    uint8_t cJingTrackControlState      ;
    f32_u8_t FSMXPos                    ;
    f32_u8_t FSMYPos                    ;
    f32_u8_t ServoYawPos                ;
    f32_u8_t ServoPitchPos              ;
    f32_u8_t YawSpeed                   ;
    f32_u8_t PitchSpeed                 ;
    f32_u8_t FSMXPosZero                ;
    f32_u8_t FSMYPosZero                ;
    f32_u8_t TurnYawOutput              ;
    f32_u8_t TurnPitchOutput            ;
    f32_u8_t FSMXOutput                 ;
    f32_u8_t FSMYOutput                 ;
    uint8_t ServoCrc                    ;
    uint8_t ServoCheck                  ;
}ServoRevTypedef_t;


// 补充报警码
typedef struct _SupplementaryAlarm_Typedef
{
    
}SupplementaryAlarm_Typedef;

typedef struct ServoSendData
{
    uint8_t TurnMode; //定义转台工作模式
    uint8_t MotorEnable; //定义转台电机使能
    int16_t Turntable_Yaw; //定义单杆方位 -2048~2048
    int16_t Turntable_Pitch; //定义单杆俯仰 -2048~2048
    uint8_t TrackingDataValid; //定义引导数据有效
    f32_u8_t TargetYaw; //定义目标方位位置 0-360°
    f32_u8_t TargetPitch; //定义目标俯仰位置 0-80°
    uint8_t TrackingFaultCleanCu; //定义粗跟踪故障清除
    uint8_t FSMMode; //定义快反镜工作模式
    uint8_t ControlEnableJing; //定义精跟踪控制使能
    f32_u8_t TargetFSMYaw; //快反镜指向方位位置 °
    f32_u8_t TargetFSMPitch; //快反镜指向俯仰位置 °
    f32_u8_t ZeroCorrectionFSMX; //快反镜x轴零点修正
    f32_u8_t ZeroCorrectionFSMY; //快反镜y轴零点修正
    uint16_t MousePointX; //定义精电视鼠标点偏x轴
    uint16_t MousePointY; //定义精电视鼠标点偏y轴
    uint8_t MousePointJing; //定义精电视鼠标点偏有效
    uint8_t TrackingFaultCleanJing; //定义精跟踪故障清除
    f32_u8_t AngleJing; //定义精电视相旋角度
}ServoSendData_Typedef;

extern SendCmd_Typedef ServoSendTypedef; //定义发送数据结构体

// extern uint8_t TurnMode; //定义转台工作模式
// extern uint8_t MotorEnable; //定义转台电机使能
// extern int16_t Turntable_Yaw; //定义单杆方位 -2048~2048
// extern int16_t Turntable_Pitch; //定义单杆俯仰 -2048~2048
// extern uint8_t TrackingDataValid; //定义引导数据有效
// extern f32_u8_t TargetYaw; //定义目标方位位置 0-360°
// extern f32_u8_t TargetPitch; //定义目标俯仰位置 0-80°
// extern uint8_t TrackingFaultCleanCu; //定义粗跟踪故障清除
// extern uint8_t FSMMode; //定义快反镜工作模式
// extern uint8_t ControlEnableJing; //定义精跟踪控制使能
// extern f32_u8_t TargetFSMYaw; //快反镜指向方位位置 °
// extern f32_u8_t TargetFSMPitch; //快反镜指向俯仰位置 °
// extern f32_u8_t ZeroCorrectionFSMX; //快反镜x轴零点修正
// extern f32_u8_t ZeroCorrectionFSMY; //快反镜y轴零点修正
// extern uint16_t MousePointX; //定义精电视鼠标点偏x轴
// extern uint16_t MousePointY; //定义精电视鼠标点偏y轴
// extern uint8_t MousePointJing; //定义精电视鼠标点偏有效
// extern uint8_t TrackingFaultCleanJing; //定义精跟踪故障清除
// extern f32_u8_t AngleJing; //定义精电视相旋角度
// extern uint8_t flagServo;

extern TrackingFaultCu_Typedef TrackingFaultCodeCu;
extern TrackingFaultJing_Typedef TrackingFaultCodeJing;
extern ServoRevTypedef_t ServoRevTypedef;
extern ServoSendData_Typedef ServoDataSendTypedef;



uint8_t CheckSum(uint8_t *ptr, uint8_t len);
uint8_t CRC8_Check(uint8_t *ptr, uint8_t len);
void ServoDataSend(SendCmd_Typedef *SendType,ServoSendData_Typedef *ServoData);
void ServoDataPack(SendCmd_Typedef *SendType,ServoSendData_Typedef *ServoData);

#endif /* INC_SERVOMODULE_H_ */
