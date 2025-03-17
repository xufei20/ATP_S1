/*
 * cmdPC.c
 *
 *  Created on: Nov 24, 2024
 *      Author: Fei
 */

#include "cmdPC.h"
#include "ServoModule.h"
#include "ImageModule.h"
#include "Focus.h"
#include "RangefinderModule.h"
#include "tcpecho.h"


uint8_t state = STATE_INIT;
uint8_t stateChange = 0;

sifuFlag_t sifuFlag = {
    .turnStateFlag = 0,
    .motorEnableFlag = 0,
    .trackingDataValidFlag = 0,
    .FSMModeFlag = 0,
    .controlEnableJingFlag = 0,
    .dgFlag = 0
};


CommandTypedef_t CommandTypedef = {
    .state = 0,
    .go2Zero = 0x00,
    .standbyEnable = 0x00, // 待机 默认开启
    .standbyDisable = 0,
    .guideEnable = 0x00,
    .catchEnable = 0,
    .manualEnable = 0,
    .trackCuEnable = 0,
    .trackJingEnable = 0x33,
    .rangefinderOpen = 0,
    .trackOpen = 0,
    .focusAdjust = 0,
    .turnYaw = {0},
    .turnPitch = {0},
    .FSM_Yaw = {0},
    .FSM_Pitch = {0},
    .cuFocusAdjust = 0,
    .zoomAdjust = 1,
    .exposureCU = 100,
    .exposureJING = 100,
    .turnYawSpeed = 0,
    .turnPitchSpeed = 0,
    .manual_center_x = 0,
    .manual_center_y = 0,
    .manual_width = 0,
    .manual_height = 0,
    .clearerror = 0,
    .x_offset = {0},
    .y_offset = {0},
    .setTurnState = 0,
    .setFSMState = 0,
    .motorEnable = 0,
    .laserAdjust = 0,
    .dgPitch = 0,
    .dgYaw = 0,
    .imgEnableCu = 0,
    .imgEnableJing = 0
};

uint8_t manual_flag = 1;
uint8_t expose_flag = 0;
uint8_t exposeJing_flag = 0;
uint8_t rangefander_flag = 0;
uint8_t rangefinder_close = 0;
uint8_t servo_flag = 0;
uint8_t zoom_flag = 0;
uint8_t focus_flag = 0;
uint8_t clearerror_flag = 0;
uint8_t error_cnt = 0;
uint8_t offset_flag = 0;
uint8_t laserChange = 0;
uint8_t imgFlag[2] = {0};

void processControl()
{
    if(state == STATE_INIT){
        
        RangefinderDataFrameSend.FuncCode = SetMultiFreq;
        cSetMultiFreq = 0x0A;
        Send2RangefinderModule(&RangefinderDataFrameSend);
        state = STATE_STANDBY;

//        setFocusPos.f = 0;
//        SendFocusFrame.func = SetPos;
//        SendFocusData(&SendFocusFrame);
         SendFocusFrame.func = ClearPos;
         SendFocusData(&SendFocusFrame);
    }



    // 系统状态 
    if(stateChange == 1)
    {
        stateChange = 0;
        if(CommandTypedef.go2Zero == 1)
        {
            // go2Zero(); //! 还没定下零位
        // HAL_GPIO_WritePin(RF_PWD_GPIO_Port,RF_PWD_Pin,GPIO_PIN_SET);
        // RangefinderDataFrameSend.FuncCode = SetMultiFreq;
        // cSetMultiFreq = 0x0A;
        // Send2RangefinderModule(&RangefinderDataFrameSend);
            state = STATE_ZERO;
        }
        else if(CommandTypedef.standbyEnable == 1)
        {
            standbyEnable();
            state = STATE_STANDBY;
        }
        else if(CommandTypedef.catchEnable == 1)
        {
            catchEnable();
            state = STATE_CATCH;
        }
        else if(CommandTypedef.guideEnable == 1)
        {
            guideEnable();
            state = STATE_GUIDE;
        }
        else if(CommandTypedef.standbyDisable == 1 && state == STATE_STANDBY)
        {
            standbyDisable();
            state = STATE_STANDBYDISABLE; // 待机状态进入待机禁止状态
        }
    }

    // 伺服状态
    if((servo_flag == 1) || (sifuFlag.turnStateFlag == 1) || (sifuFlag.FSMModeFlag == 1) || (sifuFlag.controlEnableJingFlag == 1) || (sifuFlag.motorEnableFlag == 1) || (sifuFlag.dgFlag == 1))
    {
        if(servo_flag == 1 && state == STATE_GUIDE)
        {
            servo_flag = 0;
            ServoDataSendTypedef.TargetYaw = CommandTypedef.turnYaw;
            ServoDataSendTypedef.TargetPitch = CommandTypedef.turnPitch;
            ServoDataSendTypedef.TargetFSMYaw = CommandTypedef.FSM_Yaw;
            ServoDataSendTypedef.TargetFSMPitch = CommandTypedef.FSM_Pitch;
            uart_printf("TargetYaw:%f,TargetPitch:%f,TargetFSMYaw:%f,TargetFSMPitch:%f\n",CommandTypedef.turnYaw.f,CommandTypedef.turnPitch.f,CommandTypedef.FSM_Yaw.f,CommandTypedef.FSM_Pitch.f);
        }
        if(sifuFlag.turnStateFlag == 1) // 设置转台工作模式
        {
            sifuFlag.turnStateFlag = 0;
            ServoDataSendTypedef.TurnMode = CommandTypedef.setTurnState;
            if(ServoDataSendTypedef.TurnMode == TurntableTrackingState_GUIDE)
            {
                ServoDataSendTypedef.TargetPitch.f = ServoRevTypedef.ServoPitchPos.f;
                ServoDataSendTypedef.TargetYaw.f = ServoRevTypedef.ServoYawPos.f;
                ServoDataSendTypedef.TargetFSMYaw.f = ServoRevTypedef.FSMXPos.f;
                ServoDataSendTypedef.TargetFSMPitch.f = ServoRevTypedef.FSMYPos.f;
            }
        }
        if(sifuFlag.FSMModeFlag == 1)
        {
            sifuFlag.FSMModeFlag = 0;
            ServoDataSendTypedef.FSMMode = CommandTypedef.setFSMState;
        }
        if(sifuFlag.controlEnableJingFlag == 1)
        {
            sifuFlag.controlEnableJingFlag = 0;
            ServoDataSendTypedef.ControlEnableJing = CommandTypedef.trackJingEnable;
            uart_printf("trackJingEnable:%d\n",CommandTypedef.trackJingEnable);
        }
        if(sifuFlag.motorEnableFlag == 1) // 设置转台电机使能
        {
            sifuFlag.motorEnableFlag = 0;
            ServoDataSendTypedef.MotorEnable = CommandTypedef.motorEnable;
        }
        if(ServoDataSendTypedef.TurnMode == TurntableTrackingState_CLOSE_LOOP && sifuFlag.dgFlag == 1)
        {
            sifuFlag.dgFlag = 0;
            ServoDataSendTypedef.Turntable_Pitch = CommandTypedef.dgPitch;
            ServoDataSendTypedef.Turntable_Yaw = CommandTypedef.dgYaw;
            // uart_printf("dgPitch:%d,dgYaw:%d\n",CommandTypedef.dgPitch,CommandTypedef.dgYaw);
        }
        ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);
    }

    if(laserChange == 1)
    {
        laserChange = 0;
        setFocusPos.f = (float)CommandTypedef.laserAdjust;
        SendFocusFrame.func = SetPos;
        SendFocusData(&SendFocusFrame);
    }

    if(manual_flag == 1 && state == STATE_CATCH) //手动模式
    {
        manual_flag = 0;
        if(CommandTypedef.manual_cj == 1)
        {
            ImgSendTypedef_CU.FuncCode = SETTING_TRACK;
            ImgSendTypedef_CU.FuncSubCode = MANUAL_TRACK;
            ImgSendDataTypedef_CU.ManualTracking.TrackingCenterX = CommandTypedef.manual_center_x;
            ImgSendDataTypedef_CU.ManualTracking.TrackingCenterY = CommandTypedef.manual_center_y;
            ImgSendDataTypedef_CU.ManualTracking.TrackingWidth = CommandTypedef.manual_width;
            ImgSendDataTypedef_CU.ManualTracking.TrackingHeight = CommandTypedef.manual_height;
            uart_printf("cu\n");
            Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
        }else if(CommandTypedef.manual_cj == 2){
            // TODO: 参数还需要改 这是粗相机的参数

            ImgSendTypedef_JING.FuncCode = SETTING_TRACK;
            ImgSendTypedef_JING.FuncSubCode = MANUAL_TRACK;
            ImgSendDataTypedef_JING.ManualTracking.TrackingCenterX = CommandTypedef.manual_center_x;
            ImgSendDataTypedef_JING.ManualTracking.TrackingCenterY = CommandTypedef.manual_center_y;
            ImgSendDataTypedef_JING.ManualTracking.TrackingWidth = CommandTypedef.manual_width;
            ImgSendDataTypedef_JING.ManualTracking.TrackingHeight = CommandTypedef.manual_height;
            Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);
            uart_printf("jing\n");
        }
        else if(CommandTypedef.manual_cj == 3)
        {
            ImgSendTypedef_CU.FuncCode = SETTING_TRACK;
            ImgSendTypedef_CU.FuncSubCode = STANDBY_MODE;
            Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
            ImgSendTypedef_JING.FuncCode = SETTING_TRACK;
            ImgSendTypedef_JING.FuncSubCode = STANDBY_MODE;
            Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);
            uart_printf("out\r\n");
        }
//                uart_printf("%d,%d",CommandTypedef.trackCuEnable,CommandTypedef.trackJingEnable);
        uart_printf("manual_cj:%d,TrackingCenterX:%d,TrackingCenterY:%d,TrackingWidth:%d,TrackingHeight:%d\n",CommandTypedef.manual_cj,CommandTypedef.manual_center_x,CommandTypedef.manual_center_y,CommandTypedef.manual_width,CommandTypedef.manual_height);
    }
    else if(manual_flag == 2 && state == STATE_CATCH) //自动模式
    {
        // manual_flag = 0;
        if(CommandTypedef.imgEnableCu == 1 && imgFlag[0] == 1) //!:条件待修改
        {
            imgFlag[0] = 0;
            ImgSendTypedef_CU.FuncCode = SETTING_TRACK;
            ImgSendTypedef_CU.FuncSubCode = AUTO_TRACK;
            Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
        }
        if(CommandTypedef.imgEnableJing == 1 && imgFlag[1] == 1) //!:条件待修改
        {
            imgFlag[1] = 0;
            ImgSendTypedef_JING.FuncCode = SETTING_TRACK;
            ImgSendTypedef_JING.FuncSubCode = AUTO_TRACK;  //TODO: 数据来源于上位机 待修改
            Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);
        }
        
    }


    // 曝光调节
    if(expose_flag == 1)
    {
        // 粗电视曝光调节
        ImgSendTypedef_CU.FuncCode = SETTING_DEVICEPARAM; 
        ImgSendDataTypedef_CU.exposure = CommandTypedef.exposureCU;
        ImgSendTypedef_CU.FuncSubCode = EXPOSURE;
        Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
        uart_printf("exposureCU:%ld\n",CommandTypedef.exposureCU);
        
        expose_flag = 0;
    }
    if(exposeJing_flag == 1)
    {
        ImgSendTypedef_JING.FuncCode = SETTING_DEVICEPARAM;
        ImgSendDataTypedef_JING.exposure = CommandTypedef.exposureJING;
        ImgSendTypedef_JING.FuncSubCode = EXPOSURE;
        Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);
        exposeJing_flag = 0;
    }

    if(focus_flag == 1)
    {
        // 粗电视调焦
        ImgSendTypedef_CU.FuncCode = SETTING_DEVICEPARAM;
        ImgSendDataTypedef_CU.focusAdjust = CommandTypedef.cuFocusAdjust;
        ImgSendTypedef_CU.FuncSubCode = FOCUS_ADJUST;
        Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
        focus_flag = 0;
    }
    if(zoom_flag == 1)
    {
        // 粗电视变倍调节
        ImgSendTypedef_CU.FuncCode = SETTING_DEVICEPARAM;
        ImgSendDataTypedef_CU.zoomAdjust = CommandTypedef.zoomAdjust;
        ImgSendTypedef_CU.FuncSubCode = ZOOM_ADJUST;
        uart_printf("zoomAdjust:%ld\n",CommandTypedef.zoomAdjust);
        Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
        zoom_flag = 0;
    }
    if(offset_flag == 1)
    {
        // 粗电视光学中心设置
        ImgSendTypedef_CU.FuncCode = SETTING_DEVICEPARAM;
        ImgSendDataTypedef_CU.offsetX = CommandTypedef.x_offset.u32t * 1024 / 640;
        ImgSendDataTypedef_CU.offsetY = CommandTypedef.y_offset.u32t * 1024 / 640;
        ImgSendTypedef_CU.FuncSubCode = OPTICAL_CENTER;
        Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
        offset_flag = 0;
    }

    // 测距机测距开启关闭
    if(rangefander_flag == 1)
    {
        RangefinderDataFrameSend.FuncCode = MultiMeasure;
        Send2RangefinderModule(&RangefinderDataFrameSend);
        rangefander_flag = 0;
    }else if(rangefinder_close == 1)
    {
        RangefinderDataFrameSend.FuncCode = StopMeasure;
        Send2RangefinderModule(&RangefinderDataFrameSend);
        rangefinder_close = 0;
    }

    // 清除故障
    if(clearerror_flag == 1 && error_cnt < 2)
    {
        error_cnt++;
        if(error_cnt == 1)
        {
            ServoDataSendTypedef.TrackingFaultCleanCu = 0x66;
            ServoDataSendTypedef.TrackingFaultCleanJing = 0x33;
        }else if(error_cnt == 2)
        {
            clearerror_flag = 0;
            error_cnt = 0;
        }
        ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);
    }

//     switch (state)
//     {
//         case STATE_INIT:
//         {
//             if(CommandTypedef.go2Zero == 0x01)
//             {
//                 go2Zero();
//                 uart_printf("go2Zero\n");
//                 state = STATE_ZERO;
//             }
//             else if(CommandTypedef.standbyEnable == 0x01)
//             {
//                 standbyEnable();
//                 uart_printf("standbyEnable\n");
//                 state = STATE_STANDBY;
//             }
//             else if(CommandTypedef.scanEnable == 0x01)
//             {
//                 scanEnable();
//                 uart_printf("scanEnable\n");
//                 state = STATE_SCAN;
//             }
//             else if(CommandTypedef.guideEnable == 0x01)
//             {
//                 guideEnable();
//                 uart_printf("guideEnable\n");
//                 state = STATE_GUIDE;
//             }
//             else if(CommandTypedef.manualEnable == 0x01)
//             {
//                 state = STATE_MANUAL;
//             }
//             else if (CommandTypedef.standbyDisable == 0x01)
//             {
//             //                standbyDisable();
//             //                uart_printf("standbyDisable\n");
//                         	ImgSendTypedef_CU.FuncCode = SETTING_TRACK;
//                         	ImgSendTypedef_CU.FuncSubCode = AUTO_TRACK;
//                         	Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
//                             state = STATE_STANDBYDISABLE; // 待机状态进入待机禁止状态
//                             uart_printf("STATE_STANDBYDISABLE\n");

//             }

//         }
//         break;
//         case STATE_ZERO:
//         {
//             if(CommandTypedef.standbyEnable == 0x01)
//             {
//                 standbyEnable();
//                 uart_printf("standbyEnable\n");
//                 state = STATE_STANDBY;
//             }
//             else if(CommandTypedef.scanEnable == 0x01)
//             {
//                 scanEnable();
//                 uart_printf("scanEnable\n");
//                 state = STATE_SCAN;
//             }
//             else if(CommandTypedef.guideEnable == 0x01)
//             {
//                 guideEnable();
//                 uart_printf("guideEnable\n");
//                 state = STATE_GUIDE;
//             }
//             else if(CommandTypedef.manualEnable == 0x01)
//             {
//                 state = STATE_MANUAL;
//             }
//             else if (CommandTypedef.standbyDisable == 0x01)
//             {
// //                standbyDisable();
// //                uart_printf("standbyDisable\n");
//             	ImgSendTypedef_CU.FuncCode = SETTING_TRACK;
//             	ImgSendTypedef_CU.FuncSubCode = AUTO_TRACK;
//             	Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
//                 state = STATE_STANDBYDISABLE; // 待机状态进入待机禁止状态

//             }
//         }break;
//         case STATE_STANDBY:
//         {
//             if (CommandTypedef.standbyDisable == 0x01)
//             {
// //                standbyDisable();
// //                uart_printf("standbyDisable\n");
//             	ImgSendTypedef_CU.FuncCode = SETTING_TRACK;
//             	ImgSendTypedef_CU.FuncSubCode = AUTO_TRACK;
//             	Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
//                 state = STATE_STANDBYDISABLE; // 待机状态进入待机禁止状态

//             }
//             else if (CommandTypedef.guideEnable == 0x01)
//             {
//                 guideEnable();
//                 uart_printf("guideEnable\n");
//                 state = STATE_GUIDE;          // 待机状态进入引导状态
//             }
//             else if(CommandTypedef.scanEnable == 0x01)
//             {
//                 scanEnable();
//                 uart_printf("scanEnable\n");
//                 state = STATE_SCAN;          // 待机状态进入扫描状态
//             }
//             else if(CommandTypedef.go2Zero == 0x01)
//             {
//                 go2Zero();
//                 uart_printf("go2Zero\n");
//                 state = STATE_ZERO;
//             }
//             else if(CommandTypedef.manualEnable == 0x01)
//             {
//                 state = STATE_MANUAL;
//             }
//         }break;
//         case STATE_STANDBYDISABLE:
//         {
//             if (CommandTypedef.standbyEnable == 0x01)
//             {
//                 standbyEnable();
//                 uart_printf("standbyEnable\n");
//                 state = STATE_STANDBY; // 待机禁止状态进入待机状态
//             }
//             else if(CommandTypedef.manualEnable == 0x01)
//             {
//                 state = STATE_MANUAL;
//             }
//             else if(CommandTypedef.go2Zero == 0x01)
//             {
//                 go2Zero();
//                 uart_printf("go2Zero\n");
//                 state = STATE_ZERO;
//             }
//         }break;
//         case STATE_GUIDE:
//         {
//             if(CommandTypedef.standbyEnable == 0x01)
//             {
//             	standbyEnable();
//                 uart_printf("standbyEnable\n");
//                 state = STATE_STANDBY;
//             }
//             else if (CommandTypedef.trackCuEnable == 1 && ImgRecvDataTypedef_CU.code2.CheckState == 1) //TODO:还需一个粗电视有目标的条件
//             {
//                 catchEnable();
//                 uart_printf("catchEnable\n");
//                 state = STATE_CATCH; // 跟踪状态进入捕获状态
//             }
//             else if(CommandTypedef.scanEnable == 0x01)
//             {
//                 scanEnable();
//                 uart_printf("scanEnable\n");
//                 state = STATE_SCAN; // 引导状态进入扫描状态
//             }
//             else if(CommandTypedef.manualEnable == 0x01)
//             {
//                 state = STATE_MANUAL;
//             }
//             else if(servo_flag == 1)
//             {
//                 ServoDataSendTypedef.TurnMode = TURNTABLE_GUIDE;
//                 ServoDataSendTypedef.MotorEnable = MotorPowerOn;
//                 ServoDataSendTypedef.TrackingDataValid = DATA_VALID;
//                 ServoDataSendTypedef.TargetYaw.f   = CommandTypedef.turnYaw.f;
//                 ServoDataSendTypedef.TargetPitch.f = CommandTypedef.turnPitch.f;
//                 ServoDataSendTypedef.FSMMode = FSM_DIRECTION;
//                 ServoDataSendTypedef.ControlEnableJing = ENABLE_JING;
//                 ServoDataSendTypedef.TargetFSMYaw.f = CommandTypedef.FSM_Yaw.f;
//                 ServoDataSendTypedef.TargetFSMPitch.f = CommandTypedef.FSM_Pitch.f;
//                 ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);
//                 servo_flag = 0;
//             }
//         }break;
//         case STATE_SCAN:
//         {
//             if(CommandTypedef.standbyEnable == 0x01)
//             {
//                 standbyEnable();
//                 uart_printf("standbyEnable\n");
//                 state = STATE_STANDBY;
//             }
//             else if(CommandTypedef.guideEnable == 0x01)
//             {
//                 guideEnable();
//                 uart_printf("guideEnable\n");
//                 state = STATE_GUIDE; // 扫描状态进入引导状态
//             }
//             else if(CommandTypedef.trackCuEnable == 1 && ImgRecvDataTypedef_CU.code2.CheckState == 1)
//             {
//                 catchEnable();
//                 uart_printf("catchEnable\n");
//                 state = STATE_CATCH; // 扫描状态进入捕获状态
//             }
//         }break;
//         case STATE_CATCH:
//         {
//             if(CommandTypedef.standbyEnable == 0x01)
//             {
//                 standbyEnable();
//                 uart_printf("standbyEnable\n");
//                 state = STATE_STANDBY;
//             }
//             else if(ImgRecvDataTypedef_CU.code2.TrackLose == 1) //TODO:捕获目标丢失 来自于转台的消息
//             {
//                 // guideEnable();
//                 scanEnable();
//                 uart_printf("scanEnable\n");
//                 state = STATE_SCAN; // 捕获状态进入引导状态或者扫描状态 这里需要判断
//             }
//             else {
//                 ServoDataSendTypedef.TurnMode = TURNTABLE_CU_TRACKING;
//                 ServoDataSendTypedef.MotorEnable = MotorPowerOn;
//                 if(ImgRecvDataTypedef_JING.code2.CheckState == 1 && CommandTypedef.trackJingEnable == 1)
//                 {
//                     ServoDataSendTypedef.ControlEnableJing = ENABLE_JING;
//                     ServoDataSendTypedef.FSMMode = FSM_TRACKING;
//                 }
//                 else if(ImgRecvDataTypedef_JING.code2.CheckState == 0 || CommandTypedef.trackJingEnable == 0)
//                 {
//                     ServoDataSendTypedef.ControlEnableJing = DISABLE_JING;
//                     ServoDataSendTypedef.FSMMode = FSM_ZERO;
//                 }
//                 ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);
//             }
//         }break;


//         #if testcode

//         if(manual_flag == 1)
//         {
//             manual_flag = 0;
//             if(CommandTypedef.trackCuEnable == 1)
//             {
//                 ImgSendTypedef_CU.FuncCode = SETTING_TRACK;
//                 ImgSendTypedef_CU.FuncSubCode = MANUAL_TRACK;
//                 ImgSendDataTypedef_CU.ManualTracking.TrackingCenterX = CommandTypedef.manual_center_x;
//                 ImgSendDataTypedef_CU.ManualTracking.TrackingCenterY = CommandTypedef.manual_center_y;
//                 ImgSendDataTypedef_CU.ManualTracking.TrackingWidth = CommandTypedef.manual_width;
//                 ImgSendDataTypedef_CU.ManualTracking.TrackingHeight = CommandTypedef.manual_height;
//                 uart_printf("粗跟踪\n");
//                 Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
//             }else if(CommandTypedef.trackJingEnable == 1){
//                 // TODO: 参数还需要改 这是粗相机的参数

//                 ImgSendTypedef_JING.FuncCode = SETTING_TRACK;
//                 ImgSendTypedef_JING.FuncSubCode = MANUAL_TRACK;
//                 ImgSendDataTypedef_JING.ManualTracking.TrackingCenterX = CommandTypedef.manual_center_x;
//                 ImgSendDataTypedef_JING.ManualTracking.TrackingCenterY = CommandTypedef.manual_center_y;
//                 ImgSendDataTypedef_JING.ManualTracking.TrackingWidth = CommandTypedef.manual_width;
//                 ImgSendDataTypedef_JING.ManualTracking.TrackingHeight = CommandTypedef.manual_height;
//                 Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);
//                 uart_printf("精跟踪\n");
//             }
// //                uart_printf("%d,%d",CommandTypedef.trackCuEnable,CommandTypedef.trackJingEnable);
//             uart_printf("TrackingCenterX:%d,TrackingCenterY:%d,TrackingWidth:%d,TrackingHeight:%d\n",CommandTypedef.manual_center_x,CommandTypedef.manual_center_y,CommandTypedef.manual_width,CommandTypedef.manual_height);
//         }

//         #elif
//         case STATE_MANUAL:
//         {
            
//             if(CommandTypedef.standbyEnable == 0x01)
//             {
//                 standbyEnable();
//                 uart_printf("standbyEnable\n");
//                 state = STATE_STANDBY;
//             }
//             else if(CommandTypedef.go2Zero == 0x01)
//             {
//                 go2Zero();
//                 uart_printf("go2Zero\n");
//                 state = STATE_ZERO;
//             }
//             else if(CommandTypedef.guideEnable == 0x01)
//             {
//                 guideEnable();
//                 uart_printf("guideEnable\n");
//                 state = STATE_GUIDE;
//             }
//             else if(CommandTypedef.scanEnable == 0x01)
//             {
//                 scanEnable();
//                 uart_printf("scanEnable\n");
//                 state = STATE_SCAN;
//             }
//             else if(manual_flag == 1){
//                 manual_flag = 0;
//                 if(CommandTypedef.trackCuEnable == 1)
//                 {
//                     ImgSendTypedef_CU.FuncCode = SETTING_TRACK;
//                     ImgSendTypedef_CU.FuncSubCode = MANUAL_TRACK;
//                     ImgSendDataTypedef_CU.ManualTracking.TrackingCenterX = CommandTypedef.manual_center_x;
//                     ImgSendDataTypedef_CU.ManualTracking.TrackingCenterY = CommandTypedef.manual_center_y;
//                     ImgSendDataTypedef_CU.ManualTracking.TrackingWidth = CommandTypedef.manual_width;
//                     ImgSendDataTypedef_CU.ManualTracking.TrackingHeight = CommandTypedef.manual_height;
//                     Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
//                 }else if(CommandTypedef.trackJingEnable == 1){
//                     // TODO: 参数还需要改 这是粗相机的参数

//                     ImgSendTypedef_JING.FuncCode = SETTING_TRACK;
//                     ImgSendTypedef_JING.FuncSubCode = MANUAL_TRACK;
//                     ImgSendDataTypedef_JING.ManualTracking.TrackingCenterX = CommandTypedef.manual_center_x;
//                     ImgSendDataTypedef_JING.ManualTracking.TrackingCenterY = CommandTypedef.manual_center_y;
//                     ImgSendDataTypedef_JING.ManualTracking.TrackingWidth = CommandTypedef.manual_width;
//                     ImgSendDataTypedef_JING.ManualTracking.TrackingHeight = CommandTypedef.manual_height;
//                     Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);
//                 }
//                 uart_printf("TrackingCenterX:%d,TrackingCenterY:%d,TrackingWidth:%d,TrackingHeight:%d\n",CommandTypedef.manual_center_x,CommandTypedef.manual_center_y,CommandTypedef.manual_width,CommandTypedef.manual_height);
//             }            
//         }
//         #endif
//     }
    
    
    //! 系统状态之间的切换
    // if(state == STATE_GUIDE)
    // {
    //     if(ImgRecvDataTypedef_CU.code2.CheckState == 1 && CommandTypedef.trackCuEnable == 1)
    //     {
    //         ServoDataSendTypedef.TurnMode = TURNTABLE_CU_TRACKING;
    //         ServoDataSendTypedef.MotorEnable = MotorPowerOn;
    //         ServoDataSendTypedef.TrackingDataValid = DATA_VALID;
    //         ServoDataSendTypedef.ControlEnableJing = ENABLE_JING;
    //         pcSend.cuEnable = 1; //判断当前跟踪状态
    //         ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);
    //     }
    //     else
    //     {
    //         ServoDataSendTypedef.TurnMode = TURNTABLE_GUIDE;
    //         pcSend.cuEnable = 0;
    //     }
    //     if(ImgRecvDataTypedef_JING.code2.CheckState == 1)
    //     {
    //         ServoDataSendTypedef.FSMMode = FSM_TRACKING;
    //         ServoDataSendTypedef.TurnMode = TURNTABLE_CU_TRACKING;
    //         ServoDataSendTypedef.MotorEnable = MotorPowerOn;
    //         ServoDataSendTypedef.TrackingDataValid = DATA_VALID;
    //         ServoDataSendTypedef.ControlEnableJing = ENABLE_JING;
    //         pcSend.jingEnable = 1;
    //         ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);
    //     }
    //     else
    //     {
    //         ServoDataSendTypedef.FSMMode = FSM_DIRECTION;
    //         pcSend.jingEnable = 0;
    //     }
    //     // ServoDataSendTypedef.TurnMode = TURNTABLE_GUIDE;
    //     // ServoDataSendTypedef.MotorEnable = MotorPowerOn;
    //     // ServoDataSendTypedef.TrackingDataValid = DATA_VALID;
    //     // ServoDataSendTypedef.TargetYaw.f   = CommandTypedef.turnYaw.f;
    //     // ServoDataSendTypedef.TargetPitch.f = CommandTypedef.turnPitch.f;
    //     // ServoDataSendTypedef.FSMMode = FSM_DIRECTION;
    //     // ServoDataSendTypedef.ControlEnableJing = ENABLE_JING;
    //     // ServoDataSendTypedef.TargetFSMYaw.f = CommandTypedef.FSM_Yaw.f;
    //     // ServoDataSendTypedef.TargetFSMPitch.f = CommandTypedef.FSM_Pitch.f;
    //     // ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);
    // }
    // // 还有一个速度控制
    // else if(state == STATE_SPEED)
    // {
    //     ServoDataSendTypedef.TurnMode = TURNTABLE_CLOSE_LOOP;
    //     ServoDataSendTypedef.MotorEnable = MotorPowerOn;
    //     ServoDataSendTypedef.Turntable_Yaw = CommandTypedef.turnYawSpeed;
    //     ServoDataSendTypedef.Turntable_Pitch = CommandTypedef.turnPitchSpeed;
    //     ServoDataSendTypedef.TrackingDataValid = DATA_VALID;
    //     ServoDataSendTypedef.TargetYaw.f   = ServoRevTypedef.ServoYawPos.f;
    //     ServoDataSendTypedef.TargetPitch.f = ServoRevTypedef.ServoPitchPos.f;
    //     ServoDataSendTypedef.FSMMode = FSM_DIRECTION;
    //     ServoDataSendTypedef.ControlEnableJing = ENABLE_JING;
    //     ServoDataSendTypedef.TargetFSMYaw.f = ServoRevTypedef.FSMXPosZero.f;
    //     ServoDataSendTypedef.TargetFSMPitch.f = ServoRevTypedef.FSMYPosZero.f;
    //     ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);
    // }
    // else if(state == STATE_STANDBYDISABLE)
    // {
    //     // ServoDataSendTypedef.TurnMode = TURNTABLE_OPEN_LOOP;
    //     // ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);
    // }
}

void go2init()
{
    // 转台先进单杆闭环 并清除故障
    ServoDataSendTypedef.TurnMode               = TURNTABLE_CLOSE_LOOP;
    ServoDataSendTypedef.MotorEnable            = MotorPowerOn;
    ServoDataSendTypedef.Turntable_Yaw          = SPEED_INIT_YAW;
    ServoDataSendTypedef.Turntable_Pitch        = SPEED_INIT_PITCH;
    ServoDataSendTypedef.TrackingDataValid      = DATA_VALID;
    ServoDataSendTypedef.TargetYaw.f            = POS_INIT_YAW;
    ServoDataSendTypedef.TargetPitch.f          = POS_INIT_PITCH;
    // ServoDataSendTypedef.TrackingFaultCleanCu   = CUTRACKING_FAULT_CLEAN;
    ServoDataSendTypedef.FSMMode                = FSM_ZERO;
    ServoDataSendTypedef.ControlEnableJing      = ENABLE_JING;
    ServoDataSendTypedef.TargetFSMYaw.f         = FSM_INIT_YAW;
    ServoDataSendTypedef.TargetFSMPitch.f       = FSM_INIT_PITCH;
    ServoDataSendTypedef.ZeroCorrectionFSMX.f   = 0.f;
    ServoDataSendTypedef.ZeroCorrectionFSMY.f   = 0.f;
    // ServoDataSendTypedef.TrackingFaultCleanJing   = JINGTRACKING_FAULT_CLEAN;
    ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);
    DelayMs(100);
    // 外引导回零
    ServoDataSendTypedef.TurnMode               = TURNTABLE_GUIDE;
    ServoDataSendTypedef.MotorEnable            = MotorPowerOn;
    ServoDataSendTypedef.Turntable_Yaw          = SPEED_INIT_YAW;
    ServoDataSendTypedef.Turntable_Pitch        = SPEED_INIT_PITCH;
    ServoDataSendTypedef.TrackingDataValid      = DATA_VALID;
    ServoDataSendTypedef.TargetYaw.f            = POS_INIT_YAW;
    ServoDataSendTypedef.TargetPitch.f          = POS_INIT_PITCH;
    // ServoDataSendTypedef.TrackingFaultCleanCu   = CUTRACKING_FAULT_CLEAN;
    ServoDataSendTypedef.FSMMode                = FSM_ZERO;
    ServoDataSendTypedef.ControlEnableJing      = ENABLE_JING;
    ServoDataSendTypedef.TargetFSMYaw.f         = FSM_INIT_YAW;
    ServoDataSendTypedef.TargetFSMPitch.f       = FSM_INIT_PITCH;
    ServoDataSendTypedef.ZeroCorrectionFSMX.f   = 0.f;
    ServoDataSendTypedef.ZeroCorrectionFSMY.f   = 0.f;
    // ServoDataSendTypedef.TrackingFaultCleanJing   = JINGTRACKING_FAULT_CLEAN;
    ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);


  

    // 粗电视回零  //TODO:数值都待确认 分包发送 6包  包括待机
    // ImgSendTypedef_CU.FuncCode = SETTING_DEVICEPARAM;
    // ImgSendTypedef_CU.FuncSubCode = FOCUS_ADJUST;
    // ImgSendDataTypedef_CU.focusAdjust = CommandTypedef.focusAdjust;
    // Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
    // DelayMs(2);
    // ImgSendDataTypedef_CU.zoomAdjust = CommandTypedef.zoomAdjust;
    // ImgSendTypedef_CU.FuncSubCode = ZOOM_ADJUST;
    // Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
    // DelayMs(2);
    // ImgSendDataTypedef_CU.exposure = 1000;
    // ImgSendTypedef_CU.FuncSubCode = EXPOSURE;
    // Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
    // DelayMs(2);
    // ImgSendTypedef_CU.FuncSubCode = OPTICAL_CENTER;
    // ImgSendDataTypedef_CU.offsetX = 1024;
    // ImgSendDataTypedef_CU.offsetY = 544;
    // Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
    // DelayMs(2);

    // 粗电视待机
    ImgSendTypedef_CU.FuncCode = SETTING_TRACK;
    ImgSendTypedef_CU.FuncSubCode = STANDBY_MODE;
    Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
    DelayMs(2);
    // ImgSendTypedef_CU.FuncCode = SETTING_IMGTRANS;
    // ImgSendTypedef_CU.FuncSubCode = 0x01;
    // ImgSendDataTypedef_CU.ImgTrans.ResolutionX = 2048;
    // ImgSendDataTypedef_CU.ImgTrans.ResolutionY = 1088;
    // ImgSendDataTypedef_CU.ImgTrans.OSDType = 0x01; //TODO:待确认
    // Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
    // DelayMs(2);

    // 精电视回零
    // ImgSendTypedef_JING.FuncCode = SETTING_DEVICEPARAM; 
    // ImgSendDataTypedef_JING.exposure = CommandTypedef.exposureJING;
    // ImgSendTypedef_JING.FuncSubCode = EXPOSURE;
    // Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);
    // ImgSendTypedef_JING.FuncSubCode = OPTICAL_CENTER;
    // ImgSendDataTypedef_JING.offsetX = 1024;
    // ImgSendDataTypedef_JING.offsetY = 544;
    // Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);

    // 精电视待机
    ImgSendTypedef_JING.FuncCode = SETTING_TRACK;
    ImgSendTypedef_JING.FuncSubCode = STANDBY_MODE;
    Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);
    
    // ImgSendTypedef_JING.FuncCode = SETTING_IMGTRANS;
    // ImgSendTypedef_JING.FuncSubCode = 0x01;
    // ImgSendDataTypedef_JING.ImgTrans.ResolutionX = 2048;
    // ImgSendDataTypedef_JING.ImgTrans.ResolutionY = 1088;
    // ImgSendDataTypedef_JING.ImgTrans.OSDType = 0x01; //TODO:待确认
    // Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);


    // 调焦回零 
    // TODO:还有清零
    SendFocusFrame.func = ClearPos;
    SendFocusData(&SendFocusFrame);

    // 测距机开机
    // TODO:开机 PF9
    HAL_GPIO_WritePin(RF_PWD_GPIO_Port,RF_PWD_Pin,GPIO_PIN_SET);
    RangefinderDataFrameSend.FuncCode = SetMultiFreq;
    cSetMultiFreq = 0x0A;
    Send2RangefinderModule(&RangefinderDataFrameSend);
}

void go2Zero() //! 回零需要确定数值，可最后调整
{
    // 转台回零
    ServoDataSendTypedef.TurnMode               = TURNTABLE_GUIDE;
    ServoDataSendTypedef.MotorEnable            = MotorPowerOn;
    ServoDataSendTypedef.Turntable_Yaw          = SPEED_INIT_YAW;
    ServoDataSendTypedef.Turntable_Pitch        = SPEED_INIT_PITCH;
    ServoDataSendTypedef.TrackingDataValid      = DATA_VALID;
    ServoDataSendTypedef.TargetYaw.f            = POS_INIT_YAW;
    ServoDataSendTypedef.TargetPitch.f          = POS_INIT_PITCH;
    // ServoDataSendTypedef.TrackingFaultCleanCu   = CUTRACKING_FAULT_CLEAN;
    ServoDataSendTypedef.FSMMode                = FSM_DIRECTION;
    ServoDataSendTypedef.ControlEnableJing      = ENABLE_JING;
    ServoDataSendTypedef.TargetFSMYaw.f         = FSM_INIT_YAW;
    ServoDataSendTypedef.TargetFSMPitch.f       = FSM_INIT_PITCH;
    ServoDataSendTypedef.ZeroCorrectionFSMX.f   = 0.f;
    ServoDataSendTypedef.ZeroCorrectionFSMY.f   = 0.f;
    // ServoDataSendTypedef.TrackingFaultCleanJing   = JINGTRACKING_FAULT_CLEAN;
    ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);

    // 粗电视回零  //TODO:数值都待确认 分包发送 6包  包括待机
    // ImgSendTypedef_CU.FuncCode = SETTING_DEVICEPARAM;
    // ImgSendTypedef_CU.FuncSubCode = FOCUS_ADJUST;
    // ImgSendDataTypedef_CU.focusAdjust = CommandTypedef.focusAdjust;
    // Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
    // DelayMs(2);
    // ImgSendDataTypedef_CU.zoomAdjust = CommandTypedef.zoomAdjust;
    // ImgSendTypedef_CU.FuncSubCode = ZOOM_ADJUST;
    // Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
    // DelayMs(2);
//    ImgSendDataTypedef_CU.exposure = CommandTypedef.exposureCU;
//    ImgSendTypedef_CU.FuncSubCode = EXPOSURE;
//    Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
//    DelayMs(2);
    // ImgSendTypedef_CU.FuncSubCode = OPTICAL_CENTER;
    // ImgSendDataTypedef_CU.offsetX = 1024;
    // ImgSendDataTypedef_CU.offsetY = 544;
    // Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
    // DelayMs(2);
    ImgSendTypedef_CU.FuncCode = SETTING_TRACK;
    ImgSendTypedef_CU.FuncSubCode = STANDBY_MODE;
    Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
    DelayMs(2);
    // ImgSendTypedef_CU.FuncCode = SETTING_IMGTRANS;
    // ImgSendTypedef_CU.FuncSubCode = 0x01;
    // ImgSendDataTypedef_CU.ImgTrans.ResolutionX = 2048;
    // ImgSendDataTypedef_CU.ImgTrans.ResolutionY = 1088;
    // ImgSendDataTypedef_CU.ImgTrans.OSDType = 0x01; //TODO:待确认
    // Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
    // DelayMs(2);

    // 精电视回零 
//    ImgSendTypedef_JING.FuncCode = SETTING_DEVICEPARAM;
//    ImgSendDataTypedef_JING.exposure = CommandTypedef.exposureJING;
//    ImgSendTypedef_JING.FuncSubCode = EXPOSURE;
//    Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);
//    DelayMs(2);
    // ImgSendTypedef_JING.FuncSubCode = OPTICAL_CENTER;
    // ImgSendDataTypedef_JING.offsetX = 1024;
    // ImgSendDataTypedef_JING.offsetY = 544;
    // Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);
    ImgSendTypedef_JING.FuncCode = SETTING_TRACK;
    ImgSendTypedef_JING.FuncSubCode = STANDBY_MODE;
    Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);
    // ImgSendTypedef_JING.FuncCode = SETTING_IMGTRANS;
    // ImgSendTypedef_JING.FuncSubCode = 0x01;
    // ImgSendDataTypedef_JING.ImgTrans.ResolutionX = 2048;
    // ImgSendDataTypedef_JING.ImgTrans.ResolutionY = 1088;
    // ImgSendDataTypedef_JING.ImgTrans.OSDType = 0x01; //TODO:待确认
    // Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);


    // 调焦回零 
    // TODO:还有清零
    setFocusPos.f = 0.f;
    SendFocusFrame.func = SetPos;
    SendFocusData(&SendFocusFrame);

    // 测距机回零
    RangefinderDataFrameSend.FuncCode = StopMeasure;
    Send2RangefinderModule(&RangefinderDataFrameSend);

}

void standbyEnable()
{
    // 电机使能
    ServoDataSendTypedef.TurnMode = TURNTABLE_GUIDE;
    ServoDataSendTypedef.MotorEnable = MotorPowerOn;
    ServoDataSendTypedef.Turntable_Yaw = 0;
    ServoDataSendTypedef.Turntable_Pitch = 0;
    ServoDataSendTypedef.TrackingDataValid = DATA_VALID;
    ServoDataSendTypedef.TargetYaw.f = ServoRevTypedef.ServoYawPos.f;
    ServoDataSendTypedef.TargetPitch.f = ServoRevTypedef.ServoPitchPos.f;
    ServoDataSendTypedef.FSMMode = FSM_DIRECTION;
    ServoDataSendTypedef.ControlEnableJing = ENABLE_JING;
    ServoDataSendTypedef.TargetFSMYaw.f = ServoRevTypedef.FSMXPos.f;
    ServoDataSendTypedef.TargetFSMPitch.f = ServoRevTypedef.FSMYPos.f;
    ServoDataSendTypedef.ZeroCorrectionFSMX.f = 0.f;
    ServoDataSendTypedef.ZeroCorrectionFSMY.f = 0.f;
    ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);


    // 粗电视待机
    ImgSendTypedef_CU.FuncCode = SETTING_TRACK;
    ImgSendTypedef_CU.FuncSubCode = STANDBY_MODE;
    Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
    DelayMs(2);

    // 精电视待机
    ImgSendTypedef_JING.FuncCode = SETTING_TRACK;
    ImgSendTypedef_JING.FuncSubCode = STANDBY_MODE;
    Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);

    // 调焦回零 
    // TODO:还有清零
    // setFocusPos.f = 4000.f;
    // SendFocusFrame.func = SetPos;
    // SendFocusData(&SendFocusFrame);

    // 测距机回零
    RangefinderDataFrameSend.FuncCode = StopMeasure;
    Send2RangefinderModule(&RangefinderDataFrameSend);
}

void standbyDisable()
{
    // 电机失能
    ServoDataSendTypedef.TurnMode = TURNTABLE_OPEN_LOOP;
    ServoDataSendTypedef.MotorEnable = MotorPowerOff;
    ServoDataSendTypedef.ControlEnableJing = DISABLE_JING;
    ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);
}

void guideEnable()
{
    // 电机使能
    ServoDataSendTypedef.TurnMode = TURNTABLE_GUIDE;
    ServoDataSendTypedef.MotorEnable = MotorPowerOn;
    ServoDataSendTypedef.ControlEnableJing = ENABLE_JING;
    ServoDataSendTypedef.TargetPitch.f = ServoRevTypedef.ServoPitchPos.f;
    ServoDataSendTypedef.TargetYaw.f = ServoRevTypedef.ServoYawPos.f;
    ServoDataSendTypedef.TrackingDataValid = DATA_VALID;
    ServoDataSendTypedef.FSMMode = FSM_DIRECTION;
    ServoDataSendTypedef.ControlEnableJing = ENABLE_JING;
    ServoDataSendTypedef.TargetFSMYaw.f = ServoRevTypedef.FSMXPos.f;
    ServoDataSendTypedef.TargetFSMPitch.f = ServoRevTypedef.FSMYPos.f;
    ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);

    // 粗电视引导  
    ImgSendTypedef_CU.FuncCode = SETTING_TRACK;
    // ImgSendTypedef_CU.FuncSubCode = MANUAL_TRACK;  //TODO: 数据来源于上位机 待修改
    
    ImgSendTypedef_CU.FuncSubCode = AUTO_TRACK;  //TODO: 数据来源于上位机 待修改
    Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);

    // 精电视引导
    ImgSendTypedef_JING.FuncCode = SETTING_TRACK;
    ImgSendTypedef_JING.FuncSubCode = AUTO_TRACK;
    Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);
}

void scanEnable()
{
    // 电机使能
    ServoDataSendTypedef.TurnMode = TURNTABLE_SCAN;
    ServoDataSendTypedef.MotorEnable = MotorPowerOn;
    ServoDataSendTypedef.ControlEnableJing = ENABLE_JING;
    ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);


    // ServoDataSendTypedef.TurnMode               = TurntableTrackingState_GUIDE;
    // ServoDataSendTypedef.MotorEnable            = MotorPowerOn;
    // ServoDataSendTypedef.Turntable_Yaw          = SPEED_INIT_YAW;
    // ServoDataSendTypedef.Turntable_Pitch        = SPEED_INIT_PITCH;
    // ServoDataSendTypedef.TrackingDataValid      = DATA_VALID;
    // ServoDataSendTypedef.TargetYaw.f            = POS_INIT_YAW;
    // ServoDataSendTypedef.TargetPitch.f          = POS_INIT_PITCH;
    // ServoDataSendTypedef.TrackingFaultCleanCu   = CUTRACKING_FAULT_CLEAN;
    // ServoDataSendTypedef.FSMMode                = FSM_ZERO;
    // ServoDataSendTypedef.ControlEnableJing      = ENABLE_JING;
    // ServoDataSendTypedef.TargetFSMYaw.f         = FSM_INIT_YAW;
    // ServoDataSendTypedef.TargetFSMPitch.f       = FSM_INIT_PITCH;
    // ServoDataSendTypedef.ZeroCorrectionFSMX.f   = 0.f;
    // ServoDataSendTypedef.ZeroCorrectionFSMY.f   = 0.f;
    // ServoDataSendTypedef.TrackingFaultCleanCu   = JINGTRACKING_FAULT_CLEAN;

    // 粗电视引导  
    ImgSendTypedef_CU.FuncCode = SETTING_TRACK;
    ImgSendTypedef_CU.FuncSubCode = AUTO_TRACK;  //TODO: 数据来源于上位机 待修改
    Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
    // // 粗电视调焦
    // ImgSendTypedef_CU.FuncCode = SETTING_DEVICEPARAM;
    // ImgSendTypedef_CU.FuncSubCode = FOCUS_ADJUST;
    // ImgSendDataTypedef_CU.focusAdjust = 0; //TODO: 数据来源于上位机 待修改
    // Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);

    // // 精电视引导
    ImgSendTypedef_JING.FuncCode = SETTING_TRACK;
    ImgSendTypedef_JING.FuncSubCode = AUTO_TRACK;
    Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);
}

void catchEnable()
{
    // 电机使能
    // ServoDataSendTypedef.TurnMode = TURNTABLE_CU_TRACKING;
    // ServoDataSendTypedef.MotorEnable = MotorPowerOn;
    // if(ImgRecvDataTypedef_JING.code2.CheckState == 1 && CommandTypedef.trackJingEnable == 1)
    // {
    //     ServoDataSendTypedef.ControlEnableJing = ENABLE_JING;
    //     ServoDataSendTypedef.FSMMode = FSM_TRACKING;
    // }
    // else if(ImgRecvDataTypedef_JING.code2.CheckState == 0 || CommandTypedef.trackJingEnable == 0)
    // {
    //     ServoDataSendTypedef.ControlEnableJing = DISABLE_JING;
    //     ServoDataSendTypedef.FSMMode = FSM_ZERO;
    // }
    // ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);

    // 粗电视捕获
    // ImgSendTypedef_CU.FuncCode = SETTING_TRACK;
    // ImgSendTypedef_CU.FuncSubCode = AUTO_TRACK;  //TODO: 数据来源于上位机 待修改
    // ImgSendDataTypedef_CU.focusAdjust = 0; //TODO: 数据来源于上位机 待修改
    // Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);

    // 精电视捕获
    // ImgSendTypedef_JING.FuncCode = SETTING_TRACK;
    // ImgSendTypedef_JING.FuncSubCode = AUTO_TRACK;
    // Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);

    // 电机使能
    ServoDataSendTypedef.TurnMode = TURNTABLE_GUIDE;
    ServoDataSendTypedef.MotorEnable = MotorPowerOn;
    ServoDataSendTypedef.Turntable_Yaw = 0;
    ServoDataSendTypedef.Turntable_Pitch = 0;
    ServoDataSendTypedef.TrackingDataValid = DATA_VALID;
    ServoDataSendTypedef.TargetYaw.f = ServoRevTypedef.ServoYawPos.f;
    ServoDataSendTypedef.TargetPitch.f = ServoRevTypedef.ServoPitchPos.f;
    ServoDataSendTypedef.FSMMode = FSM_DIRECTION;
    ServoDataSendTypedef.ControlEnableJing = ENABLE_JING;
    ServoDataSendTypedef.TargetFSMYaw.f = ServoRevTypedef.FSMXPos.f;
    ServoDataSendTypedef.TargetFSMPitch.f = ServoRevTypedef.FSMYPos.f;
    ServoDataSendTypedef.ZeroCorrectionFSMX.f = 0.f;
    ServoDataSendTypedef.ZeroCorrectionFSMY.f = 0.f;
    ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);
}
