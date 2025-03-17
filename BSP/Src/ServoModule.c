#include "main.h"
#include "ServoModule.h"

SendCmd_Typedef ServoSendTypedef = {
        .head = {ServoHead1, ServoHead2},
        .len = 0x2D,
        .data = {0},
        .crc = 0x00,
        .check = 0x00,
        .end = ServoTail
};

ServoSendData_Typedef ServoDataSendTypedef = {
    .TurnMode = 0xB2,
    .MotorEnable = 0x66,
    .Turntable_Yaw = 0,
    .Turntable_Pitch = 0,
    .TrackingDataValid = 0x11,
    .TargetYaw = {0},
    .TargetPitch = {0},
    .TrackingFaultCleanCu = 0x55,
    .FSMMode = 0xC0,
    .ControlEnableJing = 0x33,
    .TargetFSMYaw = {0},
    .TargetFSMPitch = {0},
    .ZeroCorrectionFSMX = {0},
    .ZeroCorrectionFSMY = {0},
    .MousePointX = 0,
    .MousePointY = 0,
    .MousePointJing = 0,
    .TrackingFaultCleanJing = 0x55,
    .AngleJing = {0}
};


// // 中断中更新数值
// uint8_t TurnMode = 0xB2; //定义转台工作模式
// uint8_t MotorEnable = 0x66; //定义转台电机使能
// int16_t Turntable_Yaw = 0; //定义单杆方位 -2048~2048
// int16_t Turntable_Pitch = 0; //定义单杆俯仰 -2048~2048
// uint8_t TrackingDataValid = 0x33; //定义引导数据有效
// f32_u8_t TargetYaw = {
//     .f = 40.f
// }; //定义目标方位位置 0-360°
// f32_u8_t TargetPitch = {
//     .f = 73.f
// }; //定义目标俯仰位置 0-80°
// uint8_t TrackingFaultCleanCu = 0x55; //定义粗跟踪故障清除
// uint8_t FSMMode = 0; //定义快反镜工作模式
// uint8_t ControlEnableJing = 0; //定义精跟踪控制使能
// f32_u8_t TargetFSMYaw; //快反镜指向方位位置 °
// f32_u8_t TargetFSMPitch; //快反镜指向俯仰位置 °
// f32_u8_t ZeroCorrectionFSMX; //快反镜x轴零点修正
// f32_u8_t ZeroCorrectionFSMY; //快反镜y轴零点修正
// uint16_t MousePointX = 0; //定义精电视鼠标点偏x轴
// uint16_t MousePointY = 0; //定义精电视鼠标点偏y轴
// uint8_t MousePointJing = 0; //定义精电视鼠标点偏有效
// uint8_t TrackingFaultCleanJing = 0; //定义精跟踪故障清除
// f32_u8_t AngleJing; //定义精电视相旋角度

// uint8_t flagServo = 0;
static uint8_t errorflag = 0;

ServoRevTypedef_t ServoRevTypedef;
TrackingFaultCu_Typedef TrackingFaultCodeCu;
TrackingFaultJing_Typedef TrackingFaultCodeJing;

// 和校验
uint8_t CheckSum(uint8_t *ptr, uint8_t len)
{
    uint8_t sum = 0;
    for(int i = 0;i < len;i++)
    {
        sum += ptr[i];
    }
    return sum;
}


void ServoDataPack(SendCmd_Typedef *SendType,ServoSendData_Typedef *ServoData)
{
    static uint8_t crc = 0;
    uint8_t check = 0;
    SendType->data[3] = ServoData->TurnMode;
    SendType->data[4] = ServoData->MotorEnable;

    #if BIG_ENDIAN
    SendType->data[5] = ServoData->Turntable_Yaw >> 8;
    SendType->data[6] = ServoData->Turntable_Yaw;
    SendType->data[7] = ServoData->Turntable_Pitch >> 8;
    SendType->data[8] = ServoData->Turntable_Pitch;
    #else
    SendType->data[5] = ServoData->Turntable_Yaw;
    SendType->data[6] = ServoData->Turntable_Yaw >> 8;
    SendType->data[7] = ServoData->Turntable_Pitch;
    SendType->data[8] = ServoData->Turntable_Pitch >> 8;
    #endif

    SendType->data[9] = ServoData->TrackingDataValid;

    for(int i = 0;i < 4;i++)
    {
        #if BIG_ENDIAN
        SendType->data[10+i] = ServoData->TargetYaw.u8t[3-i];
        SendType->data[14+i] = ServoData->TargetPitch.u8t[3-i];
        #else
        SendType->data[10+i] = ServoData->TargetYaw.u8t[i];
        SendType->data[14+i] = ServoData->TargetPitch.u8t[i];
        #endif
    }

    // if(TrackingFaultCodeCu.CommunicationFault || TrackingFaultCodeCu.EncoderFault || TrackingFaultCodeCu.DriverFaultA \
    //   || TrackingFaultCodeCu.DriverFaultE || TrackingFaultCodeCu.OverSpeedA || TrackingFaultCodeCu.OverSpeedE \
    //   || TrackingFaultCodeCu.ControlOverLimitA || TrackingFaultCodeCu.ControlOverLimitE || TrackingFaultCodeCu.PositionOverLimitE \
    //   || TrackingFaultCodeCu.PositionUnderLimitE || TrackingFaultCodeCu.TrackingFaultCuByte1 || TrackingFaultCodeCu.TrackingFaultCuByte2 \
    //   || TrackingFaultCodeCu.TrackingFaultCuByte3 || TrackingFaultCodeCu.TrackingFaultCuByte4)
    //   {
    //     errorflag = 1;
    //     ServoData->TrackingFaultCleanCu = 0x66;
    //   }
    //   else
    //   {
    //     ServoData->TrackingFaultCleanCu = 0x55;
    //     errorflag = 0;
    //   }
    if(ServoData->TrackingFaultCleanCu == 0x66)
    {
        SendType->data[18] = ServoData->TrackingFaultCleanCu;
        ServoData->TrackingFaultCleanCu = 0x55;     
    }else
    {
        SendType->data[18] = 0x55;
    }

    // SendType->data[18] = ServoData->TrackingFaultCleanCu;
    SendType->data[19] = ServoData->FSMMode;
    SendType->data[20] = ServoData->ControlEnableJing;

    for(int i = 0;i < 4;i++)
    {
        #if BIG_ENDIAN
        SendType->data[21+i] = ServoData->TargetFSMYaw.u8t[3-i];
        SendType->data[25+i] = ServoData->TargetFSMPitch.u8t[3-i];
        SendType->data[29+i] = ServoData->ZeroCorrectionFSMX.u8t[3-i];
        SendType->data[33+i] = ServoData->ZeroCorrectionFSMY.u8t[3-i];
        #else
        SendType->data[21+i] = ServoData->TargetFSMYaw.u8t[i];
        SendType->data[25+i] = ServoData->TargetFSMPitch.u8t[i];
        SendType->data[29+i] = ServoData->ZeroCorrectionFSMX.u8t[i];
        SendType->data[33+i] = ServoData->ZeroCorrectionFSMY.u8t[i];
        #endif
    }

    for(int i = 0;i < 2;i++)
    {
        #if BIG_ENDIAN
        SendType->data[37+i] = ServoData->MousePointX >> 8*(1-i);
        SendType->data[39+i] = ServoData->MousePointY >> 8*(1-i);
        #else
        SendType->data[37+i] = ServoData->MousePointX >> 8*i;
        SendType->data[39+i] = ServoData->MousePointY >> 8*i;
        #endif
    }

    SendType->data[41] = ServoData->MousePointJing;

    // if(TrackingFaultCodeJing.FSMControlOverLimitX || TrackingFaultCodeJing.FSMControlOverLimitY || TrackingFaultCodeJing.FSMPositionOverLimitX \
    //   || TrackingFaultCodeJing.FSMPositionOverLimitY || TrackingFaultCodeJing.TrackingFaultJing)
    //   {
    //     errorflag = 1;
    //     ServoData->TrackingFaultCleanJing = 0x33;
    //   }
    //   else
    //   {
    //     errorflag = 0;
    //     ServoData->TrackingFaultCleanJing = 0x55;
    //   }
    if(ServoData->TrackingFaultCleanJing == 0x33)
    {
        SendType->data[42] = ServoData->TrackingFaultCleanJing;
        ServoData->TrackingFaultCleanJing = 0x55;
    }else
    {
        SendType->data[42] = 0x55;
    }
    // SendType->data[42] = ServoData->TrackingFaultCleanJing;

    for(int i = 0;i < 4;i++)
    {
        #if BIG_ENDIAN
        SendType->data[43+i] = ServoData->AngleJing.u8t[3-i];
        #else
        SendType->data[43+i] = ServoData->AngleJing.u8t[i];
        #endif
    }
    SendType->crc = crc;
    SendType->data[0] = SendType->head[0];
    SendType->data[1] = SendType->head[1];
    SendType->data[2] = SendType->len;
    SendType->data[47] = crc;
    check = CheckSum(SendType->data, 48);
    SendType->data[48] = check;
    SendType->data[49] = SendType->end;
    crc++;

}

void ServoDataSend(SendCmd_Typedef *SendType,ServoSendData_Typedef *ServoData)
{
        ServoDataPack(SendType, ServoData);
        // 串口发送
        // for(int i = 0;i < 50;i++)
        // {
        //     uart_printf("%02x ", SendType->data[i]);
        //     // HAL_UART_Transmit(&huart5, SendType->data, 50, 0xffff);
        // }
        // uart_printf("\r\n");
        if(errorflag)
        {
            // HAL_UART_Transmit(&huart5, SendType->data, 50, 0xffff);
            errorflag = 0;
        }
        // uart_printf("errorflag = %d\r\n",errorflag);
        HAL_UART_Transmit(&huart5, SendType->data, 50, HAL_MAX_DELAY);
        // memset(SendType->data, 0, 50);
}
