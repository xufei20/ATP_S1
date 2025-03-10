#include "tcpecho.h"
#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/sys.h"
#include "cmsis_os.h"
#include "cmdPC.h"
#include "ImageModule.h"
#include "RangefinderModule.h"
#include "ServoModule.h"




#define PORT              5001
#define RECV_DATA         (1024)
#define addr             "192.168.0.100"
#define IP_ADDR          "192.168.1.80"
/*-------------------------------------------------------------*/
	int sock = -1;
    int sockw = -1;
    char *recv_data;
    __attribute__((section("._Text_Area"))) char senddata[128];
    struct sockaddr_in udp_addr,seraddr,sendaddr;

    uint32_t test = 0;
    int recv_data_len;
    socklen_t addrlen;
    f32_u8_t angle[4] = {0};

pcRecv_t pcRecv = {
    .focusAdjust_cu = 0,
    .zoomAdjust_cu = 0,
    .exposurecu = 1500,
    .exposurejing = 10000,
    .manual_center_x = 0,
    .manual_center_y = 0,
    .manual_width = 0,
    .manual_height = 0
};

pcSend_t pcSend = {
    .XoffsetCu.f = 0.f,
    .YoffsetCu.f = 0.f,
    .XoffsetJing.f = 0.f,
    .YoffsetJing.f = 0.f,
    .rangefinder.f = 0.f,
    .ServoYaw.f = 0.f,
    .ServoPitch.f = 0.f,
    .state = 0,
    .cuEnable = 0,
    .jingEnable = 0,
    .FSM_Pitch.f = 0,
    .FSM_Yaw.f = 0,
    .servoTrackingState = 0,
    .FSMTrackingState = 0,
    .cuErrorCode = {0},
    .jingErrorCode = {0},
    .cuOffsetState = 0,
    .jingOffsetState = 0
};

static void udpecho_thread(void *arg)
{
//    int sock = -1;
//    char *recv_data;
//    struct sockaddr_in udp_addr,seraddr;

//
//    int recv_data_len;
//    socklen_t addrlen;


    while (1)
    {
        recv_data = (char *)pvPortMalloc(RECV_DATA);
//        senddata = (char *)pvPortMalloc(128);
        if (recv_data == NULL)
        {
            // uart_printf("No memory\n");
            goto __exit;
        }

        sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0)
        {
        	// uart_printf("Socket error\n");
            goto __exit;
        }
        addrlen = sizeof(struct sockaddr);
        udp_addr.sin_family = AF_INET;
        udp_addr.sin_addr.s_addr = INADDR_ANY;
        udp_addr.sin_port = htons(PORT);
        memset(&(udp_addr.sin_zero), 0, sizeof(udp_addr.sin_zero));

        if (bind(sock, (struct sockaddr *)&udp_addr, sizeof(struct sockaddr)) == -1)
        {
            // uart_printf("Unable to bind\n");
            goto __exit;
        }

        while (1)
        {
//        	test++;
             recv_data_len=recvfrom(sock,recv_data,
                                 RECV_DATA,0,
                                 (struct sockaddr*)&seraddr,
                                 &addrlen);

             /*显示发送端的IP地址*/
            //  uart_printf("receive from %s\n",inet_ntoa(seraddr.sin_addr));

             /*显示发送端发来的字串*/
            //  uart_printf("recevce:%s\n",recv_data);
            for(int i = 0;i < recv_data_len;i++)
            {
                uart_printf("%02x ",recv_data[i]);
            }       
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
            //  uart_printf("recevce:%s,len:%d",mes,recv_data_len);

            if(recv_data[0] == 0x90 && recv_data[1] == 0x60 && recv_data[recv_data_len - 1] == 0xee)
            {
                #if testcode
                // --------------------------------------------------
                if(CommandTypedef.state != recv_data[3])
                {
                    stateChange = 1;
                }
                CommandTypedef.go2Zero          = recv_data[3] & bit(0) ? 1 : 0;
                CommandTypedef.standbyEnable    = recv_data[3] & bit(1) ? 1 : 0;
                CommandTypedef.standbyDisable   = recv_data[3] & bit(2) ? 1 : 0;
                CommandTypedef.guideEnable      = recv_data[3] & bit(3) ? 1 : 0;
                CommandTypedef.catchEnable      = recv_data[3] & bit(4) ? 1 : 0;
                CommandTypedef.state = recv_data[3];
                

                // uart_printf("gozeor:%d,standbyEnable:%d,standbyDisable:%d,guideEnable:%d,catchEnable:%d\n",CommandTypedef.go2Zero,CommandTypedef.standbyEnable,CommandTypedef.standbyDisable,CommandTypedef.guideEnable,CommandTypedef.catchEnable);

                if(CommandTypedef.rangefinderOpen == 0 && recv_data[4] == 1)
                {
                    rangefander_flag = 1;
                }
                if(CommandTypedef.rangefinderOpen == 1 && recv_data[4] == 0)
                {
                    rangefinder_close = 1;
                }
                if(CommandTypedef.laserAdjust != ((recv_data[6] << 8) | recv_data[5]))
                {
                    laserChange = 1;
                }

                CommandTypedef.rangefinderOpen  = recv_data[4];
                // 数据是低位在前高为在后
                CommandTypedef.laserAdjust = (recv_data[6] << 8) | recv_data[5];
                // uart_printf("laserAdjust:%d\n",CommandTypedef.laserAdjust);
                for(int i = 0;i < 4;i++)
                {
                    angle[0].u8t[i] = recv_data[7 + i];
                    angle[1].u8t[i] = recv_data[11 + i];
                    angle[2].u8t[i] = recv_data[15 + i];
                    angle[3].u8t[i] = recv_data[19 + i];
                }
                if((CommandTypedef.turnYaw.f != angle[0].f) || (CommandTypedef.turnPitch.f != angle[1].f) || (CommandTypedef.FSM_Yaw.f != angle[2].f) || (CommandTypedef.FSM_Pitch.f != angle[3].f))
                {
                    servo_flag = 1;
                }

                CommandTypedef.turnYaw.f        = angle[0].f;
                CommandTypedef.turnPitch.f      = angle[1].f;
                CommandTypedef.FSM_Yaw.f        = angle[2].f;
                CommandTypedef.FSM_Pitch.f      = angle[3].f;
                // uart_printf("turnYaw:%f,turnPitch:%f,FSM_Yaw:%f,FSM_Pitch:%f\n",CommandTypedef.turnYaw.f,CommandTypedef.turnPitch.f,CommandTypedef.FSM_Yaw.f,CommandTypedef.FSM_Pitch.f);
                
                static uint32_t temp[2];
                static uint32_t exp_temp[2];
                static int32_t offset[2];

                temp[0] = (recv_data[26]<<24) | (recv_data[25]<<16) | (recv_data[24]<<8) | recv_data[23];
                if(CommandTypedef.zoomAdjust != temp[0])
                {
                    // uart_printf("zoomAdjust:%ld,temp:%ld\n",CommandTypedef.zoomAdjust,temp[0]);
                    zoom_flag = 1;
                }
                CommandTypedef.zoomAdjust = temp[0];
                CommandTypedef.clearerror = recv_data[27];
                CommandTypedef.manualEnable = recv_data[28];
                if(CommandTypedef.manualEnable == 1)
                {
                    manual_flag = 1;
                }else if(CommandTypedef.manualEnable == 2)
                {
                    manual_flag = 2;
                }
                if(CommandTypedef.clearerror == 1)
                {
                    clearerror_flag = 1;
                }
                CommandTypedef.manual_center_x = (recv_data[30] << 8) | recv_data[29];
                CommandTypedef.manual_center_y = (recv_data[32] << 8) | recv_data[31];
                CommandTypedef.manual_width = (recv_data[34] << 8) | recv_data[33];
                CommandTypedef.manual_height = (recv_data[36] << 8) | recv_data[35];
                // uart_printf("manual_center_x:%d,manual_center_y:%d,manual_width:%d,manual_height:%d\n",CommandTypedef.manual_center_x,CommandTypedef.manual_center_y,CommandTypedef.manual_width,CommandTypedef.manual_height);
                CommandTypedef.manual_cj = recv_data[37];
                if(CommandTypedef.setTurnState != recv_data[38])
                {
                    sifuFlag.turnStateFlag = 1;
                }
                if(CommandTypedef.setFSMState != recv_data[39])
                {
                    sifuFlag.FSMModeFlag = 1;
                }
                if(CommandTypedef.trackJingEnable != recv_data[40])
                {
                    sifuFlag.controlEnableJingFlag = 1;
                }
                if(ServoDataSendTypedef.MotorEnable != recv_data[41])
                {
                    sifuFlag.motorEnableFlag = 1;
                }
                CommandTypedef.setTurnState = recv_data[38];
                CommandTypedef.setFSMState = recv_data[39];
                CommandTypedef.trackJingEnable = recv_data[40];
                CommandTypedef.motorEnable = recv_data[41];
                ServoDataSendTypedef.TrackingDataValid = recv_data[42];
                exp_temp[0] = recv_data[43] | (recv_data[44] << 8) | (recv_data[45] << 16) | (recv_data[46] << 24);
                exp_temp[1] = recv_data[47] | (recv_data[48] << 8) | (recv_data[49] << 16) | (recv_data[50] << 24);
                if(CommandTypedef.exposureCU != exp_temp[0])
                {
                    expose_flag = 1;
                }
                if(CommandTypedef.exposureJING != exp_temp[1])
                {
                    exposeJing_flag = 1;
                }
                CommandTypedef.exposureCU = exp_temp[0];
                CommandTypedef.exposureJING = exp_temp[1];
                // CommandTypedef.trackJingEnable = 1;
                // ----------------------------------------------
                #else
                CommandTypedef.go2Zero          = recv_data[3] & bit(0) ? 1 : 0;
                CommandTypedef.standbyEnable    = recv_data[3] & bit(1) ? 1 : 0;
                CommandTypedef.standbyDisable   = recv_data[3] & bit(2) ? 1 : 0;
                CommandTypedef.guideEnable      = recv_data[3] & bit(3) ? 1 : 0;
                CommandTypedef.scanEnable       = recv_data[3] & bit(4) ? 1 : 0;
                CommandTypedef.catchEnable      = recv_data[3] & bit(5) ? 1 : 0;
                CommandTypedef.manualEnable     = recv_data[3] & bit(6) ? 1 : 0;
                CommandTypedef.clearerror       = recv_data[3] & bit(7) ? 1 : 0; //TODO:清除错误，还要修改
                if(CommandTypedef.clearerror == 1)
                {
                    clearerror_flag = 1;
                }
                if(CommandTypedef.manualEnable == 1)
                {
                    manual_flag = 1;
                }
                
                uart_printf("gozeor:%d,standbyEnable:%d,standbyDisable:%d,guideEnable:%d,scanEnable:%d,catchEnable:%d,manualEnable:%d\n",CommandTypedef.go2Zero,CommandTypedef.standbyEnable,CommandTypedef.standbyDisable,CommandTypedef.guideEnable,CommandTypedef.scanEnable,CommandTypedef.catchEnable,CommandTypedef.manualEnable);

                CommandTypedef.trackCuEnable    = recv_data[4];
                CommandTypedef.trackJingEnable  = recv_data[5];
                uart_printf("cu:%d,jing:%d",CommandTypedef.trackCuEnable,CommandTypedef.trackJingEnable);
                CommandTypedef.trackOpen        = recv_data[6];
                if(CommandTypedef.rangefinderOpen == 0 && recv_data[7] == 1)
                {
                    rangefander_flag = 1;
                }
                if(CommandTypedef.rangefinderOpen == 1 && recv_data[7] == 0)
                {
                    rangefinder_close = 1;
                }
                CommandTypedef.rangefinderOpen  = recv_data[7];

                CommandTypedef.focusAdjust      = (recv_data[8] << 8) | recv_data[9];

                for(int i = 0;i < 4;i++)
                {
                    angle[i].u8t[3 - i] = recv_data[10 + i];
                }

                if((CommandTypedef.turnYaw.f != angle[0].f) || (CommandTypedef.turnPitch.f != angle[1].f) || (CommandTypedef.FSM_Yaw.f != angle[2].f) || (CommandTypedef.FSM_Pitch.f != angle[3].f))
                {
                    servo_flag = 1;
                }
                for(int i = 0;i < 4;i++)
                {
                    CommandTypedef.turnYaw.u8t[3 - i] = recv_data[10 + i];
                    CommandTypedef.turnPitch.u8t[3 - i] = recv_data[14 + i];
                    CommandTypedef.FSM_Yaw.u8t[3 - i] = recv_data[18 + i];
                    CommandTypedef.FSM_Pitch.u8t[3 - i] = recv_data[22 + i];
                }

                // CommandTypedef.turnYaw.f        = (recv_data[10] << 24) | (recv_data[11] << 16) | (recv_data[12] << 8) | recv_data[13];
                // CommandTypedef.turnPitch.f      = (recv_data[14] << 24) | (recv_data[15] << 16) | (recv_data[16] << 8) | recv_data[17];

                // CommandTypedef.FSM_Yaw.f        = (recv_data[18] << 24) | (recv_data[19] << 16) | (recv_data[20] << 8) | recv_data[21];
                // CommandTypedef.FSM_Pitch.f      = (recv_data[22] << 24) | (recv_data[23] << 16) | (recv_data[24] << 8) | recv_data[25];
                static uint32_t temp[4];
                static int32_t offset[2];
                temp[0] = recv_data[26] << 24 | recv_data[27] << 16 | recv_data[28] << 8 | recv_data[29];
                temp[1] = recv_data[30] << 24 | recv_data[31] << 16 | recv_data[32] << 8 | recv_data[33];
                if(CommandTypedef.cuFocusAdjust != temp[0])
                {
                    uart_printf("cuFocusAdjust:%ld,temp:%ld\n",CommandTypedef.cuFocusAdjust,temp[0]);
                    focus_flag = 1;
                }
                if(CommandTypedef.zoomAdjust != temp[1])
                {
                    uart_printf("zoomAdjust:%ld,temp:%ld\n",CommandTypedef.zoomAdjust,temp[1]);
                    zoom_flag = 1;
                }

                CommandTypedef.cuFocusAdjust    = (recv_data[26] << 24) | (recv_data[27] << 16) | (recv_data[28] << 8) | recv_data[29];
                CommandTypedef.zoomAdjust       = (recv_data[30] << 24) | (recv_data[31] << 16) | (recv_data[32] << 8) | recv_data[33];
                
                temp[2] = recv_data[34] << 24 | recv_data[35] << 16 | recv_data[36] << 8 | recv_data[37];
                temp[3] = recv_data[38] << 24 | recv_data[39] << 16 | recv_data[40] << 8 | recv_data[41];
                // if((CommandTypedef.exposureCU != (recv_data[34] << 24) | (recv_data[35] << 16) | (recv_data[36] << 8) | recv_data[37]) || (CommandTypedef.exposureJING != (recv_data[38] << 24) | (recv_data[39] << 16) | (recv_data[40] << 8) | recv_data[41]))
                if(CommandTypedef.exposureCU != temp[2] || CommandTypedef.exposureJING != temp[3])
                {
                    expose_flag = 1;
                }
                CommandTypedef.exposureCU       = (recv_data[34] << 24) | (recv_data[35] << 16) | (recv_data[36] << 8) | recv_data[37];
                CommandTypedef.exposureJING     = (recv_data[38] << 24) | (recv_data[39] << 16) | (recv_data[40] << 8) | recv_data[41];
                
                
                CommandTypedef.manual_center_x  = (recv_data[43] << 8) | recv_data[42];
                CommandTypedef.manual_center_y  = (recv_data[45] << 8) | recv_data[44];
                CommandTypedef.manual_width     = (recv_data[47] << 8) | recv_data[46];
                CommandTypedef.manual_height    = (recv_data[49] << 8) | recv_data[48];

                // offset[0] = (recv_data[53] << 24) | (recv_data[52] << 16) | (recv_data[51] << 8) | recv_data[50];
                // offset[1] = (recv_data[57] << 24) | (recv_data[56] << 16) | (recv_data[55] << 8) | recv_data[54];

                // 大端
                offset[0] = (recv_data[50] << 24) | (recv_data[51] << 16) | (recv_data[52] << 8) | recv_data[53];
                offset[1] = (recv_data[54] << 24) | (recv_data[55] << 16) | (recv_data[56] << 8) | recv_data[57];

                if(CommandTypedef.x_offset != offset[0] || CommandTypedef.y_offset != offset[1])
                {
                    offset_flag = 1;
                }
                CommandTypedef.x_offset         = offset[0];
                CommandTypedef.y_offset         = offset[1];
                #endif

                // CommandTypedef.x_offset         = (recv_data[53] << 24) | (recv_data[52] << 16) | (recv_data[51] << 8) | recv_data[50];
                // CommandTypedef.y_offset         = (recv_data[57] << 24) | (recv_data[56] << 16) | (recv_data[55] << 8) | recv_data[54];
                
                // uart_printf("cuFocusAdjust:%ld,zoomAdjust:%ld,exposureCU:%ld,exposureJING:%ld\n",CommandTypedef.cuFocusAdjust,CommandTypedef.zoomAdjust,CommandTypedef.exposureCU,CommandTypedef.exposureJING);
                // uart_printf("42:%02x,43:%02x,44:%02x,45:%02x,46:%02x,47:%02x,48:%02x\n",recv_data[42],recv_data[43],recv_data[44],recv_data[45],recv_data[46],recv_data[47],recv_data[48]);
                // uart_printf("x:%d,y:%d,width:%d,height:%d\n",CommandTypedef.manual_center_x,CommandTypedef.manual_center_y,CommandTypedef.manual_width,CommandTypedef.manual_height);
//                for(int i = 0;i<recv_data_len;i++)
//                {
//                    uart_printf("%02x ",recv_data[i]);
//                }

                // 打印数据
//                uart_printf("CommandTypedef.go2Zero:%d\n",CommandTypedef.go2Zero);
//                uart_printf("CommandTypedef.standbyEnable:%d\n",CommandTypedef.standbyEnable);
//                uart_printf("CommandTypedef.standbyDisable:%d\n",CommandTypedef.standbyDisable);
//                uart_printf("CommandTypedef.guideEnable:%d\n",CommandTypedef.guideEnable);
//                uart_printf("CommandTypedef.scanEnable:%d\n",CommandTypedef.scanEnable);
//                uart_printf("CommandTypedef.catchEnable:%d\n",CommandTypedef.catchEnable);
//                uart_printf("CommandTypedef.trackCuEnable:%d\n",CommandTypedef.trackCuEnable);
//                uart_printf("CommandTypedef.trackJingEnable:%d\n",CommandTypedef.trackJingEnable);
//                uart_printf("CommandTypedef.trackOpen:%d\n",CommandTypedef.trackOpen);
//                uart_printf("CommandTypedef.rangefinderOpen:%d\n",CommandTypedef.rangefinderOpen);
//                uart_printf("CommandTypedef.focusAdjust:%d\n",CommandTypedef.focusAdjust);
                // uart_printf("CommandTypedef.turnYaw:%f\n",CommandTypedef.turnYaw.f);
                // uart_printf("CommandTypedef.turnPitch:%f\n",CommandTypedef.turnPitch.f);
//                uart_printf("CommandTypedef.FSM_Yaw:%f\n",CommandTypedef.FSM_Yaw.f);
//                uart_printf("CommandTypedef.FSM_Pitch:%f\n",CommandTypedef.FSM_Pitch.f);
//                uart_printf("CommandTypedef.cuFocusAdjust:%ld\n",CommandTypedef.cuFocusAdjust);
//                uart_printf("CommandTypedef.zoomAdjust:%ld\n",CommandTypedef.zoomAdjust);
            //    uart_printf("CommandTypedef.exposureCU:%ld\n",CommandTypedef.exposureCU);
            //    uart_printf("CommandTypedef.exposureJING:%ld\n",CommandTypedef.exposureJING);



                // CommandTypedef.go2Zero = recv_data[3];
                // CommandTypedef.standbyEnable = recv_data[4];
                // CommandTypedef.standbyDisable = recv_data[5];
                // CommandTypedef.guideEnable = recv_data[6];
                // CommandTypedef.catchEnable = recv_data[7];
                // CommandTypedef.trackCuEnable = recv_data[8];
                // CommandTypedef.trackJingEnable = recv_data[9];
            }

            /*将字串返回给发送端*/
//           if(sendto(sock,senddata,
//                   128,0,
//                   (struct sockaddr*)&seraddr,
//                   addrlen) > 0){
//           	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//           }
//           sendto(sock,recv_data,
//               recv_data_len,0,
//               (struct sockaddr*)&seraddr,
//               addrlen);
             osDelay(10);
        }

__exit:
        if (sock >= 0) closesocket(sock);
        if (recv_data) free(recv_data);
    }
}

// char data[256];

static void WriteEthTask(void *thread_param)
{
    //  for(int i = 0;i<128;i++)
    //  {
    //         senddata[i] = i;
    //  }
     while(1)
     {
        //  senddata = (char *)pvPortMalloc(128);
        sockw = socket(AF_INET, SOCK_DGRAM, 0);
        // if (senddata == NULL)
        // {
        //     // uart_printf("No memory\n");
        //     goto __exit;
        // }
        
        addrlen = sizeof(struct sockaddr);
        sendaddr.sin_family = AF_INET;
        sendaddr.sin_addr.s_addr = inet_addr(IP_ADDR);
        sendaddr.sin_port = htons(10802);

        //TODO :需要删除
        // RangeFinderRevData.Distance = 10.2f;
        // ServoRevTypedef.ServoYawPos.f = 100.f;
        // ServoRevTypedef.ServoPitchPos.f = 52.4;
        // ServoRevTypedef.FSMXPos.f = 30.f;
        // ServoRevTypedef.FSMYPos.f = 40.f;
//        if (bind(sock, (struct sockaddr *)&sendaddr, sizeof(struct sockaddr)) == -1)
//        {
//            // uart_printf("Unable to bind\n");
//            goto __exit;
//        }
        memset(&(sendaddr.sin_zero), 0, sizeof(sendaddr.sin_zero));
        while(1)
        {
            senddata[0] = 0x90;
            senddata[1] = 0x60;
            pcSend.XoffsetCu.f = ImgRecvDataTypedef_CU.Offset_X;
            pcSend.YoffsetCu.f = ImgRecvDataTypedef_CU.Offset_Y;
            pcSend.XoffsetJing.f = ImgRecvDataTypedef_JING.Offset_X;
            pcSend.YoffsetJing.f = ImgRecvDataTypedef_JING.Offset_Y;
            pcSend.rangefinder.f = RangeFinderRevData.Distance;
            // pcSend.ServoYaw.f = ServoRevTypedef.ServoYawPos.f;
            pcSend.ServoYaw.f += 0.01;
            pcSend.ServoPitch.f = ServoRevTypedef.ServoPitchPos.f;
            // pcSend.state = state;
            // pcSend.cuEnable = 0;
            // pcSend.jingEnable = 0;
            for(int i = 0;i < 4;i++)
            {
                senddata[2+i] = pcSend.XoffsetCu.u8t[i];
                senddata[6+i] = pcSend.YoffsetCu.u8t[i];
                senddata[10+i] = pcSend.XoffsetJing.u8t[i];
                senddata[14+i] = pcSend.YoffsetJing.u8t[i];
                senddata[18+i] = pcSend.rangefinder.u8t[i];
                senddata[22+i] = pcSend.ServoYaw.u8t[i];
                senddata[26+i] = pcSend.ServoPitch.u8t[i];
            }
            // senddata[30] = pcSend.state;
            switch(state){
                case STATE_GUIDE:
                    senddata[30] = bit(3);
                    break;
                case STATE_CATCH:
                    senddata[30] = bit(4);
                    break;
                case STATE_STANDBY:
                    senddata[30] = bit(1);
                    break;
                case STATE_STANDBYDISABLE:
                    senddata[30] = bit(2);
                    break;
                case STATE_ZERO:
                    senddata[30] = bit(0);
                    break;
            }
            senddata[31] = pcSend.cuEnable; //TODO: 还未赋值粗跟踪状态
            senddata[32] = pcSend.jingEnable; //TODO: 还未赋值粗跟踪状态
            
            pcSend.FSM_Yaw.f = ServoRevTypedef.FSMXPos.f;
            pcSend.FSM_Pitch.f = ServoRevTypedef.FSMYPos.f;

            for(int i = 0;i < 4;i++)
            {
                senddata[33+i] = pcSend.FSM_Yaw.u8t[i];
                senddata[37+i] = pcSend.FSM_Pitch.u8t[i];
            }
            pcSend.servoTrackingState = ServoRevTypedef.cTurnTableTrackState;
            pcSend.FSMTrackingState = ServoRevTypedef.cFSMTrackState;
            // senddata[41] = pcSend.servoTrackingState;
            // senddata[42] = pcSend.FSMTrackingState;
            senddata[41] = pcSend.servoTrackingState;
            senddata[42] = pcSend.FSMTrackingState;
            for(int i = 0;i < 2;i++)
            {
                senddata[43+i] = pcSend.cuErrorCode[i];
                senddata[45+i] = pcSend.jingErrorCode[i];
            }
            senddata[47] = pcSend.cuOffsetState; //TODO: 还未赋值
            senddata[48] = pcSend.jingOffsetState; //TODO: 还未赋值
            // senddata[49] = TrackingFaultCodeCu.errorCode[0];
            // senddata[50] = TrackingFaultCodeCu.errorCode[1];
            // senddata[51] = TrackingFaultCodeJing.errorCode[0];
            // senddata[52] = TrackingFaultCodeJing.errorCode[1];

            senddata[53] = 0xee;
            sendto(sockw,senddata,54,0,(struct sockaddr*)&sendaddr,addrlen);
            // sendto(sockw,senddata,52,0,(struct sockaddr*)&sendaddr,addrlen);

            // memset(senddata,0xff,128);


            // int result = sendto(sockw,senddata,128,0,(struct sockaddr*)&sendaddr,addrlen);
            // if (result < 0) {
            //     if (errno == EWOULDBLOCK || errno == EAGAIN) {
            //         // uart_printf("发送缓冲区满，非阻塞模式下返回\n");
            //     } else {
            //         // uart_printf("发送失败，错误码: %d\n", errno);
            //     }
            // }
            // if(sendto(sockw,data,128,0,(struct sockaddr*)&sendaddr,addrlen) > 0)
            // {
//                HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
            // }
            osDelay(50);
        }
// __exit:
//         if (senddata) free(senddata);
      }

}


/*---------------------------------------------------------------------*/
void udpecho_init(void)
{
    sys_thread_new("udpecho_thread", udpecho_thread, NULL, 2048, osPriorityAboveNormal);
    sys_thread_new("writeEthTask", WriteEthTask, NULL, 1024, osPriorityAboveNormal);
}

uint8_t Eth_recv_buff[256];
uint8_t Eth_recv_data[256];
//int sock = -1;
struct sockaddr_in client_addr;
uint8_t send_buf[31]= "This is a TCP Client test...\n";

//extern osSemaphoreId_t BinaryEthRecvHandle;
extern BaseType_t taskEthRecvBegin;

static void client(void *thread_param)
{
//	int sock = -1;
//	struct sockaddr_in client_addr;

    while (1)
    {
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0)
        {
            uart_printf("Socket error\n");
            vTaskDelay(10);
            continue;
        }
        client_addr.sin_family = AF_INET;
        client_addr.sin_port = htons(PORT);
        client_addr.sin_addr.s_addr = inet_addr(IP_ADDR);
        memset(&(client_addr.sin_zero), 0, sizeof(client_addr.sin_zero));
        if (connect(sock,
                    (struct sockaddr *)&client_addr,
                    sizeof(struct sockaddr)) == -1)
        {
            uart_printf("Connect failed!\n");
            closesocket(sock);
            vTaskDelay(10);
            continue;
        }
        uart_printf("Connect to iperf server successful!\n");
        ssize_t recv_len;
        while (1)
        {
            if((recv_len = recv(sock, Eth_recv_buff, sizeof(Eth_recv_buff)-1, 0)) > 0)
            {
                
                // if(osSemaphoreRelease(BinaryEthRecvHandle) != osOK)
                // {
                //     uart_printf("Release semaphore failed\n");
                // }else{
                //     taskEthRecvBegin = pdTRUE;
                //     memccpy(Eth_recv_data,Eth_recv_buff,0,recv_len);
                //     memset(Eth_recv_buff,0,sizeof(Eth_recv_buff));
                //     uart_printf("Release semaphore success\n");
                // }


//                if(write(sock,Eth_recv_buff,recv_len) > 0)
//                {
//                    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//                }
                // recv_buff[recv_len] = '\0';
                // uart_printf("Received: %s,len:%d\n", recv_buff,recv_len);
                // if (write(sock,send_buf,sizeof(send_buf)) < 0)
                //     break;
            }
                // break;
            // uart_printf("recv:%s\n",recv_buff);
//            if (write(sock,send_buf,sizeof(send_buf)) < 0)
//                break;
            osDelay(10);
        }
        closesocket(sock);
    }
}



void client_init(void)
{
    sys_thread_new("client", client, NULL, 2048, osPriorityAboveNormal);
    sys_thread_new("writeEthTask", WriteEthTask, NULL, 1024, osPriorityNormal);
}



static void
 tcpecho_thread(void *arg)
 {
     int sock = -1,connected;
     char *recv_data;
     struct sockaddr_in server_addr,client_addr;
     socklen_t sin_size;
     int recv_data_len;

     recv_data = (char *)pvPortMalloc(RECV_DATA);
     if (recv_data == NULL)
     {
         uart_printf("No memory\n");
         goto __exit;
     }

     sock = socket(AF_INET, SOCK_STREAM, 0);
     if (sock < 0)
     {
         uart_printf("Socket error\n");
         goto __exit;
     }

     server_addr.sin_family = AF_INET;
     server_addr.sin_addr.s_addr = INADDR_ANY;
//     inet_pton(AF_INET, "192.168.0.100", &server_addr.sin_addr);
     server_addr.sin_port = htons(PORT);
     memset(&(server_addr.sin_zero), 0, sizeof(server_addr.sin_zero));

     if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
     {
         uart_printf("Unable to bind\n");
         goto __exit;
     }

     if (listen(sock, 5) == -1)
     {
         uart_printf("Listen error\n");
         goto __exit;
     }

     while (1)
     {
         sin_size = sizeof(struct sockaddr_in);

         connected = accept(sock, (struct sockaddr *)&client_addr, &sin_size);

         uart_printf("new client connected from (%s, %d)\n",
             inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
         {
             int flag = 1;

             setsockopt(connected,
                     IPPROTO_TCP,     /* set option at TCP level */
                     TCP_NODELAY,     /* name of option */
                     (void *) &flag, /* the cast is historical cruft */
                     sizeof(int));    /* length of option value */
         }

         while (1)
         {
             recv_data_len = recv(connected, recv_data, RECV_DATA, 0);

             if (recv_data_len <= 0)
                 break;

             uart_printf("recv %d len data\n",recv_data_len);

             write(connected,recv_data,recv_data_len);

         }
         if (connected >= 0)
             closesocket(connected);

         connected = -1;
     }
 __exit:
     if (sock >= 0) closesocket(sock);
     if (recv_data) free(recv_data);
 }

 void
 tcpecho_init(void)
 {
     sys_thread_new("tcpecho_thread", tcpecho_thread, NULL, 2048, osPriorityRealtime);
 }
