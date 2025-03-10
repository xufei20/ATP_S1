/*
 * tcpecho.h
 *
 *  Created on: Nov 13, 2024
 *      Author: Fei
 */

#ifndef INC_TCPECHO_H_
#define INC_TCPECHO_H_

#include "main.h"
#include <lwip/sockets.h>


#define testcode 1

typedef struct _pcRecv
{
    uint32_t focusAdjust_cu;
    uint32_t zoomAdjust_cu;
    uint32_t exposurecu;
    uint32_t exposurejing;
    uint16_t manual_center_x;
    uint16_t manual_center_y;
    uint16_t manual_width;
    uint16_t manual_height;
}pcRecv_t;

typedef struct _pcSend
{
    f32_u8_t XoffsetCu;
    f32_u8_t YoffsetCu;
    f32_u8_t XoffsetJing;
    f32_u8_t YoffsetJing;
    f32_u8_t rangefinder;
    f32_u8_t ServoYaw;
    f32_u8_t ServoPitch;

    f32_u8_t FSM_Pitch;
    f32_u8_t FSM_Yaw;
    uint8_t state;
    uint8_t cuEnable;
    uint8_t jingEnable;
    uint8_t servoTrackingState;
    uint8_t FSMTrackingState;
    uint8_t cuErrorCode[2];
    uint8_t jingErrorCode[2];
    uint8_t cuOffsetState;
    uint8_t jingOffsetState;
}pcSend_t;

extern pcRecv_t pcRecv;
extern pcSend_t pcSend;
extern int sock;
extern struct sockaddr_in client_addr;
extern uint8_t send_buf[31];
extern uint8_t Eth_recv_buff[256];
extern uint8_t Eth_recv_data[256];

void udpecho_init(void);
void client_init(void);
void tcpecho_init(void);

#endif /* INC_TCPECHO_H_ */
