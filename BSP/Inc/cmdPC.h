/*
 * cmdPC.h
 *
 *  Created on: Nov 24, 2024
 *      Author: Fei
 */

#ifndef INC_CMDPC_H_
#define INC_CMDPC_H_


#include "main.h"

#define SPEED_INIT_YAW 	 0 	//方位速度初始化
#define SPEED_INIT_PITCH 0  //俯仰速度初始化
#define POS_INIT_YAW     0.f  //方位位置初始化
#define POS_INIT_PITCH   0.f  //俯仰位置初始化
#define FSM_INIT_YAW     0.f  //快反镜方位位置初始化
#define FSM_INIT_PITCH   0.f  //快反镜俯仰位置初始化


typedef struct _CommandTypedef
{
	uint8_t go2Zero;			//初始化 回零
	uint8_t standbyEnable;		//待机(使能)
	uint8_t standbyDisable;		//待机(禁止)
	uint8_t scanEnable;			//扫描(使能)
	uint8_t guideEnable;		//引导(使能)
	uint8_t catchEnable;		//捕获(使能)
	uint8_t manualEnable;		//手动(使能)
	uint8_t clearerror;			//清除故障
	uint8_t trackCuEnable;		//粗跟踪(使能)
	uint8_t trackJingEnable;	//精跟踪(使能)
	uint8_t rangefinderOpen;	//测距机(使能)
	uint8_t trackOpen;			//开始跟踪
	short   laserAdjust;		//激光调节;
	short   focusAdjust;		//调焦
	f32_u8_t turnYaw;			//方位
	f32_u8_t turnPitch;			//俯仰
	f32_u8_t FSM_Yaw;			//快反镜方位
	f32_u8_t FSM_Pitch;			//快反镜俯仰
	uint32_t cuFocusAdjust;		//粗调焦
	uint32_t zoomAdjust;			//变倍调节
	uint32_t exposureCU;			//曝光调节
	uint32_t exposureJING;		//曝光调节
	short turnYawSpeed;			//方位速度
	short turnPitchSpeed;		//俯仰速度
	uint16_t manual_center_x;	//手动中心x
	uint16_t manual_center_y;	//手动中心y
	uint16_t manual_width;		//手动宽度
	uint16_t manual_height;		//手动高度
	uint8_t  manual_cj;			//判断粗精跟踪
	// uint8_t laserAdjust;		//激光调节
	int x_offset;				//x偏移
	int y_offset;				//y偏移
	uint8_t state;				//状态
	uint8_t setTurnState;		//设置伺服状态
	uint8_t setFSMState;		//设置快反镜状态
	uint8_t motorEnable;		//电机使能
}CommandTypedef_t;

typedef struct sifu_t
{
	uint8_t turnStateFlag; //转台状态标志
	uint8_t motorEnableFlag; //电机使能
	uint8_t trackingDataValidFlag; //引导数据有效
	uint8_t FSMModeFlag; //快反镜模式
	uint8_t controlEnableJingFlag; //精电视控制使能
}sifuFlag_t;


typedef enum {
	STATE_ERROR = -1,
	STATE_INIT = 0,
	STATE_ZERO,
	STATE_STANDBY,
	STATE_STANDBYDISABLE,
	STATE_GUIDE,
	STATE_SCAN,
	STATE_CATCH,
	STATE_TRACKCU,
	STATE_TRACKJING,
	STATE_SPEED,
	STATE_MANUAL
}StateList;



void go2init();
void go2Zero();
void catchEnable();
void guideEnable();
void scanEnable();
void standbyEnable();
void standbyDisable();
void processControl();

extern uint8_t offset_flag;
extern uint8_t manual_flag;
extern uint8_t state;
extern uint8_t expose_flag;
extern uint8_t exposeJing_flag;
extern uint8_t rangefander_flag;
extern uint8_t rangefinder_close;
extern uint8_t servo_flag;
extern uint8_t zoom_flag;
extern uint8_t focus_flag;
extern uint8_t clearerror_flag;
extern CommandTypedef_t CommandTypedef;
extern uint8_t stateChange;
extern sifuFlag_t sifuFlag;
extern uint8_t laserChange;
#endif /* INC_CMDPC_H_ */
