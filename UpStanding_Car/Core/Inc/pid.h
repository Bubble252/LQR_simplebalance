#ifndef __PID_H__
#define __PID_H__

#include "stm32f1xx_hal.h"

#define PI 3.14159265							//PI圆周率
#define Control_Frequency  200.0	//编码器读取频率
#define Diameter_67  67.0 				//轮子直径67mm 
#define Wheel_spacing  161.0         //轮距
#define EncoderMultiples   4.0 		//编码器倍频数
#define Encoder_precision  11.0 	//编码器精度 11线
#define Reduction_Ratio  30.0			//减速比30
#define Perimeter  210.4867 			//周长，单位mm
void Control(void);	//每隔10ms调用一次
void LQR_Control(void);
#endif
