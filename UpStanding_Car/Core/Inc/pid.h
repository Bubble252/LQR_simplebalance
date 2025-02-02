#ifndef __PID_H__
#define __PID_H__

#include "stm32f1xx_hal.h"

#define PI 3.14159265							//PIԲ����
#define Control_Frequency  200.0	//��������ȡƵ��
#define Diameter_67  67.0 				//����ֱ��67mm 
#define Wheel_spacing  161.0         //�־�
#define EncoderMultiples   4.0 		//��������Ƶ��
#define Encoder_precision  11.0 	//���������� 11��
#define Reduction_Ratio  30.0			//���ٱ�30
#define Perimeter  210.4867 			//�ܳ�����λmm
void Control(void);	//ÿ��10ms����һ��
void LQR_Control(void);
#endif
