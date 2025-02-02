#include "pid.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"
//***********************************************************************

//相关状态数据
float x_pose=0, x_speed, angle_x, gyro_x, angle_z=0, gyro_z, last_angle=0;
float L_accel, R_accel, velocity_L, velocity_R;
//LQR状态反馈系数 1 位移 2 速度 3 角度 4 角速度
//float K1=-77.4597, K2=-113.9570, K3=-357.2249, K4=-33.3211, K5=22.3607, K6=22.8301;
float K1=-31.6228, K2=-60.3097, K3=-700.4016, K4=-6.8960, K5=22.3607, K6=22.8301;
//速度换算成PWM占空比的比例系数
float Ratio_accel=5948;	//5948

//目标状态值
float Target_x_speed=0, Target_angle_x=0.0349, Target_gyro_z=0;
int Motor_Left,Motor_Right;                 //电机PWM变量 应是Motor的 向Moto致敬




//传感器数据变量
int Encoder_Left,Encoder_Right;
float pitch,roll,yaw;
short gyrox,gyroy,gyroz;
short	aacx,aacy,aacz;

//闭环控制中间变量
int Vertical_out,Velocity_out,Turn_out,Target_Speed,Target_turn,MOTO1,MOTO2;
float Med_Angle=1;//平衡时角度值偏移量（机械中值）  //原始3
//参数
float Vertical_Kp=240,Vertical_Kd=2.1;			//直立环 数量级（Kp：0~1000、Kd：0~10） //原始 180 2
float Velocity_Kp=0.5,Velocity_Ki=0.0025;		//速度环 数量级（Kp：0~1）            //原始 0.6 0.003
float Turn_Kp=8,Turn_Kd=0.6;											//转向环                        //原始 10 0.8

uint8_t stop;

extern TIM_HandleTypeDef htim2,htim4;
extern float distance;
extern uint8_t Fore,Back,Left,Right;
#define SPEED_Y 30 //俯仰(前后)最大设定速度
#define SPEED_Z 150//偏航(左右)最大设定速度 
//直立环PD控制器
//输入：期望角度、真实角度、角速度
int Vertical(float Med,float Angle,float gyro_Y)
{
	int temp;
	temp=Vertical_Kp*(Angle-Med)+Vertical_Kd*gyro_Y;
	return temp;
}

//速度环PI控制器
//输入：期望速度、左编码器、右编码器
int Velocity(int Target,int encoder_L,int encoder_R)
{
	static int Err_LowOut_last,Encoder_S;
	static float a=0.7;
	int Err,Err_LowOut,temp;
	Velocity_Ki=Velocity_Kp/200;
	//1、计算偏差值
	Err=(encoder_L+encoder_R)-Target;
	//2、低通滤波
	Err_LowOut=(1-a)*Err+a*Err_LowOut_last;
	Err_LowOut_last=Err_LowOut;
	//3、积分
	Encoder_S+=Err_LowOut;
	//4、积分限幅(-20000~20000)
	Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
	if(stop==1)Encoder_S=0,stop=0;
	//5、速度环计算
	temp=Velocity_Kp*Err_LowOut+Velocity_Ki*Encoder_S;
	return temp;
}


//转向环PD控制器
//输入：角速度、角度值
int Turn(float gyro_Z,int Target_turn)
{
	int temp;
	temp=Turn_Kp*Target_turn+Turn_Kd*gyro_Z;
	return temp;
}

void Control(void)	//每隔10ms调用一次
{
	int PWM_out;
	//1、读取编码器和陀螺仪的数据
	Encoder_Left=Read_Speed(&htim2);//读取编码器
	Encoder_Right=-Read_Speed(&htim4);//大概是原始值
	mpu_dmp_get_data(&pitch,&roll,&yaw);//这个俯仰角是以直立为中心的
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
	//**************************************感觉遥控部分可以忽视*****************************************************************//
	//遥控
	if((Fore==0)&&(Back==0))Target_Speed=0;//未接受到前进后退指令-->速度清零，稳在原地
	if(Fore==1)
	{
		if(distance<50)
			Target_Speed--;
		else
			Target_Speed++;
	}
	if(Back==1){Target_Speed--;}//
	Target_Speed=Target_Speed>SPEED_Y?SPEED_Y:(Target_Speed<-SPEED_Y?(-SPEED_Y):Target_Speed);//限幅
	
	/*左右*/
	if((Left==0)&&(Right==0))Target_turn=0;
	if(Left==1)Target_turn-=30;	//左转
	if(Right==1)Target_turn+=30;	//右转
	Target_turn=Target_turn>SPEED_Z?SPEED_Z:(Target_turn<-SPEED_Z?(-SPEED_Z):Target_turn);//限幅( (20*100) * 100   )
	
	/*转向约束*/
	if((Left==0)&&(Right==0))Turn_Kd=0.6;//若无左右转向指令，则开启转向约束
	else if((Left==1)||(Right==1))Turn_Kd=0;//若左右转向指令接收到，则去掉转向约束
	
	//**************************************感觉遥控部分可以忽视**************************************************************//	
	
	//2、将数据传入PID控制器，计算输出结果，即左右电机转速值
	Velocity_out=Velocity(Target_Speed,Encoder_Left,Encoder_Right);
	Vertical_out=Vertical(Velocity_out+Med_Angle,roll,gyrox);
	Turn_out=Turn(gyroz,Target_turn);
	PWM_out=Vertical_out;//累加上直立环 更前进速度也有关系
	MOTO1=PWM_out-Turn_out;
	MOTO2=PWM_out+Turn_out;//累加上转向PID 对各个轮速有微调
	Limit(&MOTO1,&MOTO2);
	Load(MOTO1,MOTO2);
}





/**************************************************************************
Function: Encoder reading is converted to speed (mm/s)
Input   : none
Output  : none
函数功能：编码器读数转换为速度（mm/s）
入口参数：无
返回  值：无
**************************************************************************/
float Velocity_Left,Velocity_Right;	//车轮速度(mm/s)
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{ 	
	float Rotation_Speed_L,Rotation_Speed_R;						//电机转速  转速=编码器读数（5ms每次）*读取频率/倍频数/减速比/编码器精度
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Left = Rotation_Speed_L*PI*Diameter_67;		//求出编码器速度=转速*周长
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Right = Rotation_Speed_R*PI*Diameter_67;		//求出编码器速度=转速*周长
}

//PWM限幅
//限幅大小6900
int PWM_Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}
void LQR_Control(void)
{
	int PWM_out;
	//1、读取编码器和陀螺仪的数据
	Encoder_Left=Read_Speed(&htim2);
	Encoder_Right=-Read_Speed(&htim4);
	mpu_dmp_get_data(&pitch,&roll,&yaw);
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
	
	
		//**************************************感觉遥控部分可以忽视*****************************************************************//
	//遥控
//	if((Fore==0)&&(Back==0))Target_Speed=0;//未接受到前进后退指令-->速度清零，稳在原地
//	if(Fore==1)
//	{
//		if(distance<50)
//			Target_Speed--;
//		else
//			Target_Speed++;
//	}
//	if(Back==1){Target_Speed--;}//
//	Target_Speed=Target_Speed>SPEED_Y?SPEED_Y:(Target_Speed<-SPEED_Y?(-SPEED_Y):Target_Speed);//限幅
//	
//	/*左右*/
//	if((Left==0)&&(Right==0))Target_turn=0;
//	if(Left==1)Target_turn-=30;	//左转
//	if(Right==1)Target_turn+=30;	//右转
//	Target_turn=Target_turn>SPEED_Z?SPEED_Z:(Target_turn<-SPEED_Z?(-SPEED_Z):Target_turn);//限幅( (20*100) * 100   )
//	
//	/*转向约束*/
//	if((Left==0)&&(Right==0))Turn_Kd=0.6;//若无左右转向指令，则开启转向约束
//	else if((Left==1)||(Right==1))Turn_Kd=0;//若左右转向指令接收到，则去掉转向约束
	
	//**************************************感觉遥控部分可以忽视**************************************************************//
	
	//获取速度(m/s)、位移(m)
	Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);
	x_speed=(Encoder_Left+Encoder_Right)/2*PI*Diameter_67/1000/(EncoderMultiples*Reduction_Ratio*Encoder_precision)*Control_Frequency;//回归m/s
	x_pose+=x_speed/Control_Frequency;
	//获取倾角(rad)、角速度(rad/s)
	angle_x=roll/180*PI;
	gyro_x=(angle_x-last_angle)*Control_Frequency;
	last_angle=angle_x;	
//	gyro_x=gyrox;

	//获取转向速度(rad/s)、转向角(rad)
	gyro_z=(Encoder_Right-Encoder_Left)/Wheel_spacing/1000*PI*Diameter_67/1000/1560*Control_Frequency;
	angle_z+=gyro_z/Control_Frequency;	
	
	
	Target_x_speed = 0;				//平衡速度(m/s)
	Target_gyro_z = 0;						//平衡转向速度(rad/s)
//	x_pose = 0;	//这个是为了让他稳在原地的
	K5=0;
	K6=0;
		
	//计算输入变量(LQR控制器)
	L_accel=-(K1*x_pose+K2*(x_speed-Target_x_speed)+K3*(angle_x-Target_angle_x)+K4*gyro_x+K5*angle_z+K6*(gyro_z-Target_gyro_z));
	R_accel=-(K1*x_pose+K2*(x_speed-Target_x_speed)+K3*(angle_x-Target_angle_x)+K4*gyro_x-K5*angle_z-K6*(gyro_z-Target_gyro_z));
	//速度换算成PWM占空比
	velocity_L=(int)(Ratio_accel*(x_speed+L_accel/Control_Frequency));
	velocity_R=(int)(Ratio_accel*(x_speed+R_accel/Control_Frequency));
	//	
	
	//限幅 并加载给电机
	Motor_Left=PWM_Limit(velocity_L,7200,-7200);
	Motor_Right=PWM_Limit(velocity_R,7200,-7200);	
	Load(Motor_Left,Motor_Left);
}