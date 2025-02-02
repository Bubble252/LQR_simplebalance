#include "pid.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"
//***********************************************************************

//���״̬����
float x_pose=0, x_speed, angle_x, gyro_x, angle_z=0, gyro_z, last_angle=0;
float L_accel, R_accel, velocity_L, velocity_R;
//LQR״̬����ϵ�� 1 λ�� 2 �ٶ� 3 �Ƕ� 4 ���ٶ�
//float K1=-77.4597, K2=-113.9570, K3=-357.2249, K4=-33.3211, K5=22.3607, K6=22.8301;
float K1=-31.6228, K2=-60.3097, K3=-700.4016, K4=-6.8960, K5=22.3607, K6=22.8301;
//�ٶȻ����PWMռ�ձȵı���ϵ��
float Ratio_accel=5948;	//5948

//Ŀ��״ֵ̬
float Target_x_speed=0, Target_angle_x=0.0349, Target_gyro_z=0;
int Motor_Left,Motor_Right;                 //���PWM���� Ӧ��Motor�� ��Moto�¾�




//���������ݱ���
int Encoder_Left,Encoder_Right;
float pitch,roll,yaw;
short gyrox,gyroy,gyroz;
short	aacx,aacy,aacz;

//�ջ������м����
int Vertical_out,Velocity_out,Turn_out,Target_Speed,Target_turn,MOTO1,MOTO2;
float Med_Angle=1;//ƽ��ʱ�Ƕ�ֵƫ��������е��ֵ��  //ԭʼ3
//����
float Vertical_Kp=240,Vertical_Kd=2.1;			//ֱ���� ��������Kp��0~1000��Kd��0~10�� //ԭʼ 180 2
float Velocity_Kp=0.5,Velocity_Ki=0.0025;		//�ٶȻ� ��������Kp��0~1��            //ԭʼ 0.6 0.003
float Turn_Kp=8,Turn_Kd=0.6;											//ת��                        //ԭʼ 10 0.8

uint8_t stop;

extern TIM_HandleTypeDef htim2,htim4;
extern float distance;
extern uint8_t Fore,Back,Left,Right;
#define SPEED_Y 30 //����(ǰ��)����趨�ٶ�
#define SPEED_Z 150//ƫ��(����)����趨�ٶ� 
//ֱ����PD������
//���룺�����Ƕȡ���ʵ�Ƕȡ����ٶ�
int Vertical(float Med,float Angle,float gyro_Y)
{
	int temp;
	temp=Vertical_Kp*(Angle-Med)+Vertical_Kd*gyro_Y;
	return temp;
}

//�ٶȻ�PI������
//���룺�����ٶȡ�����������ұ�����
int Velocity(int Target,int encoder_L,int encoder_R)
{
	static int Err_LowOut_last,Encoder_S;
	static float a=0.7;
	int Err,Err_LowOut,temp;
	Velocity_Ki=Velocity_Kp/200;
	//1������ƫ��ֵ
	Err=(encoder_L+encoder_R)-Target;
	//2����ͨ�˲�
	Err_LowOut=(1-a)*Err+a*Err_LowOut_last;
	Err_LowOut_last=Err_LowOut;
	//3������
	Encoder_S+=Err_LowOut;
	//4�������޷�(-20000~20000)
	Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
	if(stop==1)Encoder_S=0,stop=0;
	//5���ٶȻ�����
	temp=Velocity_Kp*Err_LowOut+Velocity_Ki*Encoder_S;
	return temp;
}


//ת��PD������
//���룺���ٶȡ��Ƕ�ֵ
int Turn(float gyro_Z,int Target_turn)
{
	int temp;
	temp=Turn_Kp*Target_turn+Turn_Kd*gyro_Z;
	return temp;
}

void Control(void)	//ÿ��10ms����һ��
{
	int PWM_out;
	//1����ȡ�������������ǵ�����
	Encoder_Left=Read_Speed(&htim2);//��ȡ������
	Encoder_Right=-Read_Speed(&htim4);//�����ԭʼֵ
	mpu_dmp_get_data(&pitch,&roll,&yaw);//�������������ֱ��Ϊ���ĵ�
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
	//**************************************�о�ң�ز��ֿ��Ժ���*****************************************************************//
	//ң��
	if((Fore==0)&&(Back==0))Target_Speed=0;//δ���ܵ�ǰ������ָ��-->�ٶ����㣬����ԭ��
	if(Fore==1)
	{
		if(distance<50)
			Target_Speed--;
		else
			Target_Speed++;
	}
	if(Back==1){Target_Speed--;}//
	Target_Speed=Target_Speed>SPEED_Y?SPEED_Y:(Target_Speed<-SPEED_Y?(-SPEED_Y):Target_Speed);//�޷�
	
	/*����*/
	if((Left==0)&&(Right==0))Target_turn=0;
	if(Left==1)Target_turn-=30;	//��ת
	if(Right==1)Target_turn+=30;	//��ת
	Target_turn=Target_turn>SPEED_Z?SPEED_Z:(Target_turn<-SPEED_Z?(-SPEED_Z):Target_turn);//�޷�( (20*100) * 100   )
	
	/*ת��Լ��*/
	if((Left==0)&&(Right==0))Turn_Kd=0.6;//��������ת��ָ�����ת��Լ��
	else if((Left==1)||(Right==1))Turn_Kd=0;//������ת��ָ����յ�����ȥ��ת��Լ��
	
	//**************************************�о�ң�ز��ֿ��Ժ���**************************************************************//	
	
	//2�������ݴ���PID�������������������������ҵ��ת��ֵ
	Velocity_out=Velocity(Target_Speed,Encoder_Left,Encoder_Right);
	Vertical_out=Vertical(Velocity_out+Med_Angle,roll,gyrox);
	Turn_out=Turn(gyroz,Target_turn);
	PWM_out=Vertical_out;//�ۼ���ֱ���� ��ǰ���ٶ�Ҳ�й�ϵ
	MOTO1=PWM_out-Turn_out;
	MOTO2=PWM_out+Turn_out;//�ۼ���ת��PID �Ը���������΢��
	Limit(&MOTO1,&MOTO2);
	Load(MOTO1,MOTO2);
}





/**************************************************************************
Function: Encoder reading is converted to speed (mm/s)
Input   : none
Output  : none
�������ܣ�����������ת��Ϊ�ٶȣ�mm/s��
��ڲ�������
����  ֵ����
**************************************************************************/
float Velocity_Left,Velocity_Right;	//�����ٶ�(mm/s)
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{ 	
	float Rotation_Speed_L,Rotation_Speed_R;						//���ת��  ת��=������������5msÿ�Σ�*��ȡƵ��/��Ƶ��/���ٱ�/����������
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Left = Rotation_Speed_L*PI*Diameter_67;		//����������ٶ�=ת��*�ܳ�
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Right = Rotation_Speed_R*PI*Diameter_67;		//����������ٶ�=ת��*�ܳ�
}

//PWM�޷�
//�޷���С6900
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
	//1����ȡ�������������ǵ�����
	Encoder_Left=Read_Speed(&htim2);
	Encoder_Right=-Read_Speed(&htim4);
	mpu_dmp_get_data(&pitch,&roll,&yaw);
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
	
	
		//**************************************�о�ң�ز��ֿ��Ժ���*****************************************************************//
	//ң��
//	if((Fore==0)&&(Back==0))Target_Speed=0;//δ���ܵ�ǰ������ָ��-->�ٶ����㣬����ԭ��
//	if(Fore==1)
//	{
//		if(distance<50)
//			Target_Speed--;
//		else
//			Target_Speed++;
//	}
//	if(Back==1){Target_Speed--;}//
//	Target_Speed=Target_Speed>SPEED_Y?SPEED_Y:(Target_Speed<-SPEED_Y?(-SPEED_Y):Target_Speed);//�޷�
//	
//	/*����*/
//	if((Left==0)&&(Right==0))Target_turn=0;
//	if(Left==1)Target_turn-=30;	//��ת
//	if(Right==1)Target_turn+=30;	//��ת
//	Target_turn=Target_turn>SPEED_Z?SPEED_Z:(Target_turn<-SPEED_Z?(-SPEED_Z):Target_turn);//�޷�( (20*100) * 100   )
//	
//	/*ת��Լ��*/
//	if((Left==0)&&(Right==0))Turn_Kd=0.6;//��������ת��ָ�����ת��Լ��
//	else if((Left==1)||(Right==1))Turn_Kd=0;//������ת��ָ����յ�����ȥ��ת��Լ��
	
	//**************************************�о�ң�ز��ֿ��Ժ���**************************************************************//
	
	//��ȡ�ٶ�(m/s)��λ��(m)
	Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);
	x_speed=(Encoder_Left+Encoder_Right)/2*PI*Diameter_67/1000/(EncoderMultiples*Reduction_Ratio*Encoder_precision)*Control_Frequency;//�ع�m/s
	x_pose+=x_speed/Control_Frequency;
	//��ȡ���(rad)�����ٶ�(rad/s)
	angle_x=roll/180*PI;
	gyro_x=(angle_x-last_angle)*Control_Frequency;
	last_angle=angle_x;	
//	gyro_x=gyrox;

	//��ȡת���ٶ�(rad/s)��ת���(rad)
	gyro_z=(Encoder_Right-Encoder_Left)/Wheel_spacing/1000*PI*Diameter_67/1000/1560*Control_Frequency;
	angle_z+=gyro_z/Control_Frequency;	
	
	
	Target_x_speed = 0;				//ƽ���ٶ�(m/s)
	Target_gyro_z = 0;						//ƽ��ת���ٶ�(rad/s)
//	x_pose = 0;	//�����Ϊ����������ԭ�ص�
	K5=0;
	K6=0;
		
	//�����������(LQR������)
	L_accel=-(K1*x_pose+K2*(x_speed-Target_x_speed)+K3*(angle_x-Target_angle_x)+K4*gyro_x+K5*angle_z+K6*(gyro_z-Target_gyro_z));
	R_accel=-(K1*x_pose+K2*(x_speed-Target_x_speed)+K3*(angle_x-Target_angle_x)+K4*gyro_x-K5*angle_z-K6*(gyro_z-Target_gyro_z));
	//�ٶȻ����PWMռ�ձ�
	velocity_L=(int)(Ratio_accel*(x_speed+L_accel/Control_Frequency));
	velocity_R=(int)(Ratio_accel*(x_speed+R_accel/Control_Frequency));
	//	
	
	//�޷� �����ظ����
	Motor_Left=PWM_Limit(velocity_L,7200,-7200);
	Motor_Right=PWM_Limit(velocity_R,7200,-7200);	
	Load(Motor_Left,Motor_Left);
}