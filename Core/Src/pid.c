#include "pid.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"

int Encoder_Left,Encoder_Right;
float pitch,roll,yaw;
short gyrox,gyroy,gyroz;
short aacx,aacy,aacz;

int Vertical_out,Velocity_out,Turn_out,Target_Speed,Target_turn,MOTO1,MOTO2;
float Med_Angle;

float Vertical_Kp,Vertical_Kd;
float Velocity_Kp,Velocity_Ki;
float Turn_Kp,Turn_Kd;
uint8_t stop;

extern TIM_HandleTypeDef htim2,htim4;
extern float distance;
extern uint8_t Fore,Back,Left,Right;
#define SPEED_Y 12
#define SPEED_Z 150

int Vertical(float Med,float Angle,float gyro_Y)
{
	int temp;
	temp=Vertical_Kp*(Angle-Med)+Vertical_Kd*gyro_Y;
	return temp;
}

int Velocity(int Target,int encoder_L,int encoder_R)
{
	static int Err_LowOut_last,Encoder_S;
	static float a=0.7;
	int Err,Err_LowOut,temp;
	Err=(encoder_L+encoder_R)-Target;
	Err_LowOut=(1-a)*Err+a*Err_LowOut_last;
	Err_LowOut_last=Err_LowOut;
	Encoder_S+=Err_LowOut;
	Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
	if(stop==1) Encoder_S=0,stop=0;
	temp=Velocity_Kp*Err_LowOut+Velocity_Ki*Encoder_S;
	return temp;
}

int Turn(float gyro_Z,int Target_turn)
{
	int temp;
	temp=Turn_Kp*Target_turn+Turn_Kd*gyro_Z;
	return temp;
}

void Control(void)
{
	int PWM_out;
	Encoder_Left=Read_Speed(&htim2);
	Encoder_Right=-Read_Speed(&htim4);
	mpu_dmp_get_data(&pitch,&roll,&yaw);
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
	
	if(Fore==0 && Back==0) Target_Speed=0;
	if(Fore==1)
	{
		if(distance<50)
			Target_Speed++;
		else
			Target_Speed--;
	}
	if(Back==1) Target_Speed++;
	Target_Speed=Target_Speed>SPEED_Y?SPEED_Y:(Target_Speed<(-SPEED_Y)?(-SPEED_Y):Target_Speed);
	
	if(Left==0 && Right==0) Target_turn=0;
	if(Left==1) Target_turn+=30;
	if(Right==1) Target_turn-=30;
	Target_turn=Target_turn>SPEED_Z?SPEED_Z:(Target_turn<(-SPEED_Z)?(-SPEED_Z):Target_turn);
	
	if(Left==0 && Right==0) Turn_Kd=0.6;
	else if(Left==1 || Right==1) Turn_Kd=0;
	
	Velocity_out=Velocity(Target_Speed,Encoder_Left,Encoder_Right);
	Vertical_out=Vertical(Velocity_out+Med_Angle,roll,gyrox);
	Turn_out=Turn(gyroz,Target_turn);
	PWM_out=Vertical_out;
	MOTO1=PWM_out-Turn_out;
	MOTO2=PWM_out+Turn_out;
	Limit(&MOTO1,&MOTO2);
	Load(MOTO1,MOTO2);
}