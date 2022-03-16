/**

 */
#include "pid.h"

void PID_Postion_Cal(PID_TYPE *PID,float target,float measure)
{
	PID->Error = target - measure;				//误差
	PID->Differ = PID->Error - PID->PreError;	//微分量
	PID->Integral+=PID->Error;					//对误差进行积分
	if(PID->Integral > PID->Irang)				//积分限幅
		PID->Integral = PID->Irang;				//
	if(PID->Integral < -PID->Irang)				//积分限幅
		PID->Integral = -PID->Irang;			
	
	PID->Pout = PID->P * PID->Error;			//比例控制输出
	PID->Iout = PID->I * PID->Integral;			//积分控制输出
	PID->Dout = PID->D * PID->Differ;			//微分控制输出
	
	PID->OutPut = PID->Pout + PID->Iout + PID->Dout;	//PID控制总输出
	
	PID->PreError = PID->Error;					//前一个误差值
}

void pidParameter_init(void)
{
	//poll轴
	PID_ROL_Angle.P=PID_ROLL_KP;
	PID_ROL_Angle.I=PID_ROLL_KI;
	PID_ROL_Angle.D=PID_ROLL_KD;
	PID_ROL_Angle.Ilimit_flag = 0;
	PID_ROL_Angle.Irang = PID_ROLL_INTEGRATION_LIMIT;
	
	//pitch
	PID_PIT_Angle.P=PID_PITCH_KP;
	PID_PIT_Angle.I=PID_PITCH_KI;
	PID_PIT_Angle.D=PID_PITCH_KD;
	PID_PIT_Angle.Ilimit_flag = 0;
	PID_PIT_Angle.Irang = PID_PITCH_INTEGRATION_LIMIT;
	
	//yaw
	PID_YAW_Angle.P=PID_YAW_KP;
	PID_YAW_Angle.I=PID_YAW_KI;
	PID_YAW_Angle.D=PID_YAW_KD;
	PID_YAW_Angle.Ilimit_flag = 0;
	PID_YAW_Angle.Irang = PID_YAW_INTEGRATION_LIMIT;
}
