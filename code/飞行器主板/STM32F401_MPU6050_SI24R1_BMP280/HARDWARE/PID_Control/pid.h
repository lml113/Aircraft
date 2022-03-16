/**

 */
#ifndef PID_H_
#define PID_H_

#include "stm32f401xc.h"
#include <stdbool.h>

//#define PID_ROLL_RATE_KP  70.0
//#define PID_ROLL_RATE_KI  0.0
//#define PID_ROLL_RATE_KD  0.0
//#define PID_ROLL_RATE_INTEGRATION_LIMIT    100.0

//#define PID_PITCH_RATE_KP  70.0
//#define PID_PITCH_RATE_KI  0.0
//#define PID_PITCH_RATE_KD  0.0
//#define PID_PITCH_RATE_INTEGRATION_LIMIT   100.0

//#define PID_YAW_RATE_KP  50.0
//#define PID_YAW_RATE_KI  25.0
//#define PID_YAW_RATE_KD  0.0
//#define PID_YAW_RATE_INTEGRATION_LIMIT     500.0

#define PID_ROLL_KP  1.0
#define PID_ROLL_KI  0.0
#define PID_ROLL_KD  0.0
#define PID_ROLL_INTEGRATION_LIMIT    20.0

#define PID_PITCH_KP  1.0
#define PID_PITCH_KI  0.0
#define PID_PITCH_KD  0.0
#define PID_PITCH_INTEGRATION_LIMIT   20.0

#define PID_YAW_KP  0.0
#define PID_YAW_KI  0.0
#define PID_YAW_KD  0.0
#define PID_YAW_INTEGRATION_LIMIT     360.0


#define DEFAULT_PID_INTEGRATION_LIMIT  5000.0

typedef struct PID
{
	float P;		//PID参数
	float I;
	float D;
	float Error;	//比例项
	float Integral;	//积分项
	float Differ;	//微分项
	float PreError;	//上一次误差
	float Ilimit;	//积分分离
	float Irang;	//积分限幅
	uint8_t Ilimit_flag;	//积分分离标志
	float Pout;		//比例项输出
	float Iout;		//积分项输出
	float Dout;		//微分项输出
	float OutPut;	//PID控制器总输出
}PID_TYPE;

// 角度环PID
extern PID_TYPE PID_ROL_Angle;
extern PID_TYPE PID_PIT_Angle;
extern PID_TYPE PID_YAW_Angle;


void PID_Postion_Cal(PID_TYPE *PID,float target,float measure);
void pidParameter_init(void);

///**
// * PID object initialization.
// *
// * @param[out] pid   A pointer to the pid object to initialize.
// * @param[in] desired  The initial set point.
// * @param[in] kp        The proportional gain
// * @param[in] ki        The integral gain
// * @param[in] kd        The derivative gain
// */
//void pidInit(PID_TYPE* pid, const float desired, const float kp,
//             const float ki, const float kd);

///**
// * Set the integral limit for this PID in deg.
// *
// * @param[in] pid   A pointer to the pid object.
// * @param[in] limit Pid integral swing limit.
// */
//void pidSetIntegralLimit(PID_TYPE* pid, const float limit);

///**
// * Reset the PID error values
// *
// * @param[in] pid   A pointer to the pid object.
// * @param[in] limit Pid integral swing limit.
// */
//void pidReset(PID_TYPE* pid);

///**
// * Update the PID parameters.
// *
// * @param[in] pid         A pointer to the pid object.
// * @param[in] measured    The measured value
// * @param[in] updateError Set to TRUE if error should be calculated.
// *                        Set to False if pidSetError() has been used.
// * @return PID algorithm output
// */
//float pidUpdate(PID_TYPE* pid, const float measured, const bool updateError);

///**
// * Set a new set point for the PID to track.
// *
// * @param[in] pid   A pointer to the pid object.
// * @param[in] angle The new set point
// */
//void pidSetDesired(PID_TYPE* pid, const float desired);

///**
// * Set a new set point for the PID to track.
// * @return The set point
// */
//float pidGetDesired(PID_TYPE* pid);

///**
// * Find out if PID is active
// * @return TRUE if active, FALSE otherwise
// */
//bool pidIsActive(PID_TYPE* pid);

///**
// * Set a new error. Use if a special error calculation is needed.
// *
// * @param[in] pid   A pointer to the pid object.
// * @param[in] error The new error
// */
//void pidSetError(PID_TYPE* pid, const float error);

///**
// * Set a new proportional gain for the PID.
// *
// * @param[in] pid   A pointer to the pid object.
// * @param[in] kp    The new proportional gain
// */
//void pidSetKp(PID_TYPE* pid, const float kp);

///**
// * Set a new integral gain for the PID.
// *
// * @param[in] pid   A pointer to the pid object.
// * @param[in] ki    The new integral gain
// */
//void pidSetKi(PID_TYPE* pid, const float ki);

///**
// * Set a new derivative gain for the PID.
// *
// * @param[in] pid   A pointer to the pid object.
// * @param[in] kd    The derivative gain
// */
//void pidSetKd(PID_TYPE* pid, const float kd);

#endif /* PID_H_ */
