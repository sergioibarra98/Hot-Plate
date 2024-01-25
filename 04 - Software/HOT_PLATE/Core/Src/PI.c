#include "PI.h"

void PID_Init(PIDREG3 *pid) {
	/** Initial PID Temperature values */
	pid->Up			= 0;
	pid->Ui			= 0;
	pid->Ud			= 0;
	pid->OutPreSat	= 0;
	pid->Out		= 0;
	pid->SatErr		= 0;
	pid->Up1		= 0;
	pid->Kp			= 1;
	pid->Ki			= 1;
	pid->Kc			= 1;
	pid->Kd			= 0;
	pid->OutMax		= 1520;
	pid->OutMin		= 1;
}

//int16_t PID_Update(PIDREG3 *pid) {
//	pid->Err = pid->Ref - pid->Fdb;
//	pid->Up= ((pid->Err >> 9) + (pid->Err >> 11) + (pid->Err >> 15));
//	pid->Ui= pid->Ui + ((pid->Up >> 7) + (pid->Up >> 9) + (pid->Up >> 13)) + ((pid->SatErr >> 7) + (pid->SatErr >> 9) + (pid->SatErr >> 13));
//	pid->OutPreSat = pid->Up + pid->Ui;
//	pid->Out = _IQsat(pid->OutPreSat, pid->OutMax, pid->OutMin);
//	pid->SatErr = pid->Out - pid->OutPreSat;
//	return pid->Out;
//}

int16_t PID_Update(PIDREG3 *pid) {																					\
	pid->Err = pid->Ref - pid->Fdb; 									/* Compute the error */						\
	pid->Up= (pid->Kp*pid->Err);										/* Compute the proportional output */		\
	pid->Ui= pid->Ui + (pid->Ki*pid->Up) + (pid->Kc*pid->SatErr);				/* Compute the integral output */			\
	pid->OutPreSat= pid->Up + pid->Ui;								/* Compute the pre-saturated output */		\
	pid->Out = _IQsat(pid->OutPreSat, pid->OutMax, pid->OutMin);		/* Saturate the output */					\
	pid->SatErr = pid->Out - pid->OutPreSat;
	return pid->Out;
}
