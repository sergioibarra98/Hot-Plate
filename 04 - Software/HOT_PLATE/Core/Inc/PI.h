/*
 * PI.h
 *
 *  Created on: 23 ene. 2024
 *      Author: sergio
 */

#ifndef INC_PI_H_
#define INC_PI_H_

#include "stdio.h"


#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#define   _IQsat(A, Pos, Neg)  (MAX(((MIN((A),(Pos)))),(Neg)))

typedef struct {
	int16_t  Ref;   			// Input: Reference input
	int16_t  Fdb;   			// Input: Feedback input
	int16_t  Err;				// Variable: Error
	int16_t  Kp;				// Parameter: Proportional gain
	int16_t  Up;				// Variable: Proportional output
	int16_t  Ui;				// Variable: Integral output
	int16_t  Ud;				// Variable: Derivative output
	int16_t  OutPreSat; 		// Variable: Pre-saturated output
	int16_t  OutMax;		    // Parameter: Maximum output
	int16_t  OutMin;	    	// Parameter: Minimum output
	int16_t  Out;   			// Output: PID output
	int16_t  SatErr;			// Variable: Saturated difference
	int16_t  Ki;			    // Parameter: Integral gain
	int16_t  Kc;		     	// Parameter: Integral correction gain
	int16_t  Kd; 		        // Parameter: Derivative gain
	int16_t  Up1;		   	    // History: Previous proportional output
} PIDREG3;

void PID_Init(PIDREG3 *pid);
int16_t PID_Update(PIDREG3 *pid);
#endif /* INC_PI_H_ */
