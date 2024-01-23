/*
 * PI.h
 *
 *  Created on: 23 ene. 2024
 *      Author: sergio
 */

#ifndef INC_PI_H_
#define INC_PI_H_


#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#define   _IQsat(A, Pos, Neg)  (MAX(((MIN((float)(A),(Pos)))),(Neg)))

typedef struct {   float  Ref;   			// Input: Reference input
				   float  Fdb;   			// Input: Feedback input
				   float  Err;				// Variable: Error
				   float  Kp;				// Parameter: Proportional gain
				   float  Up;				// Variable: Proportional output
				   float  Ui;				// Variable: Integral output
				   float  Ud;				// Variable: Derivative output
				   float  OutPreSat; 		// Variable: Pre-saturated output
				   float  OutMax;		    // Parameter: Maximum output
				   float  OutMin;	    	// Parameter: Minimum output
				   float  Out;   			// Output: PID output
				   float  SatErr;			// Variable: Saturated difference
				   float  Ki;			    // Parameter: Integral gain
				   float  Kc;		     	// Parameter: Integral correction gain
				   float  Kd; 		        // Parameter: Derivative gain
				   float  Up1;		   	    // History: Previous proportional output
		 	 	} PIDREG3;

#define PID_MACRO(v)																					\
	v.Err = v.Ref - v.Fdb; 									/* Compute the error */						\
	v.Up= (v.Kp*v.Err);										/* Compute the proportional output */		\
	v.Ui= v.Ui + (v.Ki*v.Up) + (v.Kc*v.SatErr);				/* Compute the integral output */			\
	v.OutPreSat= v.Up + v.Ui;								/* Compute the pre-saturated output */		\
	v.Out = _IQsat(v.OutPreSat, v.OutMax, v.OutMin);		/* Saturate the output */					\
	v.SatErr = v.Out - v.OutPreSat;							/* Compute the saturate difference */		\

void PID_Init(void);
#endif /* INC_PI_H_ */
