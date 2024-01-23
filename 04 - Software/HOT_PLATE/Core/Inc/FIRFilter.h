/*
 * FIRFilter.h
 *
 *  Created on: Jan 20, 2024
 *      Author: Sergio
 */

#ifndef INC_FIRFILTER_H_
#define INC_FIRFILTER_H_

#include "stdio.h"

#define FIR_FILTER_LENGHT 10

typedef struct {
	int16_t buf[FIR_FILTER_LENGHT];
	int16_t bufIndex;
	int16_t output;
} FIRFilter;

void FIRFilter_Init(FIRFilter *fir);
int16_t FIRFilter_Update(FIRFilter *fir, int16_t inp);

#endif /* INC_FIRFILTER_H_ */
