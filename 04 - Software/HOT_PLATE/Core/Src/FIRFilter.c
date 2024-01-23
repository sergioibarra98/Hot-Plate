#include "FIRFilter.h"
//FIR equirriple
//Density factor 20
//Fs 40kHz
//Fpass 1kHz
//Fstop 10kHz
//Apass 0.1dB
//Astop 55dB
int16_t H_FIR[FIR_FILTER_LENGHT] = {0, 3, 11, 22, 30, 30, 22, 11, 3, 0};

void FIRFilter_Init(FIRFilter *fir) {
	//Clear fiilter buffer
	for(int16_t n = 0; n < FIR_FILTER_LENGHT; n++) {
		fir->buf[n] = 0;
	}

	//Reset buffer Index
	fir->bufIndex = 0;

	//Clear filter output
	fir->output = 0;
}

int16_t FIRFilter_Update(FIRFilter *fir, int16_t inp) {
	//Store latest sample in buffer in degrees and transformed to fixed point (8,7)
	inp = ((inp >> 5) - (inp >> 7) + (inp >> 9) - (inp >> 11) + 3) << 7;		//CSD 0.025027 * inp + 3.056846

	fir->buf[fir->bufIndex] = inp;

	//Increment buffer index and wrap if necessary
	fir->bufIndex = fir->bufIndex + 1;

	if (fir->bufIndex == FIR_FILTER_LENGHT) {
		fir->bufIndex = 0;
	}

	//Compute new output sample
	fir->output = 0;

	int16_t sumIndex = fir->bufIndex;

	for (int16_t n = 1; n < FIR_FILTER_LENGHT>>1; n++) {	//n = 1 por coeficientes 0
		//Decrement Index and wrap if necessary
		if (sumIndex > 0) {
			sumIndex--;
		} else {
			sumIndex = FIR_FILTER_LENGHT - 1;
		}

		fir->output = fir->output + (((H_FIR[n] + H_FIR[FIR_FILTER_LENGHT - n - 1]) * fir->buf[sumIndex])>>7);
	}
	fir->output = fir->output >> 7;

	//Return filter output
	return fir->output;
}
