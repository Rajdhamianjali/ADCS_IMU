/*
 * RCFilter.h
 *
 *  Created on: Jun 17, 2024
 *      Author: Anjali
 */

#ifndef INC_RCFILTER_H_
#define INC_RCFILTER_H_

typedef struct {

	float coeff[2];
	float out[2];

} RCFilter;

RCFilter *myRCFilter;

void RCFilter_Init(RCFilter *filt, float cutoffFreqHz, float sampleTimeS);
float RCFilter_Update(RCFilter *filt, float inp);

#endif /* INC_RCFILTER_H_ */
