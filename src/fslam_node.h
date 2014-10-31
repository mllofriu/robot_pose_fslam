/*
 * FSLAMNode.h
 *
 *  Created on: Oct 31, 2014
 *      Author: biorob
 */

#ifndef FSLAMNODE_H_
#define FSLAMNODE_H_

#include "fslamfilter.h"

#define MU_SYSTEM_NOISE_X 0.0
#define MU_SYSTEM_NOISE_Y 0.0
#define MU_SYSTEM_NOISE_THETA 0.0
#define SIGMA_SYSTEM_NOISE_X pow(0.01,2)
#define SIGMA_SYSTEM_NOISE_Y pow(0.01,2)
#define SIGMA_SYSTEM_NOISE_THETA pow(2*M_PI/180,2)

#define MU_MEAS_NOISE_X 0.0
#define MU_MEAS_NOISE_Y 0.0
#define MU_MEAS_NOISE_Z 0.0
#define SIGMA_MEAS_NOISE_X .1
#define SIGMA_MEAS_NOISE_Y .1
#define SIGMA_MEAS_NOISE_Z .1

#define NUM_SAMPLES 1 // Default Number of Samples
#define RESAMPLE_PERIOD 0 // Default Resample Period
#define RESAMPLE_THRESHOLD (NUM_SAMPLES/4.0) // Threshold for Dynamic Resampling

class FSLAMNode {
public:
	FSLAMNode(int argc, char ** argv);
	virtual ~FSLAMNode();

private:
	FSLAMFilter * filter;
};

#endif /* FSLAMNODE_H_ */
