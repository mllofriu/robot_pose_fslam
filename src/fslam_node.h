/*
 * FSLAMNode.h
 *
 *  Created on: Oct 31, 2014
 *      Author: biorob
 */

#ifndef FSLAMNODE_H_
#define FSLAMNODE_H_

#include "fslamfilter.h"
#include <ros/ros.h>
#include <boost/thread.hpp>

#define MU_SYSTEM_NOISE_X 0.0
#define MU_SYSTEM_NOISE_Y 0.0
#define MU_SYSTEM_NOISE_THETA 0.0
#define SIGMA_SYSTEM_NOISE_X 0.2
#define SIGMA_SYSTEM_NOISE_Y 0.2
#define SIGMA_SYSTEM_NOISE_THETA 0.05

#define MU_MEAS_NOISE_X 0.0
#define MU_MEAS_NOISE_Y 0.0
#define MU_MEAS_NOISE_Z 0.0
#define SIGMA_MEAS_NOISE_X .2
#define SIGMA_MEAS_NOISE_Y .2
#define SIGMA_MEAS_NOISE_Z .2

#define NUM_SAMPLES 100 // Default Number of Samples
#define RESAMPLE_PERIOD 0 // Default Resample Period
#define RESAMPLE_THRESHOLD (NUM_SAMPLES/4.0) // Threshold for Dynamic Resampling

#define FILTER_RATE 1 

class FSLAMNode {
public:
	FSLAMNode(ros::NodeHandle & n);
	virtual ~FSLAMNode();
  
  void doSLAM();
private:
  

	FSLAMFilter * filter;
	bool publish_tf_;
  boost::thread * slamThread;
  ros::NodeHandle n;
};

#endif /* FSLAMNODE_H_ */
