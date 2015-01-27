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

#define INITIAL_X 0
#define INITIAL_Y 0
#define INITIAL_Z 0
#define INITIAL_THETA 1.57

#define MU_SYSTEM_NOISE_X 0.0
#define MU_SYSTEM_NOISE_Y 0.0
#define MU_SYSTEM_NOISE_THETA 0.0
#define SIGMA_SYSTEM_NOISE_X 0.4
#define SIGMA_SYSTEM_NOISE_Y 0.4
#define SIGMA_SYSTEM_NOISE_THETA 0.1

#define MU_MEAS_NOISE_X 0.0
#define MU_MEAS_NOISE_Y 0.0
#define MU_MEAS_NOISE_Z 0.0
#define SIGMA_MEAS_NOISE_X .2
#define SIGMA_MEAS_NOISE_Y .2
#define SIGMA_MEAS_NOISE_Z .2

#define NUM_SAMPLES 100 // Default Number of Samples
#define RESAMPLE_PERIOD 0 // Default Resample Period
#define RESAMPLE_THRESHOLD (NUM_SAMPLES/1.0) // Threshold for Dynamic Resampling

#define PUBLISH_LMS_TIME 30

#define FILTER_RATE 2 

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
