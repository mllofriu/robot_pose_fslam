/*
 * fslamfilter.h
 *
 *  Created on: Oct 31, 2014
 *      Author: biorob
 */

#ifndef FSLAMFILTER_H_
#define FSLAMFILTER_H_

#include <filter/bootstrapfilter.h>
#include "nonlinearSystemPdf.h"
#include "nonlinearMeasurementPdf.h"
#include <robot_pose_fslam/TransformWithCovarianceStamped.h>
#include <robot_pose_fslam/ResetPosition.h>
#include <tf/transform_broadcaster.h>
#include <semaphore.h>

class FSLAMFilter: public BFL::BootstrapFilter<
		std::vector<robot_pose_fslam::TransformWithCovarianceStamped>,
		robot_pose_fslam::TransformWithCovarianceStamped> {
public:
	FSLAMFilter(
			BFL::MCPdf<
					std::vector<robot_pose_fslam::TransformWithCovarianceStamped> > * prior,
			int resample_period, double resample_thrs, sem_t & mtx);
	void mapping(
			const robot_pose_fslam::TransformWithCovarianceStamped & measurement);
	void publishTF(tf::TransformBroadcaster &, std::string &,
			bool publishLandmarks);
	bool resetPosition(robot_pose_fslam::ResetPosition::Request& request,
			robot_pose_fslam::ResetPosition::Response& response);
	robot_pose_fslam::TransformWithCovarianceStamped * getTransform(
			std::vector<robot_pose_fslam::TransformWithCovarianceStamped> & states,
			std::string & frame);
private:

	void publishVisualMarker(std::string frame_id, ros::Time stamp,
			std::string mId);

	ros::Publisher rvizMarkerPub_;
	sem_t * mtx;
};

#endif /* FSLAMFILTER_H_ */
