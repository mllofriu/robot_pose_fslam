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
#include <tf/transform_broadcaster.h>

class FSLAMFilter: public BFL::BootstrapFilter<
		std::vector<robot_pose_fslam::TransformWithCovarianceStamped>,
		robot_pose_fslam::TransformWithCovarianceStamped> {
public:
	FSLAMFilter(
			BFL::MCPdf<
					std::vector<robot_pose_fslam::TransformWithCovarianceStamped> > * prior,
			int resample_period, double resample_thrs);
	void mapping(
			const robot_pose_fslam::TransformWithCovarianceStamped & measurement);
	void publishTF(tf::TransformBroadcaster &, std::string &,
			bool publishLandmarks);

private:
<<<<<<< HEAD
	void publishVisualMarker(std::string frame_id,
=======
	void publishVisualMarker(tf::StampedTransform pose, std::string frame_id,
>>>>>>> f2e58ca4a926d34f9ec8464eea297c8b51324ddd
			ros::Time stamp, std::string mId);

	ros::Publisher rvizMarkerPub_;
};

#endif /* FSLAMFILTER_H_ */
