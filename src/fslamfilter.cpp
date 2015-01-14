#include "fslamfilter.h"

#include <filter/bootstrapfilter.h>

using namespace robot_pose_fslam;
using namespace BFL;
using namespace std;
using namespace tf;

FSLAMFilter::FSLAMFilter(MCPdf<vector<TransformWithCovarianceStamped> > * prior,
		int resample_period, double resample_thrs) :
		BootstrapFilter<vector<TransformWithCovarianceStamped>,
				TransformWithCovarianceStamped>(prior, resample_period,
				resample_thrs) {
}

void FSLAMFilter::mapping(const TransformWithCovarianceStamped & m) {

	MCPdf<vector<TransformWithCovarianceStamped> > * mcpdf = PostGet();

	for (int i = 0; i < mcpdf->NumSamplesGet(); i++) {
		TransformWithCovarianceStamped measurement(m);
		ROS_INFO("Measurement frame %s", measurement.header.frame_id.c_str());
		ROS_INFO(
				"Measurement child frame %s", measurement.child_frame_id.c_str());
		vector<TransformWithCovarianceStamped> state =
				mcpdf->SampleGet(i).ValueGet();

		vector<TransformWithCovarianceStamped>::iterator stateIter;
		for (stateIter = state.begin();
				stateIter != state.end()
						&& stateIter->child_frame_id.compare(
								measurement.header.frame_id) != 0; stateIter++)
			;

		// If robot transform cannot be found, skip the measurement
		if (stateIter == state.end()) {
			ROS_WARN("The measurement frame is not found in the robot");
		}

		vector<TransformWithCovarianceStamped>::iterator lmIter;
		for (lmIter = state.begin();
				lmIter != state.end()
						&& lmIter->child_frame_id.compare(
								measurement.child_frame_id) != 0; lmIter++)
			;

		if (lmIter == state.end()) {
			// TODO: covariance change?
			ROS_INFO("Adding lm to the map ");
			tf::Transform stateT;
			transformMsgToTF(stateIter->transform.transform, stateT);
			tf::Transform measurementT;
			transformMsgToTF(measurement.transform.transform, measurementT);
			measurementT.mult(stateT,measurementT);
			transformTFToMsg(measurementT, measurement.transform.transform);
			measurement.header.frame_id = stateIter->header.frame_id;
			// TODO: Covariance
			state.push_back(measurement);
			mcpdf->SampleGet(i).ValueSet(state);
		}

		// TODO: Kalman filter
	}

}

void FSLAMFilter::publishTF(tf::TransformBroadcaster & br, std::string & robotFrame) {
  tf::Transform robotT;

	MCPdf<vector<TransformWithCovarianceStamped> > * mcpdf = PostGet();
	for (int i = 0; i < mcpdf->NumSamplesGet(); i++) {
		vector<TransformWithCovarianceStamped> state =
				mcpdf->SampleGet(i).ValueGet();

		vector<TransformWithCovarianceStamped>::iterator stateIter;
		for (stateIter = state.begin();
				stateIter != state.end()
						&& stateIter->child_frame_id.compare(robotFrame) != 0;
				stateIter++)
			;

		// If robot transform cannot be found, skip the measurement
		if (stateIter == state.end()) {
			ROS_WARN("No robot transform in state");
		}

		tf::Transform t;
		transformMsgToTF(stateIter->transform.transform, t);
    
//		ROS_INFO("Particle x %f", t.getOrigin().getX());
		char child_frame[10];
    ros::Time partTime = stateIter->header.stamp;
		sprintf(&child_frame[0], "part%d", i);
		tf::StampedTransform st(t, partTime,
				stateIter->header.frame_id, child_frame);
		br.sendTransform(st);

    // Acumulate transform for the robot pos
    //t.setOrigin(t.getOrigin() / mcpdf->NumSamplesGet());
    //t.setRotation(t.getRotation() / mcpdf->NumSamplesGet());
    robotT.mult(robotT, t);

		// Publish landmarks
		int j = 0;
		for (stateIter = state.begin(); stateIter != state.end(); stateIter++) {
			if (stateIter->child_frame_id.compare(robotFrame) != 0) {
				tf::Transform t;
				transformMsgToTF(stateIter->transform.transform, t);
				//		ROS_INFO("Particle x %f", t.getOrigin().getX());
				char marker_frame[15];
				sprintf(&marker_frame[0], "part%d%s", i, stateIter->child_frame_id.c_str());
				tf::StampedTransform st(t, partTime,
						stateIter->header.frame_id, marker_frame);
				br.sendTransform(st);
				j++;
			}
		}

	}

  
  br.sendTransform(tf::StampedTransform(robotT, ros::Time::now(), "map", "robot"));
	ROS_INFO("TF particles published");
}

