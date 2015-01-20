#include "fslamfilter.h"

#include <filter/bootstrapfilter.h>

#include <visualization_msgs/Marker.h>

using namespace robot_pose_fslam;
using namespace BFL;
using namespace std;
using namespace tf;
using namespace ros;

FSLAMFilter::FSLAMFilter(MCPdf<vector<TransformWithCovarianceStamped> > * prior,
		int resample_period, double resample_thrs) :
		BootstrapFilter<vector<TransformWithCovarianceStamped>,
				TransformWithCovarianceStamped>(prior, resample_period,
				resample_thrs) {

	rvizMarkerPub_ = NodeHandle().advertise<visualization_msgs::Marker>(
			"visualization_marker", 0);
}

void FSLAMFilter::mapping(const TransformWithCovarianceStamped & m) {
	MCPdf<vector<TransformWithCovarianceStamped> > * mcpdf = PostGet();

	for (int i = 0; i < mcpdf->NumSamplesGet(); i++) {
		TransformWithCovarianceStamped measurement(m);
		ROS_DEBUG("Measurement frame %s", measurement.header.frame_id.c_str());
		ROS_DEBUG(
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
			ROS_DEBUG("Adding lm to the map ");
			tf::Transform stateT;
			transformMsgToTF(stateIter->transform.transform, stateT);
			tf::Transform measurementT;
			transformMsgToTF(measurement.transform.transform, measurementT);
			measurementT.mult(stateT, measurementT);
			transformTFToMsg(measurementT, measurement.transform.transform);
			measurement.header.frame_id = stateIter->header.frame_id;
			// TODO: Covariance
			state.push_back(measurement);
			mcpdf->SampleGet(i).ValueSet(state);

			char marker_frame[15];
			sprintf(&marker_frame[0], "part%d%s", i,
					measurement.child_frame_id.c_str());
			publishVisualMarker(marker_frame, measurement.header.stamp, measurement.child_frame_id);
		}

		// TODO: Kalman filter
	}

}

void FSLAMFilter::publishTF(tf::TransformBroadcaster & br,
		std::string & robotFrame, bool publishLandmarks) {
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
		tf::StampedTransform st(t, partTime, stateIter->header.frame_id,
				child_frame);
		br.sendTransform(st);

		// Acumulate transform for the robot pos
		t.setOrigin(t.getOrigin() / mcpdf->NumSamplesGet());
		t.setRotation(t.getRotation());
		robotT.setOrigin(robotT.getOrigin() + t.getOrigin());
		robotT.setRotation(robotT.getRotation() + t.getRotation());
		//robotT.mult(robotT, t);

		// Publish landmarks
		if (publishLandmarks) {
			int j = 0;
			for (stateIter = state.begin(); stateIter != state.end();
					stateIter++) {
				if (stateIter->child_frame_id.compare(robotFrame) != 0) {
					// Send the transform
					tf::Transform t;
					transformMsgToTF(stateIter->transform.transform, t);
					//		ROS_INFO("Particle x %f", t.getOrigin().getX());
					char marker_frame[15];
					sprintf(&marker_frame[0], "part%d%s", i,
							stateIter->child_frame_id.c_str());
					tf::StampedTransform st(t, partTime,
							stateIter->header.frame_id, marker_frame);
					br.sendTransform(st);
					// Send the landmark
//					publishVisualMarker(st, stateIter->header.frame_id,
//							partTime, i * 4 + j, stateIter->child_frame_id);
					j++;
				}
			}
		}

	}

	br.sendTransform(
			tf::StampedTransform(robotT, ros::Time::now(), "map", "robot"));
	ROS_DEBUG("TF particles published");
}

void FSLAMFilter::publishVisualMarker(string frame_id, Time stamp, string mId) {
	static int id = 0;

	visualization_msgs::Marker rvizMarker_;
	tf::poseTFToMsg(tf::Transform(tf::Quaternion(tf::Vector3(0,0,1), 0)), rvizMarker_.pose);

	rvizMarker_.header.frame_id = frame_id;
	rvizMarker_.header.stamp = stamp;
	rvizMarker_.id = id++;

	rvizMarker_.scale.x = .1;
	rvizMarker_.scale.y = .1;
	rvizMarker_.scale.z = .05;
	rvizMarker_.ns = "basic_shapes";
	rvizMarker_.type = visualization_msgs::Marker::CUBE;
	rvizMarker_.action = visualization_msgs::Marker::ADD;
	rvizMarker_.frame_locked = true;

	if (mId.compare("/M1") == 0) {
		rvizMarker_.color.r = 0.0f;
		rvizMarker_.color.g = 0.0f;
		rvizMarker_.color.b = 1.0f;
	} else if (mId.compare("/M2") == 0) {
		rvizMarker_.color.r = 1.0f;
		rvizMarker_.color.g = 0.0f;
		rvizMarker_.color.b = 0.0f;
	} else if (mId.compare("/M3") == 0) {
		rvizMarker_.color.r = 1.0f;
		rvizMarker_.color.g = 0.0f;
		rvizMarker_.color.b = 1.0f;
	} else if (mId.compare("/M4") == 0) {
		rvizMarker_.color.r = 1.0f;
		rvizMarker_.color.g = 1.0f;
		rvizMarker_.color.b = 0.0f;
	} else {
		rvizMarker_.color.r = 0.0f;
		rvizMarker_.color.g = 1.0f;
		rvizMarker_.color.b = 0.0f;
	}
	rvizMarker_.color.a = 0.5;
	rvizMarker_.lifetime = ros::Duration();

	rvizMarkerPub_.publish(rvizMarker_);
}
