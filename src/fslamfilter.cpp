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

	bool lmAdded = false;
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
			lmAdded = true;
		}
		// TODO: Kalman filter
	}

	// If an LM was added, publish its marker once, using lock_frame to then correspond it to the tf published
	if (lmAdded) {
		char marker_frame[15];
		sprintf(&marker_frame[0], "slam%s", m.child_frame_id.c_str());
		publishVisualMarker(marker_frame, ros::Time::now(), m.child_frame_id);
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
//		char child_frame[10];
//		ros::Time partTime = stateIter->header.stamp;
//		sprintf(&child_frame[0], "part%d", i);
//		tf::StampedTransform st(t, partTime, stateIter->header.frame_id,
//				child_frame);
//		br.sendTransform(st);

// Acumulate transform for the robot pos
		t.setOrigin(t.getOrigin() / mcpdf->NumSamplesGet());
		robotT.setOrigin(robotT.getOrigin() + t.getOrigin());
		robotT.setRotation(robotT.getRotation() + t.getRotation());
		//robotT.mult(robotT, t);
	}

	br.sendTransform(
			tf::StampedTransform(robotT, ros::Time::now(), "map", robotFrame));
	ROS_DEBUG("TF particles published");

	// Publish landmarks
	if (publishLandmarks) {
		vector<tf::Transform> landmarks;
		// Push back particle 0's landmarks transforms
		vector<TransformWithCovarianceStamped> state =
				mcpdf->SampleGet(0).ValueGet();
		vector<TransformWithCovarianceStamped>::iterator stateIter;
		for (stateIter = state.begin(); stateIter != state.end(); stateIter++) {
			if (stateIter->child_frame_id.compare(robotFrame) != 0) {
				tf::Transform t;
				transformMsgToTF(stateIter->transform.transform, t);
				t.setOrigin(t.getOrigin() / mcpdf->NumSamplesGet());
				landmarks.push_back(t);
			}
		}

		// Average the position for each landmark
		for (int i = 1; i < mcpdf->NumSamplesGet(); i++) {
			int j = 0;
			vector<TransformWithCovarianceStamped> state =
					mcpdf->SampleGet(i).ValueGet();

			for (stateIter = state.begin(); stateIter != state.end();
					stateIter++) {
				if (stateIter->child_frame_id.compare(robotFrame) != 0) {
					// Send the transform
					tf::Transform t;
					transformMsgToTF(stateIter->transform.transform, t);
					t.setOrigin(t.getOrigin() / mcpdf->NumSamplesGet());
					landmarks.at(j).setOrigin(
							landmarks.at(j).getOrigin() + t.getOrigin());
					landmarks.at(j).setRotation(
							landmarks.at(j).getRotation() * t.getRotation());
					j++;
				}
			}
		}
		// Publish the transform for each landmark
		for (stateIter = state.begin(); stateIter != state.end(); stateIter++) {
			int j = 0;
			if (stateIter->child_frame_id.compare(robotFrame) != 0) {
				char marker_frame[15];
				sprintf(&marker_frame[0], "slam%s",
						stateIter->child_frame_id.c_str());
				br.sendTransform(
						tf::StampedTransform(landmarks.at(j), ros::Time::now(),
								"map", marker_frame));
				j++;
			}
		}
	}
}

void FSLAMFilter::publishVisualMarker(string frame_id, Time stamp, string mId) {
	static int id = 0;

	visualization_msgs::Marker rvizMarker_;
	tf::poseTFToMsg(tf::Transform(tf::Quaternion(tf::Vector3(0, 0, 1), 0)),
			rvizMarker_.pose);

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
