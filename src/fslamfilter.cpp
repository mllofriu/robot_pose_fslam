#include "fslamfilter.h"

#include <filter/bootstrapfilter.h>

#include <visualization_msgs/Marker.h>

using namespace robot_pose_fslam;
using namespace BFL;
using namespace std;
using namespace tf;
using namespace ros;

FSLAMFilter::FSLAMFilter(MCPdf<vector<TransformWithCovarianceStamped> > * prior,
		int resample_period, double resample_thrs, sem_t & mtx) :
		BootstrapFilter<vector<TransformWithCovarianceStamped>,
				TransformWithCovarianceStamped>(prior, resample_period,
				resample_thrs) {

	rvizMarkerPub_ = NodeHandle().advertise<visualization_msgs::Marker>(
			"visualization_marker", 0);
	this->mtx = &mtx;
}

bool FSLAMFilter::resetPosition(
		robot_pose_fslam::ResetPosition::Request& request,
		robot_pose_fslam::ResetPosition::Response& response) {
	sem_wait(mtx);
	// Iterate over every particle
	MCPdf<vector<TransformWithCovarianceStamped> > * mcpdf = PostGet();
	for (int i = 0; i < mcpdf->NumSamplesGet(); i++) {
		vector<TransformWithCovarianceStamped> state =
				mcpdf->SampleGet(i).ValueGet();

		// Find the particles transform for the robot
		vector<TransformWithCovarianceStamped>::iterator stateIter;
		for (stateIter = state.begin();
				stateIter != state.end()
						&& stateIter->child_frame_id.compare("robot") != 0;
				stateIter++)
			;
		// Update it with the sent transform
		stateIter->transform.transform = request.pose;
		stateIter->header.stamp = request.header.stamp;

		mcpdf->SampleGet(i).ValueSet(state);
	}
	sem_post(mtx);

	ROS_INFO("Position has been reseted");

	return true;
}

void FSLAMFilter::mapping(const TransformWithCovarianceStamped & m) {
	sem_wait(mtx);
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
		}
		// TODO: Kalman filter
	}
	sem_post(mtx);
}

void FSLAMFilter::publishTF(tf::TransformBroadcaster & br,
		std::string & robotFrame, bool publishLandmarks) {

	sem_wait(mtx);
	// Pick the most weighted particle to avoid problems with rotation averaging
	MCPdf<vector<TransformWithCovarianceStamped> > * mcpdf = PostGet();
	vector<WeightedSample<vector<TransformWithCovarianceStamped> > > wSamples =
			mcpdf->ListOfSamplesGet();
	double maxWeight = -INFINITY;
	int index = 0;
	for (int i = 0; i < wSamples.size(); i++)
		if (wSamples.at(i).WeightGet() > maxWeight) {
			index = i;
			maxWeight = wSamples.at(i).WeightGet();
		}

	tf::Transform robotT;
	TransformWithCovarianceStamped * mostWeighted = getTransform(
			wSamples.at(index).ValueGet(), robotFrame);
	transformMsgToTF(mostWeighted->transform.transform, robotT);
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
			state = mcpdf->SampleGet(i).ValueGet();

			for (stateIter = state.begin(); stateIter != state.end();
					stateIter++) {
				if (stateIter->child_frame_id.compare(robotFrame) != 0) {
					// Send the transform
					tf::Transform t;
					transformMsgToTF(stateIter->transform.transform, t);
					t.setOrigin(t.getOrigin() / mcpdf->NumSamplesGet());
					landmarks.at(j).setOrigin(
							landmarks.at(j).getOrigin() + t.getOrigin());
					//landmarks.at(j).setRotation(
					//	landmarks.at(j).getRotation() * t.getRotation());
					j++;
				}
			}
		}

		// Publish the transform for each landmark
		state = mcpdf->SampleGet(0).ValueGet();
		int j = 0;
		for (stateIter = state.begin(); stateIter != state.end(); stateIter++) {
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

		// Publish Marker
		state = mcpdf->SampleGet(0).ValueGet();
		for (stateIter = state.begin(); stateIter != state.end(); stateIter++) {
			if (stateIter->child_frame_id.compare(robotFrame) != 0) {
				char marker_frame[15];
				sprintf(&marker_frame[0], "slam%s",
						stateIter->child_frame_id.c_str());
				publishVisualMarker(marker_frame, ros::Time::now(),
						marker_frame);
			}
		}

	}
	sem_post(mtx);
}

TransformWithCovarianceStamped * getTransform(
		vector<TransformWithCovarianceStamped> & state, string & frame) {
	vector<TransformWithCovarianceStamped>::iterator stateIter;
	for (stateIter = state.begin();
			stateIter != state.end()
					&& stateIter->child_frame_id.compare(frame) != 0;
			stateIter++)
		;
	return &(*stateIter);
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

	if (mId.compare("slam/M1") == 0) {
		rvizMarker_.color.r = 0.0f;
		rvizMarker_.color.g = 0.0f;
		rvizMarker_.color.b = 1.0f;
	} else if (mId.compare("slam/M2") == 0) {
		rvizMarker_.color.r = 1.0f;
		rvizMarker_.color.g = 0.0f;
		rvizMarker_.color.b = 0.0f;
	} else if (mId.compare("slam/M3") == 0) {
		rvizMarker_.color.r = 1.0f;
		rvizMarker_.color.g = 0.0f;
		rvizMarker_.color.b = 1.0f;
	} else if (mId.compare("slam/M4") == 0) {
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
