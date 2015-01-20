/*
 * FSLAMNode.cpp
 *
 *  Created on: Oct 31, 2014
 *      Author: biorob
 */

#include "fslam_node.h"

#include <message_filters/subscriber.h>
#include <message_filters/cache.h>

#include <model/systemmodel.h>
#include <model/measurementmodel.h>

#include "nonlinearSystemPdf.h"
#include "nonlinearMeasurementPdf.h"

// Include file with properties
#include "mobile_robot_wall_cts.h"

#include <ar_pose/ARMarkers.h>

#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <robot_pose_fslam/TransformWithCovarianceStamped.h>

using namespace robot_pose_fslam;
using namespace MatrixWrapper;
using namespace BFL;
using namespace std;
using namespace tf;
using namespace ros;

FSLAMNode::FSLAMNode(NodeHandle & nh) :
		n(nh) {
	slamThread = new boost::thread(boost::bind(&FSLAMNode::doSLAM, this));
}

void FSLAMNode::doSLAM() {
// TODO: read from parameters
	publish_tf_ = true;

	string robotFrame;
	n.getParam("robotFrame", robotFrame);
	ROS_INFO_STREAM("Robot frame: " << robotFrame);

	tf::TransformBroadcaster br;

	//	message_filters::Subscriber<TransformWithCovarianceStamped> sub(nh,
	//			"/landmark", 1);
	//	message_filters::Cache<TransformWithCovarianceStamped> cache(sub, 1);
	message_filters::Subscriber<ar_pose::ARMarkers> sub(n,
			"/ar_pose/ar_pose_marker", 1);
	message_filters::Cache<ar_pose::ARMarkers> cache(sub, 1);

	// TODO: decrease this
	TransformListener tfl(Duration(60));

	// create gaussian
	ColumnVector sys_noise_Mu(3);
	sys_noise_Mu(1) = MU_SYSTEM_NOISE_X;
	sys_noise_Mu(2) = MU_SYSTEM_NOISE_Y;
	sys_noise_Mu(3) = MU_SYSTEM_NOISE_THETA;

	SymmetricMatrix sys_noise_Cov(3);
	sys_noise_Cov = 0.0;
	sys_noise_Cov(1, 1) = SIGMA_SYSTEM_NOISE_X;
	sys_noise_Cov(1, 2) = 0.0;
	sys_noise_Cov(1, 3) = 0.0;
	sys_noise_Cov(2, 1) = 0.0;
	sys_noise_Cov(2, 2) = SIGMA_SYSTEM_NOISE_Y;
	sys_noise_Cov(2, 3) = 0.0;
	sys_noise_Cov(3, 1) = 0.0;
	sys_noise_Cov(3, 2) = 0.0;
	sys_noise_Cov(3, 3) = SIGMA_SYSTEM_NOISE_THETA;

	Gaussian system_Uncertainty(sys_noise_Mu, sys_noise_Cov);

	// create the nonlinear system model
	NonlinearSystemPdf sys_pdf(system_Uncertainty);
	SystemModel<vector<TransformWithCovarianceStamped> > sys_model(&sys_pdf);

	/*********************************
	 * NonLinear Measurement model   *
	 ********************************/

	// create gaussian
	ColumnVector meas_noise_Mu(3);
	meas_noise_Mu(1) = MU_MEAS_NOISE_X;
	meas_noise_Mu(2) = MU_MEAS_NOISE_Y;
	meas_noise_Mu(3) = MU_MEAS_NOISE_Z;

	SymmetricMatrix meas_noise_Cov(3);
	meas_noise_Cov = 0.0;
	meas_noise_Cov(1, 1) = SIGMA_MEAS_NOISE_X;
	meas_noise_Cov(1, 2) = 0.0;
	meas_noise_Cov(1, 3) = 0.0;
	meas_noise_Cov(2, 1) = 0.0;
	meas_noise_Cov(2, 2) = SIGMA_MEAS_NOISE_Y;
	meas_noise_Cov(2, 3) = 0.0;
	meas_noise_Cov(3, 1) = 0.0;
	meas_noise_Cov(3, 2) = 0.0;
	meas_noise_Cov(3, 3) = SIGMA_MEAS_NOISE_Z;

	Gaussian meas_Uncertainty(meas_noise_Mu, meas_noise_Cov);
	NonlinearMeasurementPdf meas_pdf(meas_Uncertainty);
	MeasurementModel<TransformWithCovarianceStamped,
			vector<TransformWithCovarianceStamped> > meas_model(&meas_pdf);

	/****************************
	 * Linear prior DENSITY     *
	 ***************************/
	// Discrete prior for Particle filter (using the continuous Gaussian prior)
	vector<Sample<vector<TransformWithCovarianceStamped> > > prior_samples(
			NUM_SAMPLES);
	Transform initial_t(tf::Quaternion(tf::Vector3(0, 0, 1), 0));
	ros::Time tTime = ros::Time::now();

//	initial_t.setRotation(initial_t.getRotation() * tf::Quaternion(tf::Vector3(0,1,0), -M_PI_2));
	for (vector<Sample<vector<TransformWithCovarianceStamped> > >::iterator iter =
			prior_samples.begin(); iter != prior_samples.end(); iter++) {
		TransformWithCovarianceStamped sample;
		sample.child_frame_id = robotFrame;
		sample.header.frame_id = "map";
		sample.header.stamp = tTime;
		// Copy initial odometry to samples
		transformTFToMsg(initial_t, sample.transform.transform);
		iter->ValueGet().push_back(sample);
	}
	MCPdf<vector<TransformWithCovarianceStamped> > * prior_discr = new MCPdf<
			vector<TransformWithCovarianceStamped> >(NUM_SAMPLES, 1);
	prior_discr->ListOfSamplesSet(prior_samples);

	filter = new FSLAMFilter(prior_discr, 0, NUM_SAMPLES / 4.0);

	ros::Rate r(FILTER_RATE);
	ros::Time lastUpdate = ros::Time::now();
	int timeSinceLMPublished = 0;
	bool updated = false;
	while (ros::ok()) {
		// Spin to let the caches fill themselves
		ros::spinOnce();
		// This update time
		ros::Time now = ros::Time::now();

		// First update with odometry
		// Find odometry transform since last update
		StampedTransform t;
		try {
			tfl.waitForTransform("odom", lastUpdate, "odom", now, "map",
					Duration(.5));
			tfl.lookupTransform("odom", lastUpdate, "odom", now, "map", t);
		} catch (TransformException tfe) {
			ROS_ERROR("%s", tfe.what());
		}
		// Build odometry transform
		TransformWithCovarianceStamped odomT;
		odomT.child_frame_id = robotFrame;
		odomT.header.stamp = now;
		odomT.header.frame_id = "map";
//		t.setOrigin(tf::Vector3(t.getOrigin().getX(),t.getOrigin().getY(),0));
//		t.setRotation(tf::Quaternion().getIdentity());
		transformTFToMsg(t, odomT.transform.transform);
//		ROS_INFO("Z translation: %f", odomT.transform.transform.translation.z);
		vector<TransformWithCovarianceStamped> input;
		input.push_back(odomT);

		filter->Update(&sys_model, input);

		// Get landmark
		ar_pose::ARMarkersConstPtr lm = cache.getElemBeforeTime(now);

		// Filters: only one marker at a time, z coordinate > 0, confidence > 80, not much movement
		if (lm != NULL && lm->markers.size() == 1
				&& lm->markers[0].pose.pose.position.z > 0
				&& lm->markers[0].confidence > 80
				&& lm->markers[0].header.stamp > lastUpdate
				&& t.getOrigin().length() < 0.05 && t.getRotation().getAngle() < M_PI / 20) {
			ROS_DEBUG("Received lm at %d", lm->markers[0].header.stamp.sec);
			// Look for transform between robotFrame and marker
			StampedTransform robotToMarker;
			char child_frame[10];
			sprintf(&child_frame[0], "/M%d", lm->markers[0].id + 1);
			try {
				tfl.waitForTransform(robotFrame, child_frame, now, Duration(3));
				tfl.lookupTransform(robotFrame, child_frame, now,
						robotToMarker);
				// Build measurement transform

				TransformWithCovarianceStamped tlm;
				tlm.header = lm->markers[0].header;
				tlm.child_frame_id = child_frame;
				tlm.header.frame_id = robotFrame;
				transformTFToMsg(robotToMarker, tlm.transform.transform);
				filter->Update(&meas_model, tlm);
				filter->mapping(tlm);

				updated = true;
			} catch (TransformException tfe) {
				ROS_ERROR("%s", tfe.what());
			}

		} else {
			ROS_DEBUG("No lm received");
		}

		if (publish_tf_){
			timeSinceLMPublished++;

			filter->publishTF(br, robotFrame, timeSinceLMPublished > PUBLISH_LMS_TIME || updated);

			if (timeSinceLMPublished || updated)
				timeSinceLMPublished = 0;

			updated = false;
		}


		lastUpdate = now;
		r.sleep();
	}
}

FSLAMNode::~FSLAMNode() {
	// TODO Auto-generated destructor stub
}

int main(int argc, char** argv) {
	init(argc, argv, "robot_pose_fslam");
	NodeHandle nh("~");
	FSLAMNode filter(nh);

	ros::spin();
}

