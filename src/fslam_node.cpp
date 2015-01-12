/*
 * FSLAMNode.cpp
 *
 *  Created on: Oct 31, 2014
 *      Author: biorob
 */

#include "ros/ros.h"
#include <ros/node_handle.h>

#include <message_filters/subscriber.h>
#include <message_filters/cache.h>

#include "fslam_node.h"

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

FSLAMNode::FSLAMNode(int argc, char ** argv) {
	// TODO: read from parameters
	publish_tf_ = true;

	init(argc, argv, "robot_pose_fslam");

	NodeHandle nh;
	tf::TransformBroadcaster br;

	//	message_filters::Subscriber<TransformWithCovarianceStamped> sub(nh,
	//			"/landmark", 1);
	//	message_filters::Cache<TransformWithCovarianceStamped> cache(sub, 1);
	message_filters::Subscriber<ar_pose::ARMarkers> sub(nh,
			"/ar_pose/ar_pose_marker", 1);
	message_filters::Cache<ar_pose::ARMarkers> cache(sub, 1);

	TransformListener tfl;

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
	StampedTransform initial_t;
	bool gotTransform = false;
	ros::Time tTime = ros::Time::now();
	while (!gotTransform){
		try {
			tfl.waitForTransform("map", "odom", ros::Time(0),
					Duration(20));
			tfl.lookupTransform("map", "odom", ros::Time(0),
					initial_t);
			gotTransform = true;
		} catch (TransformException tfe) {
			ROS_ERROR("%s", tfe.what());
		}
		if (!gotTransform)
			Duration(1).sleep();
	}

//	initial_t.setRotation(initial_t.getRotation() * tf::Quaternion(tf::Vector3(0,1,0), -M_PI_2));
	for (vector<Sample<vector<TransformWithCovarianceStamped> > >::iterator iter =
			prior_samples.begin(); iter != prior_samples.end(); iter++) {
		TransformWithCovarianceStamped sample;
		sample.child_frame_id = "odom";
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
	while (ros::ok()) {
		// Spin to let the caches fill themselves
		ros::spinOnce();
		// This update time
		ros::Time now = ros::Time::now();

		// First update with odometry
		// Find odometry transform since last update
		StampedTransform t;
		try {
			tfl.waitForTransform("odom", lastUpdate, "odom", now,
					"map", Duration(.5));
			tfl.lookupTransform("odom", lastUpdate, "odom", now,
					"map", t);
		} catch (TransformException tfe) {
			ROS_ERROR("%s", tfe.what());
		}
		// Build odometry transform
		TransformWithCovarianceStamped odomT;
		odomT.child_frame_id = "odom";
		odomT.header.stamp = t.stamp_;
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

		if (lm != NULL && lm->markers.size() > 0
				&& lm->markers[0].header.stamp > lastUpdate) {
			ROS_INFO("Received lm at %d", lm->markers[0].header.stamp.sec);
			// Build measurement transform
			TransformWithCovarianceStamped tlm;
			tlm.header = lm->markers[0].header;
			char child_frame[10];
			sprintf(&child_frame[0], "lm%d", lm->markers[0].id);
			tlm.child_frame_id = child_frame;
			tlm.header.frame_id = "robot";
			tlm.transform.transform.translation.x =
					lm->markers[0].pose.pose.position.x;
			tlm.transform.transform.translation.y =
					lm->markers[0].pose.pose.position.y;
			tlm.transform.transform.translation.z =
					lm->markers[0].pose.pose.position.z;
			tlm.transform.transform.rotation =
					lm->markers[0].pose.pose.orientation;
			filter->Update(&meas_model, tlm);
			filter->mapping(tlm);
		} else {
			ROS_INFO("No lm received");
		}

		if (publish_tf_)
			filter->publishTF(br);

		lastUpdate = now;
		r.sleep();
	}

}

FSLAMNode::~FSLAMNode() {
	// TODO Auto-generated destructor stub
}

int main(int argc, char** argv) {
	FSLAMNode filter(argc, argv);
}

