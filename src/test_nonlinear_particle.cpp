// $Id: test_nonlinear_particle.cpp 5925 2006-03-14 21:23:49Z tdelaet $
// Copyright (C) 2006 Tinne De Laet <first dot last at mech dot kuleuven dot be>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

/* Demonstration program for the Bayesian Filtering Library.
 Mobile robot localization with respect to wall with different possibilities for filter
 */

#include <tf/transform_datatypes.h>
#include <robot_pose_fslam/TransformWithCovarianceStamped.h>
#include <ros/ros.h>
#include <ros/node_handle.h>

using namespace robot_pose_fslam;
using namespace std;
using namespace tf;

int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_pose_fslam_test");

	ros::NodeHandle nh;
	ros::Publisher chatter_pub = nh.advertise<TransformWithCovarianceStamped>("/landmark", 1);


//
//	/***************************
//	 * initialise MOBILE ROBOT *
//	 **************************/
//	// Model of mobile robot in world with one wall
//	// The model is used to simultate the distance measurements.
//	/*******************
//	 * ESTIMATION LOOP *
//	 *******************/
//	cout << "MAIN: Starting estimation" << endl;
//	unsigned int time_step;
	ros::Rate r(.5);
	for (int i = 0; i < 20; i++) {
		// DO ONE STEP WITH MOBILE ROBOT
		//mobile_robot.Move(input);
//		vector<TransformWithCovarianceStamped> input;
//		TransformWithCovarianceStamped odomInput;
//		odomInput.transform.transform.translation.x = 0.05;
//		odomInput.transform.transform.translation.y = 0;
//		odomInput.transform.transform.translation.z = 0;
//		odomInput.transform.transform.rotation.x = 0;
//		odomInput.transform.transform.rotation.y = 0;
//		odomInput.transform.transform.rotation.z = 0;
//		odomInput.transform.transform.rotation.w = 1;
//		odomInput.child_frame_id = "robot";
//		odomInput.header.frame_id = "world";
//		input.push_back(odomInput);
		// DO ONE MEASUREMENT
		//Transform measurement = mobile_robot.Measure();
		TransformWithCovarianceStamped measurement;
		measurement.child_frame_id = "lm1";
		measurement.header.frame_id = "robot";
		measurement.header.stamp = ros::Time::now();
		measurement.transform.transform.translation.x = 1;
		measurement.transform.transform.translation.y = 0;
		measurement.transform.transform.translation.z = 0;
		measurement.transform.transform.rotation.x = 0;
		measurement.transform.transform.rotation.y = 0;
		measurement.transform.transform.rotation.z = 0;
		measurement.transform.transform.rotation.w = 1;
		chatter_pub.publish(measurement);
		ROS_INFO("message published");
		r.sleep();
	} // estimation loop

//	Pdf<vector<TransformWithCovarianceStamped> > * posterior = filter.PostGet();
//	cout << "After " << time_step + 1 << " timesteps " << endl;
//	cout << " Posterior Mean = " << endl << posterior->ExpectedValueGet()
//			<< endl << " Covariance = " << endl << posterior->CovarianceGet()
//			<< "" << endl;
//
//	cout << "======================================================" << endl
//			<< "End of the Bootstrap filter for mobile robot localisation"
//			<< endl << "======================================================"
//			<< endl;

	return 0;
}
