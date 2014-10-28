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

#include <filter/bootstrapfilter.h>

#include <model/systemmodel.h>
#include <model/measurementmodel.h>

#include "nonlinearSystemPdf.h"
#include "nonlinearMeasurementPdf.h"

#include "mobile_robot.h"

#include <iostream>
#include <fstream>

// Include file with properties
#include "mobile_robot_wall_cts.h"

#include <tf/transform_datatypes.h>
#include <robot_pose_fslam/TransformWithCovarianceStamped.h>

using namespace robot_pose_fslam;
using namespace MatrixWrapper;
using namespace BFL;
using namespace std;
using namespace tf;

/* The purpose of this program is to construct a kalman filter for the problem
 of localisation of a mobile robot equipped with an ultrasonic sensor.
 In this case the orientation is known, which simplifies the model considerably:
 The system model will become linear.
 The ultrasonic measures the distance to the wall (it can be switched off:
 see mobile_robot_wall_cts.h)

 The necessary SYSTEM MODEL is:

 x_k      = x_{k-1} + v_{k-1} * cos(theta) * delta_t
 y_k      = y_{k-1} + v_{k-1} * sin(theta) * delta_t

 The used MEASUREMENT MODEL:
 measuring the (perpendicular) distance z to the wall y = ax + b

 set WALL_CT = 1/sqrt(pow(a,2) + 1)
 z = WALL_CT * a * x - WALL_CT * y + WALL_CT * b + GAUSSIAN_NOISE
 or Z = H * X_k + J * U_k

 where

 H = [ WALL_CT * a       - WALL_CT      0 ]
 and GAUSSIAN_NOISE = N((WALL_CT * b), SIGMA_MEAS_NOISE)

 */

// System Noise
#define MU_SYSTEM_NOISE_X 0.0
#define MU_SYSTEM_NOISE_Y 0.0
#define MU_SYSTEM_NOISE_THETA 0.0
#define SIGMA_SYSTEM_NOISE_X pow(0.01,2)
#define SIGMA_SYSTEM_NOISE_Y pow(0.01,2)
#define SIGMA_SYSTEM_NOISE_THETA pow(2*M_PI/180,2)

#define MU_MEAS_NOISE_X 0.0
#define MU_MEAS_NOISE_Y 0.0
#define MU_MEAS_NOISE_Z 0.0
#define SIGMA_MEAS_NOISE_X .1
#define SIGMA_MEAS_NOISE_Y .1
#define SIGMA_MEAS_NOISE_Z .1

#define NUM_SAMPLES 1 // Default Number of Samples
#define RESAMPLE_PERIOD 0 // Default Resample Period
#define RESAMPLE_THRESHOLD (NUM_SAMPLES/4.0) // Threshold for Dynamic Resampling
int main(int argc, char** argv) {
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
	for (vector<Sample<vector<TransformWithCovarianceStamped> > >::iterator iter =
			prior_samples.begin(); iter != prior_samples.end(); iter++) {
		TransformWithCovarianceStamped sample;
		sample.child_frame_id = "robot";
		sample.transform.transform.translation.x = 1;
		sample.transform.transform.translation.y = 0;
		sample.transform.transform.translation.z = 0;
		sample.transform.transform.rotation.x = 0;
		sample.transform.transform.rotation.y = 0;
		sample.transform.transform.rotation.z = 0;
		sample.transform.transform.rotation.w = 1;
		iter->ValueGet().push_back(sample);
	}
	MCPdf<vector<TransformWithCovarianceStamped> > prior_discr(NUM_SAMPLES, 1);
	prior_discr.ListOfSamplesSet(prior_samples);

	/******************************
	 * Construction of the Filter *
	 ******************************/
	BootstrapFilter<vector<TransformWithCovarianceStamped>,
			TransformWithCovarianceStamped> filter(&prior_discr, 0,
			NUM_SAMPLES / 4.0);

	/***************************
	 * initialise MOBILE ROBOT *
	 **************************/
	// Model of mobile robot in world with one wall
	// The model is used to simultate the distance measurements.


	/*******************
	 * ESTIMATION LOOP *
	 *******************/
	cout << "MAIN: Starting estimation" << endl;
	unsigned int time_step;
	for (time_step = 0; time_step < NUM_TIME_STEPS - 1; time_step++) {
		// DO ONE STEP WITH MOBILE ROBOT
		//mobile_robot.Move(input);
		vector<TransformWithCovarianceStamped> input;
		TransformWithCovarianceStamped odomInput;
		odomInput.transform.transform.translation.x = 0.05;
		odomInput.transform.transform.translation.y = 0;
		odomInput.transform.transform.translation.z = 0;
		odomInput.transform.transform.rotation.x = 0;
		odomInput.transform.transform.rotation.y = 0;
		odomInput.transform.transform.rotation.z = 0;
		odomInput.transform.transform.rotation.w = 1;
		odomInput.child_frame_id =  "robot";
		input.push_back(odomInput);
		// DO ONE MEASUREMENT
		//Transform measurement = mobile_robot.Measure();
		TransformWithCovarianceStamped measurement;
		measurement.child_frame_id = "lm1";
		measurement.header.frame_id = "robot";
		measurement.transform.transform.translation.x = 10;
		// UPDATE FILTER
		filter.Update(&sys_model, input, &meas_model, measurement);

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
