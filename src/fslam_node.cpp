/*
 * FSLAMNode.cpp
 *
 *  Created on: Oct 31, 2014
 *      Author: biorob
 */

#include "fslam_node.h"

#include <model/systemmodel.h>
#include <model/measurementmodel.h>

#include "nonlinearSystemPdf.h"
#include "nonlinearMeasurementPdf.h"

// Include file with properties
#include "mobile_robot_wall_cts.h"

#include <tf/transform_datatypes.h>
#include <robot_pose_fslam/TransformWithCovarianceStamped.h>

using namespace robot_pose_fslam;
using namespace MatrixWrapper;
using namespace BFL;
using namespace std;
using namespace tf;

FSLAMNode::FSLAMNode() {
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
		sample.header.frame_id = "world";
		sample.transform.transform.translation.x = 0;
		sample.transform.transform.translation.y = 0;
		sample.transform.transform.translation.z = 0;
		sample.transform.transform.rotation.x = 0;
		sample.transform.transform.rotation.y = 0;
		sample.transform.transform.rotation.z = 0;
		sample.transform.transform.rotation.w = 1;
		iter->ValueGet().push_back(sample);
	}
	MCPdf<vector<TransformWithCovarianceStamped> > * prior_discr = new MCPdf<
			vector<TransformWithCovarianceStamped> >(NUM_SAMPLES, 1);
	prior_discr->ListOfSamplesSet(prior_samples);

	filter = new FSLAMFilter(prior_discr, 0, NUM_SAMPLES / 4.0);
}

FSLAMNode::~FSLAMNode() {
	// TODO Auto-generated destructor stub
}

