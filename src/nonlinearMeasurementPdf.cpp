// $Id: nonlinearanalyticconditionalgaussianmobile.cpp 5823 2005-10-27 13:43:02Z TDeLaet $
// Copyright (C) 2006  Tinne De Laet <first dot last at mech dot kuleuven dot be>
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
//

#include "nonlinearMeasurementPdf.h"
#include <tf/transform_datatypes.h>
#include <vector>

namespace BFL {
using namespace tf;
using namespace geometry_msgs;
using namespace std;
using namespace robot_pose_fslam;

NonlinearMeasurementPdf::NonlinearMeasurementPdf(const Gaussian& measNoise) :
		ConditionalPdf<TransformWithCovarianceStamped,
				vector<TransformWithCovarianceStamped> >(1, 1) {
	_measNoise = measNoise;
}

NonlinearMeasurementPdf::~NonlinearMeasurementPdf() {
}

Probability NonlinearMeasurementPdf::ProbabilityGet(
		const TransformWithCovarianceStamped& m) const {
	// Find the transoform that defines the frame in which the measurement is defined
	vector<TransformWithCovarianceStamped> state = this->ConditionalArgumentGet(0);
	vector<TransformWithCovarianceStamped>::iterator stateIter;
	for (stateIter = state.begin();
			stateIter != state.end()
					&& stateIter->child_frame_id.compare(m.header.frame_id)
							!= 0; stateIter++)
		;

	// If robot transform cannot be found, skip the measurement
	if (stateIter == state.end())
		return 1;

	// Copy measurement to modify it after transforms
	TransformWithCovarianceStamped measurement(m);
	// Transform the measurement to the state's parent frame (should be static)
	tf::Transform stateT;
	transformMsgToTF(stateIter->transform.transform, stateT);
	tf::Transform measurementT;
	transformMsgToTF(measurement.transform.transform, measurementT);
	measurementT = stateT * measurementT;
	ROS_DEBUG("Measurement x in world frame: %f", measurementT.getOrigin().getX());
	//transformTFToMsg(measurementT,measurement.transform.transform);

	vector<TransformWithCovarianceStamped>::iterator lmIter;
	for (lmIter = state.begin();
			lmIter != state.end()
						&& lmIter->child_frame_id.compare(m.child_frame_id)
								!= 0; lmIter++)
			;

	// If landmark is not in the map just return
	if (lmIter == state.end())
		return 1;


	// calculate measurement probability
	// using the distance between transforms
	tf::Transform oldMeasT;
	transformMsgToTF(lmIter->transform.transform, oldMeasT);
	ROS_DEBUG("Landmark x in world frame: %f", oldMeasT.getOrigin().getX());
	measurementT = oldMeasT.inverse() * measurementT;
	// Calculate the difference between the old transform and new transform using inverse
	// and measNois prior
	// TODO: use both the lm covariance and measure covariance to find prob.
	ColumnVector diff(3);
	ROS_DEBUG("Diff x: %f", measurementT.getOrigin().getX());
	diff(1) = measurementT.getOrigin().getX();
	diff(2) = measurementT.getOrigin().getY();
	diff(3) = measurementT.getOrigin().getZ();
	ROS_DEBUG("Measurement probability %f",_measNoise.ProbabilityGet(diff).getValue() );
	return _measNoise.ProbabilityGet(diff);
}

} //namespace BFL

