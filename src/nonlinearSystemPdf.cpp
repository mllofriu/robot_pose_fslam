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

#include "nonlinearSystemPdf.h"
#include <stdio.h>
#include <tf/transform_datatypes.h>
#include <cstring>

namespace BFL {

using namespace tf;
using namespace geometry_msgs;
using namespace std;
using namespace robot_pose_fslam;

NonlinearSystemPdf::NonlinearSystemPdf(const Gaussian& additiveNoise) :
		ConditionalPdf<vector<TransformWithCovarianceStamped>,
				vector<TransformWithCovarianceStamped> >(1, 2) {
	_additiveNoise = additiveNoise;
}

NonlinearSystemPdf::~NonlinearSystemPdf() {
}

bool NonlinearSystemPdf::SampleFrom(
		Sample<vector<TransformWithCovarianceStamped> >& one_sample, int method,
		void * args) const {
	vector<TransformWithCovarianceStamped> state = ConditionalArgumentGet(0);
	vector<TransformWithCovarianceStamped> odomTransforms =
			ConditionalArgumentGet(1);

	// For each odometry transoform
	vector<TransformWithCovarianceStamped>::iterator stateIter;
	vector<TransformWithCovarianceStamped>::iterator odomIter;
	for (odomIter = odomTransforms.begin(); odomIter != odomTransforms.end();
			odomIter++) {
		// Find the state transform that matches
		for (stateIter = state.begin();
				stateIter != state.end()
						&& stateIter->child_frame_id.compare(
								odomIter->child_frame_id) != 0; stateIter++)
			;
		// If it wasn't there, exit with error
		if (stateIter == state.end()) {
			cout << "Error: the frame to transform is not part of the state";
			return false;
		}
		// Transform the state transform according to odometry
		tf::Transform oldT;
		transformMsgToTF(stateIter->transform.transform, oldT);
		tf::Transform change;
		transformMsgToTF(odomIter->transform.transform, change);
		oldT.mult(oldT,change);

		ROS_INFO("New x %f", oldT.getOrigin().getX());

		transformTFToMsg(oldT,stateIter->transform.transform);
		// TODO: add noise
	}

	one_sample.ValueSet(state);

	return true;
}

} //namespace BFL

