// $Id: nonlinearanalyticconditionalgaussianmobile.h 5374 2005-05-06 14:57:05Z TDeLaet $
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

#ifndef __NON_LINEAR_MEAS_MOBILE__
#define __NON_LINEAR_MEAS_MOBILE__

#include <pdf/conditionalpdf.h>
#include <pdf/gaussian.h>
#include <robot_pose_fslam/TransformWithCovarianceStamped.h>

namespace BFL {
/// Non Linear Conditional Gaussian
class NonlinearMeasurementPdf: public ConditionalPdf<robot_pose_fslam::TransformWithCovarianceStamped,
	std::vector<robot_pose_fslam::TransformWithCovarianceStamped> > {
public:
	/// Constructor
	/**
	 @param additiveNoise Pdf representing the additive Gaussian uncertainty
	 */
	NonlinearMeasurementPdf(const Gaussian& measNoise);

	/// Destructor
	virtual ~NonlinearMeasurementPdf();

	// implement this virtual function for measurement model of a particle filter
	virtual Probability ProbabilityGet(
			const robot_pose_fslam::TransformWithCovarianceStamped& measurement) const;

private:
	Gaussian _measNoise;

};

} // End namespace BFL

#endif //
