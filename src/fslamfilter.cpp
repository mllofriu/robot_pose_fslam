#include "fslamfilter.h"

#include <filter/bootstrapfilter.h>

using namespace robot_pose_fslam;
using namespace BFL;
using namespace std;
using namespace tf;

FSLAMFilter::FSLAMFilter(MCPdf<vector<TransformWithCovarianceStamped> > * prior,
		int resample_period, double resample_thrs) :
		BootstrapFilter<vector<TransformWithCovarianceStamped>,
				TransformWithCovarianceStamped>(prior, resample_period,
				resample_thrs) {
}

void FSLAMFilter::mapping(const TransformWithCovarianceStamped & m) {

	MCPdf<vector<TransformWithCovarianceStamped> > * mcpdf = PostGet();
	TransformWithCovarianceStamped measurement(m);

	for (int i = 0; i < mcpdf->NumSamplesGet(); i++) {
		ROS_INFO("%s", measurement.header.frame_id.c_str());
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

		// If landmark is not in the map just return
		if (lmIter == state.end()) {
			// TODO: covariance change?
			ROS_INFO("Adding lm to the map ");
			tf::Transform stateT;
			transformMsgToTF(stateIter->transform.transform, stateT);
			tf::Transform measurementT;
			transformMsgToTF(measurement.transform.transform, measurementT);
			measurementT *= stateT;
			transformTFToMsg(measurementT, measurement.transform.transform);
			measurement.header.frame_id = stateIter->header.frame_id;
			// TODO: Covariance
			state.push_back(measurement);
			mcpdf->SampleGet(i).ValueSet(state);
		}

		// TODO: Kalman filter
	}

}
