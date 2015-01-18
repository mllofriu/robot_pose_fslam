/*
 * fslam_nodelet.cpp
 *
 *  Created on: Jan 18, 2015
 *      Author: biorob
 */

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "fslam_node.h"

class FSLAMNodelet : public nodelet::Nodelet {
public:
	FSLAMNodelet() {};
	virtual ~FSLAMNodelet(){};

	virtual void onInit()
	  {
	    ros::NodeHandle nh = this->getPrivateNodeHandle();

	    // resolve node(let) name
	    std::string name = nh.getUnresolvedNamespace();
	    //NODELET_INFO_STREAM("Namespace " << name);
	    int pos = name.find_last_of('/');
	    name = name.substr(pos + 1);

	    NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
	    controller_.reset(new FSLAMNode(nh));

	  }
	private:
	  boost::shared_ptr<FSLAMNode> controller_;
};

PLUGINLIB_EXPORT_CLASS(FSLAMNodelet,
                       nodelet::Nodelet);



