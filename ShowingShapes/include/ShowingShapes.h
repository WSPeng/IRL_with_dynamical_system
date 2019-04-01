#ifndef __SHOWING_SHAPES_H__
#define __SHOWING_SHAPES_H__

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "geometry_msgs/Pose.h"


class ShowingShapes
{
private:

	uint32_t shape;

	ros::NodeHandle _n;
	ros::Rate _loopRate;
	float _dt;

	ros::Subscriber _subPoseObs;
	ros::Subscriber _subPoseTar;

	ros::Publisher _marker_pub_obs;
	ros::Publisher _marker_pub_tar;

	visualization_msgs::Marker marker_obs;
	visualization_msgs::Marker marker_tar;

	geometry_msgs::Pose _msgPositionObs;
	geometry_msgs::Pose _msgPositionTar;

	bool _stop;
	bool _recievedObsPositionInput;
	bool _recievedTarPositionInput;
	//Other variables
	static ShowingShapes* me;

public:

	ShowingShapes(ros::NodeHandle &n, double frequency);

	bool init();

	void run();

private:

    static void stopNodeCallback(int sig);

	void subPoseObs(const geometry_msgs::Pose::ConstPtr& msg);

	void subPoseTar(const geometry_msgs::Pose::ConstPtr& msg);

};



#endif
