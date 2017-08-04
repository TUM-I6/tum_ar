#ifndef TABLE_LOCALIZATION_H
#define TABLE_LOCALIZATION_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <string.h>
#include <boost/circular_buffer.hpp>
#include <Eigen/Dense>

class TableLocalization {
	public:
		TableLocalization() ;
		void tagDetectionsCallback(const geometry_msgs::PoseArray::ConstPtr& msg) ;
		void run() ;

	private:
		void updateTransformation(const std::string& cornerFrame, boost::circular_buffer<Eigen::Vector3f>& buffer) ;
		static Eigen::Vector3f getAveragePosition(const boost::circular_buffer<Eigen::Vector3f>& buffer) ;

		ros::NodeHandle _nh ;
		ros::Subscriber _tagDetectionsSubscriber ;
		tf::TransformListener _tfListener ;
		tf::TransformBroadcaster _tfBroadcaster ;

		std::string _cameraFrame ;
		std::string _tableFrame ;
		std::string _tagDetectionsTopic ;

		std::string _topLeftCornerFrame ;
		std::string _topRightCornerFrame ;
		std::string _bottomLeftCornerFrame ;
		std::string _bottomRightCornerFrame ;

		boost::circular_buffer<Eigen::Vector3f> _topLeftCornerPositions ;
		boost::circular_buffer<Eigen::Vector3f> _topRightCornerPositions ;
		boost::circular_buffer<Eigen::Vector3f> _bottomLeftCornerPositions ;
		boost::circular_buffer<Eigen::Vector3f> _bottomRightCornerPositions ;
};

#endif