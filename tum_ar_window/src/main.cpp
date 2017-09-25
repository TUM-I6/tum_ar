#include <tum_ar_window/ARInspectionNode.h>
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
	ROS_INFO("[ar_window] Starting ar_window") ;

	ros::init(argc, argv, "ar_window") ;

	tum::ARInspectionNode node(argc, argv) ;
	node.run() ;

	ROS_INFO("[ar_window] Exiting...") ;
}
