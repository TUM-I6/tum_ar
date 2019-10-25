#include <tum_ar_window/ARTaskNode.h>
#include <tum_ar_window/ARWindow.h>
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[]) {

	ROS_INFO("[ar_window] Starting ar_window") ;

	// Qt must be instanciated before ROS
	QApplication qa(argc, argv) ;
	ros::init(argc, argv, "ar_window") ;

	tum::ARTaskNode node(qa) ;
	node.run() ;

	ROS_INFO("[ar_window] Exiting...") ;
	return 0;
}
