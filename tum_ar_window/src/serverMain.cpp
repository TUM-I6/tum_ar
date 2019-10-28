#include <tum_ar_window/ARServerNode.h>
#include <tum_ar_window/ARWindow.h>
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[]) {

	ROS_INFO("[ar_server] Starting ar_server");

	ros::init(argc, argv, "ar_server");

	tum::ARServerNode node;
	node.run();

	ROS_INFO("[ar_server] Exiting...");
	return 0;
}
