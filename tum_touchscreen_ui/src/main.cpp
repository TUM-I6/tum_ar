#include <tum_touchscreen_ui/TouchscreenWindowNode.h>
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[]) {

	ROS_INFO("[touchscreen_window] Starting touchscreen_window");

	// Qt must be instanciated before ROS
	QApplication qa(argc, argv);
	ros::init(argc, argv, "touchscreen_window");

	tum::TouchscreenWindowNode node(qa);
	node.run();

	ROS_INFO("[touchscreen_window] Exiting...");
	return 0;
}
