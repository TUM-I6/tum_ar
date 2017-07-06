#include <ar_window/mainwindow.h>
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
	ROS_INFO("[ar_window] Starting ar_window") ;

	ros::init(argc, argv, "ar_window");
	ros::NodeHandle nh ;

	QApplication a(argc, argv);
	MainWindow w;
	//w.show();
	w.showFullScreen();

	w.displayImage("/home/arne/data/sandbox/catkin_ws/src/ar_window/images/blank.png") ;

	ros::Rate rate(50) ;
	while(ros::ok() && w.isVisible()) {
		a.processEvents();
		ros::spinOnce() ;
		rate.sleep() ;
	}

	//a.processEvents();

	ROS_INFO("[ar_window] Exiting...") ;
}
