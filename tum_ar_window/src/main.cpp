#include <tum_ar_window/ARWindow.h>
#include <tum_ar_window/Projector.h>
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
	ROS_INFO("[ar_window] Starting ar_window") ;

	ros::init(argc, argv, "ar_window");
	ros::NodeHandle nh ;

	QApplication a(argc, argv);
	ARWindow w;
	//w.show();
	w.showFullScreen();

	//w.displayImage("/home/arne/data/sandbox/catkin_ws/src/tum_ar/tum_ar_window/images/blank.png") ;
	//w.displayImage("/home/arne/data/sandbox/catkin_ws/src/tum_ar/tum_ar_window/images/type_a/poi_04.png") ;
	w.draw() ;

	tum::Projector projector(nh) ;

	ros::Rate rate(50) ;
	while(ros::ok() && w.isVisible()) {
		w.draw() ;
		a.processEvents();
		ros::spinOnce() ;
		projector.publishViewFrustumMarker(projector.getImagePlane(2.05f)) ;
		rate.sleep() ;
	}

	//a.processEvents();

	ROS_INFO("[ar_window] Exiting...") ;
}
