#include <ros/ros.h>
#include <tum_table_localization/table_localization.h>

int main(int argc, char *argv[]) {

	// Init ROS

	ros::init(argc, argv, "table_localizaztion") ;

	// Run node

	TableLocalization tableLocalizationNode ;
	tableLocalizationNode.run() ;

	return 0;
}