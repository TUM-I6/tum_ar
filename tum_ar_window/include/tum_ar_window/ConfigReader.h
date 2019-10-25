#ifndef CONFIGREADER_H
#define CONFIGREADER_H

#include <ros/ros.h>
#include <tum_ar_msgs/ARSlide.h>

namespace YAML {
	class Node ;
}

namespace tum {
	class ConfigReader {
		public:
			static std::vector<tum_ar_msgs::ARSlide> readConfigFile(const std::string& fileName) ;

			static const std_msgs::ColorRGBA BOX_DEFAULT_BORDER_COLOR ;
			static const std_msgs::ColorRGBA BOX_DEFAULT_FILL_COLOR ;
			static const std_msgs::ColorRGBA POI_DEFAULT_BORDER_COLOR ;
			static const std_msgs::ColorRGBA POI_DEFAULT_FILL_COLOR ;

			static const std::string DEFAULT_INSTRUCTION ;
			static const std::string BOX_DEFAULT_LABEL ;
			static const std::string POI_DEFAULT_LABEL ;

			static const ros::Time HEADER_DEFAULT_TIME ;
			static const int HEADER_DEFAULT_SEQ ;

		private:
			static tum_ar_msgs::ARSlide readSlide(const YAML::Node& node);
			static tum_ar_msgs::POI readPOI(const YAML::Node& node);
			static tum_ar_msgs::Box readBox(const YAML::Node& node);
			static tum_ar_msgs::Outcome readOutcome(const YAML::Node& node);
			static std_msgs::ColorRGBA readColor(const YAML::Node& node);
			static std_msgs::Header readHeader(const YAML::Node& node);
			static geometry_msgs::Point readPoint(const YAML::Node& node);
	} ;
} ;

#endif