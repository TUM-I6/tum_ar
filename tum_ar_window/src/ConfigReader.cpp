#include <tum_ar_window/ConfigReader.h>
#include <yaml-cpp/yaml.h>

std_msgs::ColorRGBA getColorRGBA(const float r, const float g, const float b, const float a) {
	std_msgs::ColorRGBA color ;
	color.r = r ;
	color.g = g ;
	color.b = b ;
	color.a = a ;
	return color ;
}

const std_msgs::ColorRGBA tum::ConfigReader::BOX_DEFAULT_BORDER_COLOR = getColorRGBA(0, 0, 1, 1) ;
const std_msgs::ColorRGBA tum::ConfigReader::BOX_DEFAULT_FILL_COLOR   = getColorRGBA(0, 0, 0, 0) ;
const std_msgs::ColorRGBA tum::ConfigReader::POI_DEFAULT_BORDER_COLOR = getColorRGBA(1, 0, 0, 1) ;
const std_msgs::ColorRGBA tum::ConfigReader::POI_DEFAULT_FILL_COLOR   = getColorRGBA(1, 1, 1, 0.9) ;

const std::string tum::ConfigReader::DEFAULT_INSTRUCTION = "";
const std::string tum::ConfigReader::BOX_DEFAULT_LABEL   = "" ;
const std::string tum::ConfigReader::POI_DEFAULT_LABEL   = "" ;

const ros::Time tum::ConfigReader::HEADER_DEFAULT_TIME = ros::Time(0) ;
const int tum::ConfigReader::HEADER_DEFAULT_SEQ = 0 ;

std::vector<tum_ar_window::ARSlide> tum::ConfigReader::readConfigFile(const std::string& fileName) {
	ROS_INFO_STREAM("[ConfigReader] Reading "<<fileName) ;
	YAML::Node config = YAML::LoadFile(fileName) ;
	std::vector<tum_ar_window::ARSlide> result ;

	if (!config["slides"]) {
		ROS_ERROR_STREAM("[ConfigReader] No Slides list found!") ;
		return result ;
	}

	for (std::size_t i=0; i<config["slides"].size(); i++) {
		YAML::Node slide = config["slides"][i] ;
		result.push_back(readSlide(slide)) ;
	}

	ROS_INFO_STREAM("[ConfigReader] Read "<<result.size()<<" slides") ;
	return result ;
}

tum_ar_window::ARSlide tum::ConfigReader::readSlide(const YAML::Node& node) {
	tum_ar_window::ARSlide slide ;

	if (node["instruction"]) {
		slide.instruction = node["instruction"].as<std::string>() ;
	}
	else {
		slide.instruction = DEFAULT_INSTRUCTION ;
	}

	for (std::size_t i=0; i<node["boxes"].size(); i++) {
		YAML::Node box = node["boxes"][i] ;
		slide.boxes.push_back(readBox(box)) ;
	}

	for (std::size_t i=0; i<node["pois"].size(); i++) {
		YAML::Node poi = node["pois"][i] ;
		slide.pois.push_back(readPOI(poi)) ;
	}

	return slide ;
}

tum_ar_window::Box tum::ConfigReader::readBox(const YAML::Node& node) {
	tum_ar_window::Box box ;

	box.header.frame_id = node["frame_id"].as<std::string>() ;
	box.header.stamp = HEADER_DEFAULT_TIME ;
	box.header.seq = HEADER_DEFAULT_SEQ ;
	box.position = readPoint(node["position"]) ;
	box.width = node["width"].as<float>() ;
	box.height = node["height"].as<float>() ;

	if (node["border_color"]) {
		box.border_color = readColor(node["border_color"]) ;
	}
	else {
		box.border_color = BOX_DEFAULT_BORDER_COLOR ;	
	}

	if (node["fill_color"]) {
		box.fill_color = readColor(node["fill_color"]) ;
	}
	else {
		box.fill_color = BOX_DEFAULT_FILL_COLOR ;
	}

	if (node["label"]) {
		box.label = node["label"].as<std::string>() ;
	}
	else {
		box.label = BOX_DEFAULT_LABEL ;
	}

	return box ;
}

tum_ar_window::POI tum::ConfigReader::readPOI(const YAML::Node& node) {
	tum_ar_window::POI poi ;

	poi.header.frame_id = node["frame_id"].as<std::string>() ;
	poi.header.stamp = HEADER_DEFAULT_TIME ;
	poi.header.seq = HEADER_DEFAULT_SEQ ;
	poi.position = readPoint(node["position"]) ;
	poi.radius = node["radius"].as<float>() ;

	if (node["border_color"]) {
		poi.border_color = readColor(node["border_color"]) ;
	}
	else {
		poi.border_color = POI_DEFAULT_BORDER_COLOR ;	
	}

	if (node["fill_color"]) {
		poi.fill_color = readColor(node["fill_color"]) ;
	}
	else {
		poi.fill_color = POI_DEFAULT_FILL_COLOR ;
	}

	if (node["label"]) {
		poi.label = node["label"].as<std::string>() ;
	}
	else {
		poi.label = POI_DEFAULT_LABEL ;
	}

	return poi ;
}

std_msgs::ColorRGBA tum::ConfigReader::readColor(const YAML::Node& node) {
	std_msgs::ColorRGBA color ;
	color.r = node["r"].as<float>() ;
	color.g = node["g"].as<float>() ;
	color.b = node["b"].as<float>() ;
	color.a = node["a"].as<float>() ;
	return color ;
}

geometry_msgs::Point tum::ConfigReader::readPoint(const YAML::Node& node) {
	geometry_msgs::Point point ;
	point.x = node["x"].as<float>() ;
	point.y = node["y"].as<float>() ;
	point.z = node["z"].as<float>() ;
	return point ;
}