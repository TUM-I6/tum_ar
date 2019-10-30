#include <tum_ar_window/ConfigReader.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ros/package.h>

#define ROS_PACKAGE_NAME "tum_ar_window"

std_msgs::ColorRGBA getColorRGBA(const float r, const float g, const float b, const float a) {
	std_msgs::ColorRGBA color;
	color.r = r;
	color.g = g;
	color.b = b;
	color.a = a;
	return color;
}

const std_msgs::ColorRGBA tum::ConfigReader::BOX_DEFAULT_BORDER_COLOR = getColorRGBA(0, 0, 1, 1);
const std_msgs::ColorRGBA tum::ConfigReader::BOX_DEFAULT_FILL_COLOR   = getColorRGBA(0, 0, 0, 0);
const std_msgs::ColorRGBA tum::ConfigReader::POI_DEFAULT_BORDER_COLOR = getColorRGBA(1, 0, 0, 1);
const std_msgs::ColorRGBA tum::ConfigReader::POI_DEFAULT_FILL_COLOR   = getColorRGBA(1, 1, 1, 0.9);

const std::string tum::ConfigReader::DEFAULT_INSTRUCTION = "";
const std::string tum::ConfigReader::BOX_DEFAULT_LABEL   = "";
const std::string tum::ConfigReader::POI_DEFAULT_LABEL   = "";

const ros::Time tum::ConfigReader::HEADER_DEFAULT_TIME = ros::Time(0);
const int tum::ConfigReader::HEADER_DEFAULT_SEQ = 0;

bool fileExist(const char *fileName) {
	std::ifstream infile(fileName);
	return infile.good();
}

std::vector<tum_ar_msgs::ARSlide> tum::ConfigReader::readARTaskDescription(const std::string& fileName) {
	ROS_INFO_STREAM("[ConfigReader] Reading "<<fileName);
	std::vector<tum_ar_msgs::ARSlide> result;

	if (fileExist(fileName.c_str())) {
		YAML::Node config = YAML::LoadFile(fileName);

		if (!config["slides"]) {
			ROS_ERROR_STREAM("[ConfigReader] No Slides list found!");
			return result;
		}

		for (std::size_t i = 0; i < config["slides"].size(); i++) {
			YAML::Node slide = config["slides"][i];
			result.push_back(readSlide(slide));
		}

		ROS_INFO_STREAM("[ConfigReader] Read " << result.size() << " slides");
	}

	return result;
}

tum::Projector::Config tum::ConfigReader::readProjectorConfig(const std::string& fileName) {
	ROS_INFO_STREAM("[ConfigReader] Reading "<<fileName);
	tum::Projector::Config result;

	if (fileExist(fileName.c_str())) {
		YAML::Node node = YAML::LoadFile(fileName);

		if (node["projector_frame"]) {
			result.projectorFrame = node["projector_frame"].as<std::string>();
		}
		else {
			ROS_ERROR_STREAM("No projector frame found. Did you specify 'projector_frame'?");
		}

		result.k = readProjectionMatrix(node);
		result.resolution = readResolutionVector(node);
	}

	return result;
}

std::string tum::ConfigReader::preparePath(const std::string& rawPath) {
	std::string path;

	if (rawPath.size() <= 0) {
		path = "";
	}
	else if (rawPath[0] == '/') {
		// absolute path starting with "/"
		path = rawPath;
	}
	else if (rawPath.rfind("file://", 0) == 0) {
		// absolute path starting with "file://"
		std::string subpath = rawPath.substr(std::string("file://").size(), rawPath.size());
		path = "/"+subpath;
	}
	else if (rawPath.rfind("package://", 0) == 0) {
		// relative path to tos package, starting with "package://"
		std::string subpath = rawPath.substr(std::string("package://").size(), rawPath.size());
		std::string targetPackage;

		unsigned int i=0;
		while (subpath.at(i) != '/') {
			targetPackage += subpath.at(i);
			i++;
		}

		std::string remainingPath = subpath.substr(i, rawPath.size());
		path = ros::package::getPath(targetPackage)+remainingPath;
	}
	else {
		// relative path
		path = ros::package::getPath(ROS_PACKAGE_NAME)+"/"+rawPath;
	}

	return path;
}

tum_ar_msgs::ARSlide tum::ConfigReader::readSlide(const YAML::Node& node) {
	tum_ar_msgs::ARSlide slide;

	if (node["instruction"]) {
		slide.instruction = node["instruction"].as<std::string>();
	}
	else {
		slide.instruction = DEFAULT_INSTRUCTION;
	}

	for (std::size_t i=0; i<node["boxes"].size(); i++) {
		YAML::Node box = node["boxes"][i];
		slide.boxes.push_back(readBox(box));
	}

	for (std::size_t i=0; i<node["pois"].size(); i++) {
		YAML::Node poi = node["pois"][i];
		slide.pois.push_back(readPOI(poi));
	}

	for (std::size_t i=0; i<node["outcomes"].size(); i++) {
		YAML::Node outcome = node["outcomes"][i];
		slide.outcomes.push_back(readOutcome(outcome));
	}

	return slide;
}

tum_ar_msgs::Box tum::ConfigReader::readBox(const YAML::Node& node) {
	tum_ar_msgs::Box box;

	box.header.frame_id = node["frame_id"].as<std::string>();
	box.header.stamp = HEADER_DEFAULT_TIME;
	box.header.seq = HEADER_DEFAULT_SEQ;
	box.position = readPoint(node["position"]);
	box.width = node["width"].as<float>();
	box.height = node["height"].as<float>();

	if (node["border_color"]) {
		box.border_color = readColor(node["border_color"]);
	}
	else {
		box.border_color = BOX_DEFAULT_BORDER_COLOR;	
	}

	if (node["fill_color"]) {
		box.fill_color = readColor(node["fill_color"]);
	}
	else {
		box.fill_color = BOX_DEFAULT_FILL_COLOR;
	}

	if (node["label"]) {
		box.label = node["label"].as<std::string>();
	}
	else {
		box.label = BOX_DEFAULT_LABEL;
	}

	return box;
}

tum_ar_msgs::POI tum::ConfigReader::readPOI(const YAML::Node& node) {
	tum_ar_msgs::POI poi;

	poi.header.frame_id = node["frame_id"].as<std::string>();
	poi.header.stamp = HEADER_DEFAULT_TIME;
	poi.header.seq = HEADER_DEFAULT_SEQ;
	poi.position = readPoint(node["position"]);
	poi.radius = node["radius"].as<float>();

	if (node["border_color"]) {
		poi.border_color = readColor(node["border_color"]);
	}
	else {
		poi.border_color = POI_DEFAULT_BORDER_COLOR;	
	}

	if (node["fill_color"]) {
		poi.fill_color = readColor(node["fill_color"]);
	}
	else {
		poi.fill_color = POI_DEFAULT_FILL_COLOR;
	}

	if (node["label"]) {
		poi.label = node["label"].as<std::string>();
	}
	else {
		poi.label = POI_DEFAULT_LABEL;
	}

	return poi;
}

tum_ar_msgs::Outcome tum::ConfigReader::readOutcome(const YAML::Node& node) {
	tum_ar_msgs::Outcome outcome;
	outcome.id = node["id"].as<unsigned int>();
	outcome.type = node["type"].as<unsigned int>();
	outcome.name = node["name"].as<std::string>();
	return outcome;
}

std_msgs::ColorRGBA tum::ConfigReader::readColor(const YAML::Node& node) {
	std_msgs::ColorRGBA color;
	color.r = node["r"].as<float>();
	color.g = node["g"].as<float>();
	color.b = node["b"].as<float>();
	color.a = node["a"].as<float>();
	return color;
}

geometry_msgs::Point tum::ConfigReader::readPoint(const YAML::Node& node) {
	geometry_msgs::Point point;
	point.x = node["x"].as<float>();
	point.y = node["y"].as<float>();
	point.z = node["z"].as<float>();
	return point;
}

Eigen::Matrix3f tum::ConfigReader::readProjectionMatrix(const YAML::Node& node) {
	Eigen::Matrix3f matrix = Eigen::Matrix3f::Zero();

	if (!node["projection_matrix"]) {
		ROS_ERROR_STREAM("No projection matrix found. Did you specify 'projection_matrix'?");
	}
	else if (node["projection_matrix"].size() < 3) {
		ROS_ERROR_STREAM("Invalid projection matrix definition!");
	}
	else {
		for (std::size_t i = 0; i < 3; i++) {
			if (node["projection_matrix"][i].size() < 3) {
				ROS_ERROR_STREAM("Invalid projection matrix definition!");
				return matrix;
			}
		}

		matrix << node["projection_matrix"][0][0].as<float>(), node["projection_matrix"][0][1].as<float>(), node["projection_matrix"][0][2].as<float>(),
		          node["projection_matrix"][1][0].as<float>(), node["projection_matrix"][1][1].as<float>(), node["projection_matrix"][1][2].as<float>(),
		          node["projection_matrix"][2][0].as<float>(), node["projection_matrix"][2][1].as<float>(), node["projection_matrix"][2][2].as<float>();
	}

	return matrix;
}

Eigen::Vector2i tum::ConfigReader::readResolutionVector(const YAML::Node& node) {
	Eigen::Vector2i resolution = Eigen::Vector2i::Zero();

	if (!node["resolution"]) {
		ROS_ERROR_STREAM("No resolution vector found. Did you specify 'resolution'?");
	}
	else if (node["resolution"].size() != 2) {
		ROS_ERROR_STREAM("Invalid resolution definition!");
	}
	else {
		resolution.x() = node["resolution"][0].as<int>();
		resolution.y() = node["resolution"][1].as<int>();
	}

	return resolution;
}
