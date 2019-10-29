#include <tum_ar_window/ARServerNode.h>
#include <tum_ar_window/ConfigReader.h>
#include <ros/package.h>

#define ROS_PACKAGE_NAME "tum_ar_window"

tum::ARServerNode::ARServerNode()
: _actionServer(_nh, "ar_task", false) {
	_blankSlide.instruction = "";

	bool autostart;
	float autostart_timeout;
	_nh.param<bool>("autostart", autostart, false);
	_nh.param<float>("autostart_timeout", autostart_timeout, 3.0);
	_nh.param<std::string>("task_description", _taskDescriptionFile, ros::package::getPath(ROS_PACKAGE_NAME)+"/config/config.yaml");

	if (_taskDescriptionFile[0] != '/') {
		_taskDescriptionFile = ros::package::getPath(ROS_PACKAGE_NAME)+"/"+_taskDescriptionFile;
	}

	_userInputSub = _nh.subscribe("user_input", 10, &ARServerNode::userInputCallback, this);
	_arSlidePub = _nh.advertise<tum_ar_msgs::ARSlide>("ar_slide", 1);

	if (autostart) {
		_taskActive = true;
		_slides = ConfigReader::readConfigFile(_taskDescriptionFile);
	}

	_loadTaskFromFileServer = _nh.advertiseService("load_task", &ARServerNode::loadTaskFromFileServiceCB, this);
	_actionServer.registerGoalCallback(boost::bind(&ARServerNode::executeARTask, this));
	_actionServer.start();

	if (autostart) {
		ROS_INFO_STREAM("[ARServerNode] Auto-starting task without goal...");

		ros::Duration(autostart_timeout).sleep();
		_arSlidePub.publish(_slides[_step]);
	}
}

tum::ARServerNode::~ARServerNode() {
}

void tum::ARServerNode::run() {
	ros::Rate rate(30);

	while(ros::ok()) {
		// process events and messages
		ros::spinOnce();
		rate.sleep();
	}
}

void tum::ARServerNode::executeARTask() { // const tum_ar_window::ARTaskGoalConstPtr &goal
	if (_taskActive) {
		// abort current goal
		tum_ar_msgs::Outcome abort;
		abort.id = tum_ar_msgs::Outcome::ID_TASK_ABORTED;
		abort.type = tum_ar_msgs::Outcome::TYPE_WARN;
		abort.name = "Task aborted due to new goal.";
		_outcomes.push_back(abort);
		
		tum_ar_msgs::ARTaskResult result;
		result.outcomes = _outcomes;
		_actionServer.setAborted(result);
	}

	tum_ar_msgs::ARTaskGoalConstPtr goal = _actionServer.acceptNewGoal();
	_step = 0;
	_taskActive = true;
	_outcomes.clear();

	if (goal->slides.size() > 0) {
		_slides = goal->slides;
	}
	else if (goal->task_description_file != "") {
		_slides = ConfigReader::readConfigFile(preparePath(goal->task_description_file));
	}
	else {
		ROS_WARN_STREAM("[ARServerNode] No slides specified. Using sample slides from "<<_taskDescriptionFile);
		_slides = ConfigReader::readConfigFile(_taskDescriptionFile);
	}

	if (_slides.size() == 0) {
		ROS_ERROR_STREAM("[ARServerNode] No slides found - aborting task.");

		tum_ar_msgs::Outcome abort;
		abort.id = tum_ar_msgs::Outcome::ID_TASK_ABORTED;
		abort.type = tum_ar_msgs::Outcome::TYPE_WARN;
		abort.name = "No slides found - aborting task.";
		
		tum_ar_msgs::ARTaskResult result;
		result.outcomes.push_back(abort);
		_actionServer.setAborted(result);
		_taskActive = false;
		return;
	}

	// publish info to the console for the user
	ROS_INFO_STREAM("[tum_ar_server] Running AR inspection based on "<<_slides.size()<<" slides");
	_arSlidePub.publish(_slides[_step]);
}

void tum::ARServerNode::userInputCallback(const tum_ar_msgs::Outcome::ConstPtr& msg) {
	if(!_taskActive) {
		return;
	}

	//ROS_INFO_STREAM("[ARServerNode] user input received");
	publishFeedback(_step, *msg);
	_outcomes.push_back(*msg);
	_step++;

	if (_step >= _slides.size() || msg->id == tum_ar_msgs::Outcome::ID_TASK_ABORTED || msg->id == tum_ar_msgs::Outcome::ID_TIMEOUT) {
		_arSlidePub.publish(_blankSlide);

		tum_ar_msgs::ARTaskResult result;
		result.outcomes = _outcomes;
		_actionServer.setSucceeded(result);

		_outcomes.clear();
		_taskActive = false;
		_step = 0;
	}
	else {
		_arSlidePub.publish(_slides[_step]);
	}
}

std::string tum::ARServerNode::preparePath(const std::string& rawPath) {
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

bool tum::ARServerNode::loadTaskFromFileServiceCB(tum_ar_msgs::LoadTaskFromFile::Request& request, tum_ar_msgs::LoadTaskFromFile::Response& response) {
	ROS_INFO_STREAM("[tum_ar_server] Request path "<<request.task_description_file);
	std::string path = preparePath(request.task_description_file);
	ROS_INFO_STREAM("[tum_ar_server] Loading task description from "<<path);
	response.slides = ConfigReader::readConfigFile(path);
	return true;
}

