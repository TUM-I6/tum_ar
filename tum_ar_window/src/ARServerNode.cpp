#include <tum_ar_window/ARServerNode.h>
#include <tum_ar_window/ConfigReader.h>
#include <ros/package.h>

#define ROS_PACKAGE_NAME "tum_ar_window"

tum::ARServerNode::ARServerNode()
: _actionServer(_nh, "ar_inspection", false) {
	_blankSlide.instruction = "";

	bool autostart;
	_nh.param<bool>("autostart", autostart, false);
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

	_actionServer.registerGoalCallback(boost::bind(&ARServerNode::executeARTask, this));
	_actionServer.start();

	if (autostart) {
		ROS_INFO_STREAM("[ARServerNode] Auto-starting task without goal...");

		ros::Duration(3).sleep();
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
		if (goal->task_description_file[0] == '/') {
			_slides = ConfigReader::readConfigFile(goal->task_description_file);
		}
		else {
			_slides = ConfigReader::readConfigFile(ros::package::getPath(ROS_PACKAGE_NAME)+"/"+goal->task_description_file);
		}
	}
	else {
		ROS_WARN_STREAM("[ARServerNode] No slides specified. Loading task description from "<<_taskDescriptionFile);
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
	ROS_INFO_STREAM("[tum_ar_window] Running AR inspection based on "<<_slides.size()<<" slides");
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
