/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Arne Peters - arne.peters@tum.de
 * Technical University of Munich
 * Chair of Robotics, Artificial Intelligence and Real-time Systems
 * Department of Informatics / I6, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * https://www6.in.tum.de
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include <tum_ar_window/ARServerNode.h>
#include <tum_ar_window/ConfigReader.h>
#include <ros/package.h>

tum::ARServerNode::ARServerNode()
: _actionServer(_nh, "ar_task", false) {
	_blankSlide.instruction = "";

	bool autostart;
	float autostart_timeout;
	_nh.param<bool>("autostart", autostart, false);
	_nh.param<float>("autostart_timeout", autostart_timeout, 3.0);
	_nh.param<std::string>("task_description", _taskDescriptionFile, std::string("package:://")+ROS_PACKAGE_NAME+"/slides/example.yaml");

	_userInputSub = _nh.subscribe("user_input", 10, &ARServerNode::userInputCallback, this);
	_arSlidePub = _nh.advertise<tum_ar_msgs::ARSlide>("ar_slide", 1);

	if (autostart) {
		_taskActive = true;
		_slides = ConfigReader::readARTaskDescription(ConfigReader::preparePath(_taskDescriptionFile));
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
		_slides = ConfigReader::readARTaskDescription(ConfigReader::preparePath(goal->task_description_file));
	}
	else {
		ROS_WARN_STREAM("[ARServerNode] No slides specified. Using sample slides from "<<_taskDescriptionFile);
		_slides = ConfigReader::readARTaskDescription(_taskDescriptionFile);
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

bool tum::ARServerNode::loadTaskFromFileServiceCB(tum_ar_msgs::LoadTaskFromFile::Request& request, tum_ar_msgs::LoadTaskFromFile::Response& response) {
	ROS_INFO_STREAM("[tum_ar_server] Request path "<<request.task_description_file);
	std::string path = ConfigReader::preparePath(request.task_description_file);
	ROS_INFO_STREAM("[tum_ar_server] Loading task description from "<<path);
	response.slides = ConfigReader::readARTaskDescription(path);
	return true;
}

