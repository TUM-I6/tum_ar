#include <tum_ar_window/ARTaskNode.h>
#include <tum_ar_window/ConfigReader.h>
#include <ros/package.h>

#define ROS_PACKAGE_NAME "tum_ar_window"

tum::ARTaskNode::ARTaskNode(QApplication& qa)
: _projector(_nh),
  _renderer(_projector),
  _actionServer(_nh, "ar_inspection", false),
  _qa(qa) {

	_window.showFullScreen();
	_blankSlide.instruction = "";

	bool autostart;
	_nh.param<bool>("autostart", autostart, false);
	_nh.param<bool>("hide_buttons", _hideButtons, false);

	_nh.param<std::string>("task_description", _taskDescriptionFile, ros::package::getPath(ROS_PACKAGE_NAME)+"/config/config.yaml");

	if (_taskDescriptionFile[0] != '/') {
		_taskDescriptionFile = ros::package::getPath(ROS_PACKAGE_NAME)+"/"+_taskDescriptionFile;
	}

	if (autostart) {
		ROS_INFO_STREAM("[ARTaskNode] Auto-starting task without goal...");
		_taskActive = true;
		_slides = ConfigReader::readConfigFile(_taskDescriptionFile);
		if (!_hideButtons) {
			_window.addButtons(_slides[0].outcomes);
		}
	}

	_userInputSub = _nh.subscribe("user_input", 10, &ARTaskNode::userInputCallback, this);
	_actionServer.registerGoalCallback(boost::bind(&ARTaskNode::executeARTask, this));
	_actionServer.start();
}

tum::ARTaskNode::~ARTaskNode() {
}

void tum::ARTaskNode::run() {
	ros::Rate rate(30);

	QPixmap slide;
	while(ros::ok() && _window.isVisible()) {
		// process events and messages
		if (_qa.hasPendingEvents()) {
			_qa.processEvents();
		}
		ros::spinOnce();

		// render new image
		QRect canvas = _window.canvasArea();
		if (_taskActive) {
			if (_step > _slides.size()) {
				ROS_ERROR_STREAM_THROTTLE(1, "[ARTaskNode] Slide index is out of bounds! Did you load any slides?");
				slide = _renderer.renderSlide(_blankSlide, canvas);
			}
			else {
				slide = _renderer.renderSlide(_slides[_step], canvas);
			}
		}
		else {
			slide = _renderer.renderSlide(_blankSlide, canvas);
		}
		_window.display(slide);

		// other stuff
		_projector.publishViewFrustumMarker(_projector.getImagePlane(2.05f));
		rate.sleep();
	}
}

void tum::ARTaskNode::executeARTask() { // const tum_ar_window::ARTaskGoalConstPtr &goal
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
	_window.clearButtons();

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
		ROS_WARN_STREAM("[ARTaskNode] No slides specified. Loading task description from "<<_taskDescriptionFile);
		_slides = ConfigReader::readConfigFile(_taskDescriptionFile);
	}

	if (_slides.size() == 0) {
		ROS_ERROR_STREAM("[ARTaskNode] No slides found - aborting task.");

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

	if (!_hideButtons) {
		_window.addButtons(_slides[_step].outcomes);
	}
}

/*std::vector<tum_ar_msgs::ARSlide> tum::ARTaskNode::loadSlides(const std::string& file) {
	std::vector<tum_ar_msgs::ARSlide> slides;

	std::function<tum_ar_window::POI (int, int, int, const std::string&)> poi = [](int x, int y, int radius, const std::string& label="") {
		tum_ar_window::POI poi;
		poi.position.x = x;
		poi.position.y = y;
		poi.position.z = 0;
		poi.border_color.r = 1;
		poi.border_color.g = 0;
		poi.border_color.b = 0;
		poi.border_color.a = 1;
		poi.fill_color.r = 1;
		poi.fill_color.g = 1;
		poi.fill_color.b = 1;
		poi.fill_color.a = 0.9;
		poi.radius = radius;
		poi.label = label;
		return poi;
	};

	std::function<tum_ar_window::Box (int, int, int, int, const std::string&)> box = [](int x, int y, int w, int h, const std::string& label="") {
		tum_ar_window::Box box;
		box.position.x = x;
		box.position.y = y;
		box.position.z = 0;
		box.border_color.r = 0;
		box.border_color.g = 0;
		box.border_color.b = 1;
		box.border_color.a = 1;
		box.width = w;
		box.height = h;
		box.label = label;
		return box;
	};

	tum_ar_msgs::ARSlide slide1;
	slide1.instruction = "Slide 1";
	slide1.pois.push_back(poi( 500, 500,  50, "label"));
	slide1.pois.push_back(poi( 600, 700,  10, "another label"));
	slide1.pois.push_back(poi( 800, 400, 100, ""));
	slide1.boxes.push_back(box( 200,-100, 300,400, "Fancy Label"));
	slide1.boxes.push_back(box( 200,1000, 300,300, "Fancy Label"));
	slide1.boxes.push_back(box(1500, 200, 600,200, "Fancy Label"));

	tum_ar_msgs::ARSlide slide2;
	slide2.instruction = "Slide 2";
	slide2.pois.push_back(poi(1500, 600,  20, ""));
	slide2.pois.push_back(poi(1000, 600, 150, ""));
	slide2.pois.push_back(poi( 700,1100, 200, "moved label"));
	slide2.boxes.push_back(box( -10, 600, 200,300, "Fancy Label"));
	slide2.boxes.push_back(box(1400, 700, 500,500, "Fancy Label"));

	slides.push_back(slide1);
	slides.push_back(slide2);

	return slides;
}*/

void tum::ARTaskNode::userInputCallback(const tum_ar_msgs::Outcome::ConstPtr& msg) {
	if(!_taskActive) {
		return;
	}

	//ROS_INFO_STREAM("[ARTaskNode] user input received");
	_window.clearButtons();
	publishFeedback(_step, *msg);
	_outcomes.push_back(*msg);
	_step++;

	if (_step >= _slides.size() || msg->id == tum_ar_msgs::Outcome::ID_TASK_ABORTED || msg->id == tum_ar_msgs::Outcome::ID_TIMEOUT) {
		tum_ar_msgs::ARTaskResult result;
		result.outcomes = _outcomes;
		_actionServer.setSucceeded(result);

		_outcomes.clear();
		_taskActive = false;
		_step = 0;
	}
	else if (!_hideButtons) {
		_window.addButtons(_slides[_step].outcomes);
	}
}
