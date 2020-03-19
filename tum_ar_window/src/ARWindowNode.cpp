#include <tum_ar_window/ARWindowNode.h>
#include <tum_ar_window/ConfigReader.h>
#include <ros/package.h>

#define ROS_PACKAGE_NAME "tum_ar_window"

tum::ARWindowNode::ARWindowNode(QApplication& qa)
		: _projector(_nh, _projectorConfig),
		  _renderer(_projector),
		  _qa(qa) {

	_window.showFullScreen();

	_nh.param<bool>("hide_buttons", _hideButtons, false);
	_nh.param<std::string>("projector_config", _projectorConfigFile, std::string("package:://")+ROS_PACKAGE_NAME+"/config/projector_tum.yaml");

	_projectorConfig = ConfigReader::readProjectorConfig(ConfigReader::preparePath(_projectorConfigFile));
	_projector.setConfig(_projectorConfig);

	_arSlideSub = _nh.subscribe("ar_slide", 10, &ARWindowNode::arSlideCallback, this);
}

tum::ARWindowNode::~ARWindowNode() {
}

void tum::ARWindowNode::run() {
	ros::Rate rate(60);

	QPixmap slide;
	while(ros::ok() && _window.isVisible()) {
		// process events and messages
		if (_qa.hasPendingEvents()) {
			_qa.processEvents();
		}
		ros::spinOnce();

		// render new image
		QRect canvas = _window.canvasArea();
		slide = _renderer.renderSlide(_slide, canvas);
		_window.display(slide);

		// other stuff
		_projector.publishViewFrustumMarker(_projector.getImagePlane(2.05f));
		rate.sleep();
	}
}

void tum::ARWindowNode::arSlideCallback(const tum_ar_msgs::ARSlide::ConstPtr& msg) {
	ROS_DEBUG_STREAM("[ARWindowNode] Received new slide");

	_slide = *msg;

	if (!_hideButtons) {
		_window.clearButtons();
		_window.addButtons(_slide.outcomes);
	}
}
