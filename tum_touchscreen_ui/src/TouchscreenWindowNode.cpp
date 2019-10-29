//
// Created by arne on 28.10.19.
//

#include <tum_touchscreen_ui/TouchscreenWindowNode.h>

tum::TouchscreenWindowNode::TouchscreenWindowNode(QApplication& qa)
: _qa(qa) {
	_window.showMaximized();
	_arSlideSub = _nh.subscribe("ar_slide", 10, &TouchscreenWindowNode::arSlideCallback, this);

}

tum::TouchscreenWindowNode::~TouchscreenWindowNode() {
	_window.close();
}

void tum::TouchscreenWindowNode::run() {
	ros::Rate rate(30);

	while(ros::ok() && _window.isVisible()) {
		// process events and messages
		if (_qa.hasPendingEvents()) {
			_qa.processEvents();
		}
		ros::spinOnce();
		rate.sleep();
	}
}

void tum::TouchscreenWindowNode::arSlideCallback(const tum_ar_msgs::ARSlide::ConstPtr& msg) {
	if (msg->instruction.size() == 0 && msg->outcomes.size() == 0) {
		_window.clearInterface();
	}
	else {
		_window.showInterface(*msg);
	}
}