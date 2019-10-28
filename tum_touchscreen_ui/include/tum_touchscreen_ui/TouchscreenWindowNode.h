//
// Created by arne on 28.10.19.
//

#ifndef SRC_TOUCHSCREENWINDOWNODE_H
#define SRC_TOUCHSCREENWINDOWNODE_H

#include <ros/ros.h>
#include <string.h>
#include <QApplication>
#include <tum_ar_msgs/ARSlide.h>
#include <tum_touchscreen_ui/TouchscreenWindow.h>

namespace tum {
	class TouchscreenWindowNode {
		public:
			TouchscreenWindowNode(QApplication& qa);
			virtual ~TouchscreenWindowNode();

			void run();
			void arSlideCallback(const tum_ar_msgs::ARSlide::ConstPtr& msg);

		private:
			ros::NodeHandle _nh;
			ros::Subscriber _arSlideSub;
			QApplication& _qa;
			TouchscreenWindow _window;
	};
}

#endif //SRC_TOUCHSCREENWINDOWNODE_H
