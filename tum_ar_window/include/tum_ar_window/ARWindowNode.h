//
// Created by arne on 28.10.19.
//

#ifndef AR_SERVER_NODE_H
#define AR_SERVER_NODE_H

#include <ros/ros.h>
#include <string.h>
#include <QApplication>
#include <tum_ar_msgs/ARSlide.h>
#include <tum_ar_window/ARSlideRenderer.h>
#include <tum_ar_window/ARWindow.h>
#include <tum_ar_msgs/Outcome.h>
#include <tum_ar_window/Projector.h>

namespace tum {
	class ARWindowNode {
	public:
		ARWindowNode(QApplication& qa);
		virtual ~ARWindowNode();

		void run();
		void arSlideCallback(const tum_ar_msgs::ARSlide::ConstPtr& msg);

	private:
		ros::NodeHandle _nh;
		ros::Subscriber _arSlideSub;
		QApplication& _qa;
		ARWindow _window;
		Projector _projector;
		ARSlideRenderer _renderer;
		bool _hideButtons;

		std::string _taskDescriptionFile;

		tum_ar_msgs::ARSlide _slide;
	};
};

#endif //AR_SERVER_NODE_H
