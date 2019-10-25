#ifndef ARINSPECTIONNODE_H
#define ARINSPECTIONNODE_H

#include <ros/ros.h>
#include <string.h>
#include <QApplication>
#include <tum_ar_msgs/ARSlide.h>
#include <tum_ar_window/ARSlideRenderer.h>
#include <tum_ar_window/ARWindow.h>
#include <tum_ar_msgs/Outcome.h>
#include <tum_ar_window/Projector.h>

namespace tum {
	class ARTaskNode {
		public:
			ARTaskNode(QApplication& qa);
			virtual ~ARTaskNode();

			void run();
			void executeARTask();
			void userInputCallback(const tum_ar_msgs::Outcome::ConstPtr& msg);

			//static std::vector<tum_ar_msgs::ARSlide> loadSlides(const std::string& file) ;

		private:
			void publishFeedback(unsigned int slide, const tum_ar_msgs::Outcome& outcome) {
				tum_ar_msgs::ARTaskFeedback feedback;
				feedback.slide = slide;
				feedback.slides = _slides.size();
				feedback.outcome = outcome;
				_actionServer.publishFeedback(feedback);
			}

			ros::NodeHandle _nh;
			ros::Subscriber _userInputSub ;
			actionlib::SimpleActionServer<tum_ar_msgs::ARTaskAction> _actionServer;
			//QCoreApplication* _qApp;
			QApplication& _qa;
			ARWindow _window;
			Projector _projector;
			ARSlideRenderer _renderer;
			bool _hideButtons;

			std::string _taskDescriptionFile;

			std::vector<tum_ar_msgs::ARSlide> _slides;
			std::vector<tum_ar_msgs::Outcome> _outcomes;
			tum_ar_msgs::ARSlide _blankSlide;
			int _step = 0;
			bool _taskActive = false;
	};
};

#endif
