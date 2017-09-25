#ifndef ARINSPECTIONNODE_H
#define ARINSPECTIONNODE_H

#include <ros/ros.h>
#include <string.h>
#include <QApplication>
#include <tum_ar_window/ARSlide.h>
#include <tum_ar_window/ARSlideRenderer.h>
#include <tum_ar_window/ARWindow.h>
#include <tum_ar_window/InspectionResult.h>
#include <tum_ar_window/Projector.h>

namespace tum {
	class ARInspectionNode {
		public:
			ARInspectionNode(int argc, char *argv[]) ;
			virtual ~ARInspectionNode() ;

			void run() ;
			void executeARInspection() ;
			void userInputCallback(const tum_ar_window::InspectionResult::ConstPtr& msg) ;
			static std::vector<tum_ar_window::ARSlide> loadSlides(const std::string& file) ;

		private:
			void publishFeedback(unsigned int slide, unsigned char status) {
				tum_ar_window::ARInspectionFeedback feedback ;
				feedback.poi = slide ;
				feedback.status = status ;
				_actionServer.publishFeedback(feedback) ;
			}

			ros::NodeHandle _nh ;
			ros::Subscriber _userInputSub ;
			actionlib::SimpleActionServer<tum_ar_window::ARInspectionAction> _actionServer ;
			QApplication _qApp ;
			ARWindow _window ;
			Projector _projector ;
			ARSlideRenderer _renderer ;

			std::vector<tum_ar_window::ARSlide> _slides ;
			tum_ar_window::ARSlide _blankSlide ;
			int _step = 0 ;
			bool _taskActive = true ;
	} ;
} ;

#endif