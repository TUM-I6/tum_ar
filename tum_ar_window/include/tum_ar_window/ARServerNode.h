#ifndef AR_SERVER_NODE_H
#define AR_SERVER_NODE_H

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <string.h>
#include <tum_ar_msgs/ARTaskAction.h>
#include <tum_ar_msgs/LoadTaskFromFile.h>

namespace tum {
	class ARServerNode {
		public:
			ARServerNode();
			virtual ~ARServerNode();

			void run();
			void executeARTask();
			void userInputCallback(const tum_ar_msgs::Outcome::ConstPtr& msg);
			bool loadTaskFromFileServiceCB(tum_ar_msgs::LoadTaskFromFile::Request& request, tum_ar_msgs::LoadTaskFromFile::Response& response);

		private:
			void publishFeedback(unsigned int slide, const tum_ar_msgs::Outcome& outcome) {
				tum_ar_msgs::ARTaskFeedback feedback;
				feedback.slide = slide;
				feedback.slides = _slides.size();
				feedback.outcome = outcome;
				_actionServer.publishFeedback(feedback);
			}

			ros::NodeHandle _nh;
			ros::Subscriber _userInputSub;
			ros::Publisher _arSlidePub;
			ros::ServiceServer _loadTaskFromFileServer;
			actionlib::SimpleActionServer<tum_ar_msgs::ARTaskAction> _actionServer;

			std::string _taskDescriptionFile;

			std::vector<tum_ar_msgs::ARSlide> _slides;
			std::vector<tum_ar_msgs::Outcome> _outcomes;
			tum_ar_msgs::ARSlide _blankSlide;
			int _step = 0;
			bool _taskActive = false;
	};
};

#endif
