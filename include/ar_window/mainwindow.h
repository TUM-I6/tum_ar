#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QMessageBox>
#include <ros/ros.h>
#include <ros/package.h>
#include <ar_window/ARInspectionAction.h>
#include <actionlib/server/simple_action_server.h>
#include <vector>

namespace Ui {
	class MainWindow;
}

class MainWindow : public QMainWindow {
	Q_OBJECT

	public:
		explicit MainWindow(QWidget *parent = 0);
		~MainWindow();
		void displayImage(const std::string& url) ;
		void executeARInspection() ; // const ar_window::ARInspectionGoalConstPtr &goal) ;

	public slots:
		void pushButtonAcceptClicked() ;
		void pushButtonRejectClicked() ;

	private:
		Ui::MainWindow *_ui;
		ros::NodeHandle _nh ;
		actionlib::SimpleActionServer<ar_window::ARInspectionAction> _actionServer ;

		std::string _wsaType ;
		std::vector<unsigned int> _pois ;
		int _pos ;
		int _defaultSize ;

		void publishFeedback(unsigned int poi, unsigned char status) {
			ar_window::ARInspectionFeedback feedback ;
			feedback.poi = poi ;
			feedback.status = status ;
			_actionServer.publishFeedback(feedback) ;
		}

		static std::string getImageURL(const std::string& wsaType, const unsigned int poi) {
			std::stringstream stream ;
			stream << ros::package::getPath("ar_window")<<"/images/"<<wsaType<<"/poi_"<<((poi<9)?"0":"")<<poi<<".png" ;
			ROS_DEBUG_STREAM("[ar_window] File-path: "<<stream.str()) ;
			return stream.str() ;
		}

		static std::string getImageURL() {
			std::stringstream stream ;
			stream << ros::package::getPath("ar_window")<<"/images/blank.png" ;
			return stream.str() ;
		}
};

#endif // MAINWINDOW_H
