//
// Created by arne on 28.10.19.
//

#ifndef SRC_TOUCHSCREENWINDOW_H
#define SRC_TOUCHSCREENWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <memory>
#include <ros/ros.h>
#include <tum_ar_msgs/ARSlide.h>

namespace Ui {
	class TouchscreenWindow;
}

namespace tum {
	class TouchscreenWindow : public QMainWindow {
		Q_OBJECT

		public:
			explicit TouchscreenWindow(QWidget *parent = 0);
			virtual ~TouchscreenWindow();

			void showInterface(const tum_ar_msgs::ARSlide& slide);
			void clearInterface();
			static QPushButton* createButtonCopy(const QPushButton* button);
			QPushButton* getButton(const tum_ar_msgs::Outcome& outcome);

		public slots:
			void pushButtonOutcomeClicked();

		protected:
			void addButtons(std::vector<tum_ar_msgs::Outcome> outcomeOptions);
			void clearButtons();

		private:
			std::unique_ptr<Ui::TouchscreenWindow> _ui;
			ros::NodeHandle _nh;
			ros::Publisher _userInputPub;

			QPushButton* _defaultButtonTemplate;
			QPushButton* _infoButtonTemplate;
			QPushButton* _successButtonTemplate;
			QPushButton* _warnButtonTemplate;
			QPushButton* _errorButtonTemplate;
	};
}


#endif //SRC_TOUCHSCREENWINDOW_H
