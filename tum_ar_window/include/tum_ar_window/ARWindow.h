#ifndef AR_WINDOW_H
#define AR_WINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QMessageBox>
#include <QGraphicsScene>
#include <ros/ros.h>
#include <ros/package.h>
#include <tum_ar_msgs/ARTaskAction.h>
#include <tum_ar_window/ARSlideRenderer.h>
#include <tum_ar_window/Projector.h>
#include <tum_ar_msgs/Outcome.h>
#include <actionlib/server/simple_action_server.h>
#include <vector>
#include <memory>

namespace Ui {
	class ARWindow;
}

namespace tum {
	class ARWindow : public QMainWindow {
		Q_OBJECT

		public:
			explicit ARWindow(QWidget *parent = 0);
			~ARWindow();

			void display(const std::string& url);
			void display(const QPixmap& pixmap);

			void addButtons(std::vector<tum_ar_msgs::Outcome> outcomeOptions);
			void clearButtons();

			int canvasWidth() const;
			int canvasHeight() const;
			QSize canvasSize() const;
			QPoint canvasPosition() const;
			QRect canvasArea() const {
				return QRect(canvasPosition(), canvasSize());
			}

		public slots:
			void pushButtonOutcomeClicked();

		private:
			ros::NodeHandle _nh;
			ros::Publisher _userInputPub;
			std::unique_ptr<Ui::ARWindow> _ui;
			std::unique_ptr<QGraphicsScene> _scene;
	};
}

#endif // AR_WINDOW_H
