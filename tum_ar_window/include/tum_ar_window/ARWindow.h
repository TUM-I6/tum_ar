#ifndef AR_WINDOW_H
#define AR_WINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QMessageBox>
#include <ros/ros.h>
#include <ros/package.h>
#include <tum_ar_window/ARInspectionAction.h>
#include <actionlib/server/simple_action_server.h>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/core/eigen.hpp>

namespace Ui {
	class ARWindow ;
}

class ARWindow : public QMainWindow {
	Q_OBJECT

	public:
		explicit ARWindow(QWidget *parent = 0);
		~ARWindow();
		void draw() ;	
		void displayImage(const cv::Mat& image) ;
		void displayImage(const std::string& url) ;
		void executeARInspection() ;

		int canvasWidth() const ;
		int canvasHeight() const ;
		
		void drawCornerBox(QPainter& painter, const QPoint& position, const QSize& boxSize, const QColor& color=Qt::white, const int edgeLenght=50, const int lineWidth=6) const ;
		void drawLogos(QPainter& painter, const QPoint& position, const int logoHeight) const ;
		void drawPoi(QPainter& painter, const QPoint& position, const float radius, const std::string& label="", const QColor& border=Qt::red, const QColor& fill=Qt::white) const ;
		void drawArea(QPainter& painter, const QPoint& position, const QSize& size, const std::string& label="", const QColor& border=Qt::blue, const QColor& fill=Qt::transparent) const ;

		void drawBoxLabel(QPainter& painter, const QPoint& boxPosition, const QSize& boxSize, const std::string& label, const int offset, const QColor& color) const ;
		void drawCircleLabel(QPainter& painter, const QPoint& cirlePosition, const float radius, const std::string& label, const int offset, const QColor& color) const ;

		QPixmap getNewFrame(const std::string& title="") const ;

	public slots:
		void pushButtonAcceptClicked() ;
		void pushButtonRejectClicked() ;

	private:
		Ui::ARWindow *_ui;
		ros::NodeHandle _nh ;
		actionlib::SimpleActionServer<tum_ar_window::ARInspectionAction> _actionServer ;

		QIcon _tumLogo ;
		QIcon _tumI6Logo ;
		QIcon _horseLogo ;

		QFont _font ;

		std::string _wsaType ;
		std::vector<unsigned int> _pois ;
		int _pos ;
		int _defaultSize ;

		void publishFeedback(unsigned int poi, unsigned char status) {
			tum_ar_window::ARInspectionFeedback feedback ;
			feedback.poi = poi ;
			feedback.status = status ;
			_actionServer.publishFeedback(feedback) ;
		}

		static std::string getImageURL(const std::string& wsaType, const unsigned int poi) {
			std::stringstream stream ;
			stream << ros::package::getPath("tum_ar_window")<<"/images/"<<wsaType<<"/poi_"<<((poi<9)?"0":"")<<poi<<".png" ;
			ROS_DEBUG_STREAM("[tum_ar_window] File-path: "<<stream.str()) ;
			return stream.str() ;
		}

		static std::string getImageURL() {
			std::stringstream stream ;
			stream << ros::package::getPath("tum_ar_window")<<"/images/blank.png" ;
			return stream.str() ;
		}
};

#endif // AR_WINDOW_H
