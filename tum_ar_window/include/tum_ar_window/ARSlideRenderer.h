#ifndef ARSLIDERENDERER_H
#define ARSLIDERENDERER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tum_ar_msgs/ARSlide.h>
#include <tum_ar_window/Projector.h>
#include <QtCore>
#include <QtGui>
#include <string.h>
#include <Eigen/Dense>

namespace tum {
	class ARSlideRenderer {
		public:
			ARSlideRenderer(const Projector& projector) ;
			virtual ~ARSlideRenderer() ;

			QPixmap renderSlide(const tum_ar_msgs::ARSlide&) const ;
			QPixmap renderSlide(const tum_ar_msgs::ARSlide&, const QRect& area) const ;

			static void drawBackground(QPainter& painter) ;
			static void drawInstruction(QPainter& painter, const std::string& instruction) ;

			static void drawCornerBox(QPainter& painter, const QPoint& position, const QSize& boxSize, const QColor& color=Qt::white, const int edgeLenght=50, const int lineWidth=6) ;
			static void drawLogos(QPainter& painter, const QPoint& position, const int logoHeight) ;
			static void drawPoi(QPainter& painter, const QPoint& position, const float radius, const std::string& label="", const QColor& border=Qt::red, const QColor& fill=Qt::white) ;
			static void drawArea(QPainter& painter, const QPoint& position, const QSize& size, const std::string& label="", const QColor& border=Qt::blue, const QColor& fill=Qt::transparent) ;

			static void drawBoxLabel(QPainter& painter, const QPoint& boxPosition, const QSize& boxSize, const std::string& label, const int offset, const QColor& color) ;
			static void drawCircleLabel(QPainter& painter, const QPoint& cirlePosition, const float radius, const std::string& label, const int offset, const QColor& color) ;

			const static QFont INFO_FONT ;
			const static QFont INSTRUCTION_FONT ;
			
			const static QIcon TUM_LOGO ;
			const static QIcon TUM_I6_LOGO ;
			const static QIcon HORSE_LOGO ;

		protected:
			void renderPOI(QPainter& painter, const tum_ar_msgs::POI& poi, const QRect& canvasArea) const ;
			void renderBox(QPainter& painter, const tum_ar_msgs::Box& box, const QRect& canvasArea) const ;

			geometry_msgs::Point toProjectorFrame(const geometry_msgs::PointStamped& point) const ;
			QPointF projectToPixel(const geometry_msgs::Point& point) const ;
			
		private:
			const Projector& _projector ;
			tf::TransformListener _tfListener ;


	} ;
} ;

#endif