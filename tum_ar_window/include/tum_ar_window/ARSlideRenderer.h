#ifndef ARSLIDERENDERER_H
#define ARSLIDERENDERER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tum_ar_window/ARSlide.h>
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

			QPixmap renderSlide(const tum_ar_window::ARSlide&) ;
			QPixmap renderSlide(const tum_ar_window::ARSlide&, const QRect& area) ;

			static void drawBackground(QPainter& painter, const std::string& instruction="") ;

			static void drawCornerBox(QPainter& painter, const QPoint& position, const QSize& boxSize, const QColor& color=Qt::white, const int edgeLenght=50, const int lineWidth=6) ;
			static void drawLogos(QPainter& painter, const QPoint& position, const int logoHeight) ;
			static void drawPoi(QPainter& painter, const QPoint& position, const float radius, const std::string& label="", const QColor& border=Qt::red, const QColor& fill=Qt::white) ;
			static void drawArea(QPainter& painter, const QPoint& position, const QSize& size, const std::string& label="", const QColor& border=Qt::blue, const QColor& fill=Qt::transparent) ;

			static void drawBoxLabel(QPainter& painter, const QPoint& boxPosition, const QSize& boxSize, const std::string& label, const int offset, const QColor& color) ;
			static void drawCircleLabel(QPainter& painter, const QPoint& cirlePosition, const float radius, const std::string& label, const int offset, const QColor& color) ;

			const static QFont _font ;
			const static QIcon _tumLogo ;
			const static QIcon _tumI6Logo ;
			const static QIcon _horseLogo ;
			
		private:
			const Projector& _projector ;
			tf::TransformListener _tfListener ;


	} ;
} ;

#endif