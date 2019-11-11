/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Arne Peters - arne.peters@tum.de
 * Technical University of Munich
 * Chair of Robotics, Artificial Intelligence and Real-time Systems
 * Department of Informatics / I6, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * https://www6.in.tum.de
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

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