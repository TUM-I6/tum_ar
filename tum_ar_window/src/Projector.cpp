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

#include <tum_ar_window/Projector.h>
#include <tum_ar_window/Toolbox.h>

#define VIEW_FRUSTUM_TOPIC "ar_window/projection_frustum"

unsigned int projectorId = 0 ;

tum::Projector::Projector(ros::NodeHandle& nh, const Config& config, const bool publishViewFrustum, const float viewFrustumLength)
: _nh(nh) {
	init(publishViewFrustum, viewFrustumLength, config) ;
}

tum::Projector::Projector(const tum::Projector& other)
: _nh(other._nh) {
	init(other._publishViewFrustum, other._viewFrustumLength, other._config) ;
}

tum::Projector& tum::Projector::operator=(const tum::Projector& other) {
	if(&other == this) {
		return *this;
	}
	_nh = other._nh ;
	init(other._publishViewFrustum, other._viewFrustumLength, other._config) ;
	return *this;
}

tum::Projector::~Projector() {
}

void tum::Projector::init(const bool publishViewFrustum, const float viewFrustumLength, const Config& config) {
	_id = projectorId++ ;

	_publishViewFrustum = publishViewFrustum ;
	_viewFrustumLength  = viewFrustumLength ;
	_config = config;

	_viewFrustumPub = _nh.advertise<visualization_msgs::Marker>(VIEW_FRUSTUM_TOPIC, 1) ;
}

Eigen::Vector2f tum::Projector::projectToPixel(const Eigen::Vector3f& point) const {
	Eigen::Vector3f pixel = _config.k*point ;
	//ROS_INFO_STREAM("pixel: "<<pixel.transpose()) ;
	return Eigen::Vector2f(pixel.x()/pixel.z(), pixel.y()/pixel.z()) ;
}

Eigen::Matrix<float, 3, 4> tum::Projector::getImagePlane(const float dist) const {
	const Eigen::Vector2f f = getFocalWidth() ;
	const Eigen::Vector2i r = getResolution() ;

	/* Calculate view frustum:
	 * The view frustum is visualised as a pyramid with o
	 * as it's top point
	 *
	 *   2 ----------------------- 3
	 *   |                         |
	 *   |                         |
	 *   |                         |
	 *   |            o            |
	 *   |                         |
	 *   |                         |
	 *   |                         |
	 *   0 ----------------------- 1
	 *
	 * z is pointing into the scene
	 */

	Eigen::Matrix<float, 3, 4> plane ;
	plane.col(0) = Eigen::Vector3f(-(dist*r.x())/(2.f*f.x()), -(dist*r.y())/(2.f*f.y()), dist) ;
	plane.col(1) = Eigen::Vector3f(-plane(0,0),  plane(1,0), plane(2,0)) ;
	plane.col(2) = Eigen::Vector3f( plane(0,0), -plane(1,0), plane(2,0)) ;
	plane.col(3) = Eigen::Vector3f(-plane(0,0), -plane(1,0), plane(2,0)) ;
	return plane ;
}

void tum::Projector::publishViewFrustumMarker(const Eigen::Matrix<float, 3, 4>& p) {
	visualization_msgs::Marker marker ;
	marker.header.frame_id = _config.projectorFrame ;
	marker.header.stamp = ros::Time::now() ;
	marker.header.seq = 0 ;
	marker.type = marker.LINE_LIST ;
	marker.action = visualization_msgs::Marker::ADD;
	marker.ns = "view_frustum" ;
	marker.id = _id ; // todo: change to allow multiple instances of Camera class
	marker.lifetime = ros::Duration(1.0) ;
	marker.pose.position.x = 0 ;
	marker.pose.position.y = 0 ;
	marker.pose.position.z = 0 ;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.02 ; // line width
	marker.scale.y = 1.0 ;
	marker.scale.z = 1.0 ;

	marker.points.resize(8*2) ; // 2 points per line
	marker.colors.resize(8*2) ;

	Eigen::Vector4f color(1.f, 1.f, 1.f, 1.f) ;
	Eigen::Vector3f origin(0,0,0) ;

	Toolbox::addPoint( 0,   origin, color, marker) ;
	Toolbox::addPoint( 1, p.col(0), color, marker) ;

	Toolbox::addPoint( 2,   origin, color, marker) ;
	Toolbox::addPoint( 3, p.col(1), color, marker) ;

	Toolbox::addPoint( 4,   origin, color, marker) ;
	Toolbox::addPoint( 5, p.col(2), color, marker) ;

	Toolbox::addPoint( 6,   origin, color, marker) ;
	Toolbox::addPoint( 7, p.col(3), color, marker) ;

	Toolbox::addPoint( 8, p.col(0), color, marker) ;
	Toolbox::addPoint( 9, p.col(1), color, marker) ;

	Toolbox::addPoint(10, p.col(1), color, marker) ;
	Toolbox::addPoint(11, p.col(3), color, marker) ;

	Toolbox::addPoint(12, p.col(2), color, marker) ;
	Toolbox::addPoint(13, p.col(3), color, marker) ;

	Toolbox::addPoint(14, p.col(0), color, marker) ;
	Toolbox::addPoint(15, p.col(2), color, marker) ;

	_viewFrustumPub.publish(marker) ;
}
