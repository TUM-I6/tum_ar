#include <tum_ar_window/Projector.h>
#include <tum_ar_window/Toolbox.h>

#define VIEW_FRUSTUM_TOPIC "ar_window/projection_frustum"

unsigned int projectorId = 0 ;

tum::Projector::Projector(ros::NodeHandle& nh)
: _nh(nh) {
	init(true, 1.0, "beamer_optical_frame") ;
}

tum::Projector::Projector(ros::NodeHandle& nh, const bool publishViewFrustum, const float viewFrustumLength, const std::string& projectorFrame)
: _nh(nh) {
	init(publishViewFrustum, viewFrustumLength, projectorFrame) ;
}

tum::Projector::Projector(const tum::Projector& other)
: _nh(other._nh) {
	init(other._publishViewFrustum, other._viewFrustumLength, other._projectorFrame) ;
}

tum::Projector& tum::Projector::operator=(const tum::Projector& other) {
	if(&other == this) {
		return *this;
	}
	_nh = other._nh ;
	init(other._publishViewFrustum, other._viewFrustumLength, other._projectorFrame) ;
	return *this;
}

tum::Projector::~Projector() {
}

void tum::Projector::init(const bool publishViewFrustum, const float viewFrustumLength, const std::string& projectorFrame) {
	_id = projectorId++ ;

	_publishViewFrustum = publishViewFrustum ;
	_viewFrustumLength  = viewFrustumLength ;
	_projectorFrame     = projectorFrame ;

	_resolution = Eigen::Vector2i(1920,1200) ;
	_k << 2700,    0, 1920/2,
	         0, 2700, 1200/2,
	         0,    0,      1;

	_viewFrustumPub = _nh.advertise<visualization_msgs::Marker>(VIEW_FRUSTUM_TOPIC, 1) ;
}

Eigen::Matrix<float, 3, 4> tum::Projector::getImagePlane(const float dist) {
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
	marker.header.frame_id = _projectorFrame ;
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
