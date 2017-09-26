#ifndef PROJECTOR_H
#define PROJECTOR_H

#include <ros/ros.h>
#include <string.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

namespace tum {
	class Projector {
		public:
			Projector(ros::NodeHandle& nh) ;
			Projector(ros::NodeHandle& nh, const bool publishViewFrustum, const float viewFrustumLenght, const std::string& beamerFrame = "beamer_optical_frame") ;
			Projector(const Projector& other) ;
			Projector& operator=(const Projector& other) ;

			virtual ~Projector() ;

			Eigen::Vector2f getFocalWidth() const {
				return Eigen::Vector2f(_k(0,0), _k(1,1)) ; // focal width
			}

			Eigen::Vector2i getResolution() const {
				return _resolution ;
			}

			std::string getFrame() const {
				return _projectorFrame ;
			}

			Eigen::Matrix3f getCameraMatrix() const {
				return _k ;
			}

			Eigen::Vector2f projectToPixel(const Eigen::Vector3f& point) const ;

			Eigen::Matrix<float, 3, 4> getImagePlane(const float dist) const ;
			void publishViewFrustumMarker(const Eigen::Matrix<float, 3, 4>& plane) ;

		private:
			void init(const bool publishViewFrustum, const float viewFrustumLenght, const std::string& projectorFrame) ;

			ros::NodeHandle& _nh ;
			ros::Publisher _viewFrustumPub ;

			std::string _projectorFrame ;
			
			bool _publishViewFrustum ;
			float _viewFrustumLength ;

			Eigen::Matrix3f _k ;
			Eigen::Vector2i _resolution ;
			
			unsigned int _id ;
	} ;
}

#endif