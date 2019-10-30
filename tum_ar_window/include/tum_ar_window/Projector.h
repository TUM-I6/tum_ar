#ifndef PROJECTOR_H
#define PROJECTOR_H

#include <ros/ros.h>
#include <string.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

namespace tum {
	class Projector {
		
		public:
			struct Config {
				Eigen::Matrix3f k;
				Eigen::Vector2i resolution;
				std::string projectorFrame;
			};

			Projector(ros::NodeHandle& nh);
			Projector(ros::NodeHandle& nh, const Config& projectorConfig, const bool publishViewFrustum=true, const float viewFrustumLength=1.0);
			Projector(const Projector& other);
			Projector& operator=(const Projector& other);

			virtual ~Projector();

			Eigen::Vector2f getFocalWidth() const {
				return Eigen::Vector2f(_config.k(0,0), _config.k(1,1)); // focal width
			}

			Eigen::Vector2i getResolution() const {
				return _config.resolution;
			}

			std::string getFrame() const {
				return _config.projectorFrame;
			}

			Eigen::Matrix3f getProjectionMatrix() const {
				return _config.k;
			}

			Eigen::Vector2f projectToPixel(const Eigen::Vector3f& point) const;

			Eigen::Matrix<float, 3, 4> getImagePlane(const float dist) const;
			void publishViewFrustumMarker(const Eigen::Matrix<float, 3, 4>& plane);

			void setConfig(const Config& config) {
				_config = config;
			}

			const Config& config() const {
				return _config;
			}

		private:
			void init(const bool publishViewFrustum, const float viewFrustumLenght, const Config& config);

			ros::NodeHandle& _nh;
			ros::Publisher _viewFrustumPub;

			Config _config;
			
			bool _publishViewFrustum;
			float _viewFrustumLength;
			
			unsigned int _id;
	};
}

#endif