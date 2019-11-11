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