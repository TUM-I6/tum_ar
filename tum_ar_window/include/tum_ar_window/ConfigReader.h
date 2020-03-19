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

#ifndef CONFIGREADER_H
#define CONFIGREADER_H

#include <ros/ros.h>
#include <tum_ar_msgs/ARSlide.h>
#include <tum_ar_window/Projector.h>
#include <Eigen/Dense>

namespace YAML {
	class Node;
}

namespace tum {
	class ConfigReader {
		public:
			static std::vector<tum_ar_msgs::ARSlide> readARTaskDescription(const std::string& fileName);
			static Projector::Config readProjectorConfig(const std::string& fileName);

			static std::string preparePath(const std::string& path);

			static const std_msgs::ColorRGBA BOX_DEFAULT_BORDER_COLOR;
			static const std_msgs::ColorRGBA BOX_DEFAULT_FILL_COLOR;
			static const std_msgs::ColorRGBA POI_DEFAULT_BORDER_COLOR;
			static const std_msgs::ColorRGBA POI_DEFAULT_FILL_COLOR;

			static const std::string DEFAULT_INSTRUCTION;
			static const std::string BOX_DEFAULT_LABEL;
			static const std::string POI_DEFAULT_LABEL;

			static const ros::Time HEADER_DEFAULT_TIME;
			static const int HEADER_DEFAULT_SEQ;

		private:
			static tum_ar_msgs::ARSlide readSlide(const YAML::Node& node);
			static tum_ar_msgs::POI readPOI(const YAML::Node& node);
			static tum_ar_msgs::Box readBox(const YAML::Node& node);
			static tum_ar_msgs::Outcome readOutcome(const YAML::Node& node);
			static std_msgs::ColorRGBA readColor(const YAML::Node& node);
			static geometry_msgs::Point readPoint(const YAML::Node& node);
			
			static Eigen::Matrix3f readProjectionMatrix(const YAML::Node& node);
			static Eigen::Vector2i readResolutionVector(const YAML::Node& node);
	};
};

#endif