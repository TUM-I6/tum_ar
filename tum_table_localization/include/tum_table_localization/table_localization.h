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

#ifndef TABLE_LOCALIZATION_H
#define TABLE_LOCALIZATION_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <string.h>
#include <boost/circular_buffer.hpp>
#include <Eigen/Dense>

class TableLocalization {
	public:
		TableLocalization() ;
		void tagDetectionsCallback(const geometry_msgs::PoseArray::ConstPtr& msg) ;
		void run() ;
		std::string _cameraFrame ;
		std::string _tableFrame ;


	private:
		void updateTransformation(const std::string& cornerFrame, boost::circular_buffer<Eigen::Vector3f>& buffer) ;
		static Eigen::Vector3f getAveragePosition(const boost::circular_buffer<Eigen::Vector3f>& buffer) ;

		ros::NodeHandle _nh ;
		ros::Subscriber _tagDetectionsSubscriber ;
		tf::TransformListener _tfListener ;
		tf::TransformBroadcaster _tfBroadcaster ;

		std::string _tagDetectionsTopic ;

		std::string _topLeftCornerFrame ;
		std::string _topRightCornerFrame ;
		std::string _bottomLeftCornerFrame ;
		std::string _bottomRightCornerFrame ;

		boost::circular_buffer<Eigen::Vector3f> _topLeftCornerPositions ;
		boost::circular_buffer<Eigen::Vector3f> _topRightCornerPositions ;
		boost::circular_buffer<Eigen::Vector3f> _bottomLeftCornerPositions ;
		boost::circular_buffer<Eigen::Vector3f> _bottomRightCornerPositions ;
};

#endif