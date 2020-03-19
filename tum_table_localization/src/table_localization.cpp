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

#include <tum_table_localization/table_localization.h>
#include <Eigen/Geometry> 

#define CAMERA_FRAME_PARAM "camera_frame"
#define TABLE_FRAME_PARAM "table_frame"

#define DEFAULT_CAMERA_FRAME "camera"
#define DEFAULT_TABLE_FRAME "table_center"
#define DEFAULT_TAG_DETECTION_TOPIC "/tag_detections_pose"

#define TOP_LEFT_CORNER_FRAME_PARAM "top_left_corner_frame"
#define TOP_RIGHT_CORNER_FRAME_PARAM "top_right_corner_frame"
#define BOTTOM_LEFT_CORNER_FRAME_PARAM "bottom_left_corner_frame"
#define BOTTOM_RIGHT_CORNER_FRAME_PARAM "bottom_right_corner_frame"

#define DEFAULT_TOP_LEFT_CORNER_FRAME "table_corner_top_left"
#define DEFAULT_TOP_RIGHT_CORNER_FRAME "table_corner_top_right"
#define DEFAULT_BOTTOM_LEFT_CORNER_FRAME "table_corner_bottom_left"
#define DEFAULT_BOTTOM_RIGHT_CORNER_FRAME "table_corner_bottom_right"

#define BUFFER_SIZE 100

TableLocalization::TableLocalization()
: _topLeftCornerPositions(BUFFER_SIZE),
  _topRightCornerPositions(BUFFER_SIZE),
  _bottomLeftCornerPositions(BUFFER_SIZE),
  _bottomRightCornerPositions(BUFFER_SIZE) {
	_nh.param<std::string>(CAMERA_FRAME_PARAM, _cameraFrame, DEFAULT_CAMERA_FRAME) ;
	ROS_INFO_STREAM("[tum_table_localization] "<<CAMERA_FRAME_PARAM<<": "<<_cameraFrame) ;
	_nh.param<std::string>(TABLE_FRAME_PARAM, _tableFrame, DEFAULT_TABLE_FRAME) ;
	ROS_INFO_STREAM("[tum_table_localization] "<<TABLE_FRAME_PARAM<<": "<<_tableFrame) ;

	_nh.param<std::string>(TOP_LEFT_CORNER_FRAME_PARAM, _topLeftCornerFrame, DEFAULT_TOP_LEFT_CORNER_FRAME) ;
	ROS_INFO_STREAM("[tum_table_localization] "<<TOP_LEFT_CORNER_FRAME_PARAM<<": "<<_topLeftCornerFrame) ;
	_nh.param<std::string>(TOP_RIGHT_CORNER_FRAME_PARAM, _topRightCornerFrame, DEFAULT_TOP_RIGHT_CORNER_FRAME) ;
	ROS_INFO_STREAM("[tum_table_localization] "<<TOP_RIGHT_CORNER_FRAME_PARAM<<": "<<_topRightCornerFrame) ;
	_nh.param<std::string>(BOTTOM_LEFT_CORNER_FRAME_PARAM, _bottomLeftCornerFrame, DEFAULT_BOTTOM_LEFT_CORNER_FRAME) ;
	ROS_INFO_STREAM("[tum_table_localization] "<<BOTTOM_LEFT_CORNER_FRAME_PARAM<<": "<<_bottomLeftCornerFrame) ;
	_nh.param<std::string>(BOTTOM_RIGHT_CORNER_FRAME_PARAM, _bottomRightCornerFrame, DEFAULT_BOTTOM_RIGHT_CORNER_FRAME) ;
	ROS_INFO_STREAM("[tum_table_localization] "<<BOTTOM_RIGHT_CORNER_FRAME_PARAM<<": "<<_bottomRightCornerFrame) ;

	//_nh.param<std::string>("tag_detections_topic", _tagDetectionsTopic, DEFAULT_TAG_DETECTION_TOPIC) ;
	//_tagDetectionsSubscriber = _nh.subscribe(_tagDetectionsTopic, 1, &TableLocalization::tagDetectionsCallback, this) ;
}

void TableLocalization::tagDetectionsCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
	ROS_INFO_STREAM("[tum_table_localization] Received "<<msg->poses.size()<<" marker poses") ;
}

void TableLocalization::run() {
	ros::Rate rate(30) ;
	while(ros::ok()) {
		rate.sleep() ;
		//ros::spin_once() ;

		updateTransformation(_topLeftCornerFrame, _topLeftCornerPositions) ;
		updateTransformation(_topRightCornerFrame, _topRightCornerPositions) ;
		updateTransformation(_bottomLeftCornerFrame, _bottomLeftCornerPositions) ;
		updateTransformation(_bottomRightCornerFrame, _bottomRightCornerPositions) ;

		if ((_topLeftCornerPositions.size() <= 0) || (_topRightCornerPositions.size() <= 0) || (_bottomLeftCornerPositions.size() <= 0) || (_bottomRightCornerPositions.size() <= 0)) {
			ROS_WARN_STREAM_THROTTLE(2, "[tum_table_localization] Waiting for first marker detection") ;
			ROS_WARN_STREAM_THROTTLE(2, "[tum_table_localization] "<<_topLeftCornerFrame<<": "<<((_topLeftCornerPositions.size()<=0)?"missing":"found")) ;
			ROS_WARN_STREAM_THROTTLE(2, "[tum_table_localization] "<<_topRightCornerFrame<<": "<<((_topRightCornerPositions.size()<=0)?"missing":"found")) ;
			ROS_WARN_STREAM_THROTTLE(2, "[tum_table_localization] "<<_bottomLeftCornerFrame<<": "<<((_bottomLeftCornerPositions.size()<=0)?"missing":"found")) ;
			ROS_WARN_STREAM_THROTTLE(2, "[tum_table_localization] "<<_bottomRightCornerFrame<<": "<<((_bottomRightCornerPositions.size()<=0)?"missing":"found")) ;
			continue ;
		}

		/* We take the average position of every tag and
		 * then look for the base of the table frame
		 * 
		 *   a ----------------------- b
		 *   |                         |
		 *   |                         |
		 *   |                         |
		 *   |            t --> y      |
		 *   |            |            |
		 *   |            V x          |
		 *   |                         |
		 *   c ----------------------- d
		 *
		 * z is pointing upwards
		 */

		Eigen::Vector3f a = getAveragePosition(_topLeftCornerPositions) ;
		Eigen::Vector3f b = getAveragePosition(_topRightCornerPositions) ;
		Eigen::Vector3f c = getAveragePosition(_bottomLeftCornerPositions) ;
		Eigen::Vector3f d = getAveragePosition(_bottomRightCornerPositions) ;

		Eigen::Vector3f t = (a+b+c+d)/4.0f ;
		Eigen::Vector3f x = ((c-a)+(d-b))/2.0f ;
		Eigen::Vector3f y = ((b-a)+(d-c))/2.0f ;

		x.normalize() ;
		y.normalize() ;

		Eigen:: Vector3f z = x.cross(y) ;

		z.normalize() ;

		// ROS_INFO_STREAM("[tum_table_localization] y: ("<<y.x()<<", "<<y.y()<<", "<<y.z()<<")") ;

		y = z.cross(x) ;

		//ROS_INFO_STREAM("[tum_table_localization] t: ("<<t.x()<<", "<<t.y()<<", "<<t.z()<<")") ;
		//ROS_INFO_STREAM("[tum_table_localization] Base:\nx: ("<<x.x()<<", "<<x.y()<<", "<<x.z()<<"),\ny: ("<<y.x()<<", "<<y.y()<<", "<<y.z()<<"),\nz: ("<<z.x()<<", "<<z.y()<<", "<<z.z()<<")") ;

		tf::Matrix3x3 r ;
		r.setValue(x.x(), x.y(), x.z(),
		           y.x(), y.y(), y.z(),
		           z.x(), z.y(), z.z()) ;

		tf::Transform transform;
		transform.setOrigin(r*tf::Vector3(-t.x(), -t.y(), -t.z())) ;
		transform.setBasis(r) ;

		_tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _tableFrame, _cameraFrame)) ;
	}
}

void TableLocalization::updateTransformation(const std::string& cornerFrame, boost::circular_buffer<Eigen::Vector3f>& buffer) {
	try{
		tf::StampedTransform transform;
		_tfListener.lookupTransform(_cameraFrame, cornerFrame, ros::Time(0), transform) ;
		tf::Vector3 cornerPositionTf = transform.getOrigin() ;
		Eigen::Vector3f cornerPositionEigen(cornerPositionTf.getX(), cornerPositionTf.getY(), cornerPositionTf.getZ()) ;
		buffer.push_back(cornerPositionEigen) ;
	}
	catch (tf::TransformException ex){
		ROS_ERROR_STREAM_THROTTLE(1, "[tum_table_localization] TF-Error while requesting transformation from \""<<_cameraFrame<<"\" to \""<<_topLeftCornerFrame<<"\": "<<ex.what()) ;
	}
}

Eigen::Vector3f TableLocalization::getAveragePosition(const boost::circular_buffer<Eigen::Vector3f>& buffer) {
	Eigen::Vector3f result = Eigen::Vector3f::Zero() ;

	for (const Eigen::Vector3f& vec : buffer) {
		result += vec ;
	}

	result /= (float)buffer.size() ;
	return result ;
}