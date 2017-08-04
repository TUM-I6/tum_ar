#include <tum_table_localization/table_localization.h>

#define DEFAULT_CAMERA_FRAME "camera"
#define DEFAULT_TABLE_FRAME "table"
#define DEFAULT_TAG_DETECTION_TOPIC "/tag_detections_pose"

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
	_nh.param<std::string>("camera_frame", _cameraFrame, DEFAULT_CAMERA_FRAME) ;
	_nh.param<std::string>("table_frame", _tableFrame, DEFAULT_TABLE_FRAME) ;
	_nh.param<std::string>("tag_detections_topic", _tagDetectionsTopic, DEFAULT_TAG_DETECTION_TOPIC) ;

	_nh.param<std::string>("top_left_corner_frame", _topLeftCornerFrame, DEFAULT_TOP_LEFT_CORNER_FRAME) ;
	_nh.param<std::string>("top_right_corner_frame", _topRightCornerFrame, DEFAULT_TOP_RIGHT_CORNER_FRAME) ;
	_nh.param<std::string>("bottom_left_corner_frame", _bottomLeftCornerFrame, DEFAULT_BOTTOM_LEFT_CORNER_FRAME) ;
	_nh.param<std::string>("bottom_right_corner_frame", _bottomRightCornerFrame, DEFAULT_BOTTOM_RIGHT_CORNER_FRAME) ;

	_tagDetectionsSubscriber = _nh.subscribe(_tagDetectionsTopic, 1, &TableLocalization::tagDetectionsCallback, this) ;
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

		if ((_topLeftCornerPositions.size() == 0) || (_topRightCornerPositions.size() == 0) || (_bottomLeftCornerPositions.size() == 0) || (_bottomRightCornerPositions.size() == 0)) {
			ROS_ERROR_STREAM_THROTTLE(1, "[tum_table_localization] Waiting for first marker detection") ;
			continue ;
		}

		/* We take the average position of every tag and
		 * then look for the base of the table frame
		 * 
		 *   a ----------------------- b
		 *   |                         |
		 *   |            Λ x          |
		 *   |            |            |
		 *   |     y  <-- t            |
		 *   |                         |
		 *   |                         |
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

		Eigen:: Vector3f z = x.cross(y) ;
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
		ROS_ERROR_STREAM_THROTTLE(1, "[tum_table_localization] TF-Error while requesting transformation from "<<_cameraFrame<<" to "<<_topLeftCornerFrame<<":"<<ex.what()) ;
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