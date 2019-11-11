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

#ifndef TOOLBOX_H
#define TOOLBOX_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <string.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <std_msgs/Header.h>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <iostream>
#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#define DEFAULT_SPIN_RATE 20

namespace tum {
	class Toolbox {
		public:
			static inline void spin(const ros::Duration& d, const unsigned int spinRate = DEFAULT_SPIN_RATE) {
				ros::Time start = ros::Time::now() ;
				ros::Rate rate(spinRate) ;
				while(ros::ok() && ros::Time::now() - start < d) {
					ros::spinOnce() ;
					rate.sleep() ;
				}
			}

			// spins until the selected marker is set to true
			static inline void spin(const bool& updateMarker, const unsigned int spinRate = DEFAULT_SPIN_RATE) {
				ros::Rate rate(spinRate) ;
				while(ros::ok() && !updateMarker) {
					ros::spinOnce() ;
					rate.sleep() ;
				}
			}

			static inline void addPoint(const unsigned int index, const Eigen::Vector3f position, const Eigen::Vector4f color, visualization_msgs::Marker& marker) {
				marker.points[index].x = position.x() ;
				marker.points[index].y = position.y() ;
				marker.points[index].z = position.z() ;
				//ROS_INFO_STREAM("[ArmToCameraCalibration] added point ("<<marker.points[index].x<<", "<<marker.points[index].y<<", "<<marker.points[index].z<<")") ;

				marker.colors[index].r = color(0) ;
				marker.colors[index].g = color(1) ;
				marker.colors[index].b = color(2) ;
				marker.colors[index].a = color(3) ;
			} ;

			static inline std::string getDateTimeString(const std::string& format = std::string("%Y-%m-%d %H-%M-%S")) {
				auto t = std::time(nullptr);
				auto tm = *std::localtime(&t);
				std::stringstream stream ;
				stream << std::put_time(&tm, format.c_str()) ;
				return stream.str() ;
			}

			static void writeToFile(const std::string& fileName, const std::string& text) {
				std::ofstream file ;
				file.open (fileName) ;
				file << text ;
				file.close() ;
			}

			template<typename T, int Y, int X> static boost::property_tree::ptree toJson(const Eigen::Matrix<T, Y, X>& matrix) {
				boost::property_tree::ptree matrix_node ;
				for (unsigned int y=0; y<matrix.rows(); y++) {
					boost::property_tree::ptree row ;

					for (unsigned int x=0; x<matrix.cols(); x++) {
						boost::property_tree::ptree cell ;
						cell.put_value(matrix(y,x)) ;
						row.push_back(std::make_pair("", cell));
					}

					matrix_node.push_back(std::make_pair("", row)) ;
				}
				return matrix_node ;
			}

			template<typename T, int Y, int X> static Eigen::Matrix<T, Y, X> readMatrix(const boost::property_tree::ptree& root, const std::string& element) {
				Eigen::Matrix<T, Y, X> matrix ;

				// resize matrix, if size is dynamic
				if ((Y == Eigen::Dynamic) || (X == Eigen::Dynamic)) {
					unsigned int y = 0 ;
					unsigned int x = 0 ;
					for (const boost::property_tree::ptree::value_type& col : root.get_child(element)) {
						if (y == 0) {
							for (const boost::property_tree::ptree::value_type& cell : col.second) {
								x++ ;
							}
						}
						y++ ;
					}

					//ROS_INFO_STREAM("resizing matrix to "<<y<<"x"<<x) ;
					matrix.resize(y,x) ;
				}

				// fill matrix
				unsigned int y = 0 ;
				for (const boost::property_tree::ptree::value_type& col : root.get_child(element)) {
					unsigned int x = 0 ;
					for (const boost::property_tree::ptree::value_type& cell : col.second) {

						matrix(y,x) = cell.second.get_value<T>() ;
						x++ ;
					}
					y++ ;
				}
				
				return matrix ;
			}

			static Eigen::Matrix4f toEigen(const tf::StampedTransform& tf) {
				tf::Matrix3x3 basis = tf.getBasis() ;
				tf::Vector3 origin = tf.getOrigin() ;

				Eigen::Matrix4f result ;
				result << basis[0][0], basis[0][1], basis[0][2], origin[0],
				          basis[1][0], basis[1][1], basis[1][2], origin[1],
				          basis[2][0], basis[2][1], basis[2][2], origin[2],
				                  0.f,         0.f,         0.f,       1.f;

				//ROS_INFO_STREAM("[Toolbox] Eigen out:\n"<<result) ;

				return result ;
			}
	} ;
}

#endif