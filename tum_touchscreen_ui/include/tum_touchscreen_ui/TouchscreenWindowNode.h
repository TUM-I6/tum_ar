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

#ifndef SRC_TOUCHSCREENWINDOWNODE_H
#define SRC_TOUCHSCREENWINDOWNODE_H

#include <ros/ros.h>
#include <string.h>
#include <QApplication>
#include <tum_ar_msgs/ARSlide.h>
#include <tum_touchscreen_ui/TouchscreenWindow.h>

namespace tum {
	class TouchscreenWindowNode {
		public:
			TouchscreenWindowNode(QApplication& qa);
			virtual ~TouchscreenWindowNode();

			void run();
			void arSlideCallback(const tum_ar_msgs::ARSlide::ConstPtr& msg);

		private:
			ros::NodeHandle _nh;
			ros::Subscriber _arSlideSub;
			QApplication& _qa;
			TouchscreenWindow _window;
	};
}

#endif //SRC_TOUCHSCREENWINDOWNODE_H
