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

#include <tum_ar_window/ARWindowNode.h>
#include <tum_ar_window/ConfigReader.h>
#include <ros/package.h>

#define ROS_PACKAGE_NAME "tum_ar_window"

tum::ARWindowNode::ARWindowNode(QApplication& qa)
		: _projector(_nh, _projectorConfig),
		  _renderer(_projector),
		  _qa(qa) {

	_window.showFullScreen();

	_nh.param<bool>("hide_buttons", _hideButtons, false);
	_nh.param<std::string>("projector_config", _projectorConfigFile, std::string("package:://")+ROS_PACKAGE_NAME+"/config/projector_tum.yaml");

	_projectorConfig = ConfigReader::readProjectorConfig(ConfigReader::preparePath(_projectorConfigFile));
	_projector.setConfig(_projectorConfig);

	_arSlideSub = _nh.subscribe("ar_slide", 10, &ARWindowNode::arSlideCallback, this);
}

tum::ARWindowNode::~ARWindowNode() {
}

void tum::ARWindowNode::run() {
	ros::Rate rate(30);

	QPixmap slide;
	while(ros::ok() && _window.isVisible()) {
		// process events and messages
		if (_qa.hasPendingEvents()) {
			_qa.processEvents();
		}
		ros::spinOnce();

		// render new image
		QRect canvas = _window.canvasArea();
		slide = _renderer.renderSlide(_slide, canvas);
		_window.display(slide);

		// other stuff
		_projector.publishViewFrustumMarker(_projector.getImagePlane(2.05f));
		rate.sleep();
	}
}

void tum::ARWindowNode::arSlideCallback(const tum_ar_msgs::ARSlide::ConstPtr& msg) {
	ROS_DEBUG_STREAM("[ARWindowNode] Received new slide");

	_slide = *msg;

	if (!_hideButtons) {
		_window.clearButtons();
		_window.addButtons(_slide.outcomes);
	}
}
