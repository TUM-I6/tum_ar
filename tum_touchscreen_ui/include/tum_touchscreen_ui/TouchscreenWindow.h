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

#ifndef SRC_TOUCHSCREENWINDOW_H
#define SRC_TOUCHSCREENWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <memory>
#include <ros/ros.h>
#include <tum_ar_msgs/ARSlide.h>

namespace Ui {
	class TouchscreenWindow;
}

namespace tum {
	class TouchscreenWindow : public QMainWindow {
		Q_OBJECT

		public:
			explicit TouchscreenWindow(QWidget *parent = 0);
			virtual ~TouchscreenWindow();

			void showInterface(const tum_ar_msgs::ARSlide& slide);
			void clearInterface();
			static QPushButton* createButtonCopy(const QPushButton* button);
			QPushButton* getButton(const tum_ar_msgs::Outcome& outcome);

		public slots:
			void pushButtonOutcomeClicked();

		protected:
			void addButtons(std::vector<tum_ar_msgs::Outcome> outcomeOptions);
			void clearButtons();

		private:
			std::unique_ptr<Ui::TouchscreenWindow> _ui;
			ros::NodeHandle _nh;
			ros::Publisher _userInputPub;

			QPushButton* _defaultButtonTemplate;
			QPushButton* _infoButtonTemplate;
			QPushButton* _successButtonTemplate;
			QPushButton* _warnButtonTemplate;
			QPushButton* _errorButtonTemplate;
	};
}


#endif //SRC_TOUCHSCREENWINDOW_H
