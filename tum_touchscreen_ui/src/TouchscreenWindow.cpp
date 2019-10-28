//
// Created by arne on 28.10.19.
//

#include <tum_touchscreen_ui/TouchscreenWindow.h>
#include <ui_TouchscreenWindow.h>

tum::TouchscreenWindow::TouchscreenWindow(QWidget* parent)
: QMainWindow(parent),
  _ui(new Ui::TouchscreenWindow) {
	_ui->setupUi(this);
	clearInterface();

	_userInputPub = _nh.advertise<tum_ar_msgs::Outcome>("user_input", 1);
}

tum::TouchscreenWindow::~TouchscreenWindow() {
	if (isVisible()) {
		close();
	}
}

void tum::TouchscreenWindow::showInterface(const tum_ar_msgs::ARSlide& slide) {
	_ui->noTaskLabel->hide();
	clearButtons();

	_ui->instructionLabel->setText(QString::fromStdString(slide.instruction));
	_ui->instructionLabel->show();
	addButtons(slide.outcomes);
}

void tum::TouchscreenWindow::clearInterface() {
	_ui->noTaskLabel->show();
	_ui->instructionLabel->hide();
	clearButtons();
}

void tum::TouchscreenWindow::addButtons(std::vector<tum_ar_msgs::Outcome> outcomeOptions) {
	ROS_INFO_STREAM("Adding "<<outcomeOptions.size()<<" buttons...");

	for (const tum_ar_msgs::Outcome& option : outcomeOptions) {
		QPushButton* button = new QPushButton(QString::fromStdString(option.name));
		button->setProperty("option_id", QVariant(option.id));
		button->setProperty("option_type", QVariant(option.type));
		_ui->buttonBarHorizontalLayout->addWidget(button);
		QObject::connect(button, SIGNAL(clicked()),this, SLOT(pushButtonOutcomeClicked())) ;
	}
}

void tum::TouchscreenWindow::clearButtons() {
	QLayoutItem *item;
	while((item = _ui->buttonBarHorizontalLayout->takeAt(0))) {
		if (item->widget()) {
			delete item->widget();
		}
		delete item;
	}
}

void tum::TouchscreenWindow::pushButtonOutcomeClicked() {
	QPushButton* trigger = (QPushButton*)sender();
	ROS_INFO_STREAM("[tum_ar_window] Outcome button "<<trigger->property("option_id").toInt()<<" pressed.");

	tum_ar_msgs::Outcome outcome;
	outcome.id = trigger->property("option_id").toInt();
	outcome.type = trigger->property("option_type").toInt();
	outcome.name = trigger->text().toStdString();
	_userInputPub.publish(outcome);
}
