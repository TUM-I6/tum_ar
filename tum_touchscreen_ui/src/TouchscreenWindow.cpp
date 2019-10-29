//
// Created by arne on 28.10.19.
//

#include <tum_touchscreen_ui/TouchscreenWindow.h>
#include <ui_TouchscreenWindow.h>

tum::TouchscreenWindow::TouchscreenWindow(QWidget* parent)
: QMainWindow(parent),
  _ui(new Ui::TouchscreenWindow) {
	_ui->setupUi(this);

	setWindowTitle("Touchscreen UI");

	_defaultButtonTemplate = createButtonCopy(_ui->defaultButton);
	_infoButtonTemplate = createButtonCopy(_ui->infoButton);
	_successButtonTemplate = createButtonCopy(_ui->successButton);
	_warnButtonTemplate = createButtonCopy(_ui->warnButton);
	_errorButtonTemplate = createButtonCopy(_ui->errorButton);
	
	clearInterface();

	_userInputPub = _nh.advertise<tum_ar_msgs::Outcome>("user_input", 1);
}

tum::TouchscreenWindow::~TouchscreenWindow() {
	if (isVisible()) {
		close();
	}
}

QPushButton* tum::TouchscreenWindow::createButtonCopy(const QPushButton* button) {
	QPushButton* copy = new QPushButton(button->text());
	copy->setStyle(button->style());
	copy->setSizePolicy(button->sizePolicy());
	return copy;
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
		QPushButton* button = getButton(option);
		_ui->buttonBarHorizontalLayout->addWidget(button);
		QObject::connect(button, SIGNAL(clicked()),this, SLOT(pushButtonOutcomeClicked())) ;
	}
}

QPushButton* tum::TouchscreenWindow::getButton(const tum_ar_msgs::Outcome& outcome) {
	QPushButton* button;

	if (outcome.type == tum_ar_msgs::Outcome::TYPE_DEFAULT) {
		button = createButtonCopy(_defaultButtonTemplate);
	}
	else if (outcome.type == tum_ar_msgs::Outcome::TYPE_INFO) {
		button = createButtonCopy(_infoButtonTemplate);
	}
	else if (outcome.type == tum_ar_msgs::Outcome::TYPE_SUCCESS) {
		button = createButtonCopy(_successButtonTemplate);
	}
	else if (outcome.type == tum_ar_msgs::Outcome::TYPE_WARN) {
		button = createButtonCopy(_warnButtonTemplate);
	}
	else if (outcome.type == tum_ar_msgs::Outcome::TYPE_ERROR) {
		button = createButtonCopy(_errorButtonTemplate);
	}
	else {
		ROS_WARN_STREAM("Unknown outcome type: "<<outcome.type<<". Using default type instead.");
		button = createButtonCopy(_defaultButtonTemplate);
	}

	button->setText(QString::fromStdString(outcome.name));
	button->setProperty("option_id", QVariant(outcome.id));
	button->setProperty("option_type", QVariant(outcome.type));

	return button;
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
