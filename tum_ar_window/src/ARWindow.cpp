#include <tum_ar_window/ARWindow.h>
#include <tum_ar_window/Toolbox.h>
#include <ui_ARWindow.h>
#include <QtSvg>

tum::ARWindow::ARWindow(QWidget *parent)
 : QMainWindow(parent),
   _ui(new Ui::ARWindow),
   _scene(new QGraphicsScene) {
	_ui->setupUi(this);

	setWindowTitle("AR Window");

	_defaultButtonTemplate = createButtonCopy(_ui->defaultButton);
	_infoButtonTemplate = createButtonCopy(_ui->infoButton);
	_successButtonTemplate = createButtonCopy(_ui->successButton);
	_warnButtonTemplate = createButtonCopy(_ui->warnButton);
	_errorButtonTemplate = createButtonCopy(_ui->errorButton);
	clearButtons();

	_userInputPub = _nh.advertise<tum_ar_msgs::Outcome>("user_input", 1);

	_ui->arDisplay->setScene(_scene.get());
	_ui->arDisplay->show();
}

tum::ARWindow::~ARWindow() {
	if (isVisible()) {
		close();
	}
}

void tum::ARWindow::display(const std::string& url) {
	QPixmap pixmap(url.c_str());
	display(pixmap);
}

void tum::ARWindow::display(const QPixmap& pixmap) {
	QList<QGraphicsItem*> items = _scene->items();
	for (int i = 0; i < items.size(); i++) {
		_scene->removeItem(items[i]);
		delete items[i];
	}
	_scene->addPixmap(pixmap);
}

void tum::ARWindow::addButtons(std::vector<tum_ar_msgs::Outcome> outcomeOptions) {
	ROS_INFO_STREAM("Adding "<<outcomeOptions.size()<<" buttons...");

	for (const tum_ar_msgs::Outcome& option : outcomeOptions) {
		QPushButton* button = getButton(option);
		_ui->buttonBar->addWidget(button);
		QObject::connect(button, SIGNAL(clicked()),this, SLOT(pushButtonOutcomeClicked())) ;
	}
}

void tum::ARWindow::clearButtons() {
	QLayoutItem *item;
	while((item = _ui->buttonBar->takeAt(0))) {
		if (item->widget()) {
			delete item->widget();
		}
		delete item;
	}
}

int tum::ARWindow::canvasWidth() const {
	return _ui->arDisplay->width();
}

int tum::ARWindow::canvasHeight() const {
	return _ui->arDisplay->height();
}

QSize tum::ARWindow::canvasSize() const {
	return _ui->arDisplay->size();
}

QPoint tum::ARWindow::canvasPosition() const {
	return _ui->arDisplay->mapToGlobal(QPoint(0,0));	
}

QPushButton* tum::ARWindow::createButtonCopy(const QPushButton* button) {
	QPushButton* copy = new QPushButton(button->text());
	copy->setStyle(button->style());
	copy->setStyleSheet(button->styleSheet());
	copy->setSizePolicy(button->sizePolicy());
	return copy;
}

QPushButton* tum::ARWindow::getButton(const tum_ar_msgs::Outcome& outcome) {
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

void tum::ARWindow::pushButtonOutcomeClicked() {
	QPushButton* trigger = (QPushButton*)sender();
	ROS_INFO_STREAM("[tum_ar_window] Outcome button "<<trigger->property("option_id").toInt()<<" pressed.");

	tum_ar_msgs::Outcome outcome;
	outcome.id = trigger->property("option_id").toInt();
	outcome.type = trigger->property("option_type").toInt();
	outcome.name = trigger->text().toStdString();
	_userInputPub.publish(outcome);
}
