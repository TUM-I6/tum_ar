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
		QPushButton* button = new QPushButton(QString::fromStdString(option.name));
		button->setProperty("option_id", QVariant(option.id));
		button->setProperty("option_type", QVariant(option.type));
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

void tum::ARWindow::pushButtonOutcomeClicked() {
	QPushButton* trigger = (QPushButton*)sender();
	ROS_INFO_STREAM("[tum_ar_window] Outcome button "<<trigger->property("option_id").toInt()<<" pressed.");

	tum_ar_msgs::Outcome outcome;
	outcome.id = trigger->property("option_id").toInt();
	outcome.type = trigger->property("option_type").toInt();
	outcome.name = trigger->text().toStdString();
	_userInputPub.publish(outcome);
}
