#include <tum_ar_window/ARWindow.h>
#include <tum_ar_window/Toolbox.h>
#include <ui_ARWindow.h>
#include <QtSvg>

tum::ARWindow::ARWindow(QWidget *parent)
 : QMainWindow(parent),
   _ui(new Ui::ARWindow),
   _scene(new QGraphicsScene) {
	_ui->setupUi(this) ;
	setWindowTitle("AR Window") ;
	QObject::connect(_ui->pushButtonAccept, SIGNAL(clicked()),this, SLOT(pushButtonAcceptClicked())) ;
	QObject::connect(_ui->pushButtonReject, SIGNAL(clicked()),this, SLOT(pushButtonRejectClicked())) ;

	_userInputPub = _nh.advertise<tum_ar_window::InspectionResult>("user_input", 1) ;

	_ui->arDisplay->setScene(_scene.get()) ;
	_ui->arDisplay->show() ;
}

tum::ARWindow::~ARWindow() {
	//delete _ui ;
}

void tum::ARWindow::display(const std::string& url) {
	QPixmap pixmap(url.c_str());
	display(pixmap) ;
}


void tum::ARWindow::hideButtons() {
	_ui->pushButtonAccept->setVisible(0);
	_ui->pushButtonReject->setVisible(0);
}

void tum::ARWindow::display(const QPixmap& pixmap) {
	QList<QGraphicsItem*> items = _scene->items() ;
	for (int i = 0; i < items.size(); i++) {
		_scene->removeItem(items[i]);
		delete items[i];
	}
	_scene->addPixmap(pixmap);
}

int tum::ARWindow::canvasWidth() const {
	return _ui->arDisplay->width() ;
}

int tum::ARWindow::canvasHeight() const {
	return _ui->arDisplay->height() ;
}

QSize tum::ARWindow::canvasSize() const {
	return _ui->arDisplay->size() ;
}

QPoint tum::ARWindow::canvasPosition() const {
	return _ui->arDisplay->mapToGlobal(QPoint(0,0)) ;	
}

void tum::ARWindow::pushButtonAcceptClicked() {
	ROS_INFO_STREAM("[tum_ar_window] Accept button pressed.") ;

	tum_ar_window::InspectionResult userInput ;
	userInput.status = tum_ar_window::InspectionResult::ACCEPTED ;
	_userInputPub.publish(userInput) ;
}

void tum::ARWindow::pushButtonRejectClicked() {
	ROS_INFO_STREAM("[tum_ar_window] Reject button pressed.") ;

	tum_ar_window::InspectionResult userInput ;
	userInput.status = tum_ar_window::InspectionResult::REJECTED ;
	_userInputPub.publish(userInput) ;
}