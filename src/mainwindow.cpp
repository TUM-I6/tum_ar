#include <ar_window/mainwindow.h>
#include <ui_mainwindow.h>

MainWindow::MainWindow(QWidget *parent) :
 QMainWindow(parent),
 _ui(new Ui::MainWindow),
 _nh(),
 _actionServer(_nh, "ActionServer", false),
 _pos(0) {
	_ui->setupUi(this);
	QObject::connect(_ui->pushButtonAccept, SIGNAL(clicked()),this, SLOT(pushButtonAcceptClicked()));
	QObject::connect(_ui->pushButtonReject, SIGNAL(clicked()),this, SLOT(pushButtonRejectClicked()));

	for (int i=0; i<5; i++) {
		_pois.push_back(i) ;
	}
}

MainWindow::~MainWindow() {
	delete _ui;
}

void MainWindow::displayImage(const std::string& url) {
	QGraphicsScene *scene = new QGraphicsScene;
	QPixmap pixmap(url.c_str());
	scene->addPixmap(pixmap);
	_ui->arDisplay->setScene(scene);
	_ui->arDisplay->show();
	//sleep(3) ;
}

void MainWindow::executeARInspection(const ar_window::ARInspectionGoalConstPtr &goal) {
	_wsaType = goal->wsa_type ;
	_pos = 0 ;

	if (goal->pois.size() > 0) {
		_pois = goal->pois ;
	}
	else {
		_pois.clear() ;
		for (int i=0; i<5; i++) {
			_pois.push_back(i) ;
		}
	}

	// publish info to the console for the user
	ROS_INFO_STREAM("[ar_window] Running AR inspection on "<<_pois.size()<<" POIs of "<<_wsaType<<" WSA") ;
}

void MainWindow::pushButtonAcceptClicked() {
	_pos++ ;
	if (_pos >= _pois.size()) {
		_pos = 0 ;
		displayImage(getImageURL()) ;
		//todo: complete action
	}
	else {
		displayImage(getImageURL("type_a", _pos)) ;
		// todo: update action status
	}
}

void MainWindow::pushButtonRejectClicked() {
	_pos = 0 ;
	//_pois.clear() ;
	displayImage(getImageURL()) ;
	// todo: report action failed
}