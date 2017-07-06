#include <ar_window/mainwindow.h>
#include <ui_mainwindow.h>

MainWindow::MainWindow(QWidget *parent) :
 QMainWindow(parent),
 _ui(new Ui::MainWindow),
 _nh(),
 _actionServer(_nh, "ar_inspection", false),
 _pos(0),
 _wsaType("type_a"),
 _defaultSize(7) {
	_ui->setupUi(this);
	QObject::connect(_ui->pushButtonAccept, SIGNAL(clicked()),this, SLOT(pushButtonAcceptClicked()));
	QObject::connect(_ui->pushButtonReject, SIGNAL(clicked()),this, SLOT(pushButtonRejectClicked()));

	/* // for debugging only
	for (int i=0; i<_defaultSize; i++) {
		_pois.push_back(i) ;
	}*/

	_actionServer.registerGoalCallback(boost::bind(&MainWindow::executeARInspection, this));
	_actionServer.start() ;
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

void MainWindow::executeARInspection() { // const ar_window::ARInspectionGoalConstPtr &goal
	ar_window::ARInspectionGoalConstPtr goal = _actionServer.acceptNewGoal() ;


	_wsaType = goal->wsa_type ;
	_pos = 0 ;

	if (goal->pois.size() > 0) {
		_pois = goal->pois ;
	}
	else {
		_pois.clear() ;
		for (int i=0; i<_defaultSize; i++) {
			_pois.push_back(i) ;
		}
	}

	displayImage(getImageURL(_wsaType, _pois[_pos])) ;

	// publish info to the console for the user
	ROS_INFO_STREAM("[ar_window] Running AR inspection on "<<_pois.size()<<" POIs of "<<_wsaType<<" WSA") ;
}

void MainWindow::pushButtonAcceptClicked() {
	ROS_INFO_STREAM("[ar_window] POI "<<_pois[_pos]<<" accepted.") ;
	publishFeedback(_pois[_pos], ar_window::ARInspectionFeedback::ACCEPTED) ;

	_pos++ ;
	if (_pos >= _pois.size()) {
		_pos = 0 ;
		displayImage(getImageURL()) ;
		_pois.clear() ;

		ar_window::ARInspectionResult result ;
		result.status = ar_window::ARInspectionResult::ACCEPTED ;
		_actionServer.setSucceeded(result);
	}
	else {
		displayImage(getImageURL(_wsaType, _pois[_pos])) ;
	}
}

void MainWindow::pushButtonRejectClicked() {
	ROS_INFO_STREAM("[ar_window] POI "<<_pois[_pos]<<" rejected.") ;
	publishFeedback(_pois[_pos], ar_window::ARInspectionFeedback::REJECTED) ;

	displayImage(getImageURL()) ;
	_pos = 0 ;
	_pois.clear() ;
	
	ar_window::ARInspectionResult result ;
	result.status = ar_window::ARInspectionResult::REJECTED ;
	_actionServer.setSucceeded(result);
}