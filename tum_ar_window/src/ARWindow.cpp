#include <tum_ar_window/ARWindow.h>
#include <tum_ar_window/Toolbox.h>
#include <ui_ARWindow.h>
#include <QtSvg>

ARWindow::ARWindow(QWidget *parent) :
 QMainWindow(parent),
 _ui(new Ui::ARWindow),
 _nh(),
 _actionServer(_nh, "ar_inspection", false),
 _pos(0),
 _wsaType("type_a"),
 _defaultSize(7),
 _horseLogo(QString(ros::package::getPath("tum_ar_window").c_str()) + "/images/horse.svg"),
 _tumI6Logo(QString(ros::package::getPath("tum_ar_window").c_str()) + "/images/tum_i6.svg"),
 _tumLogo(QString(ros::package::getPath("tum_ar_window").c_str()) + "/images/tum.svg") {
	_ui->setupUi(this);
	setWindowTitle("AR Window") ;
	QObject::connect(_ui->pushButtonAccept, SIGNAL(clicked()),this, SLOT(pushButtonAcceptClicked()));
	QObject::connect(_ui->pushButtonReject, SIGNAL(clicked()),this, SLOT(pushButtonRejectClicked()));

	/* // for debugging only
	for (int i=0; i<_defaultSize; i++) {
		_pois.push_back(i) ;
	}*/

	_actionServer.registerGoalCallback(boost::bind(&ARWindow::executeARInspection, this));
	_actionServer.start() ;

	QGraphicsScene* scene = new QGraphicsScene ;
	_ui->arDisplay->setScene(scene) ;
	_ui->arDisplay->show() ;

	_font.setPointSize(18) ;
}

ARWindow::~ARWindow() {
	delete _ui ;
}

void ARWindow::draw() {
	int width = _ui->arDisplay->width() ;
	int height = _ui->arDisplay->height() ;
	QPixmap pixmap = getNewFrame("Waiting for start signal...") ;

	/*
	//create a QPainter and pass a pointer to the device.
	//A paint device can be a QWidget, a QPixmap or a QImage
	QPainter painter(&pixmap) ;
	QPen myPen(Qt::white, 3, Qt::SolidLine) ;
	painter.setPen(myPen) ;

	//draw a point
	myPen.setColor(Qt::red) ;
	painter.drawPoint(110,110) ;

	//draw a polygon
	QPolygon polygon ;
	polygon << QPoint(130, 140) << QPoint(180, 170)
	        << QPoint(180, -140) << QPoint(220, 110)
	        << QPoint(140, 100) ;
	painter.drawPolygon(polygon) ;

	//draw an ellipse
	//The setRenderHint() call enables antialiasing, telling QPainter to use different
	//color intensities on the edges to reduce the visual distortion that normally
	//occurs when the edges of a shape are converted into pixels
	painter.setRenderHint(QPainter::Antialiasing, true) ;
	painter.setPen(QPen(Qt::red, 3, Qt::DashDotLine, Qt::RoundCap)) ;
	painter.setBrush(QBrush(Qt::green, Qt::SolidPattern)) ;
	painter.drawEllipse(200, 80, 400, 240) ;
	*/

	QGraphicsScene* scene = _ui->arDisplay->scene();
	QList<QGraphicsItem*> items = scene->items();
	for (int i = 0; i < items.size(); i++) {
		scene->removeItem(items[i]);
		delete items[i];
	}
	scene->addPixmap(pixmap);
}

QPixmap ARWindow::getNewFrame(const std::string& title) {
	int offset = 64 ;

	int width = _ui->arDisplay->width() ;
	int height = _ui->arDisplay->height() ;

	QPixmap pixmap(width, height) ;

	pixmap.fill(Qt::transparent) ;

	QPainter painter(&pixmap) ;

	drawCornerBox(painter, QPoint(0,0), QSize(width, height)) ;	
	drawLogos(painter, QPoint(offset,offset), 64) ;


	QPen myPen(Qt::white, 3, Qt::SolidLine) ;
	painter.setPen(myPen) ;
	painter.setFont(_font) ;
	painter.drawText(QRect(QPoint(offset, height/2), QPoint(width/2, height-offset)), Qt::AlignLeft|Qt::AlignBottom, title.c_str()) ;
	std::string info = tum::Toolbox::getDateTimeString("%H:%M")+"\n"+ros::this_node::getName()+ros::this_node::getNamespace() ;
	painter.drawText(QRect(QPoint(width/2, height/2), QPoint(width-offset, height-offset)), Qt::AlignRight|Qt::AlignBottom, info.c_str()) ;

	return pixmap ;
}

void ARWindow::drawCornerBox(QPainter& painter, const QPoint& pos, const QSize& box, const QColor& color, const int edge, const int lineWidth) {
	int lwh = lineWidth/2 ;

	QPoint ul(pos.x()+lwh,             pos.y()+lwh) ;
	QPoint ur(pos.x()+box.width()-lwh, pos.y()+lwh) ;
	QPoint bl(pos.x()+lwh,             pos.y()+box.height()-lwh) ;
	QPoint br(pos.x()+box.width()-lwh, pos.y()+box.height()-lwh) ;

	painter.setPen(QPen(color, lineWidth, Qt::SolidLine)) ;

	painter.drawLine(ul.x(), ul.y(), ul.x(),      ul.y()+edge) ;
	painter.drawLine(ul.x(), ul.y(), ul.x()+edge, ul.y()) ;

	painter.drawLine(ur.x(), ur.y(), ur.x(),      ur.y()+edge) ;
	painter.drawLine(ur.x(), ur.y(), ur.x()-edge, ur.y()) ;
	
	painter.drawLine(bl.x(), bl.y(), bl.x(),      bl.y()-edge) ;
	painter.drawLine(bl.x(), bl.y(), bl.x()+edge, bl.y()) ;

	painter.drawLine(br.x(), br.y(), br.x(),      br.y()-edge) ;
	painter.drawLine(br.x(), br.y(), br.x()-edge, br.y()) ;
}

void ARWindow::drawLogos(QPainter& painter, const QPoint& position, const int logoHeight) {
	// draw logos
	 
	QPoint pos = position ;
	QSize size = _horseLogo.actualSize(QSize(1000,logoHeight)) ;
	painter.drawPixmap(pos.x(), pos.y(), _horseLogo.pixmap(size));

	pos.rx() += size.width() + logoHeight ;
	size = _tumLogo.actualSize(QSize(1000,logoHeight)) ;
	painter.drawPixmap(pos.x(), pos.y(), _tumLogo.pixmap(size));

	pos.rx() += size.width() + logoHeight ;
	size = _tumI6Logo.actualSize(QSize(1000,logoHeight)) ;
	painter.drawPixmap(pos.x(), pos.y(), _tumI6Logo.pixmap(size));
}

void ARWindow::displayImage(const cv::Mat& image) {
	/*if (frame.channels()== 3) {
		cv::cvtColor(frame, RGBframe, CV_BGR2RGB) ;
		img = QImage((const unsigned char*)(RGBframe.data), RGBframe.cols,RGBframe.rows,QImage::Format_RGB888) ;
	}
	else {
		img = QImage((const unsigned char*)(frame.data), frame.cols,frame.rows,QImage::Format_Indexed8) ;
	}

	QGraphicsPixmapItem item( QPixmap::fromImage(image));
    QGraphicsScene* scene = new QGraphicsScene;
    scene->addItem(&item);*/
}

void ARWindow::displayImage(const std::string& url) {
	QGraphicsScene *scene = new QGraphicsScene ;
	QPixmap pixmap(url.c_str()) ;
	scene->addPixmap(pixmap) ;
	_ui->arDisplay->setScene(scene) ;
	_ui->arDisplay->show() ;
	//sleep(3) ;
}

void ARWindow::executeARInspection() { // const tum_ar_window::ARInspectionGoalConstPtr &goal
	tum_ar_window::ARInspectionGoalConstPtr goal = _actionServer.acceptNewGoal() ;

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
	ROS_INFO_STREAM("[tum_ar_window] Running AR inspection on "<<_pois.size()<<" POIs of "<<_wsaType<<" WSA") ;
}

void ARWindow::pushButtonAcceptClicked() {
	if (_pois.size() == 0) {
		return ;
	}

	ROS_INFO_STREAM("[tum_ar_window] POI "<<_pois[_pos]<<" accepted.") ;
	publishFeedback(_pois[_pos], tum_ar_window::ARInspectionFeedback::ACCEPTED) ;

	_pos++ ;
	if (_pos >= _pois.size()) {
		_pos = 0 ;
		displayImage(getImageURL()) ;
		_pois.clear() ;

		tum_ar_window::ARInspectionResult result ;
		result.status = tum_ar_window::ARInspectionResult::ACCEPTED ;
		_actionServer.setSucceeded(result);
	}
	else {
		displayImage(getImageURL(_wsaType, _pois[_pos])) ;
	}
}

void ARWindow::pushButtonRejectClicked() {
	if (_pois.size() == 0) {
		return ;
	}

	ROS_INFO_STREAM("[tum_ar_window] POI "<<_pois[_pos]<<" rejected.") ;
	publishFeedback(_pois[_pos], tum_ar_window::ARInspectionFeedback::REJECTED) ;

	displayImage(getImageURL()) ;
	_pos = 0 ;
	_pois.clear() ;
	
	tum_ar_window::ARInspectionResult result ;
	result.status = tum_ar_window::ARInspectionResult::REJECTED ;
	_actionServer.setSucceeded(result);
}