#include <tum_ar_window/ARWindow.h>
#include <tum_ar_window/Toolbox.h>
#include <ui_ARWindow.h>
#include <QtSvg>

#define TEXTBOX_WIDTH 1920
#define TEXTBOX_HEIGHT 128
#define TEXTBOX_MIN_HSPACE 512
#define TEXTBOX_MIN_VSPACE 128

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

int ARWindow::canvasWidth() const {
	return _ui->arDisplay->width() ;
}

int ARWindow::canvasHeight() const {
	return _ui->arDisplay->height() ;
}

void ARWindow::draw() {
	QPixmap pixmap = getNewFrame("Waiting for start signal...") ;

	//create a QPainter and pass a pointer to the device.
	//A paint device can be a QWidget, a QPixmap or a QImage
	QPainter painter(&pixmap) ;

	drawPoi(painter, QPoint( 500, 500),  50, "label") ;
	drawPoi(painter, QPoint( 600, 700),  10, "another label") ;
	drawPoi(painter, QPoint( 800, 400), 100) ;
	drawPoi(painter, QPoint(1500, 600),  20) ;
	drawPoi(painter, QPoint(1000, 600), 150) ;
	drawPoi(painter, QPoint( 700,1100), 200, "moved label") ;

	drawArea(painter, QPoint( 200,-100), QSize(300,400), "Fancy Label") ;
	drawArea(painter, QPoint( 200,1000), QSize(300,300), "Fancy Label") ;
	drawArea(painter, QPoint(1500, 200), QSize(600,200), "Fancy Label") ;
	drawArea(painter, QPoint( -10, 600), QSize(200,300), "Fancy Label") ;
	drawArea(painter, QPoint(1400, 700), QSize(500,500), "Fancy Label") ;

	QGraphicsScene* scene = _ui->arDisplay->scene();
	QList<QGraphicsItem*> items = scene->items();
	for (int i = 0; i < items.size(); i++) {
		scene->removeItem(items[i]);
		delete items[i];
	}
	scene->addPixmap(pixmap);
}

QPixmap ARWindow::getNewFrame(const std::string& title) const {
	int offset = 64 ;

	QPixmap pixmap(canvasWidth(), canvasHeight()) ;

	pixmap.fill(Qt::transparent) ;

	QPainter painter(&pixmap) ;

	drawCornerBox(painter, QPoint(0,0), QSize(canvasWidth(), canvasHeight())) ;	
	drawLogos(painter, QPoint(offset,offset), 64) ;


	QPen myPen(Qt::white, 3, Qt::SolidLine) ;
	painter.setPen(myPen) ;
	painter.setFont(_font) ;
	painter.drawText(QRect(QPoint(offset, canvasHeight()/2), QPoint(canvasWidth()/2, canvasHeight()-offset)), Qt::AlignLeft|Qt::AlignBottom, title.c_str()) ;
	std::string info = tum::Toolbox::getDateTimeString("%H:%M")+"\n"+ros::this_node::getName()+ros::this_node::getNamespace() ;
	painter.drawText(QRect(QPoint(canvasWidth()/2, canvasHeight()/2), QPoint(canvasWidth()-offset, canvasHeight()-offset)), Qt::AlignRight|Qt::AlignBottom, info.c_str()) ;

	return pixmap ;
}

void ARWindow::drawCornerBox(QPainter& painter, const QPoint& pos, const QSize& box, const QColor& color, const int edge, const int lineWidth) const {
	int lwh = lineWidth/2 ;

	QPoint ul(pos.x()+lwh,             pos.y()+lwh) ;
	QPoint ur(pos.x()+box.width()-lwh, pos.y()+lwh) ;
	QPoint bl(pos.x()+lwh,             pos.y()+box.height()-lwh) ;
	QPoint br(pos.x()+box.width()-lwh, pos.y()+box.height()-lwh) ;

	painter.setPen(QPen(color, lineWidth, Qt::SolidLine, Qt::SquareCap)) ;

	painter.drawLine(ul.x(), ul.y(), ul.x(),      ul.y()+edge) ;
	painter.drawLine(ul.x(), ul.y(), ul.x()+edge, ul.y()) ;

	painter.drawLine(ur.x(), ur.y(), ur.x(),      ur.y()+edge) ;
	painter.drawLine(ur.x(), ur.y(), ur.x()-edge, ur.y()) ;
	
	painter.drawLine(bl.x(), bl.y(), bl.x(),      bl.y()-edge) ;
	painter.drawLine(bl.x(), bl.y(), bl.x()+edge, bl.y()) ;

	painter.drawLine(br.x(), br.y(), br.x(),      br.y()-edge) ;
	painter.drawLine(br.x(), br.y(), br.x()-edge, br.y()) ;
}

void ARWindow::drawLogos(QPainter& painter, const QPoint& position, const int logoHeight) const {
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

void ARWindow::drawPoi(QPainter& painter, const QPoint& position, const float radius, const std::string& label, const QColor& border, const QColor& fill) const {
	int borderWidth = radius/25+2 ;
	painter.setRenderHint(QPainter::Antialiasing, true) ;
	painter.setPen(QPen(border, borderWidth, Qt::SolidLine, Qt::RoundCap)) ;
	painter.setBrush(QBrush(fill, Qt::SolidPattern)) ;
	painter.drawEllipse(position, radius+borderWidth/2, radius+borderWidth/2) ;

	drawCircleLabel(painter, position, radius+borderWidth, label, 2*borderWidth, border) ;
}

void ARWindow::drawArea(QPainter& painter, const QPoint& position, const QSize& size, const std::string& label, const QColor& border, const QColor& fill) const {
	int bw  = std::min<int>(size.width(), size.height())/50+2 ; // border width
	int bwh = bw/2 ;

	painter.setRenderHint(QPainter::Antialiasing, true) ;
	painter.setPen(QPen(border, bw, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin)) ;
	painter.setBrush(QBrush(fill, Qt::SolidPattern)) ;
	painter.drawRect(position.x()-bwh, position.y()-bwh, size.width()+bw, size.height()+bw) ;

	drawBoxLabel(painter, QPoint(position.x()-bwh, position.y()-bwh), QSize(size.width()+bw, size.height()+bw), label, 2*bw, border) ;
}

void ARWindow::drawBoxLabel(QPainter& painter, const QPoint& position, const QSize& size, const std::string& label, const int offset, const QColor& color) const {
	painter.setPen(QPen(color)) ;
	painter.setFont(_font) ;

	QPoint topLeft    (position.x(),              position.y()-offset) ;
	QPoint topRight   (position.x()+size.width(), topLeft.y()) ;
	QPoint rightTop   (topRight.x()+offset,       position.y()) ;
	QPoint rightBottom(rightTop.x(),              position.y()+size.height()) ;
	QPoint bottomLeft (topLeft.x(),               rightBottom.y()+offset) ;
	QPoint bottomRight(topRight.x(),              bottomLeft.y()) ;
	QPoint leftTop    (topLeft.x()-offset,        rightTop.y()) ;
	QPoint leftBottom (leftTop.x(),               rightBottom.y()) ;

	bool spaceTop    = topLeft.y() < canvasHeight() && topLeft.y() > TEXTBOX_MIN_VSPACE ;
	bool spaceRight  = rightTop.x() > 0             && canvasWidth()-rightTop.x() > TEXTBOX_MIN_HSPACE ;
	bool spaceBottom = bottomLeft.y() > 0           && canvasHeight()-bottomLeft.y() > TEXTBOX_MIN_VSPACE ;
	bool spaceLeft   = leftTop.x() < canvasWidth()  && leftTop.x() > TEXTBOX_MIN_HSPACE ;

	QSize textboxSize(TEXTBOX_WIDTH, TEXTBOX_HEIGHT) ;

	if (spaceBottom && (canvasWidth()-bottomLeft.x() > TEXTBOX_MIN_HSPACE) && (bottomLeft.x() > 0)) {
		// bottom left
		QRect rect(bottomLeft, textboxSize) ;
		painter.drawText(rect, Qt::AlignLeft|Qt::AlignTop, label.c_str()) ;
	}
	else if (spaceBottom && (bottomRight.x() > TEXTBOX_MIN_HSPACE) && (bottomRight.x() < canvasWidth())) {
		// bottom right
		QRect rect(QPoint(bottomRight.x()-TEXTBOX_WIDTH, bottomRight.y()), textboxSize) ;
		painter.drawText(rect, Qt::AlignRight|Qt::AlignTop, label.c_str()) ;
	}
	else if (spaceTop && (canvasWidth()-topLeft.x() > TEXTBOX_MIN_HSPACE) && (topLeft.x() > 0)) {
		// top left
		QRect rect(QPoint(topLeft.x(), topLeft.y()-TEXTBOX_HEIGHT), textboxSize) ;
		painter.drawText(rect, Qt::AlignLeft|Qt::AlignBottom, label.c_str()) ;
	}
	else if (spaceTop && (topRight.x() > TEXTBOX_MIN_HSPACE) && (topRight.x() < canvasWidth())) {
		// top right
		QRect rect(QPoint(topRight.x()-TEXTBOX_WIDTH, topRight.y()-TEXTBOX_HEIGHT), textboxSize) ;
		painter.drawText(rect, Qt::AlignRight|Qt::AlignBottom, label.c_str()) ;
	}
	else if (spaceRight && (canvasHeight()-rightTop.y() > TEXTBOX_MIN_VSPACE) && (rightTop.y() > 0)) {
		// right top
		QRect rect(rightTop, textboxSize) ;
		painter.drawText(rect, Qt::AlignLeft|Qt::AlignTop, label.c_str()) ;
	}
	else if (spaceRight && (rightBottom.y() > TEXTBOX_MIN_VSPACE) && (rightBottom.y() < canvasHeight())) {
		// right bottom
		QRect rect(QPoint(rightBottom.x(), rightBottom.y()-TEXTBOX_HEIGHT), textboxSize) ;
		painter.drawText(rect, Qt::AlignLeft|Qt::AlignBottom, label.c_str()) ;
	}
	else if (spaceLeft && (canvasHeight()-leftTop.y() > TEXTBOX_MIN_VSPACE) && (leftTop.y() > 0)) {
		// left top
		QRect rect(QPoint(leftTop.x()-TEXTBOX_WIDTH, leftTop.y()), textboxSize) ;
		painter.drawText(rect, Qt::AlignRight|Qt::AlignTop, label.c_str()) ;
	}
	else if (spaceRight && (leftBottom.y() > TEXTBOX_MIN_HSPACE) && (leftBottom.y() < canvasHeight())) {
		// left bottom
		QRect rect(QPoint(leftBottom.x()-TEXTBOX_WIDTH, leftBottom.y()-TEXTBOX_HEIGHT), textboxSize) ;
		painter.drawText(rect, Qt::AlignRight|Qt::AlignBottom, label.c_str()) ;
	}
	else {
		// box either covers the full screen, or is completly out of the canvas
		// todo: draw label inside box
	}
}

void ARWindow::drawCircleLabel(QPainter& painter, const QPoint& circlePosition, const float radius, const std::string& label, const int offset, const QColor& color) const {
	int dist = radius+offset ;
	QSize textboxSize(TEXTBOX_WIDTH, TEXTBOX_HEIGHT) ;

	QPoint top   (circlePosition.x(),      circlePosition.y()-dist) ;
	QPoint right (circlePosition.x()+dist, circlePosition.y()) ;
	QPoint bottom(circlePosition.x(),      circlePosition.y()+dist) ;
	QPoint left  (circlePosition.x()-dist, circlePosition.x()) ;

	bool spaceTopH    = 0 < top.x()-TEXTBOX_MIN_HSPACE/2 && top.x()+TEXTBOX_MIN_HSPACE/2 < canvasWidth() ;
	bool spaceTopV    = top.y() < canvasHeight() && top.y() > TEXTBOX_MIN_VSPACE ;
	bool spaceRightH  = canvasWidth()-right.x() < TEXTBOX_MIN_HSPACE && right.x() > 0 ;
	bool spaceRightV  = 0 < right.y()-TEXTBOX_MIN_VSPACE/2 && right.y()+TEXTBOX_MIN_VSPACE/2 < canvasHeight() ;
	bool spaceBottomH = spaceTopH ;
	bool spaceBottomV = 0 < bottom.y() && bottom.y()+TEXTBOX_MIN_VSPACE < canvasHeight() ;
	bool spaceLeftH   = TEXTBOX_MIN_HSPACE < left.x() && left.x() < canvasWidth() ;
	bool spaceLeftV   = spaceRightV ;

	painter.setPen(QPen(color)) ;
	painter.setFont(_font) ;

	if (spaceBottomH && spaceBottomV) {
		// below, horizontally centered
		QRect rect(QPoint(bottom.x()-TEXTBOX_WIDTH/2, bottom.y()), textboxSize) ;
		painter.drawText(rect, Qt::AlignHCenter|Qt::AlignTop, label.c_str()) ;
	}
	else if (spaceTopH && spaceTopV) {
		// above, horizontally centered
		QRect rect(QPoint(top.x()-TEXTBOX_WIDTH/2, top.y()-TEXTBOX_HEIGHT), textboxSize) ;
		painter.drawText(rect, Qt::AlignHCenter|Qt::AlignBottom, label.c_str()) ;
	}
	else if (spaceRightH && spaceRightV) {
		// right, vertically centered
		QRect rect(QPoint(right.x(), right.y()-TEXTBOX_HEIGHT/2), textboxSize) ;
		painter.drawText(rect, Qt::AlignLeft|Qt::AlignVCenter, label.c_str()) ;
	}
	else if (spaceTopH && spaceTopV) {
		// left, vertically centered
		QRect rect(QPoint(left.x()-TEXTBOX_WIDTH, left.y()-TEXTBOX_HEIGHT/2), textboxSize) ;
		painter.drawText(rect, Qt::AlignRight|Qt::AlignVCenter, label.c_str()) ;
	}
	else {
		// circle either covers the full screen, or is completly out of the canvas
		// todo: draw label inside circle
	}
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