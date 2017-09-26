#include <tum_ar_window/ARSlideRenderer.h>
#include <tum_ar_window/Toolbox.h>
#include <ros/package.h>

#define TEXTBOX_WIDTH 1920
#define TEXTBOX_HEIGHT 128
#define TEXTBOX_MIN_HSPACE 512
#define TEXTBOX_MIN_VSPACE 128

const QFont tum::ARSlideRenderer::_font = QFont("TUM Neue Helvetica", 18) ;
const QIcon tum::ARSlideRenderer::_tumLogo(QString(ros::package::getPath("tum_ar_window").c_str()) + "/images/tum.svg") ;
const QIcon tum::ARSlideRenderer::_tumI6Logo(QString(ros::package::getPath("tum_ar_window").c_str()) + "/images/tum_i6.svg") ;
const QIcon tum::ARSlideRenderer::_horseLogo(QString(ros::package::getPath("tum_ar_window").c_str()) + "/images/horse.svg") ;

tum::ARSlideRenderer::ARSlideRenderer(const Projector& projector)
: _projector(projector) {
}

tum::ARSlideRenderer::~ARSlideRenderer() {
}

QPixmap tum::ARSlideRenderer::renderSlide(const tum_ar_window::ARSlide& slide) {
	return renderSlide(slide, QRect(0,0,_projector.getResolution().x(),_projector.getResolution().y())) ;
}

QPixmap tum::ARSlideRenderer::renderSlide(const tum_ar_window::ARSlide& slide, const QRect& area) {
	//ROS_INFO_STREAM("[ARSlideRenderer:"<<__LINE__<<"]") ;
	QPixmap pixmap(area.width(), area.height()) ;
	pixmap.fill(Qt::transparent) ;
	QPainter painter(&pixmap) ;

	drawBackground(painter, slide.instruction) ;

	for (const tum_ar_window::Box& box : slide.boxes) {
		drawArea(
			painter,
			QPoint(box.position.x, box.position.y),
			QSize(box.width, box.height),
			box.label,
			QColor(box.border_color.r*255, box.border_color.g*255, box.border_color.b*255, box.border_color.a*255),
			QColor(box.fill_color.r*255, box.fill_color.g*255, box.fill_color.b*255, box.fill_color.a*255)
		) ;
	}

	for (const tum_ar_window::POI& poi : slide.pois) {
		drawPoi(
			painter,
			QPoint(poi.position.x, poi.position.y),
			poi.radius,
			poi.label,
			QColor(poi.border_color.r*255, poi.border_color.g*255, poi.border_color.b*255, poi.border_color.a*255),
			QColor(poi.fill_color.r*255, poi.fill_color.g*255, poi.fill_color.b*255, poi.fill_color.a*255)
		) ;
	}

	return pixmap ;
}

void tum::ARSlideRenderer::drawBackground(QPainter& painter, const std::string& instruction) {
	int offset = 64 ;

	drawCornerBox(painter, QPoint(0,0), QSize(painter.device()->width(), painter.device()->height())) ;	
	drawLogos(painter, QPoint(offset,offset), 64) ;

	QPen myPen(Qt::white, 3, Qt::SolidLine) ;
	painter.setPen(myPen) ;
	painter.setFont(_font) ;

	//ROS_INFO_STREAM("Instruction: "<<instruction) ;
	if (instruction != "") {
		painter.drawText(QRect(QPoint(offset, painter.device()->height()/2), QPoint(painter.device()->width()/2, painter.device()->height()-offset)), Qt::AlignLeft|Qt::AlignBottom, instruction.c_str()) ;
	}
	std::string info = tum::Toolbox::getDateTimeString("%H:%M")+"\n"+ros::this_node::getName()+ros::this_node::getNamespace() ;
	painter.drawText(QRect(QPoint(painter.device()->width()/2, painter.device()->height()/2), QPoint(painter.device()->width()-offset, painter.device()->height()-offset)), Qt::AlignRight|Qt::AlignBottom, info.c_str()) ;
}

void tum::ARSlideRenderer::drawCornerBox(QPainter& painter, const QPoint& pos, const QSize& box, const QColor& color, const int edge, const int lineWidth) {
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

void tum::ARSlideRenderer::drawLogos(QPainter& painter, const QPoint& position, const int logoHeight) {
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

void tum::ARSlideRenderer::drawPoi(QPainter& painter, const QPoint& position, const float radius, const std::string& label, const QColor& border, const QColor& fill) {
	int borderWidth = radius/25+2 ;
	painter.setRenderHint(QPainter::Antialiasing, true) ;
	painter.setPen(QPen(border, borderWidth, Qt::SolidLine, Qt::RoundCap)) ;
	painter.setBrush(QBrush(fill, Qt::SolidPattern)) ;
	painter.drawEllipse(position, radius+borderWidth/2, radius+borderWidth/2) ;

	drawCircleLabel(painter, position, radius+borderWidth, label, 2*borderWidth, border) ;
}

void tum::ARSlideRenderer::drawArea(QPainter& painter, const QPoint& position, const QSize& size, const std::string& label, const QColor& border, const QColor& fill) {
	int bw  = std::min<int>(size.width(), size.height())/50+2 ; // border width
	int bwh = bw/2 ;

	painter.setRenderHint(QPainter::Antialiasing, true) ;
	painter.setPen(QPen(border, bw, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin)) ;
	painter.setBrush(QBrush(fill, Qt::SolidPattern)) ;
	painter.drawRect(position.x()-bwh, position.y()-bwh, size.width()+bw, size.height()+bw) ;

	drawBoxLabel(painter, QPoint(position.x()-bwh, position.y()-bwh), QSize(size.width()+bw, size.height()+bw), label, 2*bw, border) ;
}

void tum::ARSlideRenderer::drawBoxLabel(QPainter& painter, const QPoint& position, const QSize& size, const std::string& label, const int offset, const QColor& color) {
	if (label == "") {
		return ;
	}

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

	bool spaceTop    = topLeft.y() < painter.device()->height() && topLeft.y() > TEXTBOX_MIN_VSPACE ;
	bool spaceRight  = rightTop.x() > 0             && painter.device()->width()-rightTop.x() > TEXTBOX_MIN_HSPACE ;
	bool spaceBottom = bottomLeft.y() > 0           && painter.device()->height()-bottomLeft.y() > TEXTBOX_MIN_VSPACE ;
	bool spaceLeft   = leftTop.x() < painter.device()->width()  && leftTop.x() > TEXTBOX_MIN_HSPACE ;

	QSize textboxSize(TEXTBOX_WIDTH, TEXTBOX_HEIGHT) ;

	if (spaceBottom && (painter.device()->width()-bottomLeft.x() > TEXTBOX_MIN_HSPACE) && (bottomLeft.x() > 0)) {
		// bottom left
		QRect rect(bottomLeft, textboxSize) ;
		painter.drawText(rect, Qt::AlignLeft|Qt::AlignTop, label.c_str()) ;
	}
	else if (spaceBottom && (bottomRight.x() > TEXTBOX_MIN_HSPACE) && (bottomRight.x() < painter.device()->width())) {
		// bottom right
		QRect rect(QPoint(bottomRight.x()-TEXTBOX_WIDTH, bottomRight.y()), textboxSize) ;
		painter.drawText(rect, Qt::AlignRight|Qt::AlignTop, label.c_str()) ;
	}
	else if (spaceTop && (painter.device()->width()-topLeft.x() > TEXTBOX_MIN_HSPACE) && (topLeft.x() > 0)) {
		// top left
		QRect rect(QPoint(topLeft.x(), topLeft.y()-TEXTBOX_HEIGHT), textboxSize) ;
		painter.drawText(rect, Qt::AlignLeft|Qt::AlignBottom, label.c_str()) ;
	}
	else if (spaceTop && (topRight.x() > TEXTBOX_MIN_HSPACE) && (topRight.x() < painter.device()->width())) {
		// top right
		QRect rect(QPoint(topRight.x()-TEXTBOX_WIDTH, topRight.y()-TEXTBOX_HEIGHT), textboxSize) ;
		painter.drawText(rect, Qt::AlignRight|Qt::AlignBottom, label.c_str()) ;
	}
	else if (spaceRight && (painter.device()->height()-rightTop.y() > TEXTBOX_MIN_VSPACE) && (rightTop.y() > 0)) {
		// right top
		QRect rect(rightTop, textboxSize) ;
		painter.drawText(rect, Qt::AlignLeft|Qt::AlignTop, label.c_str()) ;
	}
	else if (spaceRight && (rightBottom.y() > TEXTBOX_MIN_VSPACE) && (rightBottom.y() < painter.device()->height())) {
		// right bottom
		QRect rect(QPoint(rightBottom.x(), rightBottom.y()-TEXTBOX_HEIGHT), textboxSize) ;
		painter.drawText(rect, Qt::AlignLeft|Qt::AlignBottom, label.c_str()) ;
	}
	else if (spaceLeft && (painter.device()->height()-leftTop.y() > TEXTBOX_MIN_VSPACE) && (leftTop.y() > 0)) {
		// left top
		QRect rect(QPoint(leftTop.x()-TEXTBOX_WIDTH, leftTop.y()), textboxSize) ;
		painter.drawText(rect, Qt::AlignRight|Qt::AlignTop, label.c_str()) ;
	}
	else if (spaceRight && (leftBottom.y() > TEXTBOX_MIN_HSPACE) && (leftBottom.y() < painter.device()->height())) {
		// left bottom
		QRect rect(QPoint(leftBottom.x()-TEXTBOX_WIDTH, leftBottom.y()-TEXTBOX_HEIGHT), textboxSize) ;
		painter.drawText(rect, Qt::AlignRight|Qt::AlignBottom, label.c_str()) ;
	}
	else {
		// box either covers the full screen, or is completly out of the canvas
		// todo: draw label inside box
	}
}

void tum::ARSlideRenderer::drawCircleLabel(QPainter& painter, const QPoint& circlePosition, const float radius, const std::string& label, const int offset, const QColor& color) {
	if (label == "") {
		return ;
	}

	int dist = radius+offset ;
	QSize textboxSize(TEXTBOX_WIDTH, TEXTBOX_HEIGHT) ;

	QPoint top   (circlePosition.x(),      circlePosition.y()-dist) ;
	QPoint right (circlePosition.x()+dist, circlePosition.y()) ;
	QPoint bottom(circlePosition.x(),      circlePosition.y()+dist) ;
	QPoint left  (circlePosition.x()-dist, circlePosition.x()) ;

	bool spaceTopH    = 0 < top.x()-TEXTBOX_MIN_HSPACE/2 && top.x()+TEXTBOX_MIN_HSPACE/2 < painter.device()->width() ;
	bool spaceTopV    = top.y() < painter.device()->height() && top.y() > TEXTBOX_MIN_VSPACE ;
	bool spaceRightH  = painter.device()->width()-right.x() < TEXTBOX_MIN_HSPACE && right.x() > 0 ;
	bool spaceRightV  = 0 < right.y()-TEXTBOX_MIN_VSPACE/2 && right.y()+TEXTBOX_MIN_VSPACE/2 < painter.device()->height() ;
	bool spaceBottomH = spaceTopH ;
	bool spaceBottomV = 0 < bottom.y() && bottom.y()+TEXTBOX_MIN_VSPACE < painter.device()->height() ;
	bool spaceLeftH   = TEXTBOX_MIN_HSPACE < left.x() && left.x() < painter.device()->width() ;
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