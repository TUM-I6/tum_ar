#include <tum_ar_window/ARInspectionNode.h>

tum::ARInspectionNode::ARInspectionNode(int argc, char *argv[])
: _projector(_nh),
  _renderer(_projector),
  _qApp(argc, argv),
  _actionServer(_nh, "ar_inspection", false) {
	_window.showFullScreen() ;
	_blankSlide.instruction = "" ;
	_slides = loadSlides("") ;

	_userInputSub = _nh.subscribe("user_input", 10, &ARInspectionNode::userInputCallback, this) ;
	_actionServer.registerGoalCallback(boost::bind(&ARInspectionNode::executeARInspection, this));
	_actionServer.start() ;
}

tum::ARInspectionNode::~ARInspectionNode() {
}

void tum::ARInspectionNode::run() {
	//sleep(2) ;
	ros::Rate rate(50) ;

	while(ros::ok() && _window.isVisible()) {
		// qt stuff
		_qApp.processEvents() ;
		QRect canvas = _window.canvasArea() ;
		QPixmap slide ;
		if (_taskActive) {
			//ROS_INFO_STREAM("Task active") ;
			slide = _renderer.renderSlide(_slides[_step], canvas) ;
		}
		else {
			slide = _renderer.renderSlide(_blankSlide, canvas) ;
		}
		_window.display(slide) ;

		// ros stuff
		ros::spinOnce() ;
		_projector.publishViewFrustumMarker(_projector.getImagePlane(2.05f)) ;
		rate.sleep() ;
	}
}

void tum::ARInspectionNode::executeARInspection() { // const tum_ar_window::ARInspectionGoalConstPtr &goal
	if (_taskActive) {
		// todo: cancel old goal
	}

	tum_ar_window::ARInspectionGoalConstPtr goal = _actionServer.acceptNewGoal() ;
	_step = 0 ;

	// publish info to the console for the user
	ROS_INFO_STREAM("[tum_ar_window] Running AR inspection based on "<<_slides.size()<<" slides") ;
}

std::vector<tum_ar_window::ARSlide> tum::ARInspectionNode::loadSlides(const std::string& file) {
	std::vector<tum_ar_window::ARSlide> slides ;

	std::function<tum_ar_window::POI (int, int, int, const std::string&)> poi = [](int x, int y, int radius, const std::string& label="") {
		tum_ar_window::POI poi ;
		poi.position.x = x ;
		poi.position.y = y ;
		poi.position.z = 0 ;
		poi.border_color.r = 1 ;
		poi.border_color.g = 0 ;
		poi.border_color.b = 0 ;
		poi.border_color.a = 1 ;
		poi.fill_color.r = 1 ;
		poi.fill_color.g = 1 ;
		poi.fill_color.b = 1 ;
		poi.fill_color.a = 0.9 ;
		poi.radius = radius ;
		poi.label = label ;
		return poi ;
	} ;

	std::function<tum_ar_window::Box (int, int, int, int, const std::string&)> box = [](int x, int y, int w, int h, const std::string& label="") {
		tum_ar_window::Box box ;
		box.position.x = x ;
		box.position.y = y ;
		box.position.z = 0 ;
		box.border_color.r = 0 ;
		box.border_color.g = 0 ;
		box.border_color.b = 1 ;
		box.border_color.a = 1 ;
		box.width = w ;
		box.height = h ;
		box.label = label ;
		return box ;
	} ;

	tum_ar_window::ARSlide slide1 ;
	slide1.instruction = "Slide 1" ;
	slide1.pois.push_back(poi( 500, 500,  50, "label")) ;
	slide1.pois.push_back(poi( 600, 700,  10, "another label")) ;
	slide1.pois.push_back(poi( 800, 400, 100, "")) ;
	slide1.boxes.push_back(box( 200,-100, 300,400, "Fancy Label")) ;
	slide1.boxes.push_back(box( 200,1000, 300,300, "Fancy Label")) ;
	slide1.boxes.push_back(box(1500, 200, 600,200, "Fancy Label")) ;

	tum_ar_window::ARSlide slide2 ;
	slide2.instruction = "Slide 2" ;
	slide2.pois.push_back(poi(1500, 600,  20, "")) ;
	slide2.pois.push_back(poi(1000, 600, 150, "")) ;
	slide2.pois.push_back(poi( 700,1100, 200, "moved label")) ;
	slide2.boxes.push_back(box( -10, 600, 200,300, "Fancy Label")) ;
	slide2.boxes.push_back(box(1400, 700, 500,500, "Fancy Label")) ;

	slides.push_back(slide1) ;
	slides.push_back(slide2) ;

	return slides ;
}

void tum::ARInspectionNode::userInputCallback(const tum_ar_window::InspectionResult::ConstPtr& msg) {
	if(!_taskActive) {
		return ;
	}

	publishFeedback(_step, msg->status) ;

	switch(msg->status) {
		case tum_ar_window::InspectionResult::ACCEPTED : {
			_step++ ;
			if (_step >= _slides.size()) {
				tum_ar_window::ARInspectionResult result ;
				result.status = tum_ar_window::ARInspectionResult::ACCEPTED ;
				_actionServer.setSucceeded(result);

				_taskActive = false ;
				_step = 0 ;
			}
			break ;
		}
		case tum_ar_window::InspectionResult::REJECTED :
			// todo implement different procedures
		case tum_ar_window::InspectionResult::REWORK : {
			tum_ar_window::ARInspectionResult result ;
			result.status = tum_ar_window::ARInspectionResult::REJECTED ;
			_actionServer.setSucceeded(result);
			
			_taskActive = false ;
			_step = 0 ;
			break ;
		}
		default: {
			ROS_ERROR_STREAM("[ARInspectionNode] User input message contains invalid status!") ;
			break ;
		}
	}
}