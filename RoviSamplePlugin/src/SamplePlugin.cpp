#include "SamplePlugin.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <opencv2/stereo.hpp>
#include <opencv2/core/eigen.hpp>

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	_timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn_im    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_scan    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
        connect(_btn_home, SIGNAL(pressed()), this, SLOT(btnPressed()) );
        connect(_performTask  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );
        connect(_btn_place    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
        connect(_btn_sparse    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

	_framegrabber = NULL;
	
	_cameras = {"Camera_Right", "Camera_Left"};
	_cameras25D = {"Scanner25D"};
}

SamplePlugin::~SamplePlugin()
{
    delete _textureRender;
    delete _bgRender;
}

void SamplePlugin::initialize()
{
    log().info() << "INITALIZE" << "\n";

    getRobWorkStudio()->stateChangedEvent().add(std::bind(&SamplePlugin::stateChangedListener, this, std::placeholders::_1), this);

	// Auto load workcell
    WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/martin/RoVi1/Project_WorkCell/Scene.wc.xml");
	getRobWorkStudio()->setWorkCell(wc);
}

void SamplePlugin::open(WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
    _wc = workcell;
    _state = _wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

    if (_wc != NULL) {
	// Add the texture render to this workcell if there is a frame for texture
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
        if (textureFrame != NULL)
        {
		getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
	}
	// Add the background render to this workcell if there is a frame for texture
	Frame* bgFrame = _wc->findFrame("Background");
        if (bgFrame != NULL)
        {
		getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
	}

	// Create a GLFrameGrabber if there is a camera frame with a Camera property set
	Frame* cameraFrame = _wc->findFrame(_cameras[0]);
        if (cameraFrame != NULL)
        {
                if (cameraFrame->getPropertyMap().has("Camera"))
                {
			// Read the dimensions and field of view
			double fovy;
			int width,height;
			std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
			std::istringstream iss (camParam, std::istringstream::in);
			iss >> fovy >> width >> height;
			// Create a frame grabber
			_framegrabber = new GLFrameGrabber(width,height,fovy);
			SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
			_framegrabber->init(gldrawer);
		}
	}
	
	Frame* cameraFrame25D = _wc->findFrame(_cameras25D[0]);
        if (cameraFrame25D != NULL)
        {
                if (cameraFrame25D->getPropertyMap().has("Scanner25D"))
                {
			// Read the dimensions and field of view
			double fovy;
			int width,height;
			std::string camParam = cameraFrame25D->getPropertyMap().get<std::string>("Scanner25D");
			std::istringstream iss (camParam, std::istringstream::in);
			iss >> fovy >> width >> height;
			// Create a frame grabber
			_framegrabber25D = new GLFrameGrabber25D(width,height,fovy);
			SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
			_framegrabber25D->init(gldrawer);
		}
	}
    _device = _wc->findDevice("UR-6-85-5-A");
    _bottle = _wc->findFrame<rw::kinematics::MovableFrame>("Bottle");
    _bottleEst = _wc->findFrame<rw::kinematics::MovableFrame>("BottleEst");
    _step = -1;
    }
}

void SamplePlugin::close()
{
    log().info() << "CLOSE" << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}
	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}
	// Delete the old framegrabber
	if (_framegrabber != NULL) {
		delete _framegrabber;
	}
	_framegrabber = NULL;
	_wc = NULL;
}

Mat SamplePlugin::toOpenCVImage(const Image& img)
{
	Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
	res.data = (uchar*)img.getImageData();
	return res;
}


void SamplePlugin::btnPressed()
{
    QObject *obj = sender();
    if(obj==_btn0)
    {
        _timer->stop();
        rw::math::Math::seed();
        double extend = 0.05;
        double maxTime = 60;
        Q from(6, 1.571, -1.572, -1.572, -1.572, 1.571, 0);
        Q to(6, 1.847, -2.465, -1.602, -0.647, 1.571, 0); //From pose estimation
        createPathRRTConnect(from, to, extend, maxTime);
    }
    else if(obj==_btn1)
    {
        log().info() << "Button 1\n";
        // Toggle the timer on and off
        if (!_timer->isActive()){
            _timer->start(100); // run 10 Hz
            _step = 0;
        }
        else
            _step = 0;
    }
    else if( obj==_btn_im ){
            getImage();
    }
    else if( obj==_btn_scan )
    {
            get25DImage();
    }
    else if (obj == _btn_home)
    {
        homePosition();
    }
    else if (obj == _btn_place)
    {
        placeBottle();
    }
    else if (obj == _btn_sparse)
    {
        sparseStereo();
    }
    else if (obj == _performTask)
    {
        performTask();
    }
}

void SamplePlugin::performTask()
{
    _path.clear();

    //Interpolation from home to place
    std::vector<rw::math::Q> qs;
    qs.resize(6);
    qs.at(0) = Q(6,  1.607, -1.903, -2.101, -2.277, -2.529, 0.001);
    qs.at(1) = Q(6,  1.368, -1.554, -1.590, -2.062,  -1.869, -0.059);
    qs.at(2) = Q(6,  0.868, -1.927, -1.814, -2.297,  -1.205, -0.101);
    qs.at(3) = Q(6, -0.030, -2.066, -1.494, -2.368, 0.312, -0.017);
    qs.at(4) = Q(6, -0.778, -2.228, -1.209, -2.381, 1.731, 0.020);
    qs.at(5) = Q(6, -1.308, -2.315, -1.389, -2.581, 2.511, -0.001);

    double t = 1;
    unsigned int t_res = 100;

    std::vector<rw::trajectory::LinearInterpolator<rw::math::Q>> ls;

    for (unsigned int i = 0; i < qs.size()-1; i++)
    {
        ls.emplace_back(qs.at(i), qs.at(i+1), t);
    }

    /*
    for (unsigned int i = 0; i < ls.size(); i++)
    {
        for (unsigned int j = 0; j < t_res; j++)
        {
            _device->setQ(ls.at(i).x(t*j/t_res), _state);
            _path.push_back(t*i+t*j/t_res, _state);
        }
    }
    */
}

void SamplePlugin::sparseStereo()
{
    getImage();
    Mat imageLeft = cv::imread("/tmp/Camera_Left.png");
    Mat imageRight = cv::imread("/tmp/Camera_Right.png");

    cv::Mat hsv_imageRight;
    cv::Mat hsv_imageLeft;
    cv::cvtColor(imageLeft, hsv_imageRight, cv::COLOR_BGR2HSV);
    cv::cvtColor(imageRight, hsv_imageLeft, cv::COLOR_BGR2HSV);

    cv::Mat lowerRedRight;
    cv::Mat lowerRedLeft;

    cv::inRange(hsv_imageLeft, cv::Scalar(100,50,50), cv::Scalar(110, 255, 255), lowerRedRight);
    cv::inRange(hsv_imageRight, cv::Scalar(100, 50, 50), cv::Scalar(110, 255, 255), lowerRedLeft);

    std::vector<cv::Vec3f> circlesRight;
    std::vector<cv::Vec3f> circlesLeft;
    cv::HoughCircles(lowerRedRight, circlesRight, CV_HOUGH_GRADIENT, 1, lowerRedRight.rows, 20, 10, 0, 15);
    cv::HoughCircles(lowerRedLeft, circlesLeft, CV_HOUGH_GRADIENT, 1, lowerRedLeft.rows, 20, 10, 0, 15);

    std::vector<cv::Point2d> CenterPointRight;
    std::vector<cv::Point2d> CenterPointLeft;

    for(size_t current_circle = 0; current_circle < circlesRight.size(); ++current_circle)
    {
            cv::Point center(std::round(circlesRight[current_circle][0]-1), std::round(circlesRight[current_circle][1]));
            int radius = std::round(circlesRight[current_circle][2]);

            CenterPointRight.push_back(center);

            cv::circle(imageRight, center, radius, cv::Scalar(0, 255, 0), 3);
    }

    for(size_t current_circle = 0; current_circle < circlesLeft.size(); ++current_circle)
    {
            cv::Point center(std::round(circlesLeft[current_circle][0]-1), std::round(circlesLeft[current_circle][1]));
            int radius = std::round(circlesLeft[current_circle][2]);

            CenterPointLeft.push_back(center);

            cv::circle(imageLeft, center, radius, cv::Scalar(0, 255, 0), 3);
    }

    //Vector of Eigenmatrix for both the right and left camera
    Eigen::Matrix<double, 3, 4> projectMatrixRight = ProjectionMatrix("Camera_Right");
    Eigen::Matrix<double, 3, 4> projectMatrixLeft = ProjectionMatrix("Camera_Left");

    cv::Mat cam0(3, 4, CV_64F);
    cv::Mat cam1(3, 4, CV_64F);

    cv::eigen2cv(projectMatrixRight, cam0);
    cv::eigen2cv(projectMatrixLeft, cam1);

    cv::Mat pnts3D(4,CenterPointRight.size(),CV_64F);
    cv::triangulatePoints(cam1, cam0, CenterPointLeft, CenterPointRight, pnts3D);

    double dif_x = pnts3D.at<double>(0,0)/pnts3D.at<double>(0,3) - _bottle->getTransform(_state).P()[0];
    double dif_y = pnts3D.at<double>(0,1)/pnts3D.at<double>(0,3) - _bottle->getTransform(_state).P()[1];

    double error = sqrt((dif_x*dif_x) + (dif_y*dif_y));

    std::cout << "The error is: " << error << std::endl;

    _bottleEst->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(pnts3D.at<double>(0,0)/pnts3D.at<double>(0,3), pnts3D.at<double>(0,1)/pnts3D.at<double>(0,3), 0.21), rw::math::RPY<>(0,0,90*Deg2Rad)), _state);
    getRobWorkStudio()->setState(_state);
}

Eigen::Matrix<double, 3, 4> SamplePlugin::ProjectionMatrix(std::string frameName)
{
    Frame* cameraFrame = _wc->findFrame(frameName);
    // Read the dimensions and field of view
    double fovy;
    int width,height;
    std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
    std::istringstream iss (camParam, std::istringstream::in);
    iss >> fovy >> width >> height;

    double fovy_pixel = height / 2 / tan(fovy * (2*M_PI) / 360.0 / 2.0 );

    Eigen::Matrix<double, 3, 4> KA;
    KA << fovy_pixel, 0, width / 2.0, 0,
          0, fovy_pixel, height / 2.0, 0,
          0, 0, 1, 0;

    Transform3D<> camPosOGL = cameraFrame->wTf(_state);
    Transform3D<> openGLToVis = Transform3D<>(RPY<>(-Pi, 0, Pi).toRotation3D());
    Transform3D<> H = inverse(camPosOGL * inverse(openGLToVis));

    Eigen::Matrix<double, 4, 4> He;
    He = H.e();

    Eigen::Matrix<double, 3, 4> P = KA* He;

    return P;
}

void SamplePlugin::placeBottle()
{
    std::mt19937 engine(rd());

    std::uniform_int_distribution<> x_rand(-350, 350);
    std::uniform_int_distribution<> y_rand(350, 550);

    double x_val = x_rand(engine) / 1000.0;
    double y_val = y_rand(engine) / 1000.0;

    _bottle->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(x_val, y_val, 0.21), rw::math::RPY<>(0,0,90*Deg2Rad)), _state);
    getRobWorkStudio()->setState(_state);
}

void SamplePlugin::homePosition()
{
    rw::math::Q qHome = Q(6,  1.202, -1.770, -1.396, -0.972, 1.296, 0);
    rw::math::Q qs = _device->getQ(_state);
    createPathRRTConnect(qs, qHome, 0.05, 60);

    // Toggle the timer on and off
    if (!_timer->isActive()){
        _timer->start(100); // run 10 Hz
        _step = 0;
    }
    else
        _step = 0;
}

void SamplePlugin::get25DImage()
{
        if (_framegrabber25D != NULL)
        {
                for(size_t i = 0; i < _cameras25D.size(); i ++)
		{
			// Get the image as a RW image
			Frame* cameraFrame25D = _wc->findFrame(_cameras25D[i]); // "Camera");
			_framegrabber25D->grab(cameraFrame25D, _state);

			//const Image& image = _framegrabber->getImage();

			const rw::geometry::PointCloud* img = &(_framegrabber25D->getImage());

			std::ofstream output(_cameras25D[i] + ".pcd");
			output << "# .PCD v.5 - Point Cloud Data file format\n";
			output << "FIELDS x y z\n";
			output << "SIZE 4 4 4\n";
			output << "TYPE F F F\n";
			output << "WIDTH " << img->getWidth() << "\n";
			output << "HEIGHT " << img->getHeight() << "\n";
			output << "POINTS " << img->getData().size() << "\n";
			output << "DATA ascii\n";
            for(const auto &p_tmp : img->getData())
			{
				rw::math::Vector3D<float> p = p_tmp;
				output << p(0) << " " << p(1) << " " << p(2) << "\n";
			}
			output.close();

		}
	}
}

void SamplePlugin::getImage()
{
	if (_framegrabber != NULL) {
                for(size_t i = 0; i < _cameras.size(); i ++)
		{
			// Get the image as a RW image
			Frame* cameraFrame = _wc->findFrame(_cameras[i]); // "Camera");
			_framegrabber->grab(cameraFrame, _state);

			const rw::sensor::Image* rw_image = &(_framegrabber->getImage());

			// Convert to OpenCV matrix.
			cv::Mat image = cv::Mat(rw_image->getHeight(), rw_image->getWidth(), CV_8UC3, (rw::sensor::Image*)rw_image->getImageData());

			// Convert to OpenCV image
			Mat imflip, imflip_mat;
			cv::flip(image, imflip, 1);
                        cv::cvtColor(imflip, imflip_mat, COLOR_RGB2BGR );

                        cv::imwrite("/tmp/" +_cameras[i] + ".png", imflip_mat );

			// Show in QLabel
			QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
			QPixmap p = QPixmap::fromImage(img);
			unsigned int maxW = 480;
			unsigned int maxH = 640;
			_label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
		}
	}
}

void SamplePlugin::timer()
{
    if(0 <= _step && _step < _path.size())
    {
        _device->setQ(_path.at(_step),_state);
        getRobWorkStudio()->setState(_state);
        _step++;
    }
}

void SamplePlugin::stateChangedListener(const State& state) {
  _state = state;
}

bool SamplePlugin::checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q)
{
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ(q,testState);
    colFrom = detector.inCollision(testState,&data);
    if (colFrom) {
        cerr << "Configuration in collision: " << q << endl;
        cerr << "Colliding frames: " << endl;
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
            cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
        }
        return false;
    }
    return true;
}

void SamplePlugin::createPathRRTConnect(Q from, Q to,  double extend, double maxTime)
{
    _device->setQ(from,_state);
    getRobWorkStudio()->setState(_state);
    CollisionDetector detector(_wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,_device,_state);
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(_device),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

    _path.clear();
    if (!checkCollisions(_device, _state, detector, from))
        cout << from << " is in colission!" << endl;
    if (!checkCollisions(_device, _state, detector, to))
        cout << to << " is in colission!" << endl;;
    Timer t;
    t.resetAndResume();
    planner->query(from,to,_path,maxTime);
    t.pause();

    if (t.getTime() >= maxTime) {
        cout << "Notice: max time of " << maxTime << " seconds reached." << endl;
    }
	const int duration = 10;

    if(_path.size() == 2){  //The interpolated path between Q start and Q goal is collision free. Set the duration with respect to the desired velocity
        LinearInterpolator<Q> linInt(from, to, duration);
        QPath tempQ;
        for(int i = 0; i < duration+1; i++){
            tempQ.push_back(linInt.x(i));
        }

        _path=tempQ;
    }
}
