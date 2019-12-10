#include "SamplePlugin.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <opencv2/stereo.hpp>
#include <opencv2/core/eigen.hpp>
// type defnitions for pcl pose estimation
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

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
      connect(_btn_pose    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );



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
    else if (obj == _btn_pose)
    {
        poseEstimation();
    }
    else if (obj == _performTask)
    {
        performTask();
    }
}

void SamplePlugin::performTask()
{
    //Interpolation from home to place

}

void SamplePlugin::poseEstimation()
{
    // Perform voxel grid / down sampling
    pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr output_cloud (new pcl::PointCloud<pcl::PointNormal>);
     Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();





     pcl::PCDReader reader;
     reader.read ("/home/student/Workspace/RobWork/RobWorkStudio/bin/release/Scanner25D.pcd", *input_cloud); // Remember to download the file first!


 // transform the input_pointcloud to world
     // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
     ; // The angle of rotation in radians
     transform_1 (0,0) = 1;
     transform_1 (0,1) = -0;
     transform_1 (0,2) = 0;
     transform_1 (1,0) = 0;
     transform_1 (1,1) = 0.906308;
     transform_1 (1,2) = 0.422618;
     transform_1 (2,0) = -0;
     transform_1 (2,1) = -0.422618;
     transform_1 (2,2) = 0.906308;
     //    (row, column)

     // Define a translation of 2.5 meters on the x axis.
     transform_1 (0,3) = 0;
     transform_1 (1,3) = 1.033;
     transform_1 (2,3) = 1.325;
     pcl::transformPointCloud (*input_cloud, *input_cloud, transform_1);
        // perform downsampling of object
           double cube_size=0.01f;
            pcl::VoxelGrid<pcl::PointNormal> sor1;
           sor1.setInputCloud (input_cloud);
            sor1.setLeafSize (cube_size, cube_size, cube_size);
            sor1.filter (*output_cloud);


      // Perform outlier removal
        // Create the filtering object
       pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
        sor.setInputCloud (output_cloud);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0f);
        sor.filter (*output_cloud);


   // Perform spatial removal
        pcl::PassThrough<pcl::PointNormal> pass1;
        pass1.setInputCloud (output_cloud);
        pass1.setFilterFieldName ("x");
        pass1.setFilterLimits (-.04f, .04f);
        pass1.filter (*output_cloud);
        pass1.setFilterFieldName ("y");
        pass1.setFilterLimits (0.4f,0.515f);
        pass1.filter (*output_cloud);
        pass1.setFilterFieldName ("z");
        pass1.setFilterLimits (0.01f, 0.3f);
    //    pass.setFilterLimitsNegative (true);
        pass1.filter (*output_cloud);



          PointCloudT::Ptr object_aligned (new PointCloudT);

          FeatureCloudT::Ptr object_features (new FeatureCloudT);
          FeatureCloudT::Ptr scene_features (new FeatureCloudT);




           pcl::PointCloud<pcl::PointNormal>::Ptr scene (new pcl::PointCloud<pcl::PointNormal>);
           pcl::PointCloud<pcl::PointNormal>::Ptr object (new pcl::PointCloud<pcl::PointNormal>);
reader.read ("/home/student/Workspace/RobWork/RobWorkStudio/bin/release/Scanner25D.pcd", *scene);

           pcl::transformPointCloud (*scene, *scene, transform_1);
           // Transform3D<> fTmf = Kinematics::frameTframe(_bottle, _WORLD, _state);

            rw::math::Transform3D<> rwbottleT=_bottle->wTf(_state);
            Eigen::Matrix4f eibottleT = Eigen::Matrix4f::Identity();
            for(int i= 0; i <3; i++){
                for (int j=0; j<4; j++){
                    eibottleT(i,j)=rwbottleT(i,j);

                }

            }
            pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", eibottleT (0,0), eibottleT (0,1), eibottleT (0,2));
            pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", eibottleT (1,0), eibottleT (1,1), eibottleT  (1,2));
            pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", eibottleT (2,0), eibottleT (2,1), eibottleT (2,2));
            pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", eibottleT (0,3), eibottleT (1,3), eibottleT (2,3));


           pcl::transformPointCloud (*object, *object, eibottleT.inverse());

          pcl::PassThrough<pcl::PointNormal> pass;
          pass.setInputCloud (scene);
          pass.setFilterFieldName ("z");
          pass.setFilterLimits (-4.0f, 1.0f);
      //    pass.setFilterLimitsNegative (true);
          pass.filter (*scene);
        // Downsample
         pcl::console::print_highlight ("Downsampling...\n");
         pcl::VoxelGrid<PointNT> grid;
         const float leaf = 0.01f;
         grid.setLeafSize (leaf, leaf, leaf);
         grid.setInputCloud (object);
         grid.filter (*object);
         grid.setInputCloud (scene);
         grid.filter (*scene);

         // Estimate normals for scene
         pcl::console::print_highlight ("Estimating scene normals...\n");
         pcl::NormalEstimationOMP<PointNT,PointNT> nest;
         nest.setRadiusSearch (0.01);
         nest.setInputCloud (scene);
         nest.compute (*scene);
         nest.setInputCloud (object);
         nest.compute (*object);


         // Estimate features
         pcl::console::print_highlight ("Estimating features...\n");
         FeatureEstimationT fest;
         fest.setRadiusSearch (0.025);
         fest.setInputCloud (object);
         fest.setInputNormals (object);
         fest.compute (*object_features);
         fest.setInputCloud (scene);
         fest.setInputNormals (scene);
         fest.compute (*scene_features);
        /* pcl::PCDWriter writer;
          writer.write<pcl::PointNormal> ("../point_cloud_processing_template/cloud_filtered_object.pcd", *object, false);
          writer.write<pcl::PointNormal> ("../point_cloud_processing_template/cloud_filtered_scene.pcd", *scene, false);*/



         // Perform alignment
         pcl::console::print_highlight ("Starting alignment...\n");
         pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
         align.setInputSource (object);
         align.setSourceFeatures (object_features);
         align.setInputTarget (scene);
         align.setTargetFeatures (scene_features);
         align.setMaximumIterations (50000); // Number of RANSAC iterations
         align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
         align.setCorrespondenceRandomness (5); // Number of nearest features to use
         align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
         align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
         align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
         {
           pcl::ScopeTime t("Alignment");
           align.align (*object_aligned);
         }

         if (align.hasConverged ())
         {
           // Print results
           printf ("\n");
           Eigen::Matrix4f transformation = align.getFinalTransformation ();
           pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
           pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
           pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
           pcl::console::print_info ("\n");
           pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
           pcl::console::print_info ("\n");
           pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

           // Show alignment
           pcl::visualization::PCLVisualizer visu("Alignment");
           visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
           visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
           visu.spin ();
         }
         else
         {
           pcl::console::print_error ("Alignment failed!\n");

         }






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
    cv::Mat upperRedRight;
    cv::Mat lowerRedLeft;
    cv::Mat upperRedLeft;

    cv::inRange(hsv_imageLeft, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lowerRedRight);
    cv::inRange(hsv_imageLeft, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upperRedRight);
    cv::inRange(hsv_imageRight, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lowerRedLeft);
    cv::inRange(hsv_imageRight, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upperRedLeft);

    cv::Mat red_hueRight;
    cv::Mat red_hueLeft;
    cv::addWeighted(lowerRedRight, 1.0, upperRedRight, 1.0, 0.0, red_hueRight);
    cv::addWeighted(lowerRedLeft, 1.0, upperRedLeft, 1.0, 0.0, red_hueLeft);

    std::vector<cv::Vec3f> circlesRight;
    std::vector<cv::Vec3f> circlesLeft;
    cv::HoughCircles(red_hueRight, circlesRight, CV_HOUGH_GRADIENT, 1, red_hueRight.rows, 20, 10, 0, 0);
    cv::HoughCircles(red_hueLeft, circlesLeft, CV_HOUGH_GRADIENT, 1, red_hueLeft.rows, 20, 10, 0, 0);

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

    cv::imwrite("/tmp/test1.png", imageRight);
    cv::imwrite("/tmp/test2.png", imageLeft);

    double dif_x = pnts3D.at<double>(0,0) - _bottle->getTransform(_state).P()[0];
    double dif_y = pnts3D.at<double>(0,1) - _bottle->getTransform(_state).P()[1];
    double dif_z = pnts3D.at<double>(0,2) - _bottle->getTransform(_state).P()[2];

    double error = sqrt((dif_x*dif_x) + (dif_y*dif_y) + (dif_z*dif_z));

    std::cout << "The error is: " << error << std::endl;

    _bottleEst->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(pnts3D.at<double>(0,0), pnts3D.at<double>(0,1), pnts3D.at<double>(0,2)), rw::math::RPY<>(0,0,90*Deg2Rad)), _state);
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
