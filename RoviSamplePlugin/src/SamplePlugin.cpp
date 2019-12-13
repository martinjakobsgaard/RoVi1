#include "SamplePlugin.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <opencv2/stereo.hpp>
#include <opencv2/core/eigen.hpp>

#include <rw/invkin.hpp>

#include <thread>
#include <chrono>

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
    //connect(_btn_im    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    //connect(_btn_scan    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    //connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    //connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn_home, SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn_performTask  ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn_place    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn_sparse    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn_sparse_test    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn_pose    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn_performTaskTop    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
     connect(_btn_pose_test    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

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
    _bottleEstTop = _wc->findFrame<rw::kinematics::MovableFrame>("BottleEstTop");
    _step = -1;

    // https://stackoverflow.com/questions/17032970/clear-data-inside-text-file-in-c
    // Delete contents if it already exists when opening the workcell.
    std::ofstream myfile;
    myfile.open("/tmp/sparse_test.DAT", std::ofstream::out | std::ofstream::trunc);
    myfile << "Error" << "\n";
    myfile.close();
    std::ofstream myfile2;
    myfile2.open("/tmp/pose_test.DAT", std::ofstream::out | std::ofstream::trunc);
    myfile2 << "Error" << "\n";
    myfile2.close();

    }
    ScanForBottle();
    FilterObject();
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

    if (obj == _btn_home)
    {
        homePosition();
    }
    else if (obj == _btn_place)
    {
        placeBottle();
    }
    else if (obj == _btn_sparse)
    {
        sparse_test = false;
        std_gaussian = 0.f;
        sparseStereo();
    }
    else if (obj == _btn_sparse_test)
    {
        sparse_test = true;
        std_gaussian = 15.f;
        for (size_t i = 0; i < 25; i++)
        {
            placeBottle();
            sparseStereo();
        }
    }
    else if (obj == _btn_pose)
    {
        pose_test = false;
        get25DImage();
        poseEstimation();
    }
    else if (obj == _btn_pose_test)
    {
        for (size_t i = 0; i < 25; i++)
        {
            pose_test = true;
            placeBottle();
            get25DImage();
            poseEstimation();
        }
    }
    else if (obj == _btn_performTask)
    {
        performTask("BottleEst");
    }
    else if (obj == _btn_performTaskTop)
    {
        performTask("BottleEstTop");
    }
}

//  Function taken from reachability for this project which is based on the solution from reachability analysis.
bool SamplePlugin::reachabilityCheck(std::string approach)
{
    State state_clone = _state;
    std::vector<rw::math::Q> collisionFreeSolutions;

    CollisionDetector detector(_wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());

    collisionFreeSolutions.clear();
    for(double rollAngle=0; rollAngle<360.0; rollAngle+=5.0)
    {
        _bottleEst->moveTo(rw::math::Transform3D<>(
                                        rw::math::Vector3D<>(_bottleEst->getTransform(state_clone).P()),
                                        rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,1.57)), state_clone);

        std::vector<rw::math::Q> solutions = getConfigurations(approach, "GraspTCP", state_clone);

        for(unsigned int i=0; i<solutions.size(); i++)
        {
            // set the robot in that configuration and check if it is in collision
            if(checkCollisions(_device, state_clone, detector, solutions[i]))
            {
                std::cout << "The solution found is: " << solutions[i] << std::endl;
                QbottleEst = solutions[i];
                return true;
            }
        }
    }
    return false;
}

//  Function taken from reachability which is based on the solution from reachability analysis.
std::vector<rw::math::Q> SamplePlugin::getConfigurations(const std::string nameGoal, const std::string nameTcp, State state_clone)
{
    rw::models::SerialDevice::Ptr robotUR5 = _wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");

    // Get, make and print name of frames
    const std::string robotName = robotUR5->getName();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp = robotName + "." + "TCP";

    // Find frames and check for existence
    rw::kinematics::Frame* frameGoal = _wc->findFrame(nameGoal);
    rw::kinematics::Frame* frameTcp = _wc->findFrame(nameTcp);
    rw::kinematics::Frame* frameRobotBase = _wc->findFrame(nameRobotBase);
    rw::kinematics::Frame* frameRobotTcp = _wc->findFrame(nameRobotTcp);
    if(frameGoal==NULL || frameTcp==NULL || frameRobotBase==NULL || frameRobotTcp==NULL)
    {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (frameGoal==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (frameTcp==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotBase << "\": " << (frameRobotBase==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==NULL ? "NO!" : "YES!") << std::endl;
    }

    // Make "helper" transformations
    rw::math::Transform3D<> frameBaseTGoal = rw::kinematics::Kinematics::frameTframe(frameRobotBase, frameGoal, state_clone);
    rw::math::Transform3D<> frameTcpTRobotTcp = rw::kinematics::Kinematics::frameTframe(frameTcp, frameRobotTcp, state_clone);

    // get grasp frame in robot tool frame
    rw::math::Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR(robotUR5, state_clone));
    return closedFormSovler->solve(targetAt, state_clone);
}

void SamplePlugin::timerStart()
{
    if (!_timer->isActive()){
        _timer->start(100);
        _step = 0;
    }
    else
        _step = 0;
}

void SamplePlugin::performTask(std::string approach)
{
    double extend = 0.3;
    double maxTime = 60;

    rw::math::Q Qpos = _device->getQ(_state);
    createPathRRTConnect(Qpos, Qhome, extend, maxTime);

    std::cout << "Going to Qhome: " << Qhome << std::endl;

    reachabilityCheck(approach);
    createPathRRTConnect(Qhome, QbottleEst, extend, maxTime);

    std::cout << "Going to QbottleEst: " << QbottleEst << std::endl;

    //_bottle->attachTo(_device.get(), _state);

    createPathRRTConnect(QbottleEst, Qhome, extend, maxTime);

    std::cout << "Going to Qhome: " << Qhome << std::endl;

    if (approach == "BottleEst")
    {
        Q Qgoal(6, -1.308, -2.315, -1.389, -2.581, 2.511, -0.001);
        createPathRRTConnect(Qhome, Qgoal, extend, maxTime);
        std::cout << "Going to Qgoal: " << Qgoal << std::endl;
        createPathRRTConnect(Qgoal, Qhome, extend, maxTime);
        std::cout << "Going to Qhome: " << Qpos << std::endl;
    }
    else
    {
        Q QgoalTop(6,-0.930, -2.036, -1.330, -4.488, -1.571, 0.584);
        createPathRRTConnect(Qhome, QgoalTop, extend, maxTime);
        std::cout << "Going to Qgoal: " << QgoalTop << std::endl;
        createPathRRTConnect(QgoalTop, Qhome, extend, maxTime);
        std::cout << "Going to Qhome: " << Qpos << std::endl;
    }

    timerStart();

    /*
    _bottle->setTransform(
            rw::math::inverse(_table->wTf(_state)) * _bottle->wTf(_state),_state);
    _bottle->attachTo(_table.get(), _state);
    */
}

void SamplePlugin::ScanForBottle()
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

            std::ofstream output("/tmp/Initial" +_cameras25D[i] + ".pcd");
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

void SamplePlugin::FilterObject()
{
// Perform voxel grid / down sampling
pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud (new pcl::PointCloud<pcl::PointNormal>);
pcl::PointCloud<pcl::PointNormal>::Ptr output_cloud (new pcl::PointCloud<pcl::PointNormal>);
 Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();





 pcl::PCDReader reader;
  pcl::PCDWriter writer;
 reader.read ("/tmp/InitialScanner25D.pcd", *input_cloud); // Remember to download the file first!



 transform_1 (0,0) = 1;
 transform_1 (0,1) = -0;
 transform_1 (0,2) = 0;
 transform_1 (1,0) = 0;
 transform_1 (1,1) = 0.906308;
 transform_1 (1,2) = 0.422618;
 transform_1 (2,0) = -0;
 transform_1 (2,1) = -0.422618;
 transform_1 (2,2) = 0.906308;

 transform_1 (0,3) = 0;
 transform_1 (1,3) = 1.033;
 transform_1 (2,3) = 1.325;

 pcl::transformPointCloud (*input_cloud, *input_cloud, transform_1);
 //writer.write<pcl::PointNormal> ("/home/student/Workspace/RobWork/RobWorkStudio/bin/release/cloud_transformation_to_world.pcd", *input_cloud, false);
    // perform downsampling of object
 // Downsample
  pcl::console::print_highlight ("Downsampling...\n");
       double cube_size=0.01f;
        pcl::VoxelGrid<pcl::PointNormal> sor1;
       sor1.setInputCloud (input_cloud);
        sor1.setLeafSize (cube_size, cube_size, cube_size);
        sor1.filter (*output_cloud);
        std::cout<<"Downsampling contains: "<<output_cloud->size()<< "Points"<<std::endl;


  // Perform outlier removal
    // Create the filtering object
   pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
    sor.setInputCloud (output_cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0f);
    sor.filter (*output_cloud);
    std::cout<<"outlier removal contains: "<<output_cloud->size()<< "Points"<<std::endl;


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
    std::cout<<"Spatial removal contains: "<<output_cloud->size()<< "Points"<<std::endl;
    rw::math::Transform3D<> rwbottleT=_bottle->wTf(_state);
    Eigen::Matrix4f eibottleT = Eigen::Matrix4f::Identity();
    for(int i= 0; i <3; i++){
        for (int j=0; j<4; j++){
            eibottleT(i,j)=rwbottleT(i,j);

        }

    }
//writer.write<pcl::PointNormal> ("/home/student/Workspace/RobWork/RobWorkStudio/bin/release/Filtratet_object_beforeT.pcd", *output_cloud, false);

   pcl::transformPointCloud (*output_cloud, *output_cloud, eibottleT.inverse());

writer.write<pcl::PointNormal> ("/tmp/Object_Filtered.pcd", *output_cloud, false);
}
void SamplePlugin::poseEstimation()
{

     pcl::PCDReader reader;
      pcl::PCDWriter writer;


            PointCloudT::Ptr object_aligned (new PointCloudT);
        FeatureCloudT::Ptr object_features (new FeatureCloudT);
        FeatureCloudT::Ptr scene_features (new FeatureCloudT);

           pcl::PointCloud<pcl::PointNormal>::Ptr scene (new pcl::PointCloud<pcl::PointNormal>);
           pcl::PointCloud<pcl::PointNormal>::Ptr object (new pcl::PointCloud<pcl::PointNormal>);
            reader.read ("/tmp/Scanner25D.pcd", *scene);
            reader.read ("/tmp/Object_Filtered.pcd", *object);
            Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();



           transform_1 (0,0) = 1;
           transform_1 (0,1) = -0;
           transform_1 (0,2) = 0;
           transform_1 (1,0) = 0;
           transform_1 (1,1) = 0.906308;
           transform_1 (1,2) = 0.422618;
           transform_1 (2,0) = -0;
           transform_1 (2,1) = -0.422618;
           transform_1 (2,2) = 0.906308;

           transform_1 (0,3) = 0;
           transform_1 (1,3) = 1.033;
           transform_1 (2,3) = 1.325;

pcl::transformPointCloud (*scene, *scene, transform_1);

          // writer.write<pcl::PointNormal> ("/home/student/Workspace/RobWork/RobWorkStudio/bin/release/Filtratet_object1.pcd", *object, false);


       // writer.write<pcl::PointNormal> ("/home/student/Workspace/RobWork/RobWorkStudio/bin/release/scene_transformation.pcd", *scene, false);

          pcl::PassThrough<pcl::PointNormal> pass;
          pass.setInputCloud (scene);
          pass.setFilterFieldName ("z");
          pass.setFilterLimits (0.01f, 0.3f);
      //    pass.setFilterLimitsNegative (true);
          pass.filter (*scene);
          pass.setFilterFieldName ("x");
          pass.setFilterLimits (-0.4f, 0.4f);
      //    pass.setFilterLimitsNegative (true);
          pass.filter (*scene);
          pass.setFilterFieldName ("y");
          pass.setFilterLimits (0.3f, 0.6f);
      //    pass.setFilterLimitsNegative (true);
          pass.filter (*scene);

//writer.write<pcl::PointNormal> ("/home/student/Workspace/RobWork/RobWorkStudio/bin/release/scene_pickandplace.pcd", *scene, false);

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

//writer.write<pcl::PointNormal> ("/home/student/Workspace/RobWork/RobWorkStudio/bin/release/object_beforeA.pcd", *object, false);
//writer.write<pcl::PointNormal> ("/home/student/Workspace/RobWork/RobWorkStudio/bin/release/scene_beforeA.pcd", *scene, false);
         // Perform alignment
        double cube_size=0.01f;
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
         align.setMaxCorrespondenceDistance (2.5f * cube_size); // Inlier threshold
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
            cout << transformation << endl;
           pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
//writer.write<pcl::PointNormal> ("/home/student/Workspace/RobWork/RobWorkStudio/bin/release/Object_afterA.pcd", *object, false);
//riter.write<pcl::PointNormal> ("/home/student/Workspace/RobWork/RobWorkStudio/bin/release/Scene_afterA.pcd", *scene, false);


 //###################################################################################
            pcl::transformPointCloud (*scene, *scene, transformation.inverse());
           // writer.write<pcl::PointNormal> ("/home/student/Workspace/RobWork/RobWorkStudio/bin/release/cloud_filtered_sceneIcp.pcd", *scene, false);

           pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
             icp.setInputSource(object);
             icp.setInputTarget(scene);
             pcl::PointCloud<pcl::PointNormal> Final;
             icp.align(Final);
             std::cout << "has converged:" << icp.hasConverged() << " score: " <<
             icp.getFitnessScore() << std::endl;
             std::cout << icp.getFinalTransformation() << std::endl;
             Eigen::Matrix4f FinalT =transformation*icp.getFinalTransformation();
             cout << endl;
             cout << FinalT << endl;
           // Show alignment
           /*pcl::visualization::PCLVisualizer visu("Alignment");
           visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
           visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
           //visu.addCoordinateSystem(1.0,);
           visu.spin ();*/
           // writer.write<pcl::PointNormal> ("/home/student/Workspace/RobWork/RobWorkStudio/bin/release/scene_aligned.pcd", *scene, false);
          //  writer.write<pcl::PointNormal> ("/home/student/Workspace/RobWork/RobWorkStudio/bin/release/object_aligned.pcd", *object_aligned, false);
           //  writer.write<pcl::PointNormal> ("/home/student/Workspace/RobWork/RobWorkStudio/bin/release/cloud_filtered_icp.pcd", Final, false);
             rw::math::Transform3D<> rwbottleT=_bottle->wTf(_state);
             Eigen::Matrix4f eibottleT = Eigen::Matrix4f::Identity();
             for(int i= 0; i <3; i++){
                 for (int j=0; j<4; j++){
                     eibottleT(i,j)=rwbottleT(i,j);

                 }

             }
           double   dif_x = FinalT(0,3) - eibottleT(0,3);//_bottle->getTransform(_state).P()[0];
              double dif_y =FinalT(1,3) - eibottleT(1,3);//_bottle->getTransform(_state).P()[1];
            double  dif_z = FinalT(2,3) - eibottleT(2,3);//_bottle->getTransform(_state).P()[2];

            cout << "x: " << _bottle->getTransform(_state).P()[0] << "y: " << _bottle->getTransform(_state).P()[1] << "z: " << _bottle->getTransform(_state).P()[2] << endl;

             double error = (sqrt((dif_x*dif_x) + (dif_y*dif_y) + (dif_z*dif_z)))*1000;
             std::cout << "The error is: " << error << "mm"<< std::endl;
             _bottleEst->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(FinalT(0,0), FinalT(1,3), FinalT(2,3)), rw::math::RPY<>(0,0,90*Deg2Rad)), _state);
             getRobWorkStudio()->setState(_state);

            if (pose_test == true)
            {
                if (error < 0.5)
                {
                    std::ofstream myfile2;
                    myfile2.open ("/tmp/pose_test.DAT", ios::app);
                    myfile2 << error << "\n";
                    myfile2.close();
                    std::cout << "Error found to be: " << error << std::endl;
                }
            }


         }


         else
         {
           pcl::console::print_error ("Alignment failed!\n");

         }






}

// Function inspired by both https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/ and
// opencv HoughCircles documentation.
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
    cv::inRange(hsv_imageLeft, cv::Scalar(85,50,50), cv::Scalar(125, 255, 255), lowerRedRight);
    cv::inRange(hsv_imageRight, cv::Scalar(85, 50, 50), cv::Scalar(125, 255, 255), lowerRedLeft);

    // https://www.opencv-srf.com/2018/03/gaussian-blur.html
    cv::GaussianBlur(lowerRedRight,lowerRedRight,Size(7,7), 0);
    cv::GaussianBlur(lowerRedLeft,lowerRedLeft,Size(7,7), 0);

    cv::imwrite("/tmp/blur_noise_10.png", lowerRedRight);

    // Values found through trial and error.
    std::vector<cv::Vec3f> circlesRight;
    std::vector<cv::Vec3f> circlesLeft;
    cv::HoughCircles(lowerRedRight, circlesRight, CV_HOUGH_GRADIENT, 1, lowerRedRight.rows, 20, 10, 5, 15);
    cv::HoughCircles(lowerRedLeft, circlesLeft, CV_HOUGH_GRADIENT, 1, lowerRedLeft.rows, 20, 10, 5, 15);

    std::vector<cv::Point2d> CenterPointRight;
    std::vector<cv::Point2d> CenterPointLeft;

    if ((circlesRight.size() != 0) && (circlesLeft.size() != 0))
    {
        for(size_t current_circle = 0; current_circle < 1; ++current_circle)
        {
                cv::Point center(std::round(circlesRight[current_circle][0]-1), std::round(circlesRight[current_circle][1]));
                int radius = std::round(circlesRight[current_circle][2]);

                CenterPointRight.push_back(center);

                cv::circle(imageRight, center, radius, cv::Scalar(0, 0, 150), 3);
        }

        for(size_t current_circle = 0; current_circle < 1; ++current_circle)
        {
                cv::Point center(std::round(circlesLeft[current_circle][0]-1), std::round(circlesLeft[current_circle][1]));
                int radius = std::round(circlesLeft[current_circle][2]);

                CenterPointLeft.push_back(center);

                cv::circle(imageLeft, center, radius, cv::Scalar(0, 0, 150), 3);
        }

        //Vector of Eigenmatrix for both the right and left camera
        Eigen::Matrix<double, 3, 4> projectMatrixRight = ProjectionMatrix("Camera_Right");
        Eigen::Matrix<double, 3, 4> projectMatrixLeft = ProjectionMatrix("Camera_Left");

        cv::Mat cam0(3, 4, CV_64F);
        cv::Mat cam1(3, 4, CV_64F);

        cv::eigen2cv(projectMatrixRight, cam0);
        cv::eigen2cv(projectMatrixLeft, cam1);

        //https://docs.opencv.org/3.1.0/d0/dbd/group__triangulation.html
        cv::Mat pnts3D(4,CenterPointRight.size(),CV_64F);
        cv::triangulatePoints(cam1, cam0, CenterPointLeft, CenterPointRight, pnts3D);

        double dif_x = pnts3D.at<double>(0,0)/pnts3D.at<double>(0,3) - _bottle->getTransform(_state).P()[0];
        double dif_y = pnts3D.at<double>(0,1)/pnts3D.at<double>(0,3) - _bottle->getTransform(_state).P()[1];

        double error = sqrt((dif_x*dif_x) + (dif_y*dif_y));

        if (sparse_test == true)
        {
            if (error < 0.5)
            {
                std::ofstream myfile;
                myfile.open ("/tmp/sparse_test.DAT", ios::app);
                myfile << error << "\n";
                myfile.close();
                std::cout << "Error found to be: " << error << std::endl;
            }
        }

        _bottleEst->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(pnts3D.at<double>(0,0)/pnts3D.at<double>(0,3), pnts3D.at<double>(0,1)/pnts3D.at<double>(0,3), 0.1), rw::math::RPY<>(0,0,90*Deg2Rad)), _state);
        getRobWorkStudio()->setState(_state);
    }
    else
    {
        std::cout << "Circles were not found." << std::endl;
    }
}

//  Mostly based on a triangulation exercise during computer vision at SDU Robotics.
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

    Eigen::Matrix<double, 3, 4> P = KA * He;

    return P;
}

// https://stackoverflow.com/questions/7560114/random-number-c-in-some-range
// How to generate a random number in a specified range c++
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
    rw::math::Q qs = _device->getQ(_state);
    createPathRRTConnect(qs, Qhome, 0.05, 60);

    // Toggle the timer on and off
    timerStart();
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

            std::ofstream output("/tmp/" + _cameras25D[i] + ".pcd");
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
    if (_framegrabber != NULL)
    {
        for(size_t i = 0; i < _cameras.size(); i ++)
        {
            // Get the image as a RW image
            Frame* cameraFrame = _wc->findFrame(_cameras[i]); // "Camera");
            _framegrabber->grab(cameraFrame, _state);

            const rw::sensor::Image* rw_image = &(_framegrabber->getImage());

            // Convert to OpenCV matrix.
            cv::Mat image = cv::Mat(rw_image->getHeight(), rw_image->getWidth(), CV_8UC3, (rw::sensor::Image*)rw_image->getImageData());

            float mean_gaussian = 10.f;
            if (std_gaussian != 0.f)
            {
                cv::Mat noise_image(image.size(), CV_16SC3);
                cv::randn(noise_image, Scalar::all(mean_gaussian), Scalar::all(std_gaussian));
                cv::Mat temp_image; image.convertTo(temp_image,CV_16SC3);
                cv::addWeighted(temp_image, 1.0, noise_image, 1.0, 0.0, temp_image);
                temp_image.convertTo(image,image.type());
            }

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
    if (colFrom)
    {
        /*
        //cerr << "Configuration in collision: " << q << endl;
        //cerr << "Colliding frames: " << endl;
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++)
        {
            //cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
        }
        */
        return false;
    }
    return true;
}

void SamplePlugin::createPathRRTConnect(Q from, Q to,  double extend, double maxTime)
{
    QPath tempPath;
    _device->setQ(from,_state);
    getRobWorkStudio()->setState(_state);
    CollisionDetector detector(_wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,_device,_state);
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(_device),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

    //_path.clear();
    if (!checkCollisions(_device, _state, detector, from))
        cout << from << " is in colission!" << endl;
    if (!checkCollisions(_device, _state, detector, to))
        cout << to << " is in colission!" << endl;;
    Timer t;
    t.resetAndResume();
    planner->query(from,to,tempPath,maxTime);
    t.pause();

    if (t.getTime() >= maxTime)
    {
        cout << "Notice: max time of " << maxTime << " seconds reached." << endl;
    }
    const int duration = 10;

    if(tempPath.size() == 2){  //The interpolated path between Q start and Q goal is collision free. Set the duration with respect to the desired velocity
        LinearInterpolator<Q> linInt(from, to, duration);
        QPath tempQ;
        for(int i = 0; i < duration+1; i++)
        {
            tempQ.push_back(linInt.x(i));
        }
        _path.insert(_path.end(), tempQ.begin(), tempQ.end());
    }
}
