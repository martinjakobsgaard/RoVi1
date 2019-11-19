#include <rw/rw.hpp>
#include <rw/invkin.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <iostream>
#include <string>

USE_ROBWORK_NAMESPACE
using namespace robwork;

std::vector<rw::math::Q> getConfigurations(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
{
    // Get, make and print name of frames
    const std::string robotName = robot->getName();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp = robotName + "." + "TCP";

    // Find frames and check for existence
    rw::kinematics::Frame* frameGoal = wc->findFrame(nameGoal);
    rw::kinematics::Frame* frameTcp = wc->findFrame(nameTcp);
    rw::kinematics::Frame* frameRobotBase = wc->findFrame(nameRobotBase);
    rw::kinematics::Frame* frameRobotTcp = wc->findFrame(nameRobotTcp);
    if(frameGoal==NULL || frameTcp==NULL || frameRobotBase==NULL || frameRobotTcp==NULL)
    {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (frameGoal==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (frameTcp==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotBase << "\": " << (frameRobotBase==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==NULL ? "NO!" : "YES!") << std::endl;
    }

    // Make "helper" transformations
    rw::math::Transform3D<> frameBaseTGoal = rw::kinematics::Kinematics::frameTframe(frameRobotBase, frameGoal, state);
    rw::math::Transform3D<> frameTcpTRobotTcp = rw::kinematics::Kinematics::frameTframe(frameTcp, frameRobotTcp, state);

    // get grasp frame in robot tool frame
    rw::math::Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(robot, state) );
    return closedFormSovler->solve(targetAt, state);
}

void addToPath(State state,  TimedStatePath& tStatePath, std::vector<rw::math::Q> collisionFreeSolutions, rw::models::SerialDevice::Ptr robotUR5)
{
    double time=0;
    for(unsigned int i=0; i<collisionFreeSolutions.size(); i++)
    {
            robotUR5->setQ(collisionFreeSolutions[i], state);
            tStatePath.push_back(TimedState(time,state));
            time+=0.01;
    }
}

void checkPlacement(rw::kinematics::MovableFrame::Ptr cylinderFrame, rw::proximity::CollisionDetector::Ptr detector, rw::models::WorkCell::Ptr wc,
                    State state, rw::models::SerialDevice::Ptr robotUR5, TimedStatePath& tStatePath, int& curr_best)
{
    State state_save1 = wc->getDefaultState(); State state_save2 = wc->getDefaultState(); State state_save3 = wc->getDefaultState(); State state_save4 = wc->getDefaultState(); State state_save5 = wc->getDefaultState();
    State state_save6 = wc->getDefaultState();
    std::vector<rw::math::Q> collisionFreeSolutions;
    for(double rollAngle=0; rollAngle<360.0; rollAngle+=1.0)
    {
        cylinderFrame->moveTo(rw::math::Transform3D<>(
                                        rw::math::Vector3D<>(cylinderFrame->getTransform(state).P()),
                                        rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,1.57)), state);

        std::vector<rw::math::Q> solutions = getConfigurations("GraspTarget", "GraspTCP", robotUR5, wc, state);

        for(unsigned int i=0; i<solutions.size(); i++)
        {
            // set the robot in that configuration and check if it is in collision
            robotUR5->setQ(solutions[i], state);
            if( !detector->inCollision(state,NULL,true) )
            {
                state_save1 = state;
                collisionFreeSolutions.push_back(solutions[i]);
                break; // we only need one
            }
        }
    }

    std::vector<rw::math::Q> collisionFreeSolutions_place;
    cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(0.3, -0.5, 0.21), rw::math::RPY<>(0,0,0)), state);
    for(double rollAngle=0; rollAngle<360.0; rollAngle+=1.0)
    {
        cylinderFrame->moveTo(rw::math::Transform3D<>(
                                        rw::math::Vector3D<>(cylinderFrame->getTransform(state).P()),
                                        rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,1.57)), state);

        std::vector<rw::math::Q> solutions_place = getConfigurations("GraspTarget", "GraspTCP", robotUR5, wc, state);

        for(unsigned int i=0; i<solutions_place.size(); i++)
        {
            // set the robot in that configuration and check if it is in collision
            robotUR5->setQ(solutions_place[i], state);
            if( !detector->inCollision(state,NULL,true) )
            {
                state_save2 = state;
                collisionFreeSolutions_place.push_back(solutions_place[i]);
                break; // we only need one
            }
        }
    }

    std::vector<rw::math::Q> collisionFreeSolutions_pick1;
    cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(-0.3, 0.5, 0.21), rw::math::RPY<>(0,0,0)), state);
    for(double rollAngle=0; rollAngle<360.0; rollAngle+=1.0)
    {
        cylinderFrame->moveTo(rw::math::Transform3D<>(
                                        rw::math::Vector3D<>(cylinderFrame->getTransform(state).P()),
                                        rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,1.57)), state);

        std::vector<rw::math::Q> solutions_pick1 = getConfigurations("GraspTarget", "GraspTCP", robotUR5, wc, state);

        for(unsigned int i=0; i<solutions_pick1.size(); i++)
        {
            // set the robot in that configuration and check if it is in collision
            robotUR5->setQ(solutions_pick1[i], state);
            if( !detector->inCollision(state,NULL,true) )
            {
                state_save3 = state;
                collisionFreeSolutions_pick1.push_back(solutions_pick1[i]);
                break; // we only need one
            }
        }
    }

    std::vector<rw::math::Q> collisionFreeSolutions_pick2;
    cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(0.3, 0.5, 0.21), rw::math::RPY<>(0,0,0)), state);
    for(double rollAngle=0; rollAngle<360.0; rollAngle+=1.0)
    {
        cylinderFrame->moveTo(rw::math::Transform3D<>(
                                        rw::math::Vector3D<>(cylinderFrame->getTransform(state).P()),
                                        rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,1.57)), state);

        std::vector<rw::math::Q> solutions_pick2 = getConfigurations("GraspTarget", "GraspTCP", robotUR5, wc, state);

        for(unsigned int i=0; i<solutions_pick2.size(); i++)
        {
            // set the robot in that configuration and check if it is in collision
            robotUR5->setQ(solutions_pick2[i], state);
            if( !detector->inCollision(state,NULL,true) )
            {
                state_save4 = state;
                collisionFreeSolutions_pick2.push_back(solutions_pick2[i]);
                break; // we only need one
            }
        }
    }

    std::vector<rw::math::Q> collisionFreeSolutions_pick3;
    cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(0.3, 0.4, 0.21), rw::math::RPY<>(0,0,0)), state);
    for(double rollAngle=0; rollAngle<360.0; rollAngle+=1.0)
    {
        cylinderFrame->moveTo(rw::math::Transform3D<>(
                                        rw::math::Vector3D<>(cylinderFrame->getTransform(state).P()),
                                        rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,1.57)), state);

        std::vector<rw::math::Q> solutions_pick3 = getConfigurations("GraspTarget", "GraspTCP", robotUR5, wc, state);

        for(unsigned int i=0; i<solutions_pick3.size(); i++)
        {
            // set the robot in that configuration and check if it is in collision
            robotUR5->setQ(solutions_pick3[i], state);
            if( !detector->inCollision(state,NULL,true) )
            {
                state_save5 = state;
                collisionFreeSolutions_pick3.push_back(solutions_pick3[i]);
                break; // we only need one
            }
        }
    }

    std::vector<rw::math::Q> collisionFreeSolutions_pick4;
    cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(-0.3, 0.4, 0.21), rw::math::RPY<>(0,0,0)), state);
    for(double rollAngle=0; rollAngle<360.0; rollAngle+=1.0)
    {
        cylinderFrame->moveTo(rw::math::Transform3D<>(
                                        rw::math::Vector3D<>(cylinderFrame->getTransform(state).P()),
                                        rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,1.57)), state);

        std::vector<rw::math::Q> solutions_pick4 = getConfigurations("GraspTarget", "GraspTCP", robotUR5, wc, state);

        for(unsigned int i=0; i<solutions_pick4.size(); i++)
        {
            // set the robot in that configuration and check if it is in collision
            robotUR5->setQ(solutions_pick4[i], state);
            if( !detector->inCollision(state,NULL,true) )
            {
                state_save6 = state;
                collisionFreeSolutions_pick4.push_back(solutions_pick4[i]);
                break; // we only need one
            }
        }
    }

    int solutions_found_pick = collisionFreeSolutions.size() + collisionFreeSolutions_pick1.size() +
            collisionFreeSolutions_pick2.size() + collisionFreeSolutions_pick3.size() + collisionFreeSolutions_pick4.size();
    int solutions_found_place = collisionFreeSolutions_place.size();
    int solutions_found = solutions_found_pick/5 + solutions_found_place;
    std::cout << "Collision free solutions: " << solutions_found << std::endl;

    if (solutions_found > curr_best)
    {
        curr_best = collisionFreeSolutions.size() + collisionFreeSolutions_place.size();
        //std::cout << "Collision free solutions: " << curr_best << std::endl;
        tStatePath.clear();
        addToPath(state_save1, tStatePath, collisionFreeSolutions, robotUR5);
        addToPath(state_save2, tStatePath, collisionFreeSolutions_place, robotUR5);
        addToPath(state_save3, tStatePath, collisionFreeSolutions_pick1, robotUR5);
        addToPath(state_save4, tStatePath, collisionFreeSolutions_pick2, robotUR5);
        addToPath(state_save5, tStatePath, collisionFreeSolutions_pick3, robotUR5);
        addToPath(state_save6, tStatePath, collisionFreeSolutions_pick4, robotUR5);
    }
}

int main(int argc, char** argv)
{
    double resolution = 0.1;
    TimedStatePath tStatePath;
    int curr_best = 0;

    //load workcell
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../../Project_WorkCell/Scene.wc.xml");
    if(NULL==wc){
            RW_THROW("COULD NOT LOAD scene... check path!");
            return -1;
    }

    // find relevant frames
    rw::kinematics::MovableFrame::Ptr cylinderFrame = wc->findFrame<rw::kinematics::MovableFrame>("Bottle");
    if(NULL==cylinderFrame){
            RW_THROW("COULD not find movable frame Cylinder ... check model");
            return -1;
    }

    rw::models::SerialDevice::Ptr robotUR5 = wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
    if(NULL==robotUR5){
            RW_THROW("COULD not find device UR5 ... check model");
            return -1;
    }

    rw::kinematics::MovableFrame::Ptr robotBaseFrame = wc->findFrame<rw::kinematics::MovableFrame>("URReference");
    if(NULL==robotBaseFrame)
    {
        RW_THROW("COULD not find robot base frame... check model");
        return -1;
    }

    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));

    // get the default state
    State state = wc->getDefaultState();

    rw::math::Transform3D<> bestRobotBaseFrame;

    for (size_t k = 0; k < 0.6/resolution; k++)
    {
        for (size_t j = 0; j < 0.6/resolution; j++)
        {
            double dx = -0.3+(resolution*k);
            double dy = -0.3+(resolution*j);
            std::cout << "Checked position: " << dx << "   " << dy << std::endl;
            robotBaseFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(dx, dy, 0.01), rw::math::RPY<>(0,0,0)), state);
            checkPlacement(cylinderFrame, detector, wc, state, robotUR5, tStatePath, curr_best);
        }
    }

    robotBaseFrame->moveTo(bestRobotBaseFrame, state);

    rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, "reachability.rwplay");

    return 0;
}














/*
rw::math::Transform3D<> newRobotBaseFrame2 = robotBaseFrame->getTransform(state) * rw::math::Transform3D<>(rw::math::Vector3D<>(-1.8, 0, 0), rw::math::RPY<>(0,0,0));
robotBaseFrame->moveTo(newRobotBaseFrame2, state);
checkPlacement(bestSolutions, collisionFreeSolutions, cylinderFrame, detector, wc, state, robotUR5);

rw::math::Transform3D<> newRobotBaseFrame3 = robotBaseFrame->getTransform(state) * rw::math::Transform3D<>(rw::math::Vector3D<>(0.1, 0, 0), rw::math::RPY<>(0,0,0));
robotBaseFrame->moveTo(newRobotBaseFrame3, state);
checkPlacement(bestSolutions, collisionFreeSolutions, cylinderFrame, detector, wc, state, robotUR5);

rw::math::Transform3D<> newRobotBaseFrame4 = robotBaseFrame->getTransform(state) * rw::math::Transform3D<>(rw::math::Vector3D<>(-0.3, 0.1, 0), rw::math::RPY<>(0,0,0));
robotBaseFrame->moveTo(newRobotBaseFrame4, state);
checkPlacement(bestSolutions, collisionFreeSolutions, cylinderFrame, detector, wc, state, robotUR5);

rw::math::Transform3D<> newRobotBaseFrame5 = robotBaseFrame->getTransform(state) * rw::math::Transform3D<>(rw::math::Vector3D<>(0.1, 1, 0), rw::math::RPY<>(0,0,0));
robotBaseFrame->moveTo(newRobotBaseFrame5, state);
checkPlacement(bestSolutions, collisionFreeSolutions, cylinderFrame, detector, wc, state, robotUR5);
*/
