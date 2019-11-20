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
                    State state, rw::models::SerialDevice::Ptr robotUR5, State& state_save, std::vector<rw::math::Q>& collisionFreeSolutions)
{
    collisionFreeSolutions.clear();
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
                state_save = state;
                collisionFreeSolutions.push_back(solutions[i]);
                break; // we only need one
            }
        }
    }
}

void checkPlacementTop(rw::kinematics::MovableFrame::Ptr cylinderFrame, rw::proximity::CollisionDetector::Ptr detector, rw::models::WorkCell::Ptr wc,
                    State state, rw::models::SerialDevice::Ptr robotUR5, State& state_save, std::vector<rw::math::Q>& collisionFreeSolutions)
{
    collisionFreeSolutions.clear();
    for(double rollAngle=0; rollAngle<360.0; rollAngle+=1.0)
    {
        cylinderFrame->moveTo(rw::math::Transform3D<>(
                                        rw::math::Vector3D<>(cylinderFrame->getTransform(state).P()),
                                        rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,1.57)), state);

        std::vector<rw::math::Q> solutions = getConfigurations("GraspTargetTop", "GraspTCP", robotUR5, wc, state);

        for(unsigned int i=0; i<solutions.size(); i++)
        {
            // set the robot in that configuration and check if it is in collision
            robotUR5->setQ(solutions[i], state);
            if( !detector->inCollision(state,NULL,true) )
            {
                state_save = state;
                collisionFreeSolutions.push_back(solutions[i]);
                break; // we only need one
            }
        }
    }
}

int main(int argc, char** argv)
{
    int worst_pos = 360;
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

    State state1 = wc->getDefaultState(); std::vector<rw::math::Q> collisionFreeSolutions1;
    State state2 = wc->getDefaultState(); std::vector<rw::math::Q> collisionFreeSolutions2;
    State state3 = wc->getDefaultState(); std::vector<rw::math::Q> collisionFreeSolutions3;
    State state4 = wc->getDefaultState(); std::vector<rw::math::Q> collisionFreeSolutions4;
    State state5 = wc->getDefaultState(); std::vector<rw::math::Q> collisionFreeSolutions5;
    State state6 = wc->getDefaultState(); std::vector<rw::math::Q> collisionFreeSolutions6;
    State state7 = wc->getDefaultState(); std::vector<rw::math::Q> collisionFreeSolutions7;
    State state8 = wc->getDefaultState(); std::vector<rw::math::Q> collisionFreeSolutions8;
    State state9 = wc->getDefaultState(); std::vector<rw::math::Q> collisionFreeSolutions9;
    State state10 = wc->getDefaultState(); std::vector<rw::math::Q> collisionFreeSolutions10;
    State state11 = wc->getDefaultState(); std::vector<rw::math::Q> collisionFreeSolutions11;
    State state12 = wc->getDefaultState(); std::vector<rw::math::Q> collisionFreeSolutions12;

    for (size_t k = 0; k < 0.6/resolution; k++)
    {
        for (size_t j = 0; j < 0.6/resolution; j++)
        {
            worst_pos = 360;
            double dx = -0.3+(resolution*k);
            double dy = -0.3+(resolution*j);
            std::cout << "Checked position: " << dx << "   " << dy << std::endl;
            cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(0, 0.474, 0.21), rw::math::RPY<>(0,0,0)), state);
            robotBaseFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(dx, dy, 0.01), rw::math::RPY<>(0,0,0)), state);
            checkPlacement(cylinderFrame, detector, wc, state, robotUR5, state1, collisionFreeSolutions1);
            checkPlacementTop(cylinderFrame, detector, wc, state, robotUR5, state7, collisionFreeSolutions7);
            cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(0.3, -0.5, 0.21), rw::math::RPY<>(0,0,0)), state);
            checkPlacement(cylinderFrame, detector, wc, state, robotUR5, state2, collisionFreeSolutions2);
            checkPlacementTop(cylinderFrame, detector, wc, state, robotUR5, state8, collisionFreeSolutions8);
            cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(-0.35, 0.55, 0.21), rw::math::RPY<>(0,0,0)), state);
            checkPlacement(cylinderFrame, detector, wc, state, robotUR5, state3, collisionFreeSolutions3);
            checkPlacementTop(cylinderFrame, detector, wc, state, robotUR5, state9, collisionFreeSolutions9);
            cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(-0.35, 0.35, 0.21), rw::math::RPY<>(0,0,0)), state);
            checkPlacement(cylinderFrame, detector, wc, state, robotUR5, state4, collisionFreeSolutions4);
            checkPlacementTop(cylinderFrame, detector, wc, state, robotUR5, state10, collisionFreeSolutions10);
            cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(0.35, 0.55, 0.21), rw::math::RPY<>(0,0,0)), state);
            checkPlacement(cylinderFrame, detector, wc, state, robotUR5, state5, collisionFreeSolutions5);
            checkPlacementTop(cylinderFrame, detector, wc, state, robotUR5, state11, collisionFreeSolutions11);
            cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(0.35, 0.35, 0.21), rw::math::RPY<>(0,0,0)), state);
            checkPlacement(cylinderFrame, detector, wc, state, robotUR5, state6, collisionFreeSolutions6);
            checkPlacementTop(cylinderFrame, detector, wc, state, robotUR5, state12, collisionFreeSolutions12);

            if (collisionFreeSolutions1.size() < worst_pos)
                worst_pos = collisionFreeSolutions1.size();
            if (collisionFreeSolutions2.size() < worst_pos)
                worst_pos = collisionFreeSolutions2.size();
            if (collisionFreeSolutions3.size() < worst_pos)
                worst_pos = collisionFreeSolutions3.size();
            if (collisionFreeSolutions4.size() < worst_pos)
                worst_pos = collisionFreeSolutions4.size();
            if (collisionFreeSolutions5.size() < worst_pos)
                worst_pos = collisionFreeSolutions5.size();
            if (collisionFreeSolutions6.size() < worst_pos)
                worst_pos = collisionFreeSolutions6.size();
            if (collisionFreeSolutions7.size() < worst_pos)
                worst_pos = collisionFreeSolutions7.size();
            if (collisionFreeSolutions8.size() < worst_pos)
                worst_pos = collisionFreeSolutions8.size();
            if (collisionFreeSolutions9.size() < worst_pos)
                worst_pos = collisionFreeSolutions9.size();
            if (collisionFreeSolutions10.size() < worst_pos)
                worst_pos = collisionFreeSolutions10.size();
            if (collisionFreeSolutions11.size() < worst_pos)
                worst_pos = collisionFreeSolutions11.size();
            if (collisionFreeSolutions12.size() < worst_pos)
                worst_pos = collisionFreeSolutions12.size();

            int solution_found = worst_pos;
            std::cout << "The worst position has: " << solution_found << " positions." << std::endl;

            if (solution_found > curr_best)
            {
                curr_best = solution_found;
                tStatePath.clear();
                addToPath(state1, tStatePath, collisionFreeSolutions1, robotUR5);
                addToPath(state2, tStatePath, collisionFreeSolutions2, robotUR5);
                addToPath(state3, tStatePath, collisionFreeSolutions3, robotUR5);
                addToPath(state4, tStatePath, collisionFreeSolutions4, robotUR5);
                addToPath(state5, tStatePath, collisionFreeSolutions5, robotUR5);
                addToPath(state6, tStatePath, collisionFreeSolutions6, robotUR5);

                addToPath(state7, tStatePath, collisionFreeSolutions7, robotUR5);
                addToPath(state8, tStatePath, collisionFreeSolutions8, robotUR5);
                addToPath(state9, tStatePath, collisionFreeSolutions9, robotUR5);
                addToPath(state10, tStatePath, collisionFreeSolutions10, robotUR5);
                addToPath(state11, tStatePath, collisionFreeSolutions11, robotUR5);
                addToPath(state12, tStatePath, collisionFreeSolutions12, robotUR5);
            }
        }
    }

    rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, "reachability.rwplay");
    return 0;
}
