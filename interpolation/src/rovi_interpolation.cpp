#include <rw/rw.hpp>
#include <rw/invkin.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <iostream>
#include <string>

USE_ROBWORK_NAMESPACE
using namespace robwork;

bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q)
{
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ(q,testState);
    colFrom = detector.inCollision(testState,&data);
    if (colFrom)
    {
        std::cout << "Configuration in collision: " << q << std::endl;
        std::cout << "Colliding frames: " << std::endl;
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++)
        {
            std::cout << (*it).first->getName() << " " << (*it).second->getName() << std::endl;
        }
        return false;
        }
    return true;
}

int main(int argc, char** argv)
{
    // Load needed objects
    rw::models::WorkCell::Ptr wc                  = rw::loaders::WorkCellLoader::Factory::load("../../Project_WorkCell/Scene.wc.xml");
    rw::models::SerialDevice::Ptr robot                 = wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
    rw::kinematics::MovableFrame::Ptr frameRobotBase    = wc->findFrame<rw::kinematics::MovableFrame>("URReference");
    rw::kinematics::MovableFrame::Ptr frameBottle       = wc->findFrame<rw::kinematics::MovableFrame>("Bottle");
    rw::kinematics::Frame* frameTCP                     = wc->findFrame("GraspTCP");
    if(wc==NULL || robot==NULL || frameRobotBase==NULL || frameBottle==NULL || frameTCP==NULL)
    {
        RW_THROW("Could not find one or more devices...");
        return -1;
    }

    State state = wc->getDefaultState();

    CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());

    frameBottle->attachTo(frameTCP, state);
    frameBottle->setTransform(rw::math::Transform3D<>(
                rw::math::Vector3D<>(0, 0, 0), rw::math::RPY<>(0, 0, 0*rw::math::Deg2Rad)),
            state);

    std::vector<rw::math::Q> qs;
    qs.resize(6);
    qs.at(0) = Q(6,  1.607, -1.903, -2.101, -2.277, -2.529, 0.001);
    qs.at(1) = Q(6,  1.368, -1.554, -1.590, -2.062,  -1.869, -0.059);
    qs.at(2) = Q(6,  0.868, -1.927, -1.814, -2.297,  -1.205, -0.101);
    qs.at(3) = Q(6, -0.030, -2.066, -1.494, -2.368, 0.312, -0.017);
    qs.at(4) = Q(6, -0.778, -2.228, -1.209, -2.381, 1.731, 0.020);
    qs.at(5) = Q(6, -1.308, -2.315, -1.389, -2.581, 2.511, -0.001);

    for (int i = 0; i < qs.size(); i++)
    {
        if (!checkCollisions(robot, state, detector, qs.at(i)))
            return 0;
    }

    double t = 1;
    unsigned int t_res = 100;

    double t_b = 0.2;

    std::vector<rw::trajectory::LinearInterpolator<rw::math::Q>> ls;

    std::vector<rw::trajectory::ParabolicBlend<rw::math::Q>> ps;

    for (unsigned int i = 0; i < qs.size()-1; i++)
    {
        ls.emplace_back(qs.at(i), qs.at(i+1), t);
    }

    for (unsigned int i = 0; i < ls.size()-1; i++)
    {
        ps.emplace_back(&ls.at(i), &ls.at(i+1), t_b);
    }

    TimedStatePath path;

    for (unsigned int i = 0; i < ls.size(); i++)
    {
        for (unsigned int j = 0; j < t_res; j++)
        {
            robot->setQ(ls.at(i).x(t*j/t_res), state);
            path.push_back(TimedState(t*i+t*j/t_res, state));
        }
    }

    TimedStatePath path_b;

    for (unsigned int i = 0; i < ps.size()+1; i++)
    {
        for (unsigned int j = 0; j < t_res; j++)
        {
            if (i == 0)
            {
                if (t*j/t_res > ls.at(i).duration() - ps.at(i).tau1())
                {
                    robot->setQ(ps.at(i).x(t*j/t_res - (ls.at(i).duration() - ps.at(i).tau1())), state);
                }
                else
                {
                    robot->setQ(ls.at(i).x(t*j/t_res), state);
                }
            }
            else if (i == ps.size())
            {
                if (t*j/t_res < ps.at(i-1).tau2())
                {
                    robot->setQ(ps.at(i-1).x(ps.at(i-1).tau1() + t*j/t_res), state);
                }
                else
                {
                    robot->setQ(ls.at(i).x(t*j/t_res), state);
                }
            }
            else
            {
                if (t*j/t_res < ps.at(i-1).tau2())
                {
                    robot->setQ(ps.at(i-1).x(ps.at(i-1).tau1() + t*j/t_res), state);
                }
                else if (t*j/t_res > ls.at(i).duration() - ps.at(i).tau1())
                {
                    robot->setQ(ps.at(i).x(t*j/t_res - (ls.at(i).duration() - ps.at(i).tau1())), state);
                }
                else
                {
                    robot->setQ(ls.at(i).x(t*j/t_res), state);
                }
            }

        path_b.push_back(TimedState(t*i+t*j/t_res, state));
        }
    }

    rw::loaders::PathLoader::storeTimedStatePath(*wc, path, "./interpolaton.rwplay");
    rw::loaders::PathLoader::storeTimedStatePath(*wc, path_b, "./interpolaton_blend.rwplay");

    return 0;
}



