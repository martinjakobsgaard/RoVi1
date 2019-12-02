#include <rw/rw.hpp>
#include <rw/invkin.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <iostream>
#include <string>

USE_ROBWORK_NAMESPACE
using namespace robwork;

int main(int argc, char** argv)
{
    // Load needed objects
    rw::models::WorkCell::Ptr workcell                  = rw::loaders::WorkCellLoader::Factory::load("../../Project_WorkCell/Scene.wc.xml");
    rw::models::SerialDevice::Ptr robot                 = workcell->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
    rw::kinematics::MovableFrame::Ptr frameRobotBase    = workcell->findFrame<rw::kinematics::MovableFrame>("URReference");
    rw::kinematics::MovableFrame::Ptr frameBottle       = workcell->findFrame<rw::kinematics::MovableFrame>("Bottle");
    rw::kinematics::Frame* frameTCP                     = workcell->findFrame("GraspTCP");
    if(workcell==NULL || robot==NULL || frameRobotBase==NULL || frameBottle==NULL || frameTCP==NULL)
    {
        RW_THROW("Could not find one or more devices...");
        return -1;
    }

    // Create WorkCell collision detector
    //rw::proximity::CollisionDetector::Ptr detector =
    //rw::common::ownedPtr( new rw::proximity::CollisionDetector(workcell,
     //   rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));

    // get the default state
    State state = workcell->getDefaultState();

    // Attach bottle to grasp
    frameBottle->attachTo(frameTCP, state);
    frameBottle->setTransform(
            rw::math::Transform3D<>(
                rw::math::Vector3D<>(0, 0, 0),
                rw::math::RPY<>(0, 0, 0*rw::math::Deg2Rad)),
            state
            );

    // The configurations to interpolate between
    std::vector<rw::math::Q> qs;
    qs.resize(6);
    qs.at(0) = Q(6,  1.607, -1.903, -2.101, -2.277, -2.529, 0.001);
    qs.at(1) = Q(6,  1.368, -1.554, -1.590, -2.062,  -1.869, -0.059);
    qs.at(2) = Q(6,  0.868, -1.927, -1.814, -2.297,  -1.205, -0.101);
    qs.at(3) = Q(6, -0.030, -2.066, -1.494, -2.368, 0.312, -0.017);
    qs.at(4) = Q(6, -0.778, -2.228, -1.209, -2.381, 1.731, 0.020);
    qs.at(5) = Q(6, -1.308, -2.315, -1.389, -2.581, 2.511, -0.001);

    double t = 1; //Seconds between configurations
    unsigned int t_res = 100; //Resolution between every configuration

    //Blend time
    double t_b = 0.2;

    //Linear Interpolators (no blend)
    std::vector<rw::trajectory::LinearInterpolator<rw::math::Q>> ls;

    //Linear Interpolators (with parabolic blend)
    std::vector<rw::trajectory::ParabolicBlend<rw::math::Q>> ps;

    for (unsigned int i = 0; i < qs.size()-1; i++)
    {
        ls.emplace_back(qs.at(i), qs.at(i+1), t);
    }

    for (unsigned int i = 0; i < ls.size()-1; i++)
    {
        ps.emplace_back(&ls.at(i), &ls.at(i+1), t_b);
    }

    TimedStatePath path; //Without blend

    for (unsigned int i = 0; i < ls.size(); i++)
    {
        for (unsigned int j = 0; j < t_res; j++)
        {
            robot->setQ(ls.at(i).x(t*j/t_res), state);
            path.push_back(TimedState(t*i+t*j/t_res, state));
        }
    }

    TimedStatePath path_b; //With blend

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

    rw::loaders::PathLoader::storeTimedStatePath(*workcell, path, "./interpolaton.rwplay");
    rw::loaders::PathLoader::storeTimedStatePath(*workcell, path_b, "./interpolaton_b.rwplay");

    return 0;
}



