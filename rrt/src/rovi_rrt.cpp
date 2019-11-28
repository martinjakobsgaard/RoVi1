#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 60.

bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
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

int main(int argc, char** argv)
{
    TimedStatePath tStatePath;

    ofstream mydata;
    mydata.open("ROBDATA.dat");
    mydata << "time\tdistance\teps\tsteps" << "\n";
    mydata.close();

    mydata.open("ROBDATA.dat", std::ios_base::app);

    for (double extend = 0.02; extend <= 1.0; extend+=0.05)
    {
        for(int trial = 0; trial < 5; trial++)
        {
            const string wcFile = "../../Project_WorkCell/Scene.wc.xml";
            const string deviceName = "UR-6-85-5-A";
            //cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;
            rw::math::Math::seed();

            WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
            Frame *tool_frame = wc->findFrame("GraspTCP");
            Frame *bottle_frame = wc->findFrame("Bottle");

            Device::Ptr device = wc->findDevice(deviceName);        //process finished with exit code 139 (interrupted by signal 11: SIGSEGV)
            if (device == NULL) {
                cerr << "Device: " << deviceName << " not found!" << endl;
                return 0;
            }

            State state = wc->getDefaultState();

            Q from(6, -1.427, -0.547, 0.848, -0.301, -2.596, 0.001);
            Q to(6,-0.984, -1.719, -2.362, -2.201, -2.363, 0.001);

            device->setQ(from,state);

            Kinematics::gripFrame(bottle_frame, tool_frame, state);

            CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
            PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

            QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
            QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
            QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

            if (!checkCollisions(device, state, detector, from))
                return 0;
            if (!checkCollisions(device, state, detector, to))
                return 0;

            //cout << "Planning from " << from << " to " << to << endl;
            QPath path;
            Timer t;
            t.resetAndResume();
            planner->query(from,to,path,MAXTIME);
            t.pause();
            double distance = 0;

            //cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << endl;
            if (t.getTime() >= MAXTIME) {
                cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
            }

            for (unsigned int i = 0; i< path.size(); i++)
            {
                if(i >= 1)
                    distance += sqrt(pow((path.at(i)(0)-path.at(i-1)(0)),2)+pow((path.at(i)(1)-path.at(i-1)(1)),2)+pow((path.at(i)(2)-path.at(i-1)(2)),2)+pow((path.at(i)(3)-path.at(i-1)(3)),2)+pow((path.at(i)(4)-path.at(i-1)(4)),2)+pow((path.at(i)(5)-path.at(i-1)(5)),2));
            }
            mydata << t.getTime() << "\t" << distance << "\t\t" << extend << "\t" << path.size() << "\n";

            cout << trial << endl;
        }
    }

    mydata.close();
    return 0;
}
