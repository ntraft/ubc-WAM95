#include <vector>
#include <string>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems.h>
#include "stdheader.h"
#include "macros.h"
#include "robot.h"

using namespace barrett;
using namespace systems;


class SensorStreamSystem : public systems::System {

public:
    Input<double> time_input;
    float time_count;

#define X(aa, bb, cc) Output<bb> output_##cc;
    TYPE_TABLE
#undef X   

protected:
#define X(aa, bb, cc) Output<bb>::Value* output_value_##cc;
    TYPE_TABLE
#undef X
    Hand* hand;
    Robot* robot;
    bool* problem;
    int problem_count;
    stringstream* debug;

public:
	SensorStreamSystem(Robot* _robot, const std::string& sysName = "SensorStreamSystem") :
		systems::System(sysName), 
        time_input(this), 
#define X(aa, bb, cc) output_##cc(this, &output_value_##cc),
        TYPE_TABLE
#undef X   
        robot(_robot) 
		{
        }

	virtual ~SensorStreamSystem() { mandatoryCleanUp(); }

protected:
    Hand::jp_type fingertip_torque_readings;

protected:
#define X(aa, bb, cc) bb readings_##cc;
    TYPE_TABLE
#undef X
	
    virtual void operate() {

        Wam<DIMENSION>* wam = robot->getWAM();
        Hand* hand = robot->getHand();
        ForceTorqueSensor* fts = robot->getFTS();

#define X(aa, bb, cc) output_value_##cc->setData(&readings_##cc);
        TYPE_TABLE
#undef X
	}
};

