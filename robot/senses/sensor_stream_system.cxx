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

#define X(aa, bb, cc, dd) Output<bb> output_##cc;
    #include "type_table.h"
#undef X   

protected:
#define X(aa, bb, cc, dd) Output<bb>::Value* output_value_##cc;
    #include "type_table.h"
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
#define X(aa, bb, cc, dd) output_##cc(this, &output_value_##cc),
    #include "type_table.h"
#undef X   
        robot(_robot) 
		{
        }

	virtual ~SensorStreamSystem() { mandatoryCleanUp(); }

protected:
    Hand::jp_type fingertip_torque_readings;

protected:
#define X(aa, bb, cc, dd) bb readings_##cc;
    #include "type_table.h"
#undef X
	
    virtual void operate() {

        Wam<DIMENSION>* wam = robot->get_wam();
        Hand* hand = robot->get_hand();
        ForceTorqueSensor* fts = robot->get_fts();

#define X(aa, bb, cc, dd) output_value_##cc->setData(&readings_##cc);
    #include "type_table.h"
#undef X
	}
};

