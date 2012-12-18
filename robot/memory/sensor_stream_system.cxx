#include <vector>
#include <string>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems.h>
#include "stdheader.h"

using namespace barrett;
using namespace systems;


class SensorStreamSystem : public systems::System {

public:
/*#define X(aa, bb, cc, dd) Input<bb> input_##cc;
    TYPE_TABLE
#undef X  */ 
    float time_count;

#define X(aa, bb, cc, dd) Output<bb> output_##cc;
    TYPE_TABLE
#undef X   

protected:
#define X(aa, bb, cc, dd) Output<bb>::Value* output_value_##cc;
    TYPE_TABLE
#undef X
    Hand* hand;
    bool* problem;
    int problem_count;
    stringstream* debug;

public:
	SensorStreamSystem(Robot* _robot, const std::string& sysName = "SensorStreamSystem") :
		systems::System(sysName), 
        time_input(this), 
#define X(aa, bb, cc, dd) output_##cc(this, &output_value_#cc),
        TYPE_TABLE
#undef X   
        robot(_robot) 
		{
        }

	virtual ~SensorStreamSystem() { mandatoryCleanUp(); }

protected:
#define X(aa, bb, cc, dd) bb readings_#cc;
    TYPE_TABLE
#undef X
	
    virtual void operate() {

        Wam<DIMENSION>* wam = robot->get_wam();
        Hand* hand = robot->get_hand();
        ForceTorqueSensor* fts = robot->get_fts();

#define X(aa, bb, cc, dd) output_value_##cc->setData(&readings_##cc);
        TYPE_TABLE
#undef X
	}
};

