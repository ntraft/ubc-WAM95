#include "stdheader.h"
#include "senses.h"
#include "memory.h"

class SensorStreamSystem : public systems::System {

public:
    Input<double> time_input;
    float time_count;

#define X(aa, bb, cc, dd, ee) \
    Output<bb> output_##cc;
#include "wam_type_table.h"
#include "tool_type_table.h"
#undef X   

protected:
#define X(aa, bb, cc, dd, ee) \
    Output<bb>::Value* output_value_##cc;
#include "wam_type_table.h"
#include "tool_type_table.h"
#undef X
    Memory* memory;
    Senses* senses;
    bool* problem;
    int problem_count;
    stringstream* debug;

public:
	SensorStreamSystem(Memory* _memory, Senses* _senses, const std::string& sysName = "SensorStreamSystem") :
		systems::System(sysName),
        memory(_memory),
        senses(_senses),
#define X(aa, bb, cc, dd, ee) \
        output_##cc(this, &output_value_##cc),
#include "wam_type_table.h"
#include "tool_type_table.h"
#undef X   
        time_input(this)
		{
        }

	virtual ~SensorStreamSystem() { mandatoryCleanUp(); }

//protected:
    //Hand::jp_type fingertip_torque_readings;

protected:
#define X(aa, bb, cc, dd, ee) \
    bb readings_##cc;
#include "wam_type_table.h"
#include "tool_type_table.h"
#undef X
	
    virtual void operate(); 

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

