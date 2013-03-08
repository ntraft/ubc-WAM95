#include "stdheader.h"
#include "senses.h"
#include "memory.h"
#include "parameter_estimator.h"

class SensorStreamSystem : public systems::System {

public:
    Input<double> time_input;
    float time_count;

#define X(aa, bb, cc, dd, ee) \
    Output<bb> output_##cc;
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X   
Output<pv_type> output_param;


protected:
#define X(aa, bb, cc, dd, ee) \
    Output<bb>::Value* output_value_##cc;
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
    Output<pv_type>::Value* output_value_param;
    Memory* memory;
    Senses* senses;
    bool* problem;
    int problem_count;
    stringstream* debug;
    pv_type params;

public:
	SensorStreamSystem(Memory* _memory, Senses* _senses, const std::string& sysName = "SensorStreamSystem") :
		systems::System(sysName),
        memory(_memory),
        senses(_senses),
#define X(aa, bb, cc, dd, ee) \
        output_##cc(this, &output_value_##cc),
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X   
        output_param(this, &output_value_param),
        time_input(this)
		{
        }

	virtual ~SensorStreamSystem() { mandatoryCleanUp(); }

//protected:
    //Hand::jp_type fingertip_torque_readings;

protected:
#define X(aa, bb, cc, dd, ee) \
    bb readings_##cc;
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
	
    virtual void operate(); 

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

