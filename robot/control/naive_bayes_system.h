#include "stdheader.h"
#include "utils-inl.h"

class Senses;
class RobotController;
class ControlStrategy;

class NaiveBayesSystem : public systems::System {

public:
    Input<double> input_time;
    Output<double> output_probability;
#define X(aa, bb, cc, dd, ee) \
    Input<bb> mean_input_##cc; \
    Input<bb> std_input_##cc; \
    Input<bb> actual_input_##cc;
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X   
protected:
    Output<double>::Value* output_value_probability;
    double running_probability; //probability at current timestep * previous timestep

protected:
    Memory* memory;
    bool* problem;

public:
    void init();
	NaiveBayesSystem(Memory* _memory, const std::string& sysName = "NaiveBayesSystem") :
		systems::System(sysName), memory(_memory),
#define X(aa, bb, cc, dd, ee) \
        mean_input_##cc(this), \
        std_input_##cc(this), \
        actual_input_##cc(this),
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X   
        output_probability(this, &output_value_probability),
        input_time(this)
		{
        }

	virtual ~NaiveBayesSystem() { mandatoryCleanUp(); }

protected:
    virtual void operate();

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

