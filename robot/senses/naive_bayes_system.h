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
    #include "input_type_table.h"
    #include "tool_type_table.h"
#undef X   
protected:
    Output<double>::Value* output_value_probability;
    double alpha; //probability at current timestep * previous timestep

protected:
    Memory* memory;

public:
    void init();
    double get_probability();
	NaiveBayesSystem(ExecutionManager* em, Memory* _memory, const std::string& sysName = "NaiveBayesSystem") :
		systems::System(sysName), 
#define X(aa, bb, cc, dd, ee) \
        mean_input_##cc(this), \
        std_input_##cc(this), \
        actual_input_##cc(this),
        #include "input_type_table.h"
        #include "tool_type_table.h"
#undef X   
        memory(_memory),
        input_time(this),
        output_probability(this, &output_value_probability)
        //input_time(this)
		{
            //em->startManaging(*this);
        }

	virtual ~NaiveBayesSystem() { mandatoryCleanUp(); }

protected:
    virtual void operate();

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

