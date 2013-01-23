#include "stdheader.h"

class Senses;
class RobotController;
class ControlStrategy;

class RTControl : public systems::System {

public:
    Input<double> input_time;
    float time_count;
#define X(aa, bb, cc, dd, ee) Input<bb> mean_input_##cc;
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X   
#define X(aa, bb, cc, dd, ee) Input<bb> std_input_##cc;
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X   
#define X(aa, bb, cc, dd, ee) Input<bb> actual_input_##cc;
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X   
    /*
*/
#define X(aa, bb, cc, dd, ee) bb* problem_count_##cc;
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X   
/*
*/
/*
#define X(aa, bb, cc, dd, ee) Output<bb> output_##cc;
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X   

protected:
#define X(aa, bb, cc, dd, ee) Output<bb>::Value* output_value_##cc;
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
*/
    Output<double> output_time;
    Output<jp_type> output_jp;
protected:
    Output<double>::Value* output_value_time;
    Output<jp_type>::Value* output_value_jp;
    Senses* senses;
    RobotController* control;
    ControlStrategy* control_strategy;
    bool* problem;
    int problem_count;
    stringstream* debug;

public:
	RTControl(stringstream* _debug, 
#define X(aa, bb, cc, dd, ee) bb* _problem_count_##cc,
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X   
            Memory* memory, Senses* senses, RobotController* control, const std::string& sysName = "RTControl") :
		systems::System(sysName), 
#define X(aa, bb, cc, dd, ee) mean_input_##cc(this),
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X   
#define X(aa, bb, cc, dd, ee) std_input_##cc(this),
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X   
#define X(aa, bb, cc, dd, ee) actual_input_##cc(this),
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X  
        /*
*/
#define X(aa, bb, cc, dd, ee) problem_count_##cc(_problem_count_##cc),
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X  
        /*
*/
        /*
#define X(aa, bb, cc, dd, ee) output_##cc(this, &output_value_##cc),
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X 
*/
        input_time(this),
        output_time(this, &output_value_time),
        output_jp(this, &output_value_jp)
		{
            debug = _debug;
            this->senses = senses;
            this->control= control;
        }

	virtual ~RTControl() { mandatoryCleanUp(); }

    void set_control_strategy(ControlStrategy* _strategy){control_strategy=_strategy;}

protected:
	
    virtual void operate();
};

