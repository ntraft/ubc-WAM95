#include "stdheader.h"
#include "utils-inl.h"

class Senses;
class RobotController;
class ControlStrategy;
//class co_type;


class RTControl : public systems::System {

public:
    Input<double> input_time;
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
    Output<cp_type> output_cp;
    Output<co_type> output_co;
    Output<Quaterniond> output_qd;
protected:
    Output<double>::Value* output_value_time;
    Output<jp_type>::Value* output_value_jp;
    Output<cp_type>::Value* output_value_cp;
    Output<co_type>::Value* output_value_co;
    Output<Quaterniond>::Value* output_value_qd;
    Memory* memory;
    Senses* senses;
    RobotController* control;
    ControlStrategy* control_strategy;
    bool* problem;
    int problem_count;
    stringstream* debug;
    jp_type offsets_jp; //modifications to realtime motion feed
    cp_type offsets_cp; //modifications to realtime motion feed
    qd_type transform_qd; //modifications to realtime motion feed

public:
    void init();
	RTControl(stringstream* _debug, 
#define X(aa, bb, cc, dd, ee) bb* _problem_count_##cc,
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X   
            Memory* memory, Senses* senses, RobotController* control, const std::string& sysName = "RTControl") :
		systems::System(sysName), control_strategy(NULL),
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
        output_jp(this, &output_value_jp),
        output_cp(this, &output_value_cp),
        output_co(this, &output_value_co),
        output_qd(this, &output_value_qd)
		{
            debug = _debug;
            this->memory= memory;
            this->senses = senses;
            this->control= control;
        }

	virtual ~RTControl() { mandatoryCleanUp(); }

    void set_control_strategy(ControlStrategy* _strategy){control_strategy=_strategy;}

protected:
    jp_type out_jp;	
    cp_type out_cp;	
    co_type out_co;	
    virtual void operate();
};

