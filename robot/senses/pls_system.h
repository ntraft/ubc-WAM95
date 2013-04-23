#include "stdheader.h"
#include "utils-inl.h"

class Senses;
class RobotController;
class ControlStrategy;

class PartialLeastSquaresSystem : public systems::System {

public:
    Input<double> input_time;
    //Input<bt_type> input_beta;
#define P(aa, bb, cc, dd, ee) \
    Input<in_type> input_beta_##cc;
#include "parameter_table.h"
#undef P 
    Input<in_type> input_sensors;
    Output<pv_type> output_predictions;
/*#define X(aa, bb, cc, dd, ee) \
    Input<bb> actual_input_##cc;
    #include "input_type_table.h"
#undef X   */
protected:
    Output<pv_type>::Value* output_value_predictions;

protected:
    Memory* memory;
    pv_type predictions;
    bt_type input_beta;

public:
    void init();
    pv_type get_predictions();
	PartialLeastSquaresSystem(Memory* _memory, const std::string& sysName = "PartialLeastSquaresSystem") :
		systems::System(sysName), 
/*#define X(aa, bb, cc, dd, ee) \
        actual_input_##cc(this),
        #include "input_type_table.h"
#undef X   */
        memory(_memory),
        input_time(this),
//        input_beta(this),
#define P(aa, bb, cc, dd, ee) \
        input_beta_##cc(this),
#include "parameter_table.h"
#undef P 
        input_sensors(this),
        output_predictions(this, &output_value_predictions)
        //input_time(this)
		{
            //em->startManaging(*this);
        }

	virtual ~PartialLeastSquaresSystem() { mandatoryCleanUp(); }

protected:
    virtual void operate();

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

