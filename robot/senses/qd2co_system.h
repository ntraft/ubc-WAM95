#include "stdheader.h"
#include "utils.h"

class Qd2CoSystem : public systems::System {

public:
    //Input<double> input_time;
    Input<Quaterniond> input_qd;

    Output<co_type> output_co;
protected:
    Output<co_type>::Value* output_value_co;

public:
	Qd2CoSystem(const std::string& sysName = "Qd2CoSystem") :
		systems::System(sysName), 
        output_co(this, &output_value_co),
        input_qd(this)
        //input_time(this)
		{}

	virtual ~Qd2CoSystem() { mandatoryCleanUp(); }

protected:
	
    virtual void operate(){
        const Quaterniond& readings_qd = input_qd.getValue();
        co_type readings_co = qd2co(&readings_qd);
        output_value_co->setData(&readings_co);
    }

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

