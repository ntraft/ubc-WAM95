#include "stdheader.h"
#include "utils.h"
#include "memory.h"

class Co2QdSystem : public systems::System {

public:
    //Input<double> input_time;
    Input<co_type> input;
    Output<qd_type> output;
protected:
    Output<qd_type>::Value* output_value;

private:
    Memory* memory;

public:
	Co2QdSystem(Memory* _memory, const std::string& sysName = "Co2QdSystem") :
        memory(_memory),
		systems::System(sysName), 
        output(this, &output_value),
        input(this)
		{
            transform_qd = 
                AngleAxisd(memory->get_float("transform_qd_x"), Vector3d::UnitX()) * 
                AngleAxisd(memory->get_float("transform_qd_y"), Vector3d::UnitY()) * 
                AngleAxisd(memory->get_float("transform_qd_z"), Vector3d::UnitZ());
        }

	virtual ~Co2QdSystem() { mandatoryCleanUp(); }

protected:
    co_type readings_co;
    qd_type readings_qd;
    qd_type transform_qd;
    qd_type test_qd;
	
    virtual void operate(){
        readings_co = input.getValue();
        readings_qd = transform_qd * co2qd(&readings_co);
        output_value->setData(&readings_qd);
    }

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

