#include "stdheader.h"
#include "utils.h"
#include "memory.h"

class CpSystem : public systems::System {

public:
    //Input<double> input_time;
    Input<cp_type> input;
    Output<cp_type> output;
protected:
    Output<cp_type>::Value* output_value;

private:
    Memory* memory;

public:
	CpSystem(Memory* _memory, const std::string& sysName = "CpSystem") :
        memory(_memory),
		systems::System(sysName), 
        output(this, &output_value),
        input(this)
		{
            init();
        }

	virtual ~CpSystem() { mandatoryCleanUp(); }
    void init(){
        transform_cp = memory->get_transform_cp();
    }

protected:
    cp_type readings_cp;
    cp_type transform_cp;
    int transform_cp_count;
    cp_type transformed_cp;
	
    virtual void operate(){
        transform_cp = memory->get_transform_cp();
        readings_cp = input.getValue();
        transformed_cp = transform_cp + readings_cp;
        output_value->setData(&transformed_cp);
    }

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

