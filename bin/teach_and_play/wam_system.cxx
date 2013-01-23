#include <vector>
#include <string>

// Base class: barrett::systems::System
#include <barrett/systems/abstract/system.h>

// Includes the rest of the barrett::systems namespace, which contains all
// non-abstract Systems
#include <barrett/systems.h>

#include "stdheader.h"
#include "utils.h"

using namespace barrett;
using namespace systems;

// A System that outputs four fingertip_torque values and four tactile pressure values 
// each tactile pressure value is the summation of individual sensors on the pad
class WamSystem : public systems::System {

public:		Input<systems::Wam<DIMENSION>::jp_type> input;
public:		Output<systems::Wam<DIMENSION>::jp_type> output;

protected:	Output<systems::Wam<DIMENSION>::jp_type>::Value* outputValue;
protected:  systems::Wam<DIMENSION>* wam;
protected:  systems::Wam<DIMENSION>::jp_type jp_offsets; //modifications to realtime motion feed
protected:  int count;

public:
	WamSystem(systems::Wam<DIMENSION>* _wam, const std::string& sysName = "WamSystem") :
		systems::System(sysName), input(this), output(this, &outputValue), wam(_wam){
            init();
        }

	virtual ~WamSystem() { mandatoryCleanUp(); }

    void init(){
        set_vector_values(&jp_offsets, 0, 0);
        count = 0;
    }

protected:
    systems::Wam<DIMENSION>::jp_type jp_out;

	// Implement System::operate(). The operate() function must be declared with
	// the "protected" access specifier.
	virtual void operate() {
        const systems::Wam<DIMENSION>::jp_type& jp_in = input.getValue();  // Pull data from the input 

        jp_out = jp_in + jp_offsets;

        count++;

        if(count < 0)
            jp_offsets[5] += 0.01;
        
        outputValue->setData(&jp_out);  // Push data into the output
	}
};
