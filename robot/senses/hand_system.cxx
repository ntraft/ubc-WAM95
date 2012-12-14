#include <vector>
#include <string>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems.h>
#include "stdheader.h"

using namespace barrett;
using namespace systems;


class HandSystem : public systems::System {
	typedef boost::tuple<Hand::jp_type, Hand::jp_type> input_ft_type;
public:		Input<Hand::jp_type> mean_input;
public:		Input<Hand::jp_type> std_input;
public:		Output<Hand::jp_type> output;
public:     float time_count;

protected:	Output<Hand::jp_type>::Value* outputValue;
protected:  Hand* hand;
protected:  bool* problem;
protected:  int problem_count;
protected:  stringstream* debug;

public:
	HandSystem(Hand* _hand, bool* _problem, std::stringstream* _debug, const std::string& sysName = "HandSystem") :
		systems::System(sysName), mean_input(this), std_input(this), output(this, &outputValue), hand(_hand), problem(_problem),debug(_debug)
		{
            time_count = 0;
            problem_count = 0;
            //cout << "hand_system instantiated" << endl;
        }

	virtual ~HandSystem() { mandatoryCleanUp(); }

protected:
    Hand::jp_type fingertip_torque_readings;

	virtual void operate() {
        
		const Hand::jp_type& expected_mean = mean_input.getValue(); // Pull data from the input
		const Hand::jp_type& expected_std  = std_input.getValue();  // Pull data from the input
        
        hand->update(Hand::S_FINGERTIP_TORQUE);
        std::vector<int> vec_readings =  hand->getFingertipTorque();
        
        int num_sigmas = 5;

        time_count += 0.002;  //500 Hz

        float contact_threshold = 1950;
        
        float mean     = expected_mean[0];
        float std_up   = mean + num_sigmas * expected_std[0];
        float std_down = mean - num_sigmas * expected_std[0];
        float val      = vec_readings[0];

        string sep = " ";
        string vm = "~";

        bool should_contact = expected_mean[0] > contact_threshold;
        bool is_contact     = fingertip_torque_readings[0] > contact_threshold;

        //if time_count is an integer 
        //if(ceil(time_count*500) == time_count*500){
        //}
        for(int i = 0; i < 1; i++){ //finger 1 only
            fingertip_torque_readings[i] = vec_readings[i]; 
            if( should_contact &&
                (fingertip_torque_readings[i] > std_up || fingertip_torque_readings[i] < std_down)){
                problem_count += 1;
                if(problem_count > 30){
                    *problem = true;
                    problem_count = 0;
                }
                /*
                debug->str("");
                *debug << time_count << ": ";

                if(val < std_down)
                    *debug << vm<<val<<vm   << sep << std_down      << sep << mean          << sep << std_up;
                else if(val < mean)
                    *debug << std_down      << sep << vm<<val<<vm   << sep << mean          << sep << std_up;
                else if(val < std_up)
                    *debug << std_down      << sep << mean          << sep << vm<<val<<vm   << sep << std_up;
                else//if(val < std_down)
                    *debug << std_down      << sep << mean          << sep << std_up        << sep << vm<<val<<vm;
                *problem = true;

                *debug << endl;
                */
            }
            else{
                problem_count = 0;
            }
        }
        outputValue->setData(&fingertip_torque_readings);  // Push data into the output
	}
};
/*cout << i << endl;
cout << expected_mean[i] + num_sigmas*expected_std[i] << endl;
cout << fingertip_torque_readings[i] << endl;
cout << expected_mean[i] - num_sigmas*expected_std[i] << endl;
cout << endl;*/

