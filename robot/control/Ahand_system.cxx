/* ex05_systems_intro.cpp
 *
 * This example introduces libbarrett's concept of "Systems". Systems are used
 * to describe the calculations that are performed in the real time control-loop
 * that runs the WAM. The code below defines and uses a simple System. For
 * clarity, this example does not execute any code in real time.
 *
 * Systems are designed to make it easy to have low level control over the
 * behavior of the WAM's control loop with out getting bogged down in the
 * details of real time programming. It's not necessary to understand Systems if
 * you don't need this functionality.
 *
 * Systems are analogous to blocks in a block diagram. They have zero or more
 * System::Inputs and zero or more System::Outputs. Inputs and Outputs are pipes
 * through which data flow. An Output can be connected to many Inputs using the
 * systems::connect() function. An Input can only be connected to one Output at
 * a time.
 *
 * Each System encodes a specific "behavior": an operation that it performs on
 * the data it receives from its Inputs in order to produce the data it puts
 * into its Outputs. The creator of the System is responsible for implementing
 * this behavior in the System::operate() function. libbarrett has many built-in
 * Systems that live in the barrett/systems/ folder, such as:
 *   - systems::Callback for quickly wrapping a function in a System
 *   - systems::ExposedOutput for taking a piece of data and making it available
 *     to other Systems (see example below)
 *   - systems::FirstOrderFilter for filtering a stream of data
 *   - systems::Gain for multiplying by a constant
 *   - systems::PIDRobotController for making simple feedback control systems
 *   - systems::Summer for adding two or more streams of data
 *   - systems::Wam for interacting with a WAM in real time (can also be used to
 *     interact asynchronously, as shown in Examples 1 through 4)
 *
 * A System's operate() function is never called directly. Instead, a
 * systems::ExecutionManager handles the task of figuring out which operate()
 * functions need to be called and making sure that the necessary Input data is
 * valid and updated in time. (Much of this default behavior can be controlled
 * by re-implementing virtual functions from the systems::System class.) There
 * are multiple kinds of ExecutionManager, the main ones being
 * systems::ManualExecutionManager and systems::RealTimeExectuionManager. The
 * simple example below uses a ManualExecutionManager. The WAM's real time
 * control loop uses a RealTimeExecutionManager.
 */


#include <vector>
#include <string>

// Base class: barrett::systems::System
#include <barrett/systems/abstract/system.h>

// Includes the rest of the barrett::systems namespace, which contains all
// non-abstract Systems
#include <barrett/systems.h>

#include "stdheader.h"


using namespace barrett;
using namespace systems;


// A System that outputs four fingertip_torque values and four tactile pressure values 
// each tactile pressure value is the summation of individual sensors on the pad
class HandSystem : public systems::System {

	typedef boost::tuple<Hand::jp_type, Hand::jp_type> input_ft_type;

// IO
// Marked as "public" because Inputs and Output are (except in special cases)
// part of a System's public interface
public:		Input<Hand::jp_type> mean_input;
public:		Input<Hand::jp_type> std_input;
public:		Output<Hand::jp_type> output;
public:     float time_count;

protected:	Output<Hand::jp_type>::Value* outputValue;
protected:  Hand* hand;
protected:  bool* problem;
protected:  bool* contact_problem;
protected:  bool* stub_problem;
protected:  int problem_count;
protected:  int contact_problem_count;
protected:  int stub_problem_count;
protected:  stringstream* debug;

public:
	HandSystem(Hand* _hand, bool* _problem, bool* _contact_problem, bool* _stub_problem, std::stringstream* _debug, const std::string& sysName = "HandSystem") :
		systems::System(sysName), mean_input(this), std_input(this), output(this, &outputValue), hand(_hand), problem(_problem), contact_problem(_contact_problem),stub_problem(_stub_problem),debug(_debug)
		{
            time_count = 0;
            problem_count = 0;
            contact_problem_count = 0;
            stub_problem_count = 0;
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
        int num_problems = 40;

        time_count += 0.002;  //500 Hz

        float contact_threshold = 1950;
        float stub_threshold = 2200;
        
        float mean     = expected_mean[0];
        float std_up   = mean + num_sigmas * expected_std[0];
        float std_down = mean - num_sigmas * expected_std[0];
        float val      = vec_readings[0];

        string sep = " ";
        string vm = "~";

        bool should_contact = expected_mean[0] > contact_threshold;
        bool is_contact     = fingertip_torque_readings[0] > contact_threshold;
        bool is_stubbed     = fingertip_torque_readings[0] > stub_threshold;

        if(*contact_problem){
            *contact_problem = false;
            contact_problem_count = 0;
        }
        if(should_contact && !is_contact){
            contact_problem_count++;
            if(contact_problem_count > num_problems*0.75){
                *contact_problem = true;
                contact_problem_count = 0;
                *debug << "contact_problem" << endl;
            }
        }
        else{
            contact_problem_count = 0;
        }

        if(*stub_problem){
            *stub_problem = false;
            stub_problem_count = 0;
        }
        if(is_stubbed){
            stub_problem_count++;
            if(stub_problem_count > num_problems*0.75){
                *stub_problem = true;
                stub_problem_count = 0;
                *debug << "stub_problem" << endl;
            }
        }
        else{
            stub_problem_count = 0;
        }

        //*contact_problem = should_contact && !is_contact;

        //if time_count is an integer 
        //if(ceil(time_count*500) == time_count*500){
        //}
        for(int i = 0; i < 1; i++){ //finger 1 only
            fingertip_torque_readings[i] = vec_readings[i]; 
            if( should_contact &&
                (fingertip_torque_readings[i] > std_up || fingertip_torque_readings[i] < std_down)){
                problem_count += 1;
                if(problem_count > num_problems){
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

