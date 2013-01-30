#include "SaS.h"
#include "control.h"
#include "memory.h"

SaS::SaS(Memory* _memory, RobotController* _controller):
    ControlStrategy(_memory, _controller){}
void SaS::invoke(jp_type* output_jp, 
        const Hand::jp_type* expected_mean_ft, 
        const Hand::jp_type* actual_ft, 
        Hand::jp_type* problem_count_ft){
    if((*problem_count_ft)[0] >= memory->get_float("problem_count_threshold")){ 
        memory->set_float("realtime_control_break",1);
        (*problem_count_ft)[0] = 0;
    }
}
