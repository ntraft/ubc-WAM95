#include "SaA.h"
#include "control.h"
#include "memory.h"

SaA0::SaA0(Memory* _memory, RobotController* _controller):ControlStrategy(_memory, _controller){}
void SaA0::invoke(jp_type* output_jp, 
        const Hand::jp_type* expected_mean_ft, 
        const Hand::jp_type* actual_ft, 
        Hand::jp_type* problem_count_ft){}
SaA1::SaA1(Memory* _memory, RobotController* _controller):ControlStrategy(_memory, _controller){}
void SaA1::invoke(jp_type* output_jp, 
        const Hand::jp_type* expected_mean_ft, 
        const Hand::jp_type* actual_ft, 
        Hand::jp_type* problem_count_ft){
    if((*problem_count_ft)[0] >= memory->get_float("problem_count_threshold")){ 
        if((*actual_ft)[0] < (*expected_mean_ft)[0]){ //contact problem
            (*output_jp)[5] += 0.02;
        }
        else if((*actual_ft)[0] > (*expected_mean_ft)[0]){ //stub problem
            (*output_jp)[5] -= 0.02;
        }
        (*problem_count_ft)[0] = 0;
    }
}
SaA2::SaA2(Memory* _memory, RobotController* _controller):ControlStrategy(_memory, _controller){}
void SaA2::invoke(jp_type* output_jp, 
        const Hand::jp_type* expected_mean_ft, 
        const Hand::jp_type* actual_ft, 
        Hand::jp_type* problem_count_ft){}
SaA3::SaA3(Memory* _memory, RobotController* _controller):ControlStrategy(_memory, _controller){}
void SaA3::invoke(jp_type* output_jp, 
        const Hand::jp_type* expected_mean_ft, 
        const Hand::jp_type* actual_ft, 
        Hand::jp_type* problem_count_ft){}
