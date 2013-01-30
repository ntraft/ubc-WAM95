#include "control_strategy.h"
#include "utils-inl.h"
#include "memory.h"

ControlStrategy::ControlStrategy(Memory* _memory, RobotController* _controller):memory(_memory),controller(_controller){}

void ControlStrategy::invoke(jp_type* output_jp, 
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
void ControlStrategy::invoke(
        cp_type* offsets_cp, 
        qd_type* transform_qd, 
        const ct_type* expected_mean_ct, 
        const ct_type* actual_ct, 
        ct_type* problem_count_ct){
    if((*problem_count_ct)[1] >= memory->get_float("problem_count_threshold")){ 
        if((*actual_ct)[1] < (*expected_mean_ct)[1]){ //contact problem
            (*transform_qd) = AngleAxisd(0.1,Vector3d::UnitZ()) * (*transform_qd);
        }
        else if((*actual_ct)[1] > (*expected_mean_ct)[1]){ //stub problem
            (*transform_qd) = AngleAxisd(-0.1,Vector3d::UnitZ()) * (*transform_qd);
        }
        (*problem_count_ct)[1] = 0;
    }

}
