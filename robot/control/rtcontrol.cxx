#include "stdheader.h"
#include "rtcontrol.h"
#include "senses.h"
#include "control.h"
#include "utils.h"
#include "utils-inl.h"
#include "control_strategy.cxx"
#include "SaS.cxx"
#include "SaA.cxx"

Hand::jp_type vec2hjp(vector<int>* v){
    Hand::jp_type j;
    j[0] = (*v)[0];
    j[1] = (*v)[1];
    j[2] = (*v)[2];
    j[3] = (*v)[3];
    return j;
}

//checks all entries in src and dest and adds 1 to each entry where mat1 is greater than mat2 
template<int R, int C, typename Units>
void rtc_entries_greater_or_less_than_count(math::Matrix<R,C, Units>* dest, const math::Matrix<R,C, Units>* mat1, math::Matrix<R,C, Units>* ub, math::Matrix<R,C, Units>* lb){
    for (int i = 0; i < dest->size(); ++i) {
        if((*mat1)[i] > (*ub)[i] || (*mat1)[i] < (*lb)[i]){
            (*dest)[i]++;
        }
        else{
            (*dest)[i] = 0;
        }
    }
}
//checks all entries in src and dest and adds 1 to each entry where mat1 is less than mat2 
template<int R, int C, typename Units>
void rtc_entries_less_than_count(math::Matrix<R,C, Units>* dest, const math::Matrix<R,C, Units>* mat1, math::Matrix<R,C, Units>* mat2){
    for (int i = 0; i < dest->size(); ++i) {
        if((*mat1)[i] < (*mat2)[i]){
            (*dest)[i]++;
        }
        else{
            (*dest)[i] = 0;
        }
     }
}

void RTControl::operate() {
        int num_sigmas = 6;
#define X(aa, bb, cc, dd, ee) const bb& expected_mean_##cc = mean_input_##cc.getValue();
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) const bb& expected_std_##cc = std_input_##cc.getValue();
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) const bb& actual_##cc = actual_input_##cc.getValue();
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) bb upper_bound_##cc;
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) bb lower_bound_##cc;
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) bb margin_##cc;
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X

#define X(aa, bb, cc, dd, ee) copy_matrix(&margin_##cc,&expected_std_##cc);
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) mult_vector_values(&margin_##cc,num_sigmas,0);
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) copy_matrix(&upper_bound_##cc,&expected_mean_##cc);
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) copy_matrix(&lower_bound_##cc,&expected_mean_##cc);
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) add_vector_values(&upper_bound_##cc,&margin_##cc);
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) sub_vector_values(&lower_bound_##cc,&margin_##cc);
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) rtc_entries_greater_or_less_than_count( problem_count_##cc,     &actual_##cc, &upper_bound_##cc, &lower_bound_##cc);
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
        /*
#define X(aa, bb, cc, dd, ee) rtc_entries_less_than_count(    problem_count_##cc,     &actual_##cc, &lower_bound_##cc);
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
*/
        /*
*/
        /*
#define X(aa, bb, cc, dd, ee) output_value_##cc->setData(problem_count_##cc);
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
*/
        debug->str("");
        //*debug << "ft[0]" << senses->get_fingertip_torque_value(0,true) << endl;
        //senses->get_hand()->update(Hand::S_FINGERTIP_TORQUE,true);
        //*debug << "ft[0]" << senses->get_hand()->getFingertipTorque()[0] << endl;
        //*debug << "cf[0]" << senses->get_force()[0] << endl;
/*
#define X(aa, bb, cc, dd, ee) \
        *debug << "ub_" << aa << ": " << upper_bound_##cc[0] << endl;\
        *debug << "lb_" << aa << ": " << lower_bound_##cc[0] << endl;\
        *debug << "ma_" << aa << ": " << margin_##cc[0] << endl;\
        *debug << "ac_" << aa << ": " << actual_##cc[0] << endl;\
        *debug << "pc_" << aa << ": " << (*problem_count_##cc)[0] << endl;\
        *debug << "~~~~" << endl;
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
        *debug << "~~~~~~~~~~" << endl;
*/
        jp_type new_jp = control_strategy->invoke(&actual_jp, &expected_mean_ft, &actual_ft, problem_count_ft);
        const double& time = input_time.getValue(); // Pull data from the input
        output_value_time->setData(&time);
        output_value_jp->setData(&new_jp);
}
