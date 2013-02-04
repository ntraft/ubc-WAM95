#include "stdheader.h"
#include "rtcontrol.h"
#include "senses.h"
#include "control.h"
#include "utils.h"
#include "utils-inl.h"
#include "control_strategy.h"
#include "SaS.h"
#include "SaA.h"
//#include "co_type.h"

using namespace Eigen;

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
        if((*mat1)[i] > (*ub)[i] || (*mat1)[i] < (*lb)[i]){ (*dest)[i]++; }
        else{ (*dest)[i] = 0; } } }
//checks all entries in src and dest and adds 1 to each entry where mat1 is less than mat2 
template<int R, int C, typename Units>
void rtc_entries_less_than_count(math::Matrix<R,C, Units>* dest, const math::Matrix<R,C, Units>* mat1, math::Matrix<R,C, Units>* mat2){
    for (int i = 0; i < dest->size(); ++i) {
        if((*mat1)[i] < (*mat2)[i]){ (*dest)[i]++; }
        else{ (*dest)[i] = 0; } } }
void RTControl::init(){
    set_vector_values(&offsets_jp, 0, 0);
    set_vector_values(&offsets_cp, 0, 0);
    transform_qd = 
        AngleAxisd(memory->get_float("transform_qd_x"), Vector3d::UnitX()) * 
        AngleAxisd(memory->get_float("transform_qd_y"), Vector3d::UnitY()) * 
        AngleAxisd(memory->get_float("transform_qd_z"), Vector3d::UnitZ());
}
void RTControl::operate() {
        int num_sigmas = (int)memory->get_float("num_sigmas");
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
        if(memory->get_float("trajectory_type") == 0){
            if(control_strategy != NULL){
                //control_strategy->invoke(&offsets_jp, &expected_mean_ft, &actual_ft, problem_count_ft);
                //*debug << "invoke" << endl;
            }
            //out_jp = actual_jp + offsets_jp;
            //output_value_jp->setData(&out_jp);
        }else{
            if(control_strategy != NULL){
                //control_strategy->invoke(&offsets_cp, &transform_qd, &expected_mean_ct, &actual_ct, problem_count_ct);
                //*debug << "invoke" << endl;
            }
            out_cp = actual_cp + offsets_cp;
            actual_qd = co2qd(&actual_co);
            new_qd = transform_qd * actual_qd;
            out_co = qd2co(&new_qd);
            output_value_cp->setData(&out_cp);
            output_value_co->setData(&out_co);
            //*debug << "transform_qd: " << transform_qd.w() << ", " << transform_qd.x() << ", " << transform_qd.y() << ", " << transform_qd.z() << endl;
            //*debug << "new_qd: " << out_co << endl;
            //output_value_qd->setData(&new_qd);
            //memory->set_string("rtc_dbg",string("qdw: ") + num2str(new_qd.w()) + "co0: " + num2str(out_co[0]));
            //output_value_qd->setData(&actual_qd);
            output_value_qd->setData(&new_qd);
           // output_value_jp->setData(&actual_jp);
        }
        //const double& time = input_time.getValue(); 
        //output_value_time->setData(&time);
}
