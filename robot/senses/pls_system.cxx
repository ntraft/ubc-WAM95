#include "stdheader.h"
#include "pls_system.h"
#include "utils.h"
#include "utils-inl.h"
#include <cfloat>
//divide v from dest and store result in dest
template<int R, int C, typename Units> void div_vector_values(math::Matrix<R,C, Units>* dest, double v){
    for(int i = 0; i < dest->size(); i++){ (*dest)[i] /= v; } }
//subtract v from dest and store result in dest
template<int R, int C, typename Units> void sub_vector_values(math::Matrix<R,C, Units>* dest, double v){
    for(int i = 0; i < dest->size(); i++){ (*dest)[i] -= v; } }
//return standard deviation of vector dest
template<int R, int C, typename Units> double std_vector_values(math::Matrix<R,C, Units>* dest, double mean = 0){
    math::Matrix<R,C,Units> copy; copy_matrix(&copy,dest);
    sub_vector_values(&copy,mean);
    double sum = sum_vector_values(&copy);
    double std = sqrt(sum / copy.size());
    return std; 
}
//return mean of vector dest
template<int R, int C, typename Units> double mean_vector_values(math::Matrix<R,C, Units>* dest){
    double sum = sum_vector_values(dest); return sum / dest->size(); }
//whiten dest and store result in dest
template<int R, int C, typename Units> void whiten_vector_values(math::Matrix<R,C, Units>* dest){
    math::Matrix<R,C,Units> copy; copy_matrix(&copy,dest);
    double mean = mean_vector_values(&copy);
    double std =  std_vector_values(&copy,mean);
    sub_vector_values(dest,mean);
    div_vector_values(dest,std);
}

void PartialLeastSquaresSystem::init(){
    zero_matrix(&predictions);
}
pv_type PartialLeastSquaresSystem::get_predictions(){
    return predictions;
}
void PartialLeastSquaresSystem::operate() {
    const double time = input_time.getValue();
    const in_type input_row = input_sensors.getValue();
    /*
#define P(aa, bb, cc, dd, ee) \
    const bb& actual_##cc = actual_input_##cc.getValue(); \
    input_beta.col(ee) = input_beta_##cc.getValue();
#include "parameter_table.h"
#undef P 
    //predictions = input_row * beta; //dot-product
    zero_matrix(&predictions);
#define P(aa, bb, cc, dd, ee) \
    in_type row_##cc = input_beta_##cc.getValue(); \
    whiten_vector_values(&row_##cc);
#include "parameter_table.h"
#undef P 
    for(int i = 0; i < input_row.SIZE; i++){
#define P(aa, bb, cc, dd, ee) \
        predictions[ee] += input_row[i] * row_##cc[i];
#include "parameter_table.h"
#undef P 
        //for(int j = 0; j < NUM_PARAMETERS; j++){
        //    predictions[j] += input_row[i] * input_beta(i,j);
        //}
    }
*/
    output_value_predictions->setData(&predictions);
}
