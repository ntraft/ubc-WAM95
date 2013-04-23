#ifndef UTILS_INL_H_
#define UTILS_INL_H_

#include "stdheader.h"
#include <cfloat>

//zeros all values in matrix
template<int R, int C, typename Units>
void zero_matrix(math::Matrix<R,C, Units>* dest){
    for (int i = 0; i < dest->size(); ++i) {
        (*dest)[i] = 0;
    }
}
// parses a string of space-separated doubles into a vector of doubles
template<int R, int C, typename Units>
inline bool parseDoubles(math::Matrix<R,C, Units>* dest, const std::string& str){
        const char* cur = str.c_str();
        const char* next = cur;

        for (int i = 0; i < dest->size(); ++i) {
                (*dest)[i] = strtod(cur, (char**) &next);
                if (cur == next) {
                        return false;
                } else {
                        cur = next;
                }
        }

        // Make sure there are no extra numbers in the string.
        double ignore = strtod(cur, (char**) &next);
        (void)ignore;  // Prevent unused variable warnings

        if (cur != next) {
                return false;
        }

        return true;
}
// converts a vector of doubles into a string of space-separated doubles
template<int R, int C, typename Units>
std::string to_string(math::Matrix<R,C, Units>* src){
        std::string str;
        int i;
        for (i = 0; i < src->size(); ++i) {
                char buff[50];
                sprintf(buff, "%f",(*src)[i]);
                str.append(buff);
                if(i < src->size()-1)
                        str.append(" ");
                else
                        str.append("\n");
        }
        return str;
}
// converts a vector of doubles into a string of sep_str-separated doubles with optional prefix/suffix
template<int R, int C, typename Units>
std::string to_string(math::Matrix<R,C, Units>* src, string sep_str, string prefix = "", string suffix = ""){
    for (int i = 0; i < src->size(); ++i) {
        char buff[50];
        sprintf(buff, "%f",(*src)[i]);
        prefix.append(buff);
        if(i < src->size()-1)
            prefix.append(sep_str);
        else
            prefix.append(suffix);
    }
    return prefix;
}
//copies values from one matrix to another (must both be same size)
template<int R, int C, typename Units>
void copy_matrix(math::Matrix<R,C, Units>* dest, const math::Matrix<R,C, Units>* src){
    for (int i = 0; i < dest->size(); ++i) {
        (*dest)[i] = (*src)[i];
    }
}
//checks all entries in src and dest and adds 1 to each entry where mat1 is greater than mat2 
template<int R, int C, typename Units>
void entries_greater_than_count(math::Matrix<R,C, Units>* dest, const math::Matrix<R,C, Units>* mat1, math::Matrix<R,C, Units>* mat2){
    for (int i = 0; i < dest->size(); ++i) {
        float x = (*mat1)[i];
        float y = (*mat2)[i];
        int inc = (int)(x > y);
        (*dest)[i] += inc;
    }
}
//checks all entries in src and dest and adds 1 to each entry where mat1 is less than mat2 
template<int R, int C, typename Units>
void entries_less_than_count(math::Matrix<R,C, Units>* dest, const math::Matrix<R,C, Units>* mat1, math::Matrix<R,C, Units>* mat2){
    for (int i = 0; i < dest->size(); ++i) {
        float x = (*mat1)[i];
        float y = (*mat2)[i];
        int inc = (int)(x < y);
        (*dest)[i] += inc;
    }
}
//copy the value v to up to n (0 for all) element s of vector dest
//negative values for n indicate all except last n elements
template<int R, int C, typename Units>
void set_vector_values(math::Matrix<R,C, Units>* dest, float v, int n){
        if(n <= 0)
                n = dest->size()+n;
        for(int i = 0; i < n; i++){
                (*dest)[i] = v;
        }
}
//add the value v to up to n (0 for all) elements of vector dest
//negative values for n indicate all except last n elements
template<int R, int C, typename Units>
math::Matrix<R,C,Units>* add_vector_values(math::Matrix<R,C, Units>* dest, float v, int n){
        if(n <= 0)
                n = dest->size()+n;
        for(int i = 0; i < n; i++){
                (*dest)[i] += v;
        }
        return dest;
}
//add two vectors dest and v and store result in dest
template<int R, int C, typename Units>
void add_vector_values(math::Matrix<R,C, Units>* dest, math::Matrix<R,C, Units>* v){
        for(int i = 0; i < v->size(); i++){
                (*dest)[i] += (*v)[i];
        }
}
//subtract the value v to up to n (0 for all) elements of vector dest
//negative values for n indicate all except last n elements
template<int R, int C, typename Units>
void sub_vector_values(math::Matrix<R,C, Units>* dest, float v, int n){
        if(n <= 0)
                n = dest->size()+n;
        for(int i = 0; i < n; i++){
                (*dest)[i] -= v;
        }
}
//subtract v from dest and store result in dest
template<int R, int C, typename Units>
void sub_vector_values(math::Matrix<R,C, Units>* dest, math::Matrix<R,C, Units>* v){
        for(int i = 0; i < v->size(); i++){
                (*dest)[i] -= (*v)[i];
        }
}
//multiply the value v to up to n (0 for all) elements of vector dest
//negative values for n indicate all except last n elements
template<int R, int C, typename Units>
void mult_vector_values(math::Matrix<R,C, Units>* dest, float v, int n){
        if(n <= 0)
                n = dest->size()+n;
        for(int i = 0; i < n; i++){
                (*dest)[i] *= v;
        }
}
//multiply elements of two vectors dest and v and store result in dest
template<int R, int C, typename Units>
void mult_vector_values(math::Matrix<R,C, Units>* dest, math::Matrix<R,C, Units>* v){
        for(int i = 0; i < v->size(); i++){
                (*dest)[i] *= (*v)[i];
        }
}
//obtain magnitude of each element-wise step from one vector to another
template<int R, int C, typename Units>
void get_interpolating_steps(     math::Matrix<R,C, Units>* step,
                                                        math::Matrix<R,C, Units>* from,
                                                        math::Matrix<R,C, Units>* to,
                                                        float num_steps){
        for(int i = 0; i < step->size(); i++){
                (*step)[i] = ((*to)[i] - (*from)[i]) / num_steps;
        }
}
//store e^dest[i] for each i in dest
template<int R, int C, typename Units> void exp_vector_values(math::Matrix<R,C, Units>* dest){
    for(int i = 0; i < dest->size(); i++){ (*dest)[i] = ((*dest)[i] < -20) ? (2.06e-9) : exp((*dest)[i]); } }
//store dest[i]^2 for each i in dest
template<int R, int C, typename Units> void square_vector_values(math::Matrix<R,C, Units>* dest){
    for(int i = 0; i < dest->size(); i++){ (*dest)[i] = pow((*dest)[i],2); } }
//divide v from dest and store result in dest
template<int R, int C, typename Units> void div_vector_values(math::Matrix<R,C, Units>* dest, math::Matrix<R,C, Units>* v){
    for(int i = 0; i < v->size(); i++){ (*dest)[i] /= (*v)[i]; } }
//store log(dest[i]) for each i in dest. If dest[i] = 0 then log(dest[i]) = -99
template<int R, int C, typename Units> void log_vector_values(math::Matrix<R,C, Units>* dest){
    for(int i = 0; i < dest->size(); i++){ (*dest)[i] = ((*dest)[i] == 0) ? (-99) : (std::log((*dest)[i])); } }
//return sum(dest)
template<int R, int C, typename Units> float sum_vector_values(math::Matrix<R,C, Units>* dest){
    float sum = 0; for(int i = 0; i < dest->size(); i++){ sum += (*dest)[i]; } return sum; }
//return max(dest)
template<int R, int C, typename Units> float max_vector_values(math::Matrix<R,C, Units>* dest){
    float max = FLT_MIN; for(int i = 0; i < dest->size(); i++){ max = (*dest)[i] > max ? (*dest)[i] : max; } return max; }
#endif /* UTILS_INL_H_ */
