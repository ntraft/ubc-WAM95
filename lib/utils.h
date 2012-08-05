#ifndef UTILS_H_
#define UTILS_H_

#include "stdheader.h"

//Function defns
//returns a random float between a and b
float randomFloat(float a, float b);
//convert a number to a string
std::string num2str(int number);
std::string num2str(float number);
std::string num2str(double number);
//The following two functions convert between a 4-element vector and quaternion
Eigen::Quaterniond hjp2quaternion(Hand::jp_type* p);
Hand::jp_type quaternion2hjp(Eigen::Quaterniond* q);
// parses a string of space-separated doubles into a vector of doubles
template<int R, int C, typename Units>
bool parseDoubles(math::Matrix<R,C, Units>* dest, const std::string& str){
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
std::string toString(math::Matrix<R,C, Units>* src){
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
//first converts quaternion to hand joint position vector (same size = 4)
std::string toString(Eigen::Quaterniond* src);
//copy the value v to up to n (0 for all) element s of vector dest
//negative values for n indicate all except last n elements
template<int R, int C, typename Units>
void setVectorValues(math::Matrix<R,C, Units>* dest, float v, int n){
        if(n <= 0)
                n = dest->size()+n;
        for(int i = 0; i < n; i++){
                (*dest)[i] = v;
        }
}
//add the value v to up to n (0 for all) elements of vector dest
//negative values for n indicate all except last n elements
template<int R, int C, typename Units>
void addVectorValues(math::Matrix<R,C, Units>* dest, float v, int n){
        if(n <= 0)
                n = dest->size()+n;
        for(int i = 0; i < n; i++){
                (*dest)[i] += v;
        }
}
//add two vectors dest and v and store result in dest
template<int R, int C, typename Units>
void addVectorValues(math::Matrix<R,C, Units>* dest, math::Matrix<R,C, Units>* v){
        for(int i = 0; i < v->size(); i++){
                (*dest)[i] += (*v)[i];
        }
}
//multiply elements of two vectors dest and v and store result in dest
template<int R, int C, typename Units>
void multVectorValues(math::Matrix<R,C, Units>* dest, math::Matrix<R,C, Units>* v){
        for(int i = 0; i < v->size(); i++){
                (*dest)[i] *= (*v)[i];
        }
}
//obtain magnitude of each element-wise step from one vector to another
template<int R, int C, typename Units>
void getInterpolatingSteps(     math::Matrix<R,C, Units>* step,
                                                        math::Matrix<R,C, Units>* from,
                                                        math::Matrix<R,C, Units>* to,
                                                        float num_steps){
        for(int i = 0; i < step->size(); i++){
                (*step)[i] = ((*to)[i] - (*from)[i]) / num_steps;
        }
}
#endif /* UTILS_H_ */
