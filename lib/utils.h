#ifndef UTILS_H_
#define UTILS_H_

#include "stdheader.h"

//Function defns
//cast integer to string
std::string itoa(int i);
//returns a random float between a and b
float random_float(float a, float b);
//convert a number to a string
std::string num2str(int number);
std::string num2str(float number);
std::string num2str(double number);
//The following two functions convert between a 4-element vector and quaternion
qd_type co2qd(const co_type* co, co_type* tare_values);
co_type qd2co(const qd_type* qd, co_type* tare_values);
qd_type co2qd(const co_type* co);
co_type qd2co(const qd_type* qd);
Eigen::Quaterniond jp2quaternion(jp_type* p);
jp_type quaternion2jp(Eigen::Quaterniond* q);
//first converts quaternion to hand joint position vector (same size = 4)
std::string to_string(qd_type* src); 
void stop_thread(bool* semaphore);

#include "utils-inl.h"

#endif /* UTILS_H_ */
