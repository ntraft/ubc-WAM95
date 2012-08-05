#include "utils.h"

//NOTE: other (template) functions are defined in the header file

//returns a random float between a and b
float randomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}
//convert a number to a string
std::string num2str(int number){
   std::stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}
std::string num2str(float number){
   std::stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}
std::string num2str(double number){
   std::stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}
//The following two functions convert between a 4-element vector and quaternion
Eigen::Quaterniond hjp2quaternion(Hand::jp_type* p){
	return Eigen::Quaterniond((*p)[0], (*p)[1], (*p)[2], (*p)[3]);
}
Hand::jp_type quaternion2hjp(Eigen::Quaterniond* q){
	return Hand::jp_type(q->w(), q->x(), q->y(), q->z());
}
//first converts quaternion to hand joint position vector (same size = 4)
std::string toString(Eigen::Quaterniond* src) {
	Hand::jp_type temp = quaternion2hjp(src); 
	return toString(&temp);
}
