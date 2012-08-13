#include "utils.h"

//NOTE: other (template) functions are defined in the header file

//returns a random float between a and b
float random_float(float a, float b) {
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
//The following two functions convert between a 7-element vector and quaternion
Eigen::Quaterniond jp2quaternion(jp_type* p){
	return Eigen::Quaterniond((*p)[0], (*p)[1], (*p)[2], (*p)[3]);
}
jp_type quaternion2jp(Eigen::Quaterniond* q){
    jp_type jp;
    jp[0] =	q->w();
    jp[1] = q->x(), 
    jp[2] = q->y(); 
    jp[3] = q->z();
    jp[4] = 0; 
    jp[5] = 0; 
    jp[6] = 0;
    return jp;
}
//first converts quaternion to hand joint position vector (same size = 4)
std::string to_string(Eigen::Quaterniond* src) {
	Hand::jp_type temp = quaternion2hjp(src); 
	return to_string(&temp);
}
void stop_thread(bool* semaphore){
	waitForEnter();
	*semaphore = !*semaphore;
}
