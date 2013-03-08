#include "utils.h"
#include "utils-inl.h"

//NOTE: other (template) functions are defined in the header file

//cast integer to string
std::string itoa(int i){std::string a = boost::lexical_cast<std::string>(i);return a;}
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
qd_type co2qd(const co_type* co, co_type* tare_values){
    qd_type qd(
            (*co)[0]-(*tare_values)[0], 
            (*co)[1]-(*tare_values)[1], 
            (*co)[2]-(*tare_values)[2], 
            (*co)[3]-(*tare_values)[3]);
    qd.normalize();
	return qd; 
}
co_type qd2co(const qd_type* qd, co_type* tare_values){
	return Hand::jp_type(qd->w(), qd->x(), qd->y(), qd->z()) - *tare_values;
}
//The following two functions convert between a 4-element vector and quaternion
qd_type co2qd(const co_type* co){
    qd_type qd((*co)[0], (*co)[1], (*co)[2], (*co)[3]);
    qd.normalize();
	return qd; 
}
co_type qd2co(const qd_type* qd){
	return Hand::jp_type(qd->w(), qd->x(), qd->y(), qd->z());
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
std::string to_string(qd_type* src) {
	co_type temp = qd2co(src); 
	return to_string(&temp);
}
void stop_thread(bool* semaphore){
	waitForEnter();
	*semaphore = !*semaphore;
}
