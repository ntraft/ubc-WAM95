#ifndef MEMORY_H_
#define MEMORY_H_

//#include "stdheader.h"

//typedef Eigen::Quaterniond qd_type;
//using namespace std;
//
#include <string>

template<class T>class VarServer;

class Memory{
private:
    VarServer<float>* float_server;
    VarServer<std::string>* string_server;
    //qd_type transform_qd;
public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    Memory();
    void reload_vars();
    std::string get_string(std::string name);
    float get_float(std::string name);
    void set_string(std::string name, std::string value);
    void set_float(std::string name, float value);
    void toggle_float(std::string name);
    //void set_qd_transform(float x, float y, float z);
    //qd_type* get_qd_transform();
};

#endif /* IO_H_ */
