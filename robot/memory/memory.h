#ifndef MEMORY_H_
#define MEMORY_H_

#include "stdheader.h"

typedef Eigen::Quaterniond qd_type;
using namespace std;

template<class T>class VarServer;

class Memory{
private:
    VarServer<float>* float_server;
    VarServer<string>* string_server;
    qd_type transform_qd;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    Memory();
    void reload_vars();
    string get_string(string name);
    float get_float(string name);
    void set_string(string name, string value);
    void set_float(string name, float value);
    void toggle_float(string name);
    void set_qd_transform(float x, float y, float z);
    qd_type* get_qd_transform();
};

#endif /* IO_H_ */
