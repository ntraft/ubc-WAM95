#ifndef MEMORY_H_
#define MEMORY_H_

//#include "stdheader.h"
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#define DIMENSION 7u
#include <Eigen/Geometry>
#include <string>
#include <barrett/math.h>  // For barrett::math::saturate()
#include <barrett/units.h>
typedef Eigen::Quaterniond qd_type;
BARRETT_UNITS_TYPEDEFS(DIMENSION);
//typedef math::Vector3<double> cp_type;
using namespace std;

template<class T>class VarServer;

class Memory{
private:
    VarServer<float>* float_server;
    VarServer<std::string>* string_server;
    qd_type transform_qd;
    cp_type transform_cp;
public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    Memory();
    void reload_vars();
    void writeout_vars();
    std::string get_string(std::string name);
    float get_float(std::string name);
    void set_string(std::string name, std::string value);
    void set_float(std::string name, float value);
    void toggle_float(std::string name);
    void set_transform_qd(float x, float y, float z);
    qd_type get_transform_qd();
    void set_transform_cp(float x, float y, float z);
    cp_type get_transform_cp();
    jp_type get_initial_jp();
};

#endif /* IO_H_ */
