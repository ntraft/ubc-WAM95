#ifndef _SAA1_h
#define _SAA1_h

#include "stdheader.h"
#include "control_strategy.h"

class RobotController;
class Memory;

class SaA0 : public ControlStrategy{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    SaA0(Memory* _memory, RobotController* _controller);
    virtual void invoke(jp_type* output_jp, 
            const Hand::jp_type* expected_mean_ft, 
            const Hand::jp_type* actual_ft, 
            Hand::jp_type* problem_count_ft);
};
class SaA1 : public ControlStrategy{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    SaA1(Memory* _memory, RobotController* _controller);
    virtual void invoke(jp_type* output_jp, 
            const Hand::jp_type* expected_mean_ft, 
            const Hand::jp_type* actual_ft, 
            Hand::jp_type* problem_count_ft);
};
class SaA2 : public ControlStrategy{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    SaA2(Memory* _memory, RobotController* _controller);
    virtual void invoke(jp_type* output_jp, 
            const Hand::jp_type* expected_mean_ft, 
            const Hand::jp_type* actual_ft, 
            Hand::jp_type* problem_count_ft);
};
class SaA3 : public ControlStrategy{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    SaA3(Memory* _memory, RobotController* _controller);
    virtual void invoke(jp_type* output_jp, 
            const Hand::jp_type* expected_mean_ft, 
            const Hand::jp_type* actual_ft, 
            Hand::jp_type* problem_count_ft);
};

#endif
