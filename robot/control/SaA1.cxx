#ifndef _SAA1_CXX
#define _SAA1_CXX

#include "control_strategy.cxx"

class RobotController;

class SaA1 : public ControlStrategy{
public:
    SaA1(RobotController* _controller):ControlStrategy(_controller){}
};

#endif
