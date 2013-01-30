#ifndef _SAA3_CXX
#define _SAA3_CXX

#include "control_strategy.cxx"

class RobotController;

class SaA3 : public ControlStrategy{
private:
    RobotController* controller;
public:
    SaA3(RobotController* _controller):ControlStrategy(_controller),controller(_controller){}
};

#endif
