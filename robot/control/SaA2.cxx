#ifndef _SAA2_CXX
#define _SAA2_CXX

#include "control_strategy.cxx"

class RobotController;

class SaA2 : public ControlStrategy{
private:
    RobotController* controller;
public:
    SaA2(RobotController* _controller):ControlStrategy(_controller),controller(_controller){}
};

#endif
