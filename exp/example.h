#ifndef EXAMPLE_H_
#define EXAMPLE_H_

#include "stdheader.h"

class RobotController;
class Senses;

class Example: public Experiment{
private:
protected:
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
	Example(RobotController* controller, Senses* senses);

    void init(std::string args);
    std::string get_help_string();
    virtual void run();
};

class HoldPosition: public Example{
private:
protected:
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    Example(RobotController* controller, Senses* senses);

    void init(std::string args);
    std::string get_help_string();
    virtual void run();
};

class RealtimeMove;
class TeachAndPlay;
class TorqueControl;
class Haptics;

#endif
