#ifndef BH_H_
#define BH_H_

class Experiment;
class RobotController;
class Senses;
class Robot;

class BHExp: public Experiment{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    BHExp(Robot* robot);
    virtual void run();
};

class BHTorque: public BHExp{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    BHTorque(Robot* robot);
    virtual void run();
};
#endif
