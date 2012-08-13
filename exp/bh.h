#ifndef BH_H_
#define BH_H_

class Experiment;
class Controller;
class Senses;

class BHExp: public Experiment{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    BHExp(Controller* controller, Senses* senses);
    virtual void run();
};

class BHTorque: public BHExp{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    BHTorque(Controller* controller, Senses* senses);
    virtual void run();
};
#endif
