#ifndef ACTIVE_H_
#define ACTIVE_H_

class Experiment;
class RobotController;
class Senses;
class Robot;

class Active: public Experiment{
public:
    Active(Robot* robot);
    virtual void run();
};

/*class SimpleShapes: public Active{
public:
    SimpleShapes(Robot* robot);
    void run();
};*/
#endif
