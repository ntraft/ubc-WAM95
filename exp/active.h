#ifndef ACTIVE_H_
#define ACTIVE_H_

class Experiment;
class Controller;
class Senses;

class Active: public Experiment{
public:
    Active(Controller* controller, Senses* senses);
    virtual void run();
};

/*class SimpleShapes: public Active{
public:
    SimpleShapes(Controller* controller, Senses* senses);
    void run();
};*/
#endif
