#ifndef TACTILE_H_
#define TACTILE_H_

class Experiment;
class Controller;
class Senses;

class Tactile: public Experiment{
public:
    Tactile(Controller* controller, Senses* senses);
    virtual void run();
};

class CartesianRaster: public Tactile{
public:
    CartesianRaster(Controller* controller, Senses* senses);
    void run();
};
/*class SimpleShapes: public Active{
public:
    SimpleShapes(Controller* controller, Senses* senses);
    void run();
};*/
#endif
