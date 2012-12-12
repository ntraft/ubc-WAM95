#ifndef TACTILE_H_
#define TACTILE_H_

class Experiment;
class RobotController;
class Senses;
class Robot;

class Tactile: public Experiment{
public:
    Tactile(Robot* robot);
    virtual void run();
};

class CartesianRaster: public Tactile{
public:
    CartesianRaster(Robot* robot);
    void run();
};
/*class SimpleShapes: public Active{
public:
    SimpleShapes(Robot* robot);
    void run();
};*/
#endif
