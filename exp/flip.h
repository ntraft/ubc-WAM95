#ifndef FLIP_H_
#define FLIP_H_

class Experiment;
class RobotController;
class Senses;
class Robot;

class Flip: public Experiment{
public:
    Flip(Robot* robot);
    virtual void run();
    void teach_and_play();
};

class FlipTilt: public Flip{
public:
    FlipTilt(Robot* robot);
    virtual void run();
};
#endif
