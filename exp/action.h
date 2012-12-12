#ifndef ACTION_H_
#define ACTION_H_

class Experiment;
class RobotController;
class Senses;
class Robot;

enum ACTION_PHASES{
        APPROACH = -1,
        PRELOAD,
        LOADING,
        TRANSITIONAL,
        STATIC,
        REPLACEMENT,
        UNLOADING,
        NUM_ACTION_PHASES
};

class Action: public Experiment{
public:
    Action(Robot* robot);
    virtual void run();
};

class ActionPhase: public Action{
public:
    ActionPhase(Robot* robot);
    void run();
};
class SimpleShapes: public Action{
public:
    SimpleShapes(Robot* robot);
    void run();
};
#endif
