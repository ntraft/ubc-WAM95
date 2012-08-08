#ifndef ACTION_H_
#define ACTION_H_

class Experiment;
class Controller;
class Senses;

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
    Action(Controller* controller, Senses* senses);
    virtual void run();
};

class ActionPhase: public Action{
public:
    ActionPhase(Controller* controller, Senses* senses);
    void run();
};
class SimpleShapes: public Action{
public:
    SimpleShapes(Controller* controller, Senses* senses);
    void run();
};
#endif
