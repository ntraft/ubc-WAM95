#ifndef ACTION_H_
#define ACTION_H_

class Experiment;

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
    Action(systems::Wam<DIMENSION>* wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm);
    virtual void run();
};

class ActionPhase: public Action{
public:
    ActionPhase(systems::Wam<DIMENSION>* wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm);
    void run();
};
class SimpleShapes: public Action{
public:
    SimpleShapes(systems::Wam<DIMENSION>* wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm);
    void run();
};
#endif
