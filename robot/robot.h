#ifndef ROBOT_H_
#define ROBOT_H_

#include "stdheader.h"
#include "mainline.h"

class Senses;
class RobotController;
class Experiment;
class RTMemory;
class Memory;

class Robot: public MainLine{
    private:
        ProductManager* pm;
        systems::Wam<DIMENSION>* wam;
        Hand* hand;
        ForceTorqueSensor* fts;
        Senses* senses;
        RobotController* controller;
        RTMemory* rtmemory;
        Memory* memory;
        Experiment* experiment;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
        Robot();
        Robot(ProductManager* pm, systems::Wam<DIMENSION>* wam);
        void instantiate_components();
        void init_wam();
        void init_hand();
        void init_fts();
        void init_rt();
        void shutdown();

        //mainline
        virtual void help();
        virtual void validate_args();
        virtual void run();

        //accessors
        ProductManager* get_pm();
        systems::Wam<DIMENSION>* get_wam();
        systems::Wam<DIMENSION>* getWAM();
        ForceTorqueSensor* get_fts();
        Hand* get_hand();
        Senses* get_senses();
        RTMemory* get_rtmemory();
        Memory* get_memory();
        RobotController* get_controller();

        //mutators
        void update_sensors();
};
#endif
