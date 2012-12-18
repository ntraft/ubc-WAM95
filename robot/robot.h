#ifndef ROBOT_H_
#define ROBOT_H_

#include "stdheader.h"
#include "mainline.h"

class Senses;
class RobotController;

class Robot: public MainLine{
    private:
        ProductManager* pm;
        systems::Wam<DIMENSION>* wam;
        Hand* hand;
        ForceTorqueSensor* fts;
        Senses* senses;
        RobotController* controller;

    public:
        Robot(ProductManager* pm, systems::Wam<DIMENSION>* wam);
        void init_wam();
        void init_hand();
        void init_fts();

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
        RobotController* get_controller();
};
#endif
