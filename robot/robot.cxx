#include "robot.h"
#include "stdheader.h"
#include "control.h"
#include "senses.h"
#include "experiment.h"

Robot::Robot(ProductManager* pm, systems::Wam<DIMENSION>* wam){
    this->pm = pm;
    this->wam = wam;
    this->fts = pm->getForceTorqueSensor();
    this->hand = pm->getHand();
    init_wam();
    init_hand();
    init_fts();
    
    senses = new Senses(pm, wam);
    controller = new RobotController(pm, wam, senses);
    experiment = new Experiment(pm, wam, senses, controller);

    module_name = "Robot";

    std::cout << "Robot initialized!" << std::endl;
    fflush(stdout);
}

void Robot::init_wam(){
}
void Robot::init_hand(){
}
void Robot::init_fts(){
   fts->tare();
}

//MAINLINE
void Robot::validate_args(){
    std::cout << "Robot validate args" << std::endl;
}
void Robot::run(){
    bool back = false;
    while (!back){//robot->get_pm()->getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
        step();
        switch (line[0]) {
#define X(aa, bb, cc, dd, ee) case bb: cc; break;
            #include "robot_table.h"
#undef X
            default: help(); break;
        }
    }
    exit();
}
void Robot::help(){
    printf("\n");
#define X(aa, bb, cc, dd, ee) printf("     bb dd\n");
    #include "robot_table.h"
#undef X
}

//ACCESSORS
ProductManager* Robot::get_pm(){return pm;}
systems::Wam<DIMENSION>* Robot::get_wam(){return wam;}
systems::Wam<DIMENSION>* Robot::getWAM(){return wam;}
ForceTorqueSensor* Robot::get_fts(){return fts;}
Hand* Robot::get_hand(){return hand;}
RobotController* Robot::get_controller(){return controller;}
Senses* Robot::get_senses(){return senses;}
