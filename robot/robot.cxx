#include "robot.h"
#include "stdheader.h"
#include "control.h"
#include "senses.h"

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

    module_name = "Robot";

    std::cout << "Robot initialized!" << std::endl;
}

void Robot::init_wam(){
}
void Robot::init_hand(){
}
void Robot::init_fts(){
   fts->tare();
}

//MAINLINE
void Robot::help(){
    std::cout << "Robot help" << std::endl;
}
void Robot::validate_args(){
    std::cout << "Robot validate args" << std::endl;
}
void Robot::run(){
    step();
}

//ACCESSORS
ProductManager* Robot::get_pm(){return pm;}
systems::Wam<DIMENSION>* Robot::get_wam(){return wam;}
systems::Wam<DIMENSION>* Robot::getWAM(){return wam;}
ForceTorqueSensor* Robot::get_fts(){return fts;}
Hand* Robot::get_hand(){return hand;}
RobotController* Robot::get_controller(){return controller;}
Senses* Robot::get_senses(){return senses;}
