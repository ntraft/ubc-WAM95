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
    
    senses = new Senses(this);
    controller = new RobotController(this);

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
ProductManager* Robot::getPM(){return pm;}
systems::Wam<DIMENSION>* Robot::getWAM(){return wam;}
ForceTorqueSensor* Robot::getFTS(){return fts;}
Hand* Robot::getHand(){return hand;}
RobotController* Robot::getRobotController(){return controller;}
Senses* Robot::getSenses(){return senses;}
