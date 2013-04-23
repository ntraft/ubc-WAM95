#include "robot.h"
#include "stdheader.h"
#include "control.h"
#include "senses.h"
#include "rtmemory.h"
#include "memory.h"
//#include "experiment.h"
Robot::Robot(){
    memory = new Memory();
}

Robot::Robot(ProductManager* pm, systems::Wam<DIMENSION>* wam){
    this->pm = pm;
    this->wam = wam;
    this->fts = pm->getForceTorqueSensor();
    this->hand = pm->getHand();
    init_wam();
    init_hand();
    init_fts();

    instantiate_components();

    module_name = "Robot";

    std::cout << "Robot instantiated!" << std::endl;
    fflush(stdout);
}
void Robot::instantiate_components(){
    senses = new Senses(pm, wam);
    controller = new RobotController(pm, wam, senses);
    memory = new Memory();
    rtmemory = new RTMemory(pm, wam, memory, senses, controller);
    //experiment = new Experiment(pm, wam, senses, controller);
}
void Robot::init_wam(){
}
void Robot::init_hand(){
}
void Robot::init_fts(){
   //fts->tare();
}
void Robot::init_rt(){
    get_rtmemory()->init();
}
void Robot::shutdown(){
    cout << "Robot shut down activated." << endl;
	// Wait for the user to press Shift-idle
	pm->getSafetyModule()->waitForMode(SafetyModule::IDLE);
    std::exit(0);
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
#define X(aa, bb, cc, dd, ee) \
        case bb: cc; break;
#include "robot_table.h"
#undef X
            default: help(); break;
        }
    }
    exit();
}
void Robot::help(){
    printf("\n");
#define X(aa, bb, cc, dd, ee) \
    printf("     bb dd\n");
#include "robot_table.h"
#undef X
}

//ACCESSORS
ProductManager* Robot::get_pm(){return pm;}
systems::Wam<DIMENSION>* Robot::get_wam(){return wam;}
systems::Wam<DIMENSION>* Robot::getWAM(){return wam;}
ForceTorqueSensor* Robot::get_fts(){return fts;}
Hand* Robot::get_hand(){return hand;}
Senses* Robot::get_senses(){return senses;}
RTMemory* Robot::get_rtmemory(){return rtmemory;}
Memory* Robot::get_memory(){return memory;}
RobotController* Robot::get_controller(){return controller;}

//MUTATORS
void Robot::update_sensors(){
    if(senses->has_hand())
        get_hand()->update(Hand::S_ALL,true); //update hand sensors
    //if(senses->has_fts()){
        get_fts()->update(true); //update f/t sensor 
        get_fts()->updateAccel(true); //update f/t acceleration sensor 
    //}
}
