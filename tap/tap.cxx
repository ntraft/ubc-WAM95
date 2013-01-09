#include "utils.h"
#include "robot.h"
#include "tap.h"
#include "teach.cxx"
//#include "play.cxx"
//#include "loop.cxx"

TeachAndPlay::TeachAndPlay(Robot* robot):MainLine(){
    this->robot = robot;
    teach = new Teach(robot, "test");
    //play = new Play(robot);
    //loop = new Loop(robot);
    /*this->pm = _pm;
    this->wam = _wam;
    this->controller = _controller;
    this->senses = _senses;
    this->hand = pm->getHand();*/
}

void TeachAndPlay::run(){
    MainLine::run();
    bool quit = false;
    while (!quit){//robot->get_pm()->getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
        step();
        switch (line[0]) {
#define X(aa, bb, cc, dd, ee) case bb: cc; break;
            #include "tap_table.h"
#undef X
            default:
                help();
                break;
        }
    }
    exit();
}

void TeachAndPlay::init(std::string args){
}

void TeachAndPlay::help(){
        printf("\n");
#define X(aa, bb, cc, dd, ee) cout << "     " << bb << ": " << dd << endl; //printf("     bb dd\n");
        #include "tap_table.h"
#undef X
}

