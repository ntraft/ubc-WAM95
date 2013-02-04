#include "utils.h"
#include "robot.h"
#include "tap.h"
#include "teach.h"
#include "play.h"
#include "rtmemory.h"

TeachAndPlay::TeachAndPlay(Robot* robot):MainLine(){
    this->robot = robot;
}
void TeachAndPlay::init_tap(){
    //cout << "initializing tap..."; fflush(stdout);
	
    // Turn on Gravity Compensation
	robot->get_wam()->gravityCompensate(true);
    
    // Modify the WAM Safety Limits
	robot->get_pm()->getSafetyModule()->setTorqueLimit(3.0);
	robot->get_pm()->getSafetyModule()->setVelocityLimit(1.5);
    

    init_teach();
    init_play();
    robot->get_rtmemory()->init();
    //cout << "Teach And Play initialized!" << endl; fflush(stdout);
}
void TeachAndPlay::init_teach(){
    teach = new Teach(robot);
}
void TeachAndPlay::init_play(){
    play = new Play(robot);
}

void TeachAndPlay::run(){
    MainLine::run();
    init_tap();
    
    bool quit = false;
    while (!quit){
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

void TeachAndPlay::help(){
        printf("\n");
#define X(aa, bb, cc, dd, ee) cout << "     " << bb << ": " << dd << endl; //printf("     bb dd\n");
        #include "tap_table.h"
#undef X
}

