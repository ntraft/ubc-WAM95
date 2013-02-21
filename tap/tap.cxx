#include "utils.h"
#include "robot.h"
#include "tap.h"
#include "teach.h"
#include "play.h"
#include "rtmemory.h"
#include "memory.h"


//Move WAM to joint position specified by string str (7 numbers separated by whitespace)
void move_to_str(systems::Wam<DIMENSION>* wam, jp_type* dest, const string& str)
{
	if (parseDoubles(dest, str)) {
		std::cout << "Moving to: " << *dest << std::endl;
		wam->moveTo(*dest, true, 1, 1);
	} else {
		printf("ERROR: Please enter exactly %d numbers separated by "
				"whitespace.\n", dest->size());
	}
}

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
void TeachAndPlay::init_flip(){
    jp_type jp;
    string str = robot->get_memory()->get_string("init_flip_jp");
    move_to_str(robot->get_wam(),&jp,str); 
    robot->get_wam()->idle();
}

void TeachAndPlay::run(){
    MainLine::run();
    init_tap();
    
    bool quit = false;
    while (!quit){
        step();
        switch (line[0]) {
#define X(aa, bb, cc, dd, ee) \
            case bb: cc; break;
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
#define X(aa, bb, cc, dd, ee) \
        cout << "     " << bb << ": " << dd << endl; //printf("     bb dd\n");
#include "tap_table.h"
#undef X
}

