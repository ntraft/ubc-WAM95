#include "stdheader.h"
#include "robot.h"
#include "display.h"
#include "teach.h"
#include "rtmemory.h"


//Teach Class

Teach::Teach(Robot* _robot){ 
    robot = _robot;
    cout << "Teach instantiated!" << endl; fflush(stdout);
}

bool Teach::init() {
    bool success = robot->get_rtmemory()->prepare_log_file();
    cout << "Teach initialized!" << endl; fflush(stdout);
    return success;
}

void Teach::run(){
	init();
	printf("\nPress [Enter] to start teaching.\n");
	waitForEnter();
	robot->get_rtmemory()->record();
	//boost::thread t(&Teach::display, &teach);

	printf("Press [Enter] to stop teaching.\n");
	waitForEnter();
	robot->get_rtmemory()->create_spline();
	
	//robot->get_pm()->getSafetyModule()->waitForMode(SafetyModule::IDLE);
}
