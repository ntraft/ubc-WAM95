#include "stdheader.h"
#include "robot.h"
#include "display.h"
#include "teach.h"
#include "rtmemory.h"


//Teach Class

Teach::Teach(Robot* _robot){ 
    cout << "instantiating teach" << endl; fflush(stdout);
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

    string saveName;
    printf("Please type name of trajectory to be recorded (press [Enter] for default): ");
    std::getline(std::cin, saveName);
    if (saveName.size() == 0) 
        saveName = robot->get_memory()->get_string("default_teach_name");

    robot->get_rtmemory()->set_teach_name(saveName);
	
    printf("\nPlease move the WAM to its initial joint configuration and press [Enter].\n");
	waitForEnter();
    robot->get_rtmemory()->set_initial_jp();

	printf("\nPress [Enter] to start teaching.\n");
	waitForEnter();
	robot->get_rtmemory()->reset_time();
	robot->get_rtmemory()->record();
	//boost::thread t(&Teach::display, &teach);

	printf("Press [Enter] to stop teaching.\n");
	waitForEnter();
	robot->get_rtmemory()->create_spline();
}
