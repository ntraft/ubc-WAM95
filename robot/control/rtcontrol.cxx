#include <iostream>  // For std::cin
#include <string>  // For std::string and std::getline()
#include <cstdlib>  // For std::atexit()
#include <unistd.h>  // For usleep() & readlink
#include <string>

#include "stdheader.h"
#include "macros.h"
#include "control_mode_switcher.h"
#include "robot.h"
#include "hand_system.cxx"
#include "wam_system.cxx"

//RTControl Class
class RTControl {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DIMENSION);
protected:
    HandSystem* hand_system; //for realtime data logging of hand sensors
    WamSystem* wam_system; //for realtime manipulation of wam trajectory
    Robot* robot;	
    Hand* hand;
    Wam<DIMENSION>* wam;
    ControlModeSwitcher<DIMENSION>* cms;
    string play_name;
    int inputType;
    const libconfig::Setting& setting;
    libconfig::Config config;
    systems::Ramp time;
    bool vcMode;
public:
	int dataSize;
	bool loop;
    bool problem;
    stringstream hand_debug;
	RTControl(Robot* _robot, std::string filename_,const libconfig::Setting& setting_) :
			robot(_robot), play_name(filename_), setting(setting_), inputType(0), cms(NULL), 
            time(robot->get_pm()->getExecutionManager()), dataSize(0), loop(false) 
    { 
    }
	
    bool init();
	void moveToStart();
	
private:
	DISALLOW_COPY_AND_ASSIGN(RTControl);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

bool RTControl::init(){
    //Turn on Gravity Compensation
	wam->gravityCompensate(true);
    // Is a Hand attached?
	//Hand* hand = NULL;
	if (robot->get_pm()->foundHand()) {
		hand = robot->get_pm()->getHand();
		//printf(">>> Press [Enter] to initialize Hand. (Make sure it has room!)");
		//waitForEnter();
		//hand->initialize();
        
        //hand system deals with realtime sensor reading
        hand_system = new HandSystem(hand,&problem,&hand_debug);
        wam_system = new WamSystem((systems::Wam<DIMENSION>*)&wam);
	}
	// Modify the WAM Safety Limits
	robot->get_pm()->getSafetyModule()->setTorqueLimit(3.0);
	robot->get_pm()->getSafetyModule()->setVelocityLimit(1.5);
	// Create our control mode switcher to go between current control mode and voltage control mode
	cms = new ControlModeSwitcher<DIMENSION>(boost::ref(*(robot->get_pm())), *wam, setting["control_mode_switcher"]);
	// Set our control mode
	if (vcMode == 1) {
		printf("Switching system to voltage control mode\n\n");
		cms->voltageControl();
		//Allow the mechanical system to settle
		btsleep(2.0);
	} else {
		printf("Verifying system is in current control mode\n\n");
		cms->currentControl();
	}
    fflush(stdout);
	robot->get_pm()->getExecutionManager()->startManaging(time); //starting time management
	return true;
}
void RTControl::moveToStart() {
	/*if (inputType == 0) {
		wam->moveTo(jpSpline->eval(jpSpline->initialS()), true);
	} else
		wam->moveTo(
				boost::make_tuple(cpSpline->eval(cpSpline->initialS()),
						qSpline->eval(qSpline->initialS())));*/
}
