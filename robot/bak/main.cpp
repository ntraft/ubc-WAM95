#include <iostream>  // For std::cin
#include <string>  // For std::string and std::getline()
#include <cstdlib>  // For std::atexit()
#include <unistd.h>  // For usleep() & readlink
#include <string>

#include "stdheader.h"

#include "data_stream.h" //for sensor data stream io
#include "rtmemory.h"
#include "rtcontrol.h"
#include "rtsenses.h"
#include "hand_system.cxx" 
#include "wam_system.cxx" 
#include "disrobot.cxx" 
#include "control_mode_switcher.h"
#include <barrett/standard_main_function.h>


 int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam& wam_) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DIMENSION);

    systems::Wam<DIMENSION>* wam = (systems::Wam<DIMENSION>*)(&wam_);

	Robot robot(wam, pm, filename, config.getRoot());

	if (!robot.init())
		return 1;

    robot.loop = true;
    curState = PLAYING;

    float sleep_s = 0.002;

	//while (roboting) {
    while(pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE){
        if(robot.problem){
            //cout << "contact!" << endl;
            //cout << "hd: " << robot.hand_debug.str() << endl;
            cout << "uh-oh" << endl;
            robot.hand_debug.str("");
            robot.problem = false;
        }
		switch (curState) {
		case QUIT:
			//roboting = false;
			break;
		case PLAYING:
			switch (lastState) {
			case STOPPED:
				robot.moveToStart();
                robot.init_data_logger();
				robot.reconnectSystems();
				robot.startplayback();
				lastState = PLAYING;
				break;
			case PAUSED:
				robot.startplayback();
				lastState = PLAYING;
				break;
			case PLAYING:
				if (robot.playbackActive()) {
					btsleep(sleep_s);
					break;
				} else if (robot.loop) {
                    loop_count++;
					robot.disconnectSystems();
					lastState = STOPPED;
					curState = PLAYING;
					break;
                } else {
					curState = STOPPED;
					break;
				}
    			default: break;
			}
			break;
		case PAUSED:
			switch (lastState) {
			case PLAYING:
				robot.pauseplayback();
				lastState = PAUSED;
				break;
			case PAUSED:
				btsleep(sleep_s);
				break;
			case STOPPED:
				break;
			default: break;
			}
			break;
		case STOPPED:
			switch (lastState) {
			case PLAYING:
				robot.disconnectSystems();
				lastState = STOPPED;
				break;
			case PAUSED:
				robot.disconnectSystems();
				lastState = STOPPED;
				break;
			case STOPPED:
				btsleep(sleep_s);
				break;
			default: break;
			}
			break;
		}
        /*if(!collecting_data){
            robot.collect_data_stream();
            collecting_data = true;
        }*/
	}
	robot.disconnectSystems();
    robot.output_data_stream();
	//wam->moveHome();
	//printf("\n\n");
	//pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
