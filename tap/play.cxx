#include "stdheader.h"
#include "data_stream.h" //for sensor data stream io
#include "hand_system.cxx" 
#include "wam_system.cxx" 
#include "sensor_stream_system.h" 
#include "display.h" 
#include "robot.h"
#include "play.h"
#include "rtmemory.h"
#include "utils.h"
#include "control_strategy.cxx"
#include "SaS.cxx"
#include "SaA.cxx"

enum STATE {
	PLAYING, STOPPED, PAUSED, QUIT
} curState = STOPPED, lastState = STOPPED;

static int loop_count = 0;

enum strategy{
    SURPRISE_AND_STOP = -1,
    SURPRISE_AND_ADAPT1,
    SURPRISE_AND_ADAPT2,
    SURPRISE_AND_ADAPT3
};

ControlStrategy* new_strategy(enum strategy s, RobotController* controller){
   switch(s){
       case SURPRISE_AND_STOP: return new SaS(controller); break;
       case SURPRISE_AND_ADAPT1: return new SaA1(controller); break;
       case SURPRISE_AND_ADAPT2: return new SaA2(controller); break;
       case SURPRISE_AND_ADAPT3: return new SaA3(controller); break;
       default: return NULL;
   } 
}

Play::Play(Robot* robot) : inputType(0), time(robot->get_pm()->getExecutionManager()), loop(false), playName("test") {                       
    this->robot = robot; 
    pm = robot->get_pm();
    wam = robot->get_wam();
    hand = robot->get_hand();
    sas = new_strategy((enum strategy)-1, robot->get_controller());
    for(int i = 0; i < 3; i++){ saa[i] = new_strategy((enum strategy)i, robot->get_controller()); }
    cout << "Play instantiated!" << endl;fflush(stdout);
}

// Initialization - Gravity compensating, setting safety limits, parsing input file and creating trajectories
bool Play::init() {
    robot->get_rtmemory()->load_trajectory();
    robot->get_rtmemory()->load_data_stream(true);
    robot->get_rtmemory()->load_data_stream(false);
    cout << "Play initialized!" << endl;fflush(stdout);
	return true;
}
void Play::move_to_start() {
    cout << "moving to start..."; fflush(stdout);
	if (inputType == 0) {
		wam->moveTo( robot->get_rtmemory()->get_initial_jp() , true);
	} else
		wam->moveTo( robot->get_rtmemory()->get_initial_tp() );
    cout << "playback started!" << endl; fflush(stdout);
}

void Play::toggle_sas(){
    sas_toggle = !sas_toggle;
    if(sas_toggle){
        strategy = sas;
    }
    else{
        strategy = NULL;
    }
    robot->get_rtmemory()->set_control_strategy(strategy);
}
void Play::toggle_saa(int type){
    if(type == 0){
        strategy = NULL;
    }else{
        strategy = saa[type-1];
    }
    robot->get_rtmemory()->set_control_strategy(strategy);
}

void Play::run(){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DIMENSION);
    init();
	//boost::thread displayThread(displayEntryPoint,pm,wam);

	//bool playing = true;
    //bool collecting_data = false;
    loop = true;
    curState = PLAYING;
    //lastState = PLAYING;

    float sleep_s = 0.002;

	//while (playing) {
    while(pm->getSafetyModule()->getMode() == SafetyModule::ACTIVE){
        robot->get_rtmemory()->check_for_problems();
		switch (curState) {
		case QUIT:
			//playing = false;
			break;
		case PLAYING:
			switch (lastState) {
			case STOPPED:
                //cout << "STOPPED -> PLAYING" << endl;
				move_to_start();
                robot->get_rtmemory()->init_data_logger();
				robot->get_rtmemory()->reconnect_systems();
				robot->get_rtmemory()->start_playback();
				lastState = PLAYING;
				break;
			case PAUSED:
                //cout << "PAUSED -> PLAYING" << endl;
				robot->get_rtmemory()->start_playback();
				lastState = PLAYING;
				break;
			case PLAYING:
                //cout << "PLAYING -> PLAYING" << endl;
				if (robot->get_rtmemory()->playback_active()) {
					btsleep(sleep_s);
                    robot->update_sensors();
					break;
				} else if (loop) {
                    loop_count++;
					robot->get_rtmemory()->disconnect_systems();
					lastState = STOPPED;
					curState = PLAYING;
					break;
                } else {
					curState = STOPPED;
					break;
				}
    			default:
				break;
			}
			break;
		case PAUSED:
			switch (lastState) {
			case PLAYING:
                robot->get_rtmemory()->pause_playback();
				lastState = PAUSED;
				break;
			case PAUSED:
				btsleep(sleep_s);
				break;
			case STOPPED:
				break;
			default:
				break;
			}
			break;
		case STOPPED:
			switch (lastState) {
			case PLAYING:
				robot->get_rtmemory()->disconnect_systems();
				lastState = STOPPED;
				break;
			case PAUSED:
				robot->get_rtmemory()->disconnect_systems();
				lastState = STOPPED;
				break;
			case STOPPED:
				btsleep(sleep_s);
				break;
			default:
				break;
			}
			break;
		}
        /*if(!collecting_data){
            collect_data_stream();
            collecting_data = true;
        }*/
	}
	robot->get_rtmemory()->disconnect_systems();
    robot->get_rtmemory()->output_data_stream();
	//wam->moveHome();
	//printf("\n\n");
	//pm->getSafetyModule()->waitForMode(SafetyModule::IDLE);
}
