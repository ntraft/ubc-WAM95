#define BARRETT_SMF_VALIDATE_ARGS
#include "stdheader.h"  //check here for all standard includes and definitions
#include <barrett/standard_main_function.h> //wam_main function

#include "utils.h"
#include "senses.h"
#include "robot.h"
#include "control.h"
#include "experiment.h"
#include "action.h"
#include "tap.h"
#include "rtmemory.h"
#include "memory.h"


bool validate_args(int argc, char** argv) { return true; }


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
/*
    rtm->set_play_name("tp");
    //rtm->load_trajectory();
    if(false){
        rtm->load_data_stream(true);
        rtm->load_data_stream(false);
    }
*/
/*
    //load_trajectory();   
    Memory memory;// = new Memory();
    memory.load_trajectory();
    Senses* senses = new Senses(&pm, ((systems::Wam<DIMENSION>*)(&wam)));
    RobotController* control = new RobotController(&pm, ((systems::Wam<DIMENSION>*)(&wam)), senses);
    RTMemory* rtm = new RTMemory(&pm, ((systems::Wam<DIMENSION>*)(&wam)), &memory, senses, control);

    bool playing = false;
	while (true) {
        if(!playing){
            wam.moveTo(rtm->get_initial_tp(), true, 0.5, 0.5);
            rtm->init_data_logger();
            rtm->disconnect_systems();
            rtm->reconnect_systems();
            rtm->start_playback();
        }
        else{
            btsleep(0.002);
            pm.getHand()->update(Hand::S_ALL,true); //update hand sensors
            pm.getForceTorqueSensor()->update(true); //update f/t sensor 
            pm.getForceTorqueSensor()->updateAccel(true); //update f/t acceleration sensor 
        }
        playing = rtm->playback_active();
    }*/
    return 0;
}
