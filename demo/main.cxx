/*
 * master_master.cpp
 *
 *  Created on: Apr 22, 2012
 *      Author: Daniel Troniak
 */

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h> //wam_main function
#include "stdheader.h"  //check here for all standard includes and definitions

#include "utils.h"
#include "senses.h"
#include "control.h"
#include "experiment.h"
#include "action.h"

std::vector<std::string> autoCmds;
std::string line;
Experiment* experiment;
Controller* controller;
Senses* senses;

bool validate_args(int argc, char** argv) {
	/*if (argc != 2  &&  argc != 3) {
		printf("Usage: %s <remoteHost> [--auto]\n", argv[0]);
		printf("  --auto : Automatically link WAMs and start Hand or Gimbals Hand Controller thread\n");

		return false;
	}*/
	return true;
}
void step_program(){
    if (autoCmds.empty()) {
        printf(">>> ");
        std::getline(std::cin, line);
    } else {
        line = autoCmds.back();
        autoCmds.pop_back();
    }
}
void exit_program(){
	/*output_log<DIMENSION>(pm, (void*)&logger);
    //output data to log file
    // Wait for the user to press Shift-idle
    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
    logger.closeLog();
    //((systems::PeriodicDataLogger<tuple_type>*)loggerin)->closeLog();
    printf("Logging stopped.\n");
    
    typedef boost::tuple<double, jp_type, jv_type, jt_type, cp_type, Eigen::Quaterniond> tuple_type;
    
    log::Reader<tuple_type> lr(tmpFile);
    lr.exportCSV(outFile);
    printf("Output written to %s.\n", outFile);
    std::remove(tmpFile);
    */
    std::cout << "Program exited normally" << std::endl;
    exit(0);
}
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
    senses = new Senses(&pm, ((systems::Wam<DIMENSION>*)(&wam)));
    controller = new Controller(senses);
    experiment = new Experiment(controller, senses);
	
	while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
        step_program();
		switch (line[0]) {
            case 's': senses->display(); break;	
            case 'j': //controller->move_wam_to_str(&jp, "joint positions", line.substr(1)); 
                    break;
            case 'p': //move_wam_to_str(&cp, "tool position", line.substr(1)); 
                    break;
            case 'i': controller->idle(); break;
            case 'h': controller->home(); break;
            case 'g': controller->grasp(); break;
            case 'u': controller->ungrasp(); break;
            case 'w': //move_hand_to_str(&hjp, "joint positions", line.substr(1));	
                    break;
            case 'b': controller->backdrive_hand();break;
            case 'r':
                experiment->init(line);
                experiment->run();
                break;
            case 'e': exit_program(); break;
            case 't': 
                senses->tare_fingertip_torque();
                senses->tare_tactile();
                break;
            case 'd': experiment->toggle_collect_data(); break;
            case '1': experiment->teach_pose(0); break;
            case '2': experiment->teach_pose(1); break;
            case '3': senses->getWAM()->gravityCompensate(); break;
            default:
                unsigned char in = atoi(line.c_str());
                controller->hand_command(in);

                printf("\n");
                printf("    'j' go to joint position\n");
                printf("    'p' go to tool  position\n");
                printf("    'w' go to hand  position\n");
                printf("    'g' grasp object\n");
                printf("    'u' ungrasp object\n");
                printf("    'b' toggle hand backdrivability\n");
                printf("    'i' to idle wam\n");
                printf("    'h' return to home position\n");
                printf("    's' to show formatted output\n");
                printf("    'r' to run experiment\n");
                printf("    'e' to end program and output to log\n");
                printf("    't' to tare the tactile and fingertip_torque sensors\n");
                printf("    'o' to cause WAM to hold its current orientation\n");
                printf("	'd' to toggle data collection on/off (default off)\n");
                printf("    '1' to record WAM bottom joint angles\n");
                printf("    '2' to record WAM top joint angles\n");
                printf("    '3' to compensate for Gravity with WAM\n");
                break;
		}
	}
	return 0;
}




