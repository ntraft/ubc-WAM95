/*
 * demo/main.cxx
 *
 *  Created on: Apr 22, 2012
 *      Author: Daniel Troniak
 */

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

bool validate_args(int argc, char** argv) { return true; }

class MainProgram: public MainLine{
    Robot* robot;
    //Experiment* experiment;
    TeachAndPlay* tap;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MainProgram(Robot* _robot):MainLine(), robot(_robot){
        tap = new TeachAndPlay(robot);
        //experiment = new Experiment(robot);
    }

    void validate_args(string line){
    }

    void run(){
        MainLine::run();
        bool quit = false;
        while (!quit){//robot->get_pm()->getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
            step();
            switch (line[0]) {
                cout << "line received: " << line << endl;
#define X(aa, bb, cc, dd, ee) case bb: cc; break;
        #include "main_table.h"
#undef X
                default:
                    //unsigned char in = atoi(line.c_str());
                    //controller->hand_command(in);
                    help();
                    break;
            }
        }
        robot->get_pm()->getSafetyModule()->waitForMode(SafetyModule::IDLE);
        exit();
    }
    void help(){
        printf("\n");
#define X(aa, bb, cc, dd, ee) cout << "     " << bb << ": " << dd << endl; //printf("     bb dd\n");
        #include "main_table.h"
#undef X
    }
};

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    Robot* robot = new Robot(&pm, ((systems::Wam<DIMENSION>*)(&wam)));
    MainProgram mp(robot);
    mp.run();
    return 0;
}




