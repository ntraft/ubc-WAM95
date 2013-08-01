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

    void run_pls_regress(){
        string traj_name;
        string num_params;
        string pls_ncomp;
        cout << "Please enter trajectory name" << endl;
        cin >> traj_name;
        float num_param = robot->get_memory()->get_float("num_environment_parameters");
        float ncomp = robot->get_memory()->get_float("pls_ncomp");

        //run script in external shell using system
        string script_fname = "script/run_pls_regress.sh";
        const char* argv[3] = {traj_name.c_str(), itoa(num_param).c_str(), itoa(ncomp).c_str()};
        char prefix[100] = "";
        snprintf(prefix, sizeof(prefix), "%s %s %s %s", script_fname.c_str(), argv[0], argv[1], argv[2]);
        system(prefix);
        
        //run script in external shell using popen (NOT working) 
        /*FILE *pp;
        pp = popen("script/run_pls_regress.sh");
        fputs(pp, 
                traj_name.c_str(), 
                robot->get_memory()->get_float("num_environment_parameters"), 
                robot->get_memory()->get_float("pls_ncomp"));
        if (pp != NULL) {
            //read output of external shell script
            while (1) {
                char *line;
                char buf[1000];
                line = fgets(buf, sizeof buf, pp);
                if (line == NULL) break;
                if (line[0] == 'd') printf("%s", line);
            }
            pclose(pp);
        }
        else{
            cout << "WARNING: could not execute script" << endl;
        }
        return 0;
        */
    }

    void run(){
        MainLine::run();
        bool quit = false;
        while (!quit){//robot->get_pm()->getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
            step();
            //cout << "stepped!" << endl;
            try{
                switch (line[0]) {
                    cout << "line received: " << line << endl;
#define X(aa, bb, cc, dd, ee) \
                    case bb: cc; break;
#include "main_table.h"
#undef X
                    default:
                        //unsigned char in = atoi(line.c_str());
                        //robot->get_controller()->hand_command(in);
                        help();
                        break;
                }
            }
            catch(int exception){
                if(exception == NoHandFound) {
                    std::cerr << "ERROR: No Hand Found!" << std::endl;
                }
            }
        }
        robot->get_pm()->getSafetyModule()->waitForMode(SafetyModule::IDLE);
        exit();
    }
    void help(){
        printf("\n");
#define X(aa, bb, cc, dd, ee) \
        cout << "     " << bb << ": " << dd << endl; //printf("     bb dd\n");
#include "main_table.h"
#undef X
    }
};

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    Robot* robot = new Robot(&pm, ((systems::Wam<DIMENSION>*)(&wam)));
    MainProgram mp(robot);
    try{
        mp.run();
    }
    catch(int i){
        std::cout << "exception at wam_main" << std::endl;
    }
    return 0;
}




