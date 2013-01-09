#include "control.h"
#include "utils.h"
#include "senses.h"
#include "robot.h"

Hand::jp_type finger_contacts;        //entries are 0 if no contact, 1 if contact
bool backdrivesemastop = true;

RobotController::RobotController(ProductManager* _pm, Wam<DIMENSION>* _wam, Senses* _senses):MainLine(){
    //, pm(_pm),wam(_wam),hand(_pm->getHand()),fts(_pm->getForceTorqueSensor()),senses(_senses){
    
    this->pm = _pm;
    this->wam = _wam;
    this->hand = pm->getHand();
    this->fts = pm->getForceTorqueSensor();
    this->senses = _senses;
    
	set_vector_values(&finger_contacts, 0, 0);
    init_wam();
    init_hand();

    std::cout << "RobotController initialized!" << std::endl;
}
void RobotController::init_wam(){
    //wam->gravityCompensate(true);
}
void RobotController::init_hand(){
    //printf(">>> Press [Enter] to initialize Hand. (Make sure it has room!)");
    //waitForEnter();
    hand->initialize();
}
//MAINLINE
void RobotController::validate_args(){
    std::cout << "Robot validate args" << std::endl;
}
void RobotController::run(){
    bool back = false;
    while (!back){//robot->get_pm()->getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
        step();
        switch (line[0]) {
#define X(aa, bb, cc, dd, ee) case bb: cc; break;
            #include "control_table.h"
#undef X
            default: help(); break;
        }
    }
    exit();
}
void RobotController::help(){
    printf("\n");
#define X(aa, bb, cc, dd, ee) printf("     bb dd\n");
    #include "control_table.h"
#undef X
}
//Close all fingers
void RobotController::close_hand(){
        Hand::jv_type velocities;
        set_vector_values(&velocities, 3.0, -1);
        hand->velocityMove(velocities);
}
//Open all fingers
void RobotController::open_hand(){
        Hand::jv_type velocities;
        set_vector_values(&velocities, -3.0, 0);
        hand->velocityMove(velocities);
}
//Close all fingers until contacts detected
void RobotController::grasp_object(){
        if(0/*backdrivesemastop*/){
                std::cout << "WARNING: Requires hand backdrivability to be set" << std::endl;
                return;
        }
        Hand::jv_type velocities;
        //graspsemastop = false;
        bool done = false;
        while(!done/* && !graspsemastop*/){
                boost::this_thread::sleep(boost::posix_time::milliseconds(5));
                done = true;
                for(int i = 0; i < finger_contacts.size()-1; i++){
                        //std::cout << finger_contacts[i] << std::endl;
                        if(!finger_contacts[i]){
                                velocities[i] = 3.0;
                                done = false;
                        }
                }
                hand->velocityMove(velocities);
        }
}
//open all fingers and reset finger contact flags
void RobotController::ungrasp_object(){
        //graspsemastop = true;
        set_vector_values(&finger_contacts, 0, 0);
        open_hand();
}

enum BACKDRIVE_PARMS{
    FINGER_JOINT_STEP,
    TACT_STOP,
    TACT_PUSH,
    FT_TORQUE_STOP,
    FT_TORQUE_PUSH,
    FT_TORQUE_PULL,
    NUM_BACKDRIVE_PARMS
};
enum FINGER_IDS{
    F1,
    F2,
    F3,
    NUM_FINGERS,
    PALM
};

int RobotController::get_fingertip_torque_value(Hand* hand, int finger_num){
    hand->update(Hand::S_FINGERTIP_TORQUE,true);
    std::vector<int> fingertip_torque = hand->getFingertipTorque();
    cout << "got torque value" << fingertip_torque[finger_num] << endl;fflush(stdout);
    return fingertip_torque[finger_num];
    return 0;
}

void RobotController::backdrive_hand_thread(){
    std::cout << "Hand Backdrivability ON" << std::endl;

    int torque_threshold_max = 2500;
    int torque_threshold_min = 1500;
    int torque_threshold_zero_min = 1750;
    int torque_threshold_zero_max = 2250;

    Hand::jt_type small_positive_torque;
    Hand::jt_type small_negative_torque;
    parseDoubles(&small_positive_torque," 1.0  1.0  1.0  1.0");
    parseDoubles(&small_negative_torque,"-1.0 -1.0 -1.0 -1.0");
    
    Hand::jv_type small_positive_velocity;
    Hand::jv_type small_negative_velocity;
    parseDoubles(&small_positive_velocity," 1.0  1.0  1.0  1.0");
    parseDoubles(&small_negative_velocity,"-1.0 -1.0 -1.0 -1.0");

    Hand::jv_type zero_velocity;
    parseDoubles(&zero_velocity,"0 0 0 0");

    cout << "vars initialized" << endl;fflush(stdout);

    //hand->setTorqueMode();
    while(!backdrivesemastop){
        for(int i = 0; i < NUM_FINGERS; i++){
            cout << "in for" << endl;fflush(stdout);
            //get_fingertip_torque_value(hand,0);
            int torque = 0; //robot->get_fingertip_torque_value(hand, i);
            cout << "got torque value" << endl;fflush(stdout);
            if(torque > torque_threshold_max){
                //std::cout << "move by " << small_negative_torque[i] << std::endl;fflush(stdout);
                //hand->setTorqueCommand(small_negative_torque,i);
                std::cout << "move by " << small_positive_velocity[i] << std::endl;fflush(stdout);
                hand->velocityMove(small_negative_velocity, 1 << i);
                usleep(200);
            }
            else if(torque < torque_threshold_min){
                //std::cout << "move by " << small_positive_torque[i] << std::endl;fflush(stdout);
                //hand->setTorqueCommand(small_positive_torque,i);
                std::cout << "move by " << small_positive_velocity[i] << std::endl;fflush(stdout);
                hand->velocityMove(small_positive_velocity, 1 << i);
                usleep(200);
            }
            else if(torque < torque_threshold_zero_min || torque > torque_threshold_zero_max){
                std::cout << "stop moving " << small_positive_velocity[i] << std::endl;fflush(stdout);
                hand->velocityMove(zero_velocity, 1 << i);
            }
        }
		usleep(10000);	//run @ 100 Hz
    }
    std::cout << "Hand Backdrivability OFF" << std::endl;
}
void RobotController::backdrive_hand(){
    if(backdrivesemastop){
        boost::thread* backDriveHandThread;
        backdrivesemastop = false;
        backDriveHandThread = new boost::thread(&RobotController::backdrive_hand_thread,this);
    }
    else{
        backdrivesemastop = true;
    }
}
void RobotController::move_hand_to_str(Hand::jp_type* dest, const std::string& description, const std::string& str)
{
	if (parseDoubles(dest, str)) {
		std::cout << "Moving Hand to " << description << ": " << *dest << std::endl;
		hand->trapezoidalMove(*dest);
	} else {
		printf("ERROR: Please enter exactly 4 numbers separated by whitespace.\n");
	}
}

double RobotController::vel_command(bool open, bool close, double speed) {
	if (open  &&  !close) {
		return -speed;
	} else if (close  &&  !open) {
		return speed;
	} else {
		return 0.0;
	}
}

void RobotController::hand_command(unsigned char data){
	Hand::jv_type hjv;
	hjv[0] = vel_command(data & (1<<2), data & (1<<3));  // Middle
	hjv[1] = vel_command(data & (1<<0), data & (1<<1));  // Pointer
	hjv[2] = vel_command(data & (1<<4), data & (1<<5));  // Thumb
	hjv[3] = vel_command(data & (1<<6), data & (1<<7));  // Rocker
	hand->velocityMove(hjv);
	std::cout << "Velocity: " << hjv << std::endl;
}
void RobotController::idle(){
    printf("WAM & Hand idled.\n");
    wam->idle();
    hand->idle();
    systems::disconnect(wam->tt2jt.output);
    systems::disconnect(wam->toController.referenceInput);
    systems::disconnect(wam->input);
}
void RobotController::home(){
    std::cout << "Moving to home position: " << wam->getHomePosition() << std::endl;
	wam->moveHome();
}
void RobotController::grasp(){
    //boost::thread* graspObjectThread;
    //graspObjectThread = new boost::thread(graspObject, hand);
}
void RobotController::ungrasp(){
    ungrasp_object();
}

