#include "control.h"
#include "utils.h"
#include "senses.h"

Hand::jp_type finger_contacts;        //entries are 0 if no contact, 1 if contact

Controller::Controller(Senses* senses){
    this->pm = senses->getPM();
    this->wam = senses->getWAM();
    this->hand = senses->getHand();

	setVectorValues(&finger_contacts, 0, 0);
    init_wam();
    init_hand();
}
void Controller::init_wam(){
    wam->gravityCompensate();
}
void Controller::init_hand(){
    //printf(">>> Press [Enter] to initialize Hand. (Make sure it has room!)");
    //waitForEnter();
    hand->initialize();
}
//Close all fingers
void Controller::close_hand(){
        Hand::jv_type velocities;
        setVectorValues(&velocities, 3.0, -1);
        hand->velocityMove(velocities);
}
//Open all fingers
void Controller::open_hand(){
        Hand::jv_type velocities;
        setVectorValues(&velocities, -3.0, 0);
        hand->velocityMove(velocities);
}
//Close all fingers until contacts detected
void Controller::grasp_object(){
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
void Controller::ungrasp_object(){
        //graspsemastop = true;
        setVectorValues(&finger_contacts, 0, 0);
        open_hand();
}
void Controller::backdrive_hand(){
			/*if(backdrivesemastop){
				boost::thread* backDriveHandThread;
				backdrivesemastop = false;
				backDriveHandThread = new boost::thread(backDriveHand, hand, &wam, &pm);
			}
			else{
				backdrivesemastop = true;
			}*/
        std::cout << "Hand Backdrivability ON" << std::endl;
        double finger_joint_step = 1;//misc_parms[0];
        double tact_stop = 1;//misc_parms[1];
        double tact_push = 1;//misc_parms[2];
        double fingertip_torque_stop = 1;//misc_parms[3];
        double fingertip_torque_push = 1;//misc_parms[4];
        double fingertip_torque_pull = 1;//misc_parms[5];
        //double temp = 1;//misc_parms[6];
        /*std::cout << "parameters: ";
        std::cout       << finger_joint_step << ", "
                                << tact_stop << ", "
                                << tact_push << ", "
                                << fingertip_torque_stop << ", "
                                << fingertip_torque_push << ", "
                                << fingertip_torque_pull << ", " << std::endl;
        //std::cout << "tact_base_vals: ";
        //std::cout     << tact_base_val[0] << ", "
        //                      << tact_base_val[1] << ", "
        //                      << tact_base_val[2] << std::endl;
        //std::cout << "fingertip_torque_base_vals: ";
        //std::cout     << fingertip_torque_base_val[0] << ", "
        //                      << fingertip_torque_base_val[1] << ", "
        //                      << fingertip_torque_base_val[2] << ", "
        //                      << fingertip_torque_base_val[3] << std::endl;
        //hand->setPositionMode();
        while(!backdrivesemastop){
                //boost::this_thread::sleep(boost::posix_time::milliseconds(1));
                int num_fingers = 3;
                hand->update(Hand::S_POSITION, true);
                Hand::jp_type hjp = hand->getInnerLinkPosition();
                //std::cout << "before: " << toString(&hjp) << std::endl;
                for(int i = 0; i < num_fingers; i++){
                        //F > zero_thresh
                        if(check_tactile_contact(hand, i, tact_base_val[i]*tact_stop)||
                                check_fingertip_torque_contact(hand, i, fingertip_torque_base_val[i]*fingertip_torque_stop)){   //F1,2,3
                                finger_contacts[i] = 1;
                                //F > push_thresh
                                if(check_tactile_contact(hand, i, tact_base_val[i]*tact_push)||
                                        check_fingertip_torque_contact(hand, i, fingertip_torque_base_val[i]*fingertip_torque_push)){
//Hand::jv_type hjv;
                                        //setVectorValues(&hjv, 0, 0);
                                        //std::cout << "Move F" << i+1 << " back slightly" << std::endl;
                                        //hjv[i] = -0.1;        //radians/sec
                                        //hand->velocityMove(hjv);
                                        //hand->trapezoidalMove(hjp, true);
                                        //std::cout << "changing for F" << i << std::endl;
                                        //std::cout << toString(&hjp) << std::endl;

                                        hjp[i] = std::max(hjp[i]-finger_joint_step, finger_joint_step);       //radians
                                        hand->setPositionMode();
                                        hand->setPositionCommand(hjp);
                                }
                        }
                        //F < pull_thresh
                        if(i != 2 && get_fingertip_torque_value(hand, i) < fingertip_torque_base_val[i]*fingertip_torque_pull){ //F1,2,3
                                //Hand::jv_type hjv;
                                //setVectorValues(&hjv, 0, 0);
                                //std::cout << "Move F" << i+1 << " back slightly" << std::endl;
                                //hjv[i] = 0.1; //radians/s
                                //hand->velocityMove(hjv);
                                //hand->trapezoidalMove(hjp, true);
                                //std::cout << toString(&hjp) << std::endl;

                                //std::cout << get_fingertip_torque_value(hand, i) << " < " << fingertip_torque_base_val[i]*fingertip_torque_pull << std::endl;

                                hjp[i] = std::min(hjp[i]+finger_joint_step, FINGER_JOINT_LIMIT-finger_joint_step);    //radians
                                hand->setPositionMode();
                                hand->setPositionCommand(hjp);
                        }
                        if(i == 2 && get_fingertip_torque_value(hand, i) < fingertip_torque_base_val[i]*fingertip_torque_pull*1.1){     //F1,2,3
                                //Hand::jv_type hjv;
                                //setVectorValues(&hjv, 0, 0);
                                //std::cout << "Move F" << i+1 << " back slightly" << std::endl;
                                //hjv[i] = 0.1; //radians/s
                                //hand->velocityMove(hjv);
                                //hand->trapezoidalMove(hjp, true);
                                //std::cout << toString(&hjp) << std::endl;

                                std::cout << get_fingertip_torque_value(hand, i) << " < " << fingertip_torque_base_val[i]*fingertip_torque_pull << std::endl;
hjp[i] = std::min(hjp[i]+finger_joint_step, FINGER_JOINT_LIMIT-finger_joint_step);    //radians
                                hand->setPositionMode();
                                hand->setPositionCommand(hjp);
                        }
                }
                //std::cout << "after: " << toString(&hjp) << std::endl;
                //if(check_tactile_contact(hand, 3, ZERO_TACTILE_THRESHOLD*3)){ //PALM
                        //cp_type
                        //std::cout << "Move hand back slightly" << std::endl;
                //}
        }*/
        std::cout << "Hand Backdrivability OFF" << std::endl;
}
void Controller::move_hand_to_str(Hand::jp_type* dest, const std::string& description, const std::string& str)
{
	if (parseDoubles(dest, str)) {
		std::cout << "Moving Hand to " << description << ": " << *dest << std::endl;
		hand->trapezoidalMove(*dest);
	} else {
		printf("ERROR: Please enter exactly 4 numbers separated by whitespace.\n");
	}
}

double Controller::vel_command(bool open, bool close, double speed) {
	if (open  &&  !close) {
		return -speed;
	} else if (close  &&  !open) {
		return speed;
	} else {
		return 0.0;
	}
}

void Controller::hand_command(unsigned char data){
	Hand::jv_type hjv;
	hjv[0] = vel_command(data & (1<<2), data & (1<<3));  // Middle
	hjv[1] = vel_command(data & (1<<0), data & (1<<1));  // Pointer
	hjv[2] = vel_command(data & (1<<4), data & (1<<5));  // Thumb
	hjv[3] = vel_command(data & (1<<6), data & (1<<7));  // Rocker
	hand->velocityMove(hjv);
	std::cout << "Velocity: " << hjv << std::endl;
}
void Controller::lock_orientation(){
    std::cout << "not yet implemented" << std::endl;
    /*Eigen::Quaterniond orientation = wam->getToolOrientation();
    std::cout 	<< "maintaining " 
                << toString(&quaternion2hjp(&orientation)) 
                << std::endl;			
    wam->moveTo(orientation, true, 0.3, 0.25);
    systems::ExposedOutput<Eigen::Quaterniond> toSetpoint(orientation);
    //causes wam to hold its tool at the desiredOrientation
    {
        BARRETT_SCOPED_LOCK(pm.getExecutionManager()->getMutex());
        wam->idle();
        forceConnect(toSetpoint.output, wam->toController.referenceInput);
        forceConnect(wam->tt2jt.output, wam->input);
    }*/
}
void Controller::idle(){
    printf("WAM & Hand idled.\n");
    wam->idle();
    hand->idle();
    systems::disconnect(wam->tt2jt.output);
    systems::disconnect(wam->toController.referenceInput);
    systems::disconnect(wam->input);
}
void Controller::home(){
    std::cout << "Moving to home position: " << wam->getHomePosition() << std::endl;
	wam->moveHome();
}
void Controller::grasp(){
    //boost::thread* graspObjectThread;
    //graspObjectThread = new boost::thread(graspObject, hand);
}
void Controller::ungrasp(){
    ungrasp_object();
}

