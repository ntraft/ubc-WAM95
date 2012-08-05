#include "control.h"
#include "utils.h"
#include "senses.h"

#ifndef DIMENSION
#define DIMENSION 7u
#define FINGERTIP_TORQUE2TORQUE_RATIO 118.0     //convert hand fingertip_torque to N-m
#define FINGER_JOINT_LIMIT 2.4435       //=140 degrees
#define ZERO_FINGERTIP_TORQUE_THRESHOLD 2000    //required threshold to be considered non-noise reading
#define ZERO_TACTILE_THRESHOLD 0.5      //required threshold to be considered non-noise reading
#endif

Hand::jp_type finger_contacts;        //entries are 0 if no contact, 1 if contact

//Close all fingers
void closeHand(Hand* hand){
        Hand::jv_type velocities;
        setVectorValues(&velocities, 3.0, -1);
        hand->velocityMove(velocities);
}
//Open all fingers
void openHand(Hand* hand){
        Hand::jv_type velocities;
        setVectorValues(&velocities, -3.0, 0);
        hand->velocityMove(velocities);
}
//Close all fingers until contacts detected
void graspObject(Hand* hand){
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
void ungraspObject(Hand* hand){
        //graspsemastop = true;
        setVectorValues(&finger_contacts, 0, 0);
        openHand(hand);
}
void backDriveHand(Hand* hand, ForceTorqueSensor* fts, void* wamin, ProductManager* pm){
        std::cout << "Hand Backdrivability ON" << std::endl;
        double finger_joint_step = 1;//misc_parms[0];
        double tact_stop = 1;//misc_parms[1];
        double tact_push = 1;//misc_parms[2];
        double fingertip_torque_stop = 1;//misc_parms[3];
        double fingertip_torque_push = 1;//misc_parms[4];
        double fingertip_torque_pull = 1;//misc_parms[5];
        //double temp = 1;//misc_parms[6];
        std::cout << "parameters: ";
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
        while(1/*!backdrivesemastop*/){
                //boost::this_thread::sleep(boost::posix_time::milliseconds(1));
                int num_fingers = 3;
                hand->update(Hand::S_POSITION, true);
                Hand::jp_type hjp = hand->getInnerLinkPosition();
                //std::cout << "before: " << toString(&hjp) << std::endl;
                for(int i = 0; i < num_fingers; i++){
                        //F > zero_thresh
                        if(check_tactile_contact(hand, i, /*tact_base_val[i]*/0.5*tact_stop)||
                                check_fingertip_torque_contact(hand, i, /*fingertip_torque_base_val[i]*/2000*fingertip_torque_stop)){   //F1,2,3
                                finger_contacts[i] = 1;
                                //F > push_thresh
                                if(check_tactile_contact(hand, i, /*tact_base_val[i]*/0.5*tact_push)||
                                        check_fingertip_torque_contact(hand, i, /*fingertip_torque_base_val[i]*/2000*fingertip_torque_push)){
//Hand::jv_type hjv;
                                        //setVectorValues(&hjv, 0, 0);
                                        //std::cout << "Move F" << i+1 << " back slightly" << std::endl;
                                        //hjv[i] = -0.1;        //radians/sec
                                        //hand->velocityMove(hjv);
                                        //hand->trapezoidalMove(hjp, true);
                                        //std::cout << "changing for F" << i << std::endl;
                                        //std::cout << toString(&hjp) << std::endl;

                                        hjp[i] /*-= finger_joint_step;*/ = std::max(hjp[i]-finger_joint_step, finger_joint_step);       //radians
                                        hand->setPositionMode();
                                        hand->setPositionCommand(hjp);
                                }
                        }
                        //F < pull_thresh
                        if(i != 2 && get_fingertip_torque_value(hand, i) < /*fingertip_torque_base_val[i]*/2000*fingertip_torque_pull){ //F1,2,3
                                //Hand::jv_type hjv;
                                //setVectorValues(&hjv, 0, 0);
                                //std::cout << "Move F" << i+1 << " back slightly" << std::endl;
                                //hjv[i] = 0.1; //radians/s
                                //hand->velocityMove(hjv);
                                //hand->trapezoidalMove(hjp, true);
                                //std::cout << toString(&hjp) << std::endl;

                                //std::cout << get_fingertip_torque_value(hand, i) << " < " << fingertip_torque_base_val[i]*fingertip_torque_pull << std::endl;

                                hjp[i] /*+= finger_joint_step;*/ = std::min(hjp[i]+finger_joint_step, FINGER_JOINT_LIMIT-finger_joint_step);    //radians
                                hand->setPositionMode();
                                hand->setPositionCommand(hjp);
                        }
                        if(i == 2 && get_fingertip_torque_value(hand, i) < /*fingertip_torque_base_val[i]*/2000*fingertip_torque_pull*1.1){     //F1,2,3
                                //Hand::jv_type hjv;
                                //setVectorValues(&hjv, 0, 0);
                                //std::cout << "Move F" << i+1 << " back slightly" << std::endl;
                                //hjv[i] = 0.1; //radians/s
                                //hand->velocityMove(hjv);
                                //hand->trapezoidalMove(hjp, true);
                                //std::cout << toString(&hjp) << std::endl;

                                std::cout << get_fingertip_torque_value(hand, i) << " < " << /*fingertip_torque_base_val[i]*/2000*fingertip_torque_pull << std::endl;
hjp[i] /*+= finger_joint_step;*/ = std::min(hjp[i]+finger_joint_step, FINGER_JOINT_LIMIT-finger_joint_step);    //radians
                                hand->setPositionMode();
                                hand->setPositionCommand(hjp);
                        }
                }
                //std::cout << "after: " << toString(&hjp) << std::endl;
                //if(check_tactile_contact(hand, 3, ZERO_TACTILE_THRESHOLD*3)){ //PALM
                        //cp_type
                        //std::cout << "Move hand back slightly" << std::endl;
                //}
        }
        std::cout << "Hand Backdrivability OFF" << std::endl;
}
void moveToStr(Hand* hand, Hand::jp_type* dest,
		const std::string& description, const std::string& str)
{
	if (parseDoubles(dest, str)) {
		std::cout << "Moving Hand to " << description << ": " << *dest << std::endl;
		hand->trapezoidalMove(*dest);
	} else {
		printf("ERROR: Please enter exactly 4 numbers separated by whitespace.\n");
	}
}

double velCommand(bool open, bool close, double speed) {
	if (open  &&  !close) {
		return -speed;
	} else if (close  &&  !open) {
		return speed;
	} else {
		return 0.0;
	}
}

void handCommand(Hand* hand, unsigned char data){
	Hand::jv_type hjv;
	hjv[0] = velCommand(data & (1<<2), data & (1<<3));  // Middle
	hjv[1] = velCommand(data & (1<<0), data & (1<<1));  // Pointer
	hjv[2] = velCommand(data & (1<<4), data & (1<<5));  // Thumb
	hjv[3] = velCommand(data & (1<<6), data & (1<<7));  // Rocker
	hand->velocityMove(hjv);
	std::cout << "Velocity: " << hjv << std::endl;
}
