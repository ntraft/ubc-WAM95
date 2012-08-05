#include "senses.h"
#include "stdheader.h"

Hand::jv_type tact_base_val; 
Hand::jv_type fingertip_torque_base_val; 
Hand::ct_type torque_epsilon;


int get_fingertip_torque_value(Hand* hand, int finger_num){
        hand->update(Hand::S_FINGERTIP_TORQUE,true);
        std::vector<int> fingertip_torque = hand->getFingertipTorque();
        return fingertip_torque[finger_num];
}
bool check_tactile_contact(Hand* hand, int finger_num){
        //std::cout << "check_tactile_contact!" << std::endl;
        hand->update(Hand::S_TACT_FULL, true);
        std::vector<TactilePuck*> tps;
        tps = hand->getTactilePucks();
        v_type finger_tact = tps[finger_num]->getFullData();
        for(int i = 0; i < finger_tact.size(); i++){
                //std::cout << finger_tact[i] << " > " << tact_base_val(finger_num) << "?" << std::endl;
                if(finger_tact[i] > tact_base_val[finger_num]){
                        return true;
                }
        }
        return false;
}
bool check_tactile_contact(Hand* hand, int finger_num, float threshold){
        //std::cout << "check_tactile_contact!" << std::endl;
        hand->update(Hand::S_TACT_FULL, true);
        std::vector<TactilePuck*> tps;
        tps = hand->getTactilePucks();
        v_type finger_tact = tps[finger_num]->getFullData();
        for(int i = 0; i < finger_tact.size(); i++){
                if(finger_tact[i] > threshold){
                        std::cout << finger_tact[i] << " > " << threshold << std::endl;
                        return true;
                }
        }
        return false;
}
bool check_tactile_contact(Hand* hand){
        return (check_tactile_contact(hand, 0) || check_tactile_contact(hand, 1) || check_tactile_contact(hand, 2));
}
bool check_fingertip_torque_contact(Hand* hand, int finger_num, int fingertip_torque_thresh){
        hand->update(Hand::S_FINGERTIP_TORQUE,true);
        std::vector<int> fingertip_torque = hand->getFingertipTorque();
        if(fingertip_torque[finger_num] > fingertip_torque_thresh){
                std::cout << fingertip_torque[finger_num] << ">" << fingertip_torque_thresh << std::endl;
                return true;
        }
        else{
                //std::cout << fingertip_torque[finger_num] << ">" << fingertip_torque_thresh << std::endl;
                return false;
        }
}
bool check_fingertip_torque_contact(Hand* hand, int fingertip_torque_thresh){
        return check_fingertip_torque_contact(hand, 0, fingertip_torque_thresh)
                || check_fingertip_torque_contact(hand, 1, fingertip_torque_thresh)
                || check_fingertip_torque_contact(hand, 2, fingertip_torque_thresh);
}
bool check_fingertip_torque_contact(Hand* hand){
        return check_fingertip_torque_contact(hand, 0, fingertip_torque_base_val[0])
                || check_fingertip_torque_contact(hand, 1, fingertip_torque_base_val[1])
                || check_fingertip_torque_contact(hand, 2, fingertip_torque_base_val[2]);
}
void tare_tactile(Hand* hand){
        //std::cout << "check_tactile_contact!" << std::endl;
        hand->update(Hand::S_TACT_FULL, true);
        std::vector<TactilePuck*> tps;
        tps = hand->getTactilePucks();
        std::cout << "tare-value for tactile pad on: " << std::endl;
        for(unsigned int finger_num = 0; finger_num < tps.size(); finger_num++){
                v_type finger_tact = tps[finger_num]->getFullData();
                float max = -1;
                for(int i = 0; i < finger_tact.size(); i++){
                        if(finger_tact[i] > max){
                                max = finger_tact[i];
                        }
                }
                std::cout << "    F" << finger_num+1 << ": " << max << std::endl;
                tact_base_val[finger_num] = max;
        }
}
//reset zero-value of fingertip torque sensors
void tare_fingertip_torque(Hand* hand){
        hand->update(Hand::S_FINGERTIP_TORQUE,true);
        std::vector<int> fingertip_torque = hand->getFingertipTorque();
        std::cout << "tare-value for fingertip_torque: " << std::endl;
        for(unsigned int finger_num = 0; finger_num < fingertip_torque.size(); finger_num++){
                fingertip_torque_base_val[finger_num] = fingertip_torque[finger_num];
                 std::cout << "    F" << finger_num+1 << ": " << fingertip_torque[finger_num] << std::endl;
        }
}
