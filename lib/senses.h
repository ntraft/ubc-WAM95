#ifndef SENSES_H_
#define SENSES_H_

#include "stdheader.h"

//Function defns
int get_fingertip_torque_value(Hand* hand, int finger_num);
bool check_tactile_contact(Hand* hand, int finger_num);
bool check_tactile_contact(Hand* hand, int finger_num, float threshold);
bool check_tactile_contact(Hand* hand);
bool check_fingertip_torque_contact(Hand* hand, int finger_num, int fingertip_torque_thresh);
bool check_fingertip_torque_contact(Hand* hand, int fingertip_torque_thresh);
bool check_fingertip_torque_contact(Hand* hand);
//reset zero-value of tactile sensors
void tare_tactile(Hand* hand);
//reset zero-value of fingertip torque sensors
void tare_fingertip_torque(Hand* hand);

#endif
