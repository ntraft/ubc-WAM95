#ifndef CONTROL_H_
#define CONTROL_H_

#include "stdheader.h"
#include "mainline.h"

class Robot;
class Senses;

class RobotController: public MainLine{
private:
    systems::Wam<DIMENSION>* wam;
    Hand* hand;
    ForceTorqueSensor* fts;
    ProductManager* pm;
    Senses* senses;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    RobotController(ProductManager* pm, Wam<DIMENSION>* wam, Senses* senses);
    void init_wam();
    void init_hand();

    void validate_args();
    void run();
    void help();

    //Close all fingers
    void close_hand();
    //Open all fingers
    void open_hand();
    //Close all fingers until contacts detected
    void grasp_object();
    //open all fingers and reset finger contact flags
    void ungrasp_object();
    int get_fingertip_torque_value(Hand* hand, int finger_num);
    void backdrive_hand_thread();
    void backdrive_hand();
    
    void move_wam_to_str(jp_type* dest, const std::string& description, const std::string& str);
        
    void move_hand_to_str(Hand::jp_type* dest,
            const std::string& description, const std::string& str);
    double vel_command(bool open, bool close, double speed = 1.25);
    void hand_command(unsigned char data);
    void lock_orientation();
    void idle();
    void home();
    void grasp();
    void ungrasp();
    void move_wam_relative_z(float amount);
    void move_wam_relative_x(float amount);

};
#endif
