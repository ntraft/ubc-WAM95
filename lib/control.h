#ifndef CONTROL_H_
#define CONTROL_H_

#include "stdheader.h"

class Senses;

class Controller{
private:
    systems::Wam<DIMENSION>* wam;
    Hand* hand;
    ForceTorqueSensor* fts;
    ProductManager* pm;
    Senses* senses;

public:
    Controller(Senses* senses);
    void init_wam();
    void init_hand();
    //Close all fingers
    void close_hand();
    //Open all fingers
    void open_hand();
    //Close all fingers until contacts detected
    void grasp_object();
    //open all fingers and reset finger contact flags
    void ungrasp_object();
    void backdrive_hand();
    template<size_t DOF, int R, int C, typename Units>
    void move_wam_to_str(math::Matrix<R,C, Units>* dest,
            const std::string& description, const std::string& str)
    {
        if (parseDoubles(dest, str)) {
            std::cout << "Moving to " << description << ": " << *dest << std::endl;
            wam->moveTo(*dest);
        } else {
            printf("ERROR: Please enter exactly %d numbers separated by "
                    "whitespace.\n", dest->size());
        }
    }
    void move_hand_to_str(Hand::jp_type* dest,
            const std::string& description, const std::string& str);
    double vel_command(bool open, bool close, double speed = 1.25);
    void hand_command(unsigned char data);
    void lock_orientation();
    void idle();
    void home();
    void grasp();
    void ungrasp();

};
#endif
