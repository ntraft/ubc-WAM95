#ifndef SENSES_H_
#define SENSES_H_

#include "stdheader.h"

//for data logging
/*char tmpFile[14] = "/tmp/btXXXXXX";
char outFile[14] = "data/data.csv";
void dataCollect(Hand* hand, ForceTorqueSensor* fts, void* wamin, ProductManager* pm, 
					enum EXPERIMENT_KEYS expnum, enum EXPERIMENT_SHAPES expshape);
void runExperiment(	Hand* hand, ForceTorqueSensor* fts, void* wamin, ProductManager* pm,
					enum EXPERIMENT_KEYS expnum);
void backDriveHand(Hand* hand, ForceTorqueSensor* fts, void* wamin, ProductManager* pm);
void graspObject(Hand* hand);
void stop_thread(bool* semaphore);*/


class Senses{
    private:
        ProductManager* pm;
        systems::Wam<DIMENSION>* wam;
        Hand* hand;
        ForceTorqueSensor* fts;
    public:
        Senses(ProductManager* pm, systems::Wam<DIMENSION>* wam);
        void init_wam();
        void init_hand();
        void init_fts();

        ProductManager* getPM();
        systems::Wam<DIMENSION>* getWAM();
        ForceTorqueSensor* getFTS();
        Hand* getHand();
        
        //Sensor utilities
        int get_fingertip_torque_value(int finger_num);
        bool check_tactile_contact(int finger_num);
        bool check_tactile_contact(int finger_num, float threshold);
        bool check_tactile_contact();
        bool check_fingertip_torque_contact(int finger_num, int fingertip_torque_thresh);
        bool check_fingertip_torque_contact(int fingertip_torque_thresh);
        bool check_fingertip_torque_contact();
        //reset zero-value of tactile sensors
        void tare_tactile();
        //reset zero-value of fingertip torque sensors
        void tare_fingertip_torque();

        void display();
        // Functions that help display data from the Hand's (optional) tactile sensors.
        // Note that the palm tactile sensor has a unique cell layout that these
        // functions do not print  correctly.
        void drawBoard(WINDOW *win, int starty, int startx, int rows, int cols,
                int tileHeight, int tileWidth);
        void graphCell(WINDOW *win, int starty, int startx, double pressure);
        void graphPressures(WINDOW *win, int starty, int startx,
                const TactilePuck::v_type& pressures);
};
#endif
