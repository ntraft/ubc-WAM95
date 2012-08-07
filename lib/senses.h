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

// Functions that help display data from the Hand's (optional) tactile sensors.
// Note that the palm tactile sensor has a unique cell layout that these
// functions do not print  correctly.
const int TACT_CELL_HEIGHT = 3;
const int TACT_CELL_WIDTH = 6;
const int TACT_BOARD_ROWS = 8;
const int TACT_BOARD_COLS = 3;
const int TACT_BOARD_STRIDE = TACT_BOARD_COLS * TACT_CELL_WIDTH + 2;
void drawBoard(WINDOW *win, int starty, int startx, int rows, int cols,
		int tileHeight, int tileWidth);
void graphPressures(WINDOW *win, int starty, int startx,
		const TactilePuck::v_type& pressures);



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
