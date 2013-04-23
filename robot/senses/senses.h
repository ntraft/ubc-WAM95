#ifndef SENSES_H_
#define SENSES_H_

#include "stdheader.h"
#include "mainline.h"

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

enum SENSOR_KEYS{
    TACT_BASE_VAL,
    FT_TORQUE_BASE_VAL,
    TORQUE_EPSILON,
    NUM_SENSOR_VARS
};

class Robot;

class Senses: public MainLine{
    private:
        ProductManager* pm;
        systems::Wam<DIMENSION>* wam;
        Hand* hand;
        ForceTorqueSensor* fts;

        Hand::jv_type sensor_vars[NUM_SENSOR_VARS]; 
        tv_type tare_value_tv;
        Hand::jp_type tare_value_ft;
#define X(aa, bb, cc, dd, ee) \
        bb tare_value_##cc;
#include "input_type_table.h"
#undef X
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
        Senses(ProductManager* pm, Wam<DIMENSION>* wam);
        void init_wam();
        void init_hand();
        void init_fts();


        //mainline
        virtual void help();
        virtual void validate_args();
        virtual void run();

        //accessors
        ProductManager* get_pm();
        systems::Wam<DIMENSION>* get_wam();
        ForceTorqueSensor* get_fts();
        Hand* get_hand();
        bool has_hand();
        
        //Sensor utilities
        cf_type get_force();
        ct_type get_torque();
        ca_type get_accel();
        jp_type get_tool_pose();
        co_type get_tool_orientation();
        Eigen::Quaterniond get_tool_orientation_q();
        Hand::jp_type get_tool_orientation_m();
        Hand::jp_type get_fingertip_torques();
        Hand::jp_type get_fingertip_torques(bool realtime);
        int get_fingertip_torque_value(int finger_num);
        int get_fingertip_torque_value(int finger_num,bool realtime);
        Hand::jp_type get_tactile_sums();
        tv_type get_tactile_vector();
        bool check_tactile_contact(int finger_num);
        bool check_tactile_contact(int finger_num, float threshold);
        bool check_tactile_contact();
        bool check_fingertip_torque_contact(int finger_num, int fingertip_torque_thresh);
        bool check_fingertip_torque_contact(int fingertip_torque_thresh);
        bool check_fingertip_torque_contact();
        void tare_all();
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
