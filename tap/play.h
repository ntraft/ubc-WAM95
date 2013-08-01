#include "stdheader.h"
#include "control_mode_switcher.h"
class Robot;
class HandSystem;
class WamSystem;
class SensorStreamSystem;
class ControlStrategy;

using barrett::detail::waitForEnter;

//Play Class
class Play {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DIMENSION);
protected:
	systems::Wam<DIMENSION>* wam;
	Hand* hand;
	ProductManager* pm;
	std::string playName;
	int inputType;
    std::string tmpStr, saveName, fileOut;
    systems::Ramp time;

    ControlStrategy* strategy;
    vector<ControlStrategy*> control_strategies;
    //bool sas_toggle;
    bool is_init;
    bool playing;
    cp_type qd_increase_count;
    cp_type cp_increase_count;
	    
public:
	bool loop_flag;
    bool problem;
    Robot* robot;
    stringstream hand_debug;
	Play(Robot* robot);
    void instantiate_control_strategies();
	bool init();
    void loop();
    void output_data_stream();
    void user_control();
    void transform_cp();
    void transform_qd();
    void increase_cp(int ind);
    void increase_qd(int ind);
    void decrease_cp(int ind);
    void decrease_qd(int ind);
    void run();
	void move_to_start();
    void toggle_var(string name);
    //void toggle_sas();
    //void toggle_saa(int);
    void set_control_strategy(int);
private:
	DISALLOW_COPY_AND_ASSIGN(Play);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
