#ifndef EXPERIMENT_H_
#define EXPERIMENT_H_

#include "stdheader.h"

class Controller;
class Senses;

enum EXPERIMENT_KEYS{
        ACTIONPHASE,
        ACTIVESENSING,
        WAMVELOCITY,
        WAMJOINTPOS,
        WAMCARTESIANPOS,
        WAMJOINTTORQUE,
        BHVELOCITY,
        BHPOSITION,
        BHTORQUE,
        BHTRAPEZOIDAL,
        SIMPLESHAPES,
        ACTIVEPROBING,
        CARTESIANRASTER,
        NUM_EXPERIMENTS
};
enum EXPERIMENT_SHAPES{
        CIRCLE,
        SQUARE,
        TRIANGLE,
        NUM_SHAPES
};

class Experiment{

public:
	enum EXPERIMENT_KEYS exp_id;
	enum EXPERIMENT_SHAPES exp_shape;
	
    //experiment variables
	bool boolrealtime;// = true;
	int prev_state;// = -1;
	int num_runs;// = 1;
	systems::Wam<DIMENSION>::jp_type wamBottom;
	systems::Wam<DIMENSION>::jp_type wamTop;
	systems::Wam<DIMENSION>::cp_type wamBottomC;
	systems::Wam<DIMENSION>::cp_type wamBottomC_ip; //intermediate point
	systems::Wam<DIMENSION>::cp_type wamTopC;
	Hand::jp_type wamBottomO;
	Hand::jp_type wamTopO;
	Eigen::Quaterniond wamTopQ;
	Eigen::Quaterniond wamBottomQ;
	Hand::jv_type handPregrasp;
	Hand::jv_type handGrasp;
	Hand::jv_type handUnGrasp;
	//Hand::jv_type tact_base_val;
	//Hand::jv_type fingertip_torque_base_val;
	//Hand::ct_type torque_epsilon;
	systems::Wam<DIMENSION>::jp_type joint_tolerance;
	systems::Wam<DIMENSION>::jp_type temp;
	systems::Wam<DIMENSION>::jp_type misc_parms;    //miscellaneous parameters
	//Hand::jp_type finger_contacts;        //entries are 0 if no contact, 1 if contact
	
	//experiment-specific flags
	bool move_and_lift;
	bool land_and_stroke;

    bool is_initialized;

    //data collection vectors
    std::vector< std::vector<int> > hfingertip_torque;      //hand strain measure
    std::vector<Hand::ct_type> ct_vector;        //cartesian torque
    Hand::ct_type min_ct; //running minimum values
    //bool collectData = false;

protected:
    void data_collect();
    bool flag_collect_data;
    Controller* controller;
    Senses* senses;

public:
	Experiment(Controller* controller, Senses* senses);

    void toggle_collect_data();
    void teach_pose(int seqnum);
    void run_experiment();
    void init(std::string args);
    void help();
	virtual void run();
    void load_exp_variables();
    void save_exp_variables();
};

#endif
