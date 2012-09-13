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
    FLIP,
    
    //examples
    HOLDPOSITION,
    SYSTEMSINTRO,
    REALTIMEMOVE,
    TEACHANDPLAY,
    TORQUECONTROL,
    HAPTICS,
    NUM_EXPERIMENTS
};
enum EXPERIMENT_SHAPES{
    CIRCLE,
    SQUARE,
    TRIANGLE,
    NUM_SHAPES
};
enum EXP_VAR_KEYS_7{
    WAM_BOTTOM,
    WAM_TOP,
    JOINT_TOLERANCE,
    MISC_PARMS,
    NUM_EXP_VARS_7
};
enum EXP_VAR_KEYS_4{
    HAND_PREGRASP,
    HAND_GRASP,
    HAND_UNGRASP,
    WAM_BOTTOM_O,
    WAM_TOP_O,
    NUM_EXP_VARS_4
};
enum EXP_VAR_KEYS_3{
    WAM_BOTTOM_C,
    WAM_TOP_C,
    NUM_EXP_VARS_3
};

class Experiment{
private:
    //data collection vectors
    std::vector< std::vector<int> > hfingertip_torque;	//hand strain measure
    std::vector<Hand::ct_type> ct_vector;	//cartesian torque
    Hand::ct_type min_ct;	//running minimum values
    //bool collectData = false;

protected:
    //experiment variables
	bool boolrealtime;// = true;
	int prev_state;// = -1;
	int num_runs;// = 1;
	systems::Wam<DIMENSION>::jp_type temp;
	//experiment-specific flags
	bool move_and_lift;
	bool land_and_stroke;
    bool is_initialized;

    void data_collect();
    bool flag_collect_data;
    Controller* controller;
    Senses* senses;
    systems::Wam<DIMENSION>* wam;
    Hand* hand;
    ProductManager* pm;

    enum EXPERIMENT_KEYS exp_id;
    enum EXPERIMENT_SHAPES exp_shape;
    
    jp_type exp_vars_7[NUM_EXP_VARS_7];
    Hand::jp_type exp_vars_4[NUM_EXP_VARS_4];
    systems::Wam<DIMENSION>::cp_type exp_vars_3[NUM_EXP_VARS_3];
    Eigen::Quaterniond wam_bottom_q;
    Eigen::Quaterniond wam_top_q;
    
    void load_exp_variables();
    void save_exp_variables();
    void load_exp_variables_3();
    void save_exp_variables_3();
    void load_exp_variables_4();
    void save_exp_variables_4();
    void load_exp_variables_7();
    void save_exp_variables_7();

    //data logging
    char* tmpFile;
	systems::Ramp* time;
	typedef boost::tuple<double, jp_type, jv_type, jt_type, Hand::cp_type, Eigen::Quaterniond> tuple_type;
    systems::TupleGrouper<double, jp_type, jv_type, jt_type, Hand::cp_type, Eigen::Quaterniond>* tg;
	systems::PeriodicDataLogger<tuple_type>* logger;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
	Experiment(Controller* controller, Senses* senses);

    void toggle_collect_data();
    void teach_pose(int seqnum);
    Experiment* get_experiment();
    void init(std::string args);
    void help();
	virtual void run();
    void set_num_runs(int num);

    //data logging
    void init_data_log();
    void start_data_log();
    void stop_data_log();
};

#endif
