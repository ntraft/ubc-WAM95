#ifndef EXPERIMENT_H_
#define EXPERIMENT_H_

#include "stdheader.h"

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
    systems::Wam<DIMENSION>* wam;
    Hand* hand;
    ForceTorqueSensor* fts;
    ProductManager* pm;

	//experiment variables
	bool boolrealtime;// = true;
	int prev_state;// = -1;
	int num_runs;// = 1;
	std::string experiment_keys[NUM_EXPERIMENTS];/* = {
		"ActionPhase",
		"ActiveSensing",
		"WAMVelocity",
		"WAMJointPos",
		"WAMCartesianPos",
		"WAMJointTorque",
		"BHVelocity",
		"BHPosition",
		"BHTorque",
		"BHTrapezoidal",
		"SimpleShapes",
		"ActiveProbing",
		"CartesianRaster"
	};*/
	std::string experiment_shapes[NUM_SHAPES];/* = {
		"circle",
		"square",
		"triangle"
	};*/
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
	enum EXPERIMENT_SHAPES expshape;
	
	//experiment-specific flags
	bool move_and_lift;
	bool land_and_stroke;

public:
	Experiment(systems::Wam<DIMENSION>* wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm);
    void load_exp_variables();
    void save_exp_variables();
	virtual void run();
};


#endif
