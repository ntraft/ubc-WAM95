#include "stdheader.h"
#include "control_mode_switcher.h"
class Robot;
class HandSystem;
class WamSystem;
class SensorStreamSystem;
class ControlStrategy;
class SaS;
class SaA1;
class SaA2;
class SaA3;

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
    vector<ControlStrategy*> saa;
    ControlStrategy* sas;
    bool sas_toggle;
	    
public:
	bool loop;
    bool problem;
    Robot* robot;
    stringstream hand_debug;
	Play(Robot* robot);
	bool init();
    void run();
	void move_to_start();
    void toggle_sas();
    void toggle_saa(int);
private:
	DISALLOW_COPY_AND_ASSIGN(Play);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
