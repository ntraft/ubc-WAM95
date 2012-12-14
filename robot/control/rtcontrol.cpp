//RTControl Class
class RTControl {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DIMENSION);
protected:
	systems::Wam<DIMENSION>* wam;
	Hand* hand;
    HandSystem* hand_system; //for realtime data logging of hand sensors
    WamSystem* wam_system; //for realtime manipulation of wam trajectory
	ProductManager& pm;
	
public:
	int dataSize;
	bool loop;
    bool problem;
    stringstream hand_debug;
	RTControl(systems::Wam<DIMENSION>* wam_, ProductManager& pm_, std::string filename_,
			const libconfig::Setting& setting_) :
			wam(wam_), hand(NULL), pm(pm_), playName(filename_), inputType(0), setting(
					setting_), cms(NULL), cpVec(NULL), qVec(NULL), jpSpline(
					NULL), cpSpline(NULL), qSpline(NULL), jpTrajectory(NULL), cpTrajectory(
					NULL), qTrajectory(NULL), time(pm.getExecutionManager()), dataSize(
					0), loop(false) 
    { 
    }
	
    bool init();
	void moveToStart();
	
private:
	DISALLOW_COPY_AND_ASSIGN(RTControl);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

bool RTControl::init(){
    //Turn on Gravity Compensation
	wam->gravityCompensate(true);
    // Is a Hand attached?
	//Hand* hand = NULL;
	if (pm.foundHand()) {
		hand = pm.getHand();
		//printf(">>> Press [Enter] to initialize Hand. (Make sure it has room!)");
		//waitForEnter();
		//hand->initialize();
        
        //hand system deals with realtime sensor reading
        hand_system = new HandSystem(hand,&problem,&hand_debug);
        wam_system = new WamSystem((systems::Wam<DIMENSION>*)&wam);
	}
	// Modify the WAM Safety Limits
	pm.getSafetyModule()->setTorqueLimit(3.0);
	pm.getSafetyModule()->setVelocityLimit(1.5);
	// Create our control mode switcher to go between current control mode and voltage control mode
	cms = new ControlModeSwitcher<DIMENSION>(pm, *wam,
			setting["control_mode_switcher"]);
	// Set our control mode
	if (vcMode == 1) {
		printf("Switching system to voltage control mode\n\n");
		cms->voltageControl();
		//Allow the mechanical system to settle
		btsleep(2.0);
	} else {
		printf("Verifying system is in current control mode\n\n");
		cms->currentControl();
	}
    fflush(stdout);
	pm.getExecutionManager()->startManaging(time); //starting time management
	return true;

}
void RTControl::moveToStart() {
	if (inputType == 0) {
		wam->moveTo(jpSpline->eval(jpSpline->initialS()), true);
	} else
		wam->moveTo(
				boost::make_tuple(cpSpline->eval(cpSpline->initialS()),
						qSpline->eval(qSpline->initialS())));
}
