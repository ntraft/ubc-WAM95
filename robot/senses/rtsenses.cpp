template<size_t DOF> void RTSenses<DOF>::disconnectSystems() {
//RTSenses Class
template<size_t DOF>
class RTSenses {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
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
	RTSenses(systems::Wam<DIMENSION>* wam_, ProductManager& pm_, std::string filename_,
			const libconfig::Setting& setting_) :
			wam(wam_), hand(NULL), pm(pm_), playName(filename_), inputType(0), setting(
					setting_), cms(NULL), cpVec(NULL), qVec(NULL), jpSpline(
					NULL), cpSpline(NULL), qSpline(NULL), jpTrajectory(NULL), cpTrajectory(
					NULL), qTrajectory(NULL), time(pm.getExecutionManager()), dataSize(
					0), loop(false) {
    }
	bool init();
    void disconnectSystems();
	void reconnectSystems();
private:
	DISALLOW_COPY_AND_ASSIGN(RTSenses);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

bool RTSenses::init(){
}

void RTSenses::disconnectSystems() {
	disconnect(wam->input);
    disconnect(logger->input);
    disconnect(tg.template getInput<0>());
	disconnect(tg.template getInput<1>());
	disconnect(tg.template getInput<2>());
	disconnect(tg.template getInput<3>());
	disconnect(tg.template getInput<4>());
	disconnect(tg.template getInput<5>());
	disconnect(tg.template getInput<6>());
    wam->idle();
	time.stop();
	time.setOutput(0.0);
}
void RTSenses::reconnectSystems(){
    wam_system->init();

	if (inputType == 0) {
		systems::forceConnect(time.output, jpTrajectory->input);
		systems::forceConnect(jpTrajectory->output, wam_system->input);
		wam->trackReferenceSignal(wam_system->output);
	} else {
		systems::forceConnect(time.output, cpTrajectory->input);
		systems::forceConnect(time.output, qTrajectory->input);
		systems::forceConnect(cpTrajectory->output, poseTg.getInput<0>());
		systems::forceConnect(qTrajectory->output, poseTg.getInput<1>());
		wam->trackReferenceSignal(poseTg.output);
	}

    systems::forceConnect(time.output, stream_ft_mean_trajectory->input);
    systems::forceConnect(time.output, stream_ft_std_trajectory->input);
    systems::forceConnect(stream_ft_mean_trajectory->output,hand_system->mean_input);
    systems::forceConnect(stream_ft_std_trajectory->output,hand_system->std_input);
	systems::forceConnect(time.output,                tg.template getInput<0>());
	systems::forceConnect(wam->jpOutput,              tg.template getInput<1>());
	systems::forceConnect(wam->jvOutput,              tg.template getInput<2>());
	systems::forceConnect(wam->jtSum.output,          tg.template getInput<3>());
	systems::forceConnect(wam->toolPosition.output,   tg.template getInput<4>());
	systems::forceConnect(wam->toolOrientation.output,tg.template getInput<5>());
    systems::forceConnect(hand_system->output,        tg.template getInput<6>());
	
    //systems::forceConnect(debug_system->output,       tg.template getInput<7>());
    //systems::forceConnect(time.output,       tg.template getInput<6>());
    systems::forceConnect(tg.output, logger->input);
    hand_system->time_count = 0;
	time.start();
	printf("Logging started.\n");
}
