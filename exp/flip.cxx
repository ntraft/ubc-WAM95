#include "experiment.h"
#include "flip.h"
#include "utils.h"
#include "control.h"
#include "senses.h"
#include "robot.h"
#include "stdheader.h"

Flip::Flip(Robot* robot)
: Experiment(robot){
}
FlipTilt::FlipTilt(Robot* robot)
: Flip(robot){
}
void Flip::run(){
    teach_and_play();
}
void FlipTilt::run(){
    Flip::teach_and_play();
}

void Flip::teach_and_play(){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DIMENSION);
	typedef boost::tuple<double, jp_type> jp_sample_type;

	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
        return;
	}

	const double T_s = pm->getExecutionManager()->getPeriod();

	wam->gravityCompensate();

	systems::Ramp time(pm->getExecutionManager());

	systems::TupleGrouper<double, jp_type> jpLogTg;

	// Record at 1/10th of the loop rate
	systems::PeriodicDataLogger<jp_sample_type> jpLogger(pm->getExecutionManager(),
			new barrett::log::RealTimeWriter<jp_sample_type>(tmpFile, 10*T_s), 10);

	printf("Teaching started.\n");
	//printf("Press [Enter] to start teaching.\n");
	//waitForEnter();
	{
		// Make sure the Systems are connected on the same execution cycle
		// that the time is started. Otherwise we might record a bunch of
		// samples all having t=0; this is bad because the Spline requires time
		// to be monotonic.
		BARRETT_SCOPED_LOCK(pm->getExecutionManager()->getMutex());

		connect(time.output, jpLogTg.getInput<0>());
		connect(wam->jpOutput, jpLogTg.getInput<1>());
		connect(jpLogTg.output, jpLogger.input);
		time.start();
	}

	printf("Press [Enter] to stop teaching.\n");
	waitForEnter();
	jpLogger.closeLog();
	disconnect(jpLogger.input);


	// Build spline between recorded points
	log::Reader<jp_sample_type> lr(tmpFile);
	std::vector<jp_sample_type> vec;
	for (size_t i = 0; i < lr.numRecords(); ++i) {
		vec.push_back(lr.getRecord());
        jp_sample_type sample = vec[i];
        //std::cout << sample[0] << std::endl;
	}
	math::Spline<jp_type> spline(vec);

    //jp_type spline_val = spline(0);
    //std::cout << spline_val[0] << std::endl;

	printf("Press [Enter] to play back the recorded trajectory.\n");
	waitForEnter();

	// First, move to the starting position
	wam->moveTo(spline.eval(spline.initialS()));

	// Then play back the recorded motion
	time.stop();
	time.setOutput(spline.initialS());

	systems::Callback<double, jp_type> trajectory(boost::ref(spline));

	connect(time.output, trajectory.input);
	wam->trackReferenceSignal(trajectory.output);

	time.start();

	while (trajectory.input.getValue() < spline.finalS()) {
		usleep(100000);
	}


	printf("Press [Enter] to idle the wam->\n");
	waitForEnter();
	wam->idle();


	std::remove(tmpFile);
	//pm->getSafetyModule()->waitForMode(SafetyModule::IDLE);
}

