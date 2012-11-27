#include <iostream>  // For std::cin
#include <string>  // For std::string and std::getline()
#include <cstdlib>  // For std::atexit()
#include <unistd.h>  // For usleep() & readlink
#include <string>

#include "stdheader.h"

#include "data_stream.h" //for sensor data stream io
#include "hand_system.cxx" 
#include "display.cxx" 

using barrett::detail::waitForEnter;

enum STATE {
	PLAYING, STOPPED, PAUSED, QUIT
} curState = STOPPED, lastState = STOPPED;

static int loop_count = 0;

//cast integer to string
std::string itoa(int i){std::string a = boost::lexical_cast<std::string>(i);return a;}

//RTLoop Class
template<size_t DOF>
class RTLoop {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
protected:
	systems::Wam<DOF>& wam;
	Hand* hand;
    HandSystem* hand_system; //for realtime data logging of hand sensors
	ProductManager& pm;
	std::string playName;
	int inputType;
	const libconfig::Setting& setting;
	libconfig::Config config;
	typedef boost::tuple<double, jp_type> input_jp_type;
	typedef boost::tuple<double, cp_type> input_cp_type;
	typedef boost::tuple<double, Eigen::Quaterniond> input_quat_type;

    std::string tmpStr, saveName, fileOut;

	ControlModeSwitcher<DOF>* cms;
	std::vector<input_cp_type, Eigen::aligned_allocator<input_cp_type> >* cpVec;
	std::vector<input_quat_type, Eigen::aligned_allocator<input_quat_type> >* qVec;
	math::Spline<jp_type>* jpSpline;
	math::Spline<cp_type>* cpSpline;
	math::Spline<Eigen::Quaterniond>* qSpline;
	systems::Callback<double, jp_type>* jpTrajectory;
	systems::Callback<double, cp_type>* cpTrajectory;
	systems::Callback<double, Eigen::Quaterniond>* qTrajectory;
    
    systems::Ramp time;
	systems::TupleGrouper<cp_type, Eigen::Quaterniond> poseTg;
	    
    //realtime data logging
    //typedef boost::tuple<double, jp_type, jv_type, jt_type, cp_type, Eigen::Quaterniond> tuple_type;
	//systems::TupleGrouper<double, jp_type, jv_type, jt_type, cp_type, Eigen::Quaterniond> tg;
    typedef boost::tuple<double, jp_type, jv_type, jt_type, cp_type, Eigen::Quaterniond, Hand::jp_type > tuple_type;
	systems::TupleGrouper<double, jp_type, jv_type, jt_type, cp_type, Eigen::Quaterniond, Hand::jp_type > tg;
    std::string data_log_headers;
    systems::PeriodicDataLogger<tuple_type>* logger;
    std::vector<std::string> tmp_filenames;
    std::string log_prefix;
    
public:
	int dataSize;
	bool loop;
	RTLoop(systems::Wam<DOF>& wam_, ProductManager& pm_, std::string filename_,
			const libconfig::Setting& setting_) :
			wam(wam_), hand(NULL), pm(pm_), playName(filename_), inputType(0), setting(
					setting_), cms(NULL), cpVec(NULL), qVec(NULL), jpSpline(
					NULL), cpSpline(NULL), qSpline(NULL), jpTrajectory(NULL), cpTrajectory(
					NULL), qTrajectory(NULL), time(pm.getExecutionManager()), dataSize(
					0), loop(false) {
                        log_prefix = "./data_streams/";
                data_log_headers = 
                    "time"
                    ",joint_pos_0,joint_pos_1,joint_pos_2,joint_pos_3,joint_pos_4,joint_pos_5,joint_pos_6,joint_pos_7"
                    ",joint_vel_0,joint_vel_1,joint_vel_2,joint_vel_3,joint_vel_4,joint_vel_5,joint_vel_6,joint_vel_7"
                    ",joint_tor_0,joint_tor_1,joint_tor_2,joint_tor_3,joint_tor_4,joint_tor_5,joint_tor_6,joint_tor_7"
                    ",cart_pos_0,cart_pos_1,cart_pos_2,cart_pos_3"
                    ",cart_ori_0,cart_ori_1,cart_ori_2,cart_ori_3";
    }
	bool init();
	void displayEntryPoint();
	void moveToStart();
	void startPlayback();
	void pausePlayback();
	bool playbackActive();
	void disconnectSystems();
	void reconnectSystems();
	void init_data_logger();
    void output_data_stream();
private:
	DISALLOW_COPY_AND_ASSIGN(RTLoop);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}
;

// Initialization - Gravity compensating, setting safety limits, parsing input file and creating trajectories
template<size_t DOF>
bool RTLoop<DOF>::init() {
	// Turn on Gravity Compensation
	wam.gravityCompensate(true);
    // Is a Hand attached?
	//Hand* hand = NULL;
	if (pm.foundHand()) {
		hand = pm.getHand();
		//printf(">>> Press [Enter] to initialize Hand. (Make sure it has room!)");
		//waitForEnter();
		hand->initialize();
        
        //hand system deals with realtime sensor reading
        hand_system = new HandSystem(pm.getExecutionManager(), hand);
	}
	// Modify the WAM Safety Limits
	pm.getSafetyModule()->setTorqueLimit(3.0);
	pm.getSafetyModule()->setVelocityLimit(1.5);
	// Create our control mode switcher to go between current control mode and voltage control mode
	cms = new ControlModeSwitcher<DOF>(pm, wam,
			setting["control_mode_switcher"]);
	//Create stream from input file
	std::ifstream fs(playName.c_str());
	std::string line;
	// Check to see the data type specified on the first line (jp_type or pose_type)
	// this will inform us if we are tracking 4DOF WAM Joint Angles, 7DOF WAM Joint Angles, or WAM Poses
	std::getline(fs, line);
	// Using a boost tokenizer to parse the data of the file into our vector.
	boost::char_separator<char> sep(",");
	typedef boost::tokenizer<boost::char_separator<char> > t_tokenizer;
	t_tokenizer tok(line, sep);
	if (strcmp(line.c_str(), "pose_type") == 0) {
		// Create our spline and trajectory if the first line of the parsed file informs us of a pose_type
		inputType = 1;
		cpVec = new std::vector<input_cp_type,
				Eigen::aligned_allocator<input_cp_type> >();
		qVec = new std::vector<input_quat_type,
				Eigen::aligned_allocator<input_quat_type> >();
		float fLine[8];
		input_cp_type cpSamp;
		input_quat_type qSamp;
		while (true) {
			std::getline(fs, line);
			if (!fs.good())
				break;
			t_tokenizer tok(line, sep);
			int j = 0;
			for (t_tokenizer::iterator beg = tok.begin(); beg != tok.end();
					++beg) {
				fLine[j] = boost::lexical_cast<float>(*beg);
				j++;
			}
			boost::get<0>(cpSamp) = fLine[0];
			boost::get<0>(qSamp) = boost::get<0>(cpSamp);

			boost::get<1>(cpSamp) << fLine[1], fLine[2], fLine[3];
			boost::get<1>(qSamp) = Eigen::Quaterniond(fLine[4], fLine[5],
					fLine[6], fLine[7]);
			boost::get<1>(qSamp).normalize();
			cpVec->push_back(cpSamp);
			qVec->push_back(qSamp);
		}
		// Make sure the vectors created are the same size
		assert(cpVec->size() == qVec->size());
		// Create our splines between points
		cpSpline = new math::Spline<cp_type>(*cpVec);
		qSpline = new math::Spline<Eigen::Quaterniond>(*qVec);
		// Create trajectories from the splines
		cpTrajectory = new systems::Callback<double, cp_type>(
				boost::ref(*cpSpline));
		qTrajectory = new systems::Callback<double, Eigen::Quaterniond>(
				boost::ref(*qSpline));
	} else if (strcmp(line.c_str(), "jp_type") == 0) {
		// Create our spline and trajectory if the first line of the parsed file informs us of a jp_type
		std::vector<input_jp_type, Eigen::aligned_allocator<input_jp_type> > jp_vec;
		float fLine[DOF + 1];
		input_jp_type samp;
		while (true) {
			std::getline(fs, line);
			if (!fs.good())
				break;
			t_tokenizer tok(line, sep);
			int j = 0;
			for (t_tokenizer::iterator beg = tok.begin(); beg != tok.end();
					++beg) {
				fLine[j] = boost::lexical_cast<float>(*beg);
				j++;
			}
			boost::get<0>(samp) = fLine[0];
			// To handle the different WAM configurations
			if (j == 5)
				boost::get<1>(samp) << fLine[1], fLine[2], fLine[3], fLine[4];
			else
				boost::get<1>(samp) << fLine[1], fLine[2], fLine[3], fLine[4], fLine[5], fLine[6], fLine[7];
			jp_vec.push_back(samp);
		}
		// Create our splines between points
		jpSpline = new math::Spline<jp_type>(jp_vec);
		// Create our trajectory
		jpTrajectory = new systems::Callback<double, jp_type>(
				boost::ref(*jpSpline));
	} else {
		// The first line does not contain "jp_type or pose_type" return false and exit.
		printf(
				"EXITING: First line of file must specify jp_type or pose_type data.");
		btsleep(1.5);
		return false;
	}
	//Close the file
	fs.close();
	printf("\nFile Contains data in the form of: %s\n\n",
			inputType == 0 ? "jp_type" : "pose_type");
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
// This function will run in a different thread and control displaying to the screen and user input
template<size_t DOF>
void RTLoop<DOF>::displayEntryPoint() {
	ForceTorqueSensor* fts = NULL;
	if (pm.foundForceTorqueSensor()) {
		fts = pm.getForceTorqueSensor();
		fts->tare();
	}   
	std::vector<TactilePuck*> tps;
	Hand::jp_type currentPos(0.0);
	Hand::jp_type nextPos(M_PI);
	nextPos[3] = 0;
	initscr();
	curs_set(0);
	noecho();
	timeout(0);
	std::atexit((void (*)())endwin);
	int wamY = 0, wamX = 0;
	int ftsY = 0, ftsX = 0;
	int handY = 0, handX = 0;
	int line = 0;
	mvprintw(line++,0, "WAM");
	mvprintw(line++,0, "     Joint Positions (rad): ");
	getyx(stdscr, wamY, wamX);
	mvprintw(line++,0, "  Joint Velocities (rad/s): ");
	mvprintw(line++,0, "       Joint Torques (N*m): ");
	mvprintw(line++,0, "         Tool Position (m): ");
	mvprintw(line++,0, "   Tool Orientation (quat): ");
	line++;
	if (fts != NULL) {
		mvprintw(line++,0, "F/T Sensor");
		mvprintw(line++,0, "             Force (N): ");
		getyx(stdscr, ftsY, ftsX);
		mvprintw(line++,0, "          Torque (N*m): ");
		mvprintw(line++,0, "  Acceleration (m/s^2): ");
		line++;
	}
	if (hand != NULL) {
		mvprintw(line++,0, "Hand");
		mvprintw(line++,0, "      Inner Position (rad): ");
		getyx(stdscr, handY, handX);
		mvprintw(line++,0, "      Outer Position (rad): ");
		mvprintw(line++,0, "  Fingertip Torque sensors: ");
		if ( !hand->hasFingertipTorqueSensors() ) {
			printw(" n/a");
		}
		mvprintw(line++,0, "           Tactile sensors: ");
		if (hand->hasTactSensors()) {
			tps = hand->getTactilePucks();
			for (size_t i = 0; i < tps.size(); ++i) {
				drawBoard(stdscr,
						line, i * TACT_BOARD_STRIDE,
						TACT_BOARD_ROWS, TACT_BOARD_COLS,
						TACT_CELL_HEIGHT, TACT_CELL_WIDTH);
			}
		} else {
			printw(" n/a");
		}
		line++;
	}
	jp_type jp;jv_type jv;jt_type jt;cp_type cp;Eigen::Quaterniond to;math::Matrix<6,DOF> J;cf_type cf;ct_type ct;ca_type ca;Hand::jp_type hjp;
	while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
		line = wamY;
		jp = math::saturate(wam.getJointPositions(), 9.999);
		mvprintw(line++,wamX, "[%6.3f", jp[0]);
		for (size_t i = 1; i < DOF; ++i) {
			printw(", %6.3f", jp[i]);}
		printw("]");
		jv = math::saturate(wam.getJointVelocities(), 9.999);
		mvprintw(line++,wamX, "[%6.3f", jv[0]);
		for (size_t i = 1; i < DOF; ++i) {
			printw(", %6.3f", jv[i]);}
		printw("]");
		jt = math::saturate(wam.getJointTorques(), 99.99);
		mvprintw(line++,wamX, "[%6.2f", jt[0]);
		for (size_t i = 1; i < DOF; ++i) {
			printw(", %6.2f", jt[i]);}
		printw("]");
		cp = math::saturate(wam.getToolPosition(), 9.999);
    	mvprintw(line++,wamX, "[%6.3f, %6.3f, %6.3f]", cp[0], cp[1], cp[2]);
		to = wam.getToolOrientation();  // We work only with unit quaternions. No saturation necessary.
    	mvprintw(line++,wamX, "%+7.4f %+7.4fi %+7.4fj %+7.4fk", to.w(), to.x(), to.y(), to.z());
		if (fts != NULL) {
			line = ftsY;
            fts->update();
            cf = math::saturate(fts->getForce(), 99.99);
        	mvprintw(line++,ftsX, "[%6.2f, %6.2f, %6.2f]", cf[0], cf[1], cf[2]);
            ct = math::saturate(fts->getTorque(), 9.999);
        	mvprintw(line++,ftsX, "[%6.3f, %6.3f, %6.3f]", ct[0], ct[1], ct[2]);
        	fts->updateAccel();
            ca = math::saturate(fts->getAccel(), 99.99);
        	mvprintw(line++,ftsX, "[%6.2f, %6.2f, %6.2f]", ca[0], ca[1], ca[2]);}
		if (hand != NULL) {
			line = handY;
			hand->update();  // Update all sensors
			hjp = math::saturate(hand->getInnerLinkPosition(), 9.999);
			mvprintw(line++,handX, "[%6.3f, %6.3f, %6.3f, %6.3f]",
					hjp[0], hjp[1], hjp[2], hjp[3]);
			hjp = math::saturate(hand->getOuterLinkPosition(), 9.999);
			mvprintw(line++,handX, "[%6.3f, %6.3f, %6.3f, %6.3f]",
					hjp[0], hjp[1], hjp[2], hjp[3]);

			if (hand->hasFingertipTorqueSensors()) {
				mvprintw(line,handX, "[%4d, %4d, %4d, %4d]",
						hand->getFingertipTorque()[0],
						hand->getFingertipTorque()[1],
						hand->getFingertipTorque()[2],
						hand->getFingertipTorque()[3]);}
			line += 2;
			if (hand->hasTactSensors()) {
				for (size_t i = 0; i < tps.size(); ++i) {
					graphPressures(stdscr, line, i * TACT_BOARD_STRIDE,
							tps[i]->getFullData());}}}
		refresh();  // Ask ncurses to display the new text
		usleep(100000);}}
// Function to evaluate and move to the first pose in the trajectory
template<size_t DOF>
void RTLoop<DOF>::moveToStart() {
	if (inputType == 0) {
		wam.moveTo(jpSpline->eval(jpSpline->initialS()), true);
	} else
		wam.moveTo(
				boost::make_tuple(cpSpline->eval(cpSpline->initialS()),
						qSpline->eval(qSpline->initialS())));
}
template<size_t DOF> void RTLoop<DOF>::startPlayback() {time.start();}
template<size_t DOF> void RTLoop<DOF>::pausePlayback() {time.stop();}
template<size_t DOF> bool RTLoop<DOF>::playbackActive() {
	if (inputType == 0)
		return (jpTrajectory->input.getValue() < jpSpline->finalS());
	else {
		return (cpTrajectory->input.getValue() < cpSpline->finalS());
	}
}
template<size_t DOF> void RTLoop<DOF>::disconnectSystems() {
	disconnect(wam.input);
    disconnect(logger->input);
    disconnect(tg.template getInput<0>());
	disconnect(tg.template getInput<1>());
	disconnect(tg.template getInput<2>());
	disconnect(tg.template getInput<3>());
	disconnect(tg.template getInput<4>());
	disconnect(tg.template getInput<5>());
	disconnect(tg.template getInput<6>());
    wam.idle();
	time.stop();
	time.setOutput(0.0);
}
template<size_t DOF>
void RTLoop<DOF>::reconnectSystems(){
	if (inputType == 0) {
		systems::forceConnect(time.output, jpTrajectory->input);
		wam.trackReferenceSignal(jpTrajectory->output);
	} else {
		systems::forceConnect(time.output, cpTrajectory->input);
		systems::forceConnect(time.output, qTrajectory->input);
		systems::forceConnect(cpTrajectory->output, poseTg.getInput<0>());
		systems::forceConnect(qTrajectory->output, poseTg.getInput<1>());
		wam.trackReferenceSignal(poseTg.output);
	}
    systems::forceConnect(time.output,               hand_system->input);
	systems::forceConnect(time.output,               tg.template getInput<0>());
	systems::forceConnect(wam.jpOutput,              tg.template getInput<1>());
	systems::forceConnect(wam.jvOutput,              tg.template getInput<2>());
	systems::forceConnect(wam.jtSum.output,          tg.template getInput<3>());
	systems::forceConnect(wam.toolPosition.output,   tg.template getInput<4>());
	systems::forceConnect(wam.toolOrientation.output,tg.template getInput<5>());
    systems::forceConnect(hand_system->output,       tg.template getInput<6>());
    //systems::forceConnect(time.output,       tg.template getInput<6>());
    systems::forceConnect(tg.output, logger->input);
	time.start();
	printf("Logging started.\n");
}

//gets called at the start of each loop
template<size_t DOF> void RTLoop<DOF>::init_data_logger(){
    //set up realtime data logging
    char tmp_filename_template[] = "/tmp/btXXXXXX";
    int tmp_file_descriptor;

    
	if ((tmp_file_descriptor = mkstemp(tmp_filename_template)) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
	}
    
    //get temporary filename created by system
    char tmp_filename_buf[14];
    std::string read_str = "/proc/self/fd/" + boost::lexical_cast<std::string>(tmp_file_descriptor); 
    readlink(read_str.c_str(),tmp_filename_buf,14);
    std::string tmp_filename(tmp_filename_buf); 
	tmp_filenames.push_back(tmp_filename);

    if(loop_count > 0)
        logger->closeLog();
    const size_t PERIOD_MULTIPLIER = 1;
    logger = new systems::PeriodicDataLogger<tuple_type> (
			pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>((char*)tmp_filename.c_str(), PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);
}
//export to csv files
template<size_t DOF>
void RTLoop<DOF>::output_data_stream(){
    logger->closeLog(); //clost outstanding log
    //save headers for data log of entire trajectory
    std::string log_name = playName.substr(9,playName.length()-4-9); //strip recorded/ and .csv from playName
    std::string header_filename = log_prefix+log_name+".h";
    std::ofstream out(header_filename.c_str());
    out << data_log_headers << std::endl;
    out.close();
    int counter = 0;
    std::vector<std::string>::iterator tmp_filename_it;	
    for (tmp_filename_it=tmp_filenames.begin(); tmp_filename_it < tmp_filenames.end(); tmp_filename_it++){
        std::string tmp_filename = *tmp_filename_it;
        std::string out_filename = log_prefix+itoa(counter++)+log_name+".csv";
        log::Reader<tuple_type> lr((tmp_filename).c_str());
        lr.exportCSV(out_filename.c_str()); 
        std::remove((tmp_filename).c_str());
        printf("Data log saved to the location: %s \n", out_filename.c_str());
    }
        std::cout <<  "All data logs saved successfully!" << std::endl;
}

template<size_t DOF> int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	std::string filename(argv[1]);
    // Load our vc_calibration file.
	libconfig::Config config;
	std::string calibration_file;
	if (DOF == 4)
		calibration_file = "calibration4.conf";
	else
		calibration_file = "calibration7.conf";
	config.readFile(calibration_file.c_str());
	config.getRoot();

	RTLoop<DOF> play(wam, pm, filename, config.getRoot());

	if (!play.init())
		return 1;

	//boost::thread displayThread(&RTLoop<DOF>::displayEntryPoint, &play);

	bool playing = true;
    //bool collecting_data = false;
    play.loop = true;
    curState = PLAYING;
    //lastState = PLAYING;

	//while (playing) {
    while(pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE){
		switch (curState) {
		case QUIT:
			playing = false;
			break;
		case PLAYING:
			switch (lastState) {
			case STOPPED:
				play.moveToStart();
                play.init_data_logger();
				play.reconnectSystems();
				play.startPlayback();
				lastState = PLAYING;
				break;
			case PAUSED:
				play.startPlayback();
				lastState = PLAYING;
				break;
			case PLAYING:
				if (play.playbackActive()) {
					btsleep(0.1);
					break;
				} else if (play.loop) {
                    loop_count++;
					play.disconnectSystems();
					lastState = STOPPED;
					curState = PLAYING;
					break;
                } else {
					curState = STOPPED;
					break;
				}
    			default:
				break;
			}
			break;
		case PAUSED:
			switch (lastState) {
			case PLAYING:
				play.pausePlayback();
				lastState = PAUSED;
				break;
			case PAUSED:
				btsleep(0.1);
				break;
			case STOPPED:
				break;
			default:
				break;
			}
			break;
		case STOPPED:
			switch (lastState) {
			case PLAYING:
				play.disconnectSystems();
				lastState = STOPPED;
				break;
			case PAUSED:
				play.disconnectSystems();
				lastState = STOPPED;
				break;
			case STOPPED:
				btsleep(0.1);
				break;
			default:
				break;
			}
			break;
		}
        /*if(!collecting_data){
            play.collect_data_stream();
            collecting_data = true;
        }*/
	}
	play.disconnectSystems();
    play.output_data_stream();
	//wam.moveHome();
	//printf("\n\n");
	//pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
