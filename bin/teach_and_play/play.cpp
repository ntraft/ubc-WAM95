/*
 * play.cpp
 *
 *  Created on: Sept 26, 2012
 *      Author: Kyle Maroney
 */

#include <string>
#include <boost/tuple/tuple.hpp>
#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <libconfig.h++>
#include <Eigen/Core>

#include <barrett/exception.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

#include "control_mode_switcher.h"

#include <iostream>  // For std::cin
#include <string>  // For std::string and std::getline()
#include <cstdlib>  // For std::atexit()

#include <unistd.h>  // For usleep()

// The ncurses library allows us to write text to any location on the screen
#include <curses.h>

#include <barrett/math.h>  // For barrett::math::saturate()
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>


using namespace barrett;
using detail::waitForEnter;

enum STATE {
	PLAYING, STOPPED, PAUSED, QUIT
} curState = STOPPED, lastState = STOPPED;

char* ctrlMode = NULL;
bool vcMode = false;

// Functions that help display data from the Hand's (optional) tactile sensors.
// Note that the palm tactile sensor has a unique cell layout that these
// functions do not print correctly.
const int TACT_CELL_HEIGHT = 3;
const int TACT_CELL_WIDTH = 6;
const int TACT_BOARD_ROWS = 8;
const int TACT_BOARD_COLS = 3;
const int TACT_BOARD_STRIDE = TACT_BOARD_COLS * TACT_CELL_WIDTH + 2;
void drawBoard(WINDOW *win, int starty, int startx, int rows, int cols,
		int tileHeight, int tileWidth);
void graphPressures(WINDOW *win, int starty, int startx,
		const TactilePuck::v_type& pressures);

void drawBoard(WINDOW *win, int starty, int startx, int rows, int cols,
		int tileHeight, int tileWidth) {
	int endy, endx, i, j;

	endy = starty + rows * tileHeight;
	endx = startx + cols * tileWidth;

	for (j = starty; j <= endy; j += tileHeight)
		for (i = startx; i <= endx; ++i)
			mvwaddch(win, j, i, ACS_HLINE);
	for (i = startx; i <= endx; i += tileWidth)
		for (j = starty; j <= endy; ++j)
			mvwaddch(win, j, i, ACS_VLINE);
	mvwaddch(win, starty, startx, ACS_ULCORNER);
	mvwaddch(win, endy, startx, ACS_LLCORNER);
	mvwaddch(win, starty, endx, ACS_URCORNER);
	mvwaddch(win, endy, endx, ACS_LRCORNER);
	for (j = starty + tileHeight; j <= endy - tileHeight; j += tileHeight) {
		mvwaddch(win, j, startx, ACS_LTEE);
		mvwaddch(win, j, endx, ACS_RTEE);
		for (i = startx + tileWidth; i <= endx - tileWidth; i += tileWidth)
			mvwaddch(win, j, i, ACS_PLUS);
	}
	for (i = startx + tileWidth; i <= endx - tileWidth; i += tileWidth) {
		mvwaddch(win, starty, i, ACS_TTEE);
		mvwaddch(win, endy, i, ACS_BTEE);
	}
}

void graphCell(WINDOW *win, int starty, int startx, double pressure) {
	int i, chunk;
	char c;

	int value = (int)(pressure * 256.0) / 102;  // integer division
//	int value = (int)(pressure * 256.0) / 50; // integer division
	for (i = 4; i >= 0; --i) {
		chunk = (value <= 7) ? value : 7;
		value -= chunk;

		switch (chunk) {
		default:  c = '#'; break;
		case 2:   c = '~'; break;
		case 1:   c = '-'; break;
		case 0:   c = '_'; break;
		}
		mvwprintw(win, starty + 1, startx + i, "%c", c);

		switch (chunk - 4) {
		case 3:   c = '#'; break;
		case 2:   c = '~'; break;
		case 1:   c = '-'; break;
		case 0:   c = '_'; break;
		default:  c = ' '; break;
		}
		mvwprintw(win, starty, startx + i, "%c", c);
	}
}

void graphPressures(WINDOW *win, int starty, int startx,
		const TactilePuck::v_type& pressures) {
	for (int i = 0; i < pressures.size(); ++i) {
		graphCell(win,
				starty + 1 + TACT_CELL_HEIGHT *
						(TACT_BOARD_ROWS - 1 - (i / 3 /* integer division */)),
				startx + 1 + TACT_CELL_WIDTH * (i % TACT_BOARD_COLS),
				pressures[i]);
	}
}




bool validate_args(int argc, char** argv) {
	switch (argc) {
	case 2:
		if (boost::filesystem::exists(argv[1])) {
			printf("\nTrajectory to be played in current control mode: %s\n\n",
					argv[1]);
			return true;
			break;
		} else {
			printf("\nTrajectory not found in location specified: %s\n\n",
					argv[1]);
			return false;
			break;
		}
	case 3:
		ctrlMode = argv[2];
		if (boost::filesystem::exists(argv[1])
				&& (strcmp(ctrlMode, "cc") == 0 || strcmp(ctrlMode, "-cc") == 0
						|| strcmp(ctrlMode, "vc") == 0
						|| strcmp(ctrlMode, "-vc") == 0)) {
			printf(
					"\nTrajectory to be played in %s mode: %s\n\n",
					strcmp(ctrlMode, "vc") == 0
							|| strcmp(ctrlMode, "-vc") == 0 ?
							"voltage control" : "current control", argv[1]);
			if (strcmp(ctrlMode, "vc") == 0 || strcmp(ctrlMode, "-vc") == 0)
				vcMode = true;
			return true;
			break;
		} else {
			printf("\nTrajectory not found in location specified: %s\n\n",
					argv[1]);
			return false;
			break;
		}

	default:
		printf("Usage: %s <path/to/trajectory> [<Control Mode (cc or vc)>]\n",
				argv[0]);
		return false;
		break;
	}
}

//Play Class
template<size_t DOF>
class Play {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
protected:
	systems::Wam<DOF>& wam;
	Hand* hand;
	ProductManager& pm;
	std::string playName;
	int inputType;
	const libconfig::Setting& setting;
	libconfig::Config config;
	typedef boost::tuple<double, jp_type> input_jp_type;
	typedef boost::tuple<double, cp_type> input_cp_type;
	typedef boost::tuple<double, Eigen::Quaterniond> input_quat_type;

	ControlModeSwitcher<DOF>* cms;

	std::vector<input_cp_type, Eigen::aligned_allocator<input_cp_type> >* cpVec;
	std::vector<input_quat_type, Eigen::aligned_allocator<input_quat_type> >* qVec;
	math::Spline<jp_type>* jpSpline;
	math::Spline<cp_type>* cpSpline;
	math::Spline<Eigen::Quaterniond>* qSpline;
	systems::Callback<double, jp_type>* jpTrajectory;
	systems::Callback<double, cp_type>* cpTrajectory;
	systems::Callback<double, Eigen::Quaterniond>* qTrajectory;
	systems::TupleGrouper<cp_type, Eigen::Quaterniond> poseTg;
	systems::Ramp time;

public:
	int dataSize;
	bool loop;

	Play(systems::Wam<DOF>& wam_, ProductManager& pm_, std::string filename_,
			const libconfig::Setting& setting_) :
			wam(wam_), hand(NULL), pm(pm_), playName(filename_), inputType(0), setting(
					setting_), cms(NULL), cpVec(NULL), qVec(NULL), jpSpline(
					NULL), cpSpline(NULL), qSpline(NULL), jpTrajectory(NULL), cpTrajectory(
					NULL), qTrajectory(NULL), time(pm.getExecutionManager()), dataSize(
					0), loop(false) {
	}
	bool
	init();

	~Play() {
	}

	void
	displayEntryPoint();
	void
	moveToStart();
	void
	startPlayback();
	void
	pausePlayback();
	bool
	playbackActive();
	void
	disconnectSystems();
	void
	reconnectSystems();

private:
	DISALLOW_COPY_AND_ASSIGN(Play);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

}
;

// Initialization - Gravity compensating, setting safety limits, parsing input file and creating trajectories
template<size_t DOF>
bool Play<DOF>::init() {
	// Turn on Gravity Compensation
	wam.gravityCompensate(true);

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
		printf("Switching system to voltage control mode");
		cms->voltageControl();
		//Allow the mechanical system to settle
		btsleep(2.0);
	} else {
		printf("Verifying system is in current control mode");
		cms->currentControl();
	}

	pm.getExecutionManager()->startManaging(time); //starting time management
	return true;
}

// This function will run in a different thread and control displaying to the screen and user input
template<size_t DOF>
void Play<DOF>::displayEntryPoint() {

    // Is an FTS attached?
	ForceTorqueSensor* fts = NULL;
	if (pm.foundForceTorqueSensor()) {
		fts = pm.getForceTorqueSensor();
		fts->tare();
	}

	// Is a Hand attached?
	Hand* hand = NULL;
	std::vector<TactilePuck*> tps;
	if (pm.foundHand()) {
		hand = pm.getHand();

		//printf(">>> Press [Enter] to initialize Hand. (Make sure it has room!)");
		//waitForEnter();
		//hand->initialize();
	}   
    
    
    // Some hand variables allow for switching between open and close positions
	Hand::jp_type currentPos(0.0);
	Hand::jp_type nextPos(M_PI);
	nextPos[3] = 0;

/* Instructions displayed to screen.
	printf("\n");
	printf("Commands:\n");
	printf("  p    Play\n");
	printf("  i    Pause\n");
	printf("  s    Stop\n");
	printf("  l    Loop\n");
	printf("  q    Quit\n");
	printf("  At any time, press [Enter] to open or close the Hand.\n");
	printf("\n");

	std::string line;*/

    // Set up the ncurses environment
	initscr();
	curs_set(0);
	noecho();
	timeout(0);

	// Make sure we cleanup after ncurses when the program exits
	std::atexit((void (*)())endwin);



	// Set up the static text on the screen
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


	// Display loop!
	jp_type jp;
	jv_type jv;
	jt_type jt;
	cp_type cp;
	Eigen::Quaterniond to;
	math::Matrix<6,DOF> J;

	cf_type cf;
	ct_type ct;
	ca_type ca;

	Hand::jp_type hjp;

	//while (true) {




	// Fall out of the loop once the user Shift-idles
	while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
		// WAM
		line = wamY;

		// math::saturate() prevents the absolute value of the joint positions
		// from exceeding 9.9999. This puts an upper limit on the length of the
		// string that gets printed to the screen below. We do this to make sure
		// that the string will fit properly on the screen.
		jp = math::saturate(wam.getJointPositions(), 9.999);
		mvprintw(line++,wamX, "[%6.3f", jp[0]);
		for (size_t i = 1; i < DOF; ++i) {
			printw(", %6.3f", jp[i]);
		}
		printw("]");

		jv = math::saturate(wam.getJointVelocities(), 9.999);
		mvprintw(line++,wamX, "[%6.3f", jv[0]);
		for (size_t i = 1; i < DOF; ++i) {
			printw(", %6.3f", jv[i]);
		}
		printw("]");

		jt = math::saturate(wam.getJointTorques(), 99.99);
		mvprintw(line++,wamX, "[%6.2f", jt[0]);
		for (size_t i = 1; i < DOF; ++i) {
			printw(", %6.2f", jt[i]);
		}
		printw("]");

		cp = math::saturate(wam.getToolPosition(), 9.999);
    	mvprintw(line++,wamX, "[%6.3f, %6.3f, %6.3f]", cp[0], cp[1], cp[2]);

		to = wam.getToolOrientation();  // We work only with unit quaternions. No saturation necessary.
    	mvprintw(line++,wamX, "%+7.4f %+7.4fi %+7.4fj %+7.4fk", to.w(), to.x(), to.y(), to.z());


		// FTS
		if (fts != NULL) {
			line = ftsY;

            fts->update();
            cf = math::saturate(fts->getForce(), 99.99);
        	mvprintw(line++,ftsX, "[%6.2f, %6.2f, %6.2f]", cf[0], cf[1], cf[2]);
            ct = math::saturate(fts->getTorque(), 9.999);
        	mvprintw(line++,ftsX, "[%6.3f, %6.3f, %6.3f]", ct[0], ct[1], ct[2]);

        	fts->updateAccel();
            ca = math::saturate(fts->getAccel(), 99.99);
        	mvprintw(line++,ftsX, "[%6.2f, %6.2f, %6.2f]", ca[0], ca[1], ca[2]);
		}


		// Hand
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
						hand->getFingertipTorque()[3]);
			}

			line += 2;
			if (hand->hasTactSensors()) {
				for (size_t i = 0; i < tps.size(); ++i) {
					graphPressures(stdscr, line, i * TACT_BOARD_STRIDE,
							tps[i]->getFullData());
				}
			}
		}


		refresh();  // Ask ncurses to display the new text
		usleep(100000);  // Slow the loop rate down to roughly 10 Hz
	//}

        /*
		// Continuously parse our line input
		printf(">> ");
		std::getline(std::cin, line);

		if (line.size() == 0) { // Enter press recognized without any input
			// If hand is present, open or close accordingly
			if (hand != NULL) {
				hand->trapezoidalMove(nextPos, false);
				std::swap(currentPos, nextPos);
			}
		} else { // User input - Set State
			switch (line[0]) {
			case 'p':
				curState = PLAYING;
				break;
			case 'i':
				curState = PAUSED;
				break;
			case 's':
				loop = false;
				curState = STOPPED;
				break;
			case 'l':
				loop = true;
				curState = PLAYING;
				break;
			case 'q':
				loop = false;
				cms->currentControl();
				curState = QUIT;
				break;
			default:
				break;
			}
		}*/
        loop = true;
		curState = PLAYING;
		//break;
	}
}

// Function to evaluate and move to the first pose in the trajectory
template<size_t DOF>
void Play<DOF>::moveToStart() {
	if (inputType == 0) {
		wam.moveTo(jpSpline->eval(jpSpline->initialS()), true);
	} else
		wam.moveTo(
				boost::make_tuple(cpSpline->eval(cpSpline->initialS()),
						qSpline->eval(qSpline->initialS())));
}

template<size_t DOF>
void Play<DOF>::startPlayback() {
	time.start();
}

template<size_t DOF>
void Play<DOF>::pausePlayback() {
	time.stop();
}

template<size_t DOF>
bool Play<DOF>::playbackActive() {
	if (inputType == 0)
		return (jpTrajectory->input.getValue() < jpSpline->finalS());
	else {
		return (cpTrajectory->input.getValue() < cpSpline->finalS());
	}
}

template<size_t DOF>
void Play<DOF>::disconnectSystems() {
	disconnect(wam.input);
	wam.idle();
	time.stop();
	time.setOutput(0.0);
}

template<size_t DOF>
void Play<DOF>::reconnectSystems() {
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
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
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

	Play<DOF> play(wam, pm, filename, config.getRoot());

	if (!play.init())
		return 1;

	boost::thread displayThread(&Play<DOF>::displayEntryPoint, &play);

	bool playing = true;
    //play.loop = true;
    //curState = PLAYING;
    //lastState = PLAYING;

	while (playing) {
		switch (curState) {
		case QUIT:
			playing = false;
			break;
		case PLAYING:
			switch (lastState) {
			case STOPPED:
				play.moveToStart();
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
	}

	wam.moveHome();
	printf("\n\n");
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
