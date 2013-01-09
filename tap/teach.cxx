/*
 * teach.cpp
 *
 *  Created on: Aug 31, 2012
 *      Author: km
 */
/*
#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/thread.hpp>
#include <curses.h>

#include <barrett/detail/stl_utils.h>
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>
*/
//#define BARRETT_SMF_VALIDATE_ARGS
//#include <barrett/standard_main_function.h>

#include "control_mode_switcher.h"

#include "stdheader.h"

/*using namespace barrett;
using detail::waitForEnter;
using systems::connect;
using systems::disconnect;
using systems::reconnect;
*/
int recordType = 0;

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

/*
bool validate_args(int argc, char** argv) {
	switch (argc) {
	case 2:
		printf("\nTrajectory to be recorded in joint space: %s\n\n", argv[1]);
		return true;
		break;
	case 3: {
		char* recordMode(argv[2]);
		if ((strcmp(recordMode, "jp") == 0 || strcmp(recordMode, "-jp") == 0
				|| strcmp(recordMode, "pose") == 0
				|| strcmp(recordMode, "-pose") == 0)) {
			printf(
					"\nTrajectory to be recorded in %s: %s\n\n",
					strcmp(recordMode, "jp") == 0
							|| strcmp(recordMode, "-jp") == 0 ?
							"joint space" : "Cartesian space", argv[1]);
			if (strcmp(recordMode, "pose") == 0
					|| strcmp(recordMode, "-pose") == 0)
				recordType = 1;
			return true;
			break;
		} else {
			printf(
					"\nIncorrect recording data type specified: must be jp or pose\n\n");
			return false;
			break;
		}
	}
	default:
		printf("Usage: %s <trajectory_name> [<Record Mode (jp or pose)>]\n",
				argv[0]);
		return false;
		break;
	}
}
*/
//Teach Class

class Teach {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DIMENSION);
protected:
    Robot* robot;
	Wam<DIMENSION>* wam;
	ProductManager* pm;
	std::string tmpStr, saveName, fileOut;
	char* tmpFile;
	struct LateInitializedDataMembers;
	LateInitializedDataMembers* d;
	int dLine, dX, dY, key;
	bool initCurses, displaying;
	typedef boost::tuple<double, jp_type> input_jp_type;
	systems::Ramp time;
	systems::TupleGrouper<double, jp_type> jpLogTg;
	systems::TupleGrouper<double, pose_type> poseLogTg;
	typedef boost::tuple<double, jp_type> jp_sample_type;
	typedef boost::tuple<double, pose_type> pose_sample_type;
	systems::PeriodicDataLogger<jp_sample_type>* jpLogger;
	systems::PeriodicDataLogger<pose_sample_type>* poseLogger;

public:
	Teach(//systems::Wam<DIMENSION>& wam_, ProductManager& pm_, 
            Robot* _robot, std::string filename_) :
                robot(_robot), wam(robot->get_wam()), pm(robot->get_pm()), 
                    tmpStr("/tmp/btXXXXXX"), saveName(filename_), 
                    dLine( 0), dX(0), dY(0), key(0), initCurses(true), displaying(true), time(NULL) {
        //Teach teach(wam, pm, filename);
	}
	bool init();
    void run();
	void record();
	void display();
	void createSpline();
}
;


bool Teach::init() {
	tmpFile = new char[tmpStr.length() + 1];
	strcpy(tmpFile, tmpStr.c_str());
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return false;
	}

	pm->getSafetyModule()->setVelocityLimit(1.5);
	pm->getSafetyModule()->setTorqueLimit(3.0);

	wam->gravityCompensate();

	pm->getExecutionManager()->startManaging(time); //starting time manager

	if (recordType == 0)
		jpLogger = new systems::PeriodicDataLogger<jp_sample_type>(
				pm->getExecutionManager(),
				new barrett::log::RealTimeWriter<jp_sample_type>(tmpFile,
						pm->getExecutionManager()->getPeriod()), 1);
	else
		poseLogger = new systems::PeriodicDataLogger<pose_sample_type>(
				pm->getExecutionManager(),
				new barrett::log::RealTimeWriter<pose_sample_type>(tmpFile,
						pm->getExecutionManager()->getPeriod()), 1);
	return true;
}


void Teach::record() {
	BARRETT_SCOPED_LOCK(pm->getExecutionManager()->getMutex());

	if (recordType == 0) {
		connect(time.output, jpLogTg.getInput<0>());
		connect(wam->jpOutput, jpLogTg.getInput<1>());
		connect(jpLogTg.output, jpLogger->input);
	} else {
		connect(time.output, poseLogTg.getInput<0>());
		connect(wam->toolPose.output, poseLogTg.getInput<1>());
		connect(poseLogTg.output, poseLogger->input);
	}

	time.start();
}


void Teach::display() {
	/*BARRETT_UNITS_TEMPLATE_TYPEDEFS(DIMENSION);
	int cnt = 0;
	int scnt = 0;
	if (initCurses) {
		// Set up the ncurses environment
		initscr();
		//curs_set(0);
		noecho();
		timeout(0);
		cbreak();

		//mvprintw(dLine++, 0, "WAM");
		//mvprintw(dLine++, 0, "     Joint Positions (rad): ");
		getyx(stdscr, dY, dX);

		for (size_t i = 0; i < DIMENSION; i++)
			//mvprintw(dLine++, 0, "                        J%zu: ", i + 1);
			refresh();
		initCurses = false;
	}
	while (displaying) {
		dLine = 2;
		//key = getch();
		//jp_type jpc = wam->getJointPositions();
		for (size_t i = 0; i < DIMENSION; i++)
			//mvprintw(dLine++, dX, "[%6.3f]", jpc[i]);
			if (getch() != -1) {
				cnt++;
			}
		mvprintw(dLine++, dX, "Key = %d", getch());
		mvprintw(dLine++, dX, "Count = %d", cnt);
		mvprintw(dLine++, dX, "Count = %d", scnt);
		scnt++;
		refresh();
		btsleep(0.1);
		if (scnt > 50)
			displaying = false;
	}
	endwin();*/
    // Is an FTS attached?
	ForceTorqueSensor* fts = NULL;
	if (pm->foundForceTorqueSensor()) {
		fts = pm->getForceTorqueSensor();
		fts->tare();
	}

	// Is a Hand attached?
	Hand* hand = NULL;
	std::vector<TactilePuck*> tps;
	if (pm->foundHand()) {
		hand = pm->getHand();

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
	math::Matrix<6,DIMENSION> J;

	cf_type cf;
	ct_type ct;
	ca_type ca;

	Hand::jp_type hjp;

	//while (true) {




	// Fall out of the loop once the user Shift-idles
	while (pm->getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
		// WAM
		line = wamY;

		// math::saturate() prevents the absolute value of the joint positions
		// from exceeding 9.9999. This puts an upper limit on the length of the
		// string that gets printed to the screen below. We do this to make sure
		// that the string will fit properly on the screen.
		jp = math::saturate(wam->getJointPositions(), 9.999);
		mvprintw(line++,wamX, "[%6.3f", jp[0]);
		for (size_t i = 1; i < DIMENSION; ++i) {
			printw(", %6.3f", jp[i]);
		}
		printw("]");

		jv = math::saturate(wam->getJointVelocities(), 9.999);
		mvprintw(line++,wamX, "[%6.3f", jv[0]);
		for (size_t i = 1; i < DIMENSION; ++i) {
			printw(", %6.3f", jv[i]);
		}
		printw("]");

		jt = math::saturate(wam->getJointTorques(), 99.99);
		mvprintw(line++,wamX, "[%6.2f", jt[0]);
		for (size_t i = 1; i < DIMENSION; ++i) {
			printw(", %6.2f", jt[i]);
		}
		printw("]");

		cp = math::saturate(wam->getToolPosition(), 9.999);
    	mvprintw(line++,wamX, "[%6.3f, %6.3f, %6.3f]", cp[0], cp[1], cp[2]);

		to = wam->getToolOrientation();  // We work only with unit quaternions. No saturation necessary.
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
    }
}


void Teach::createSpline() {
	saveName = "recorded/" + saveName;
	if (recordType == 0) {
		jpLogger->closeLog();
		disconnect(jpLogger->input);
		// Build spline between recorded points
		log::Reader<jp_sample_type> lr(tmpFile);
		lr.exportCSV(saveName.c_str());
	}
	else{
		poseLogger->closeLog();
		disconnect(poseLogger->input);
		log::Reader<pose_sample_type> pr(tmpFile);
		pr.exportCSV(saveName.c_str());
	}
	// Adding our datatype as the first line of the recorded trajectory
	fileOut = saveName + ".csv";
	std::ifstream in(saveName.c_str());
	std::ofstream out(fileOut.c_str());
	if (recordType == 0)
		out << "jp_type\n";
	else
		out << "pose_type\n";
	out << in.rdbuf();
	out.close();
	in.close();
	remove(saveName.c_str());
	printf("Trajectory saved to the location: %s \n\n ", fileOut.c_str());
}

void Teach::run(){
	//std::string filename(argv[1]);


	init();

	printf("\nPress [Enter] to start teaching.\n");
	waitForEnter();
	record();
	//boost::thread t(&Teach::display, &teach);

	printf("Press [Enter] to stop teaching.\n");
	waitForEnter();
	createSpline();
	
	pm->getSafetyModule()->waitForMode(SafetyModule::IDLE);
}
