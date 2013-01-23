#include "display.h"


// Functions that help display data from the Hand's (optional) tactile sensors.
// Note that the palm tactile sensor has a unique cell layout that these
// functions do not print correctly.
const int TACT_CELL_HEIGHT = 3;
const int TACT_CELL_WIDTH = 6;
const int TACT_BOARD_ROWS = 8;
const int TACT_BOARD_COLS = 3;
const int TACT_BOARD_STRIDE = TACT_BOARD_COLS * TACT_CELL_WIDTH + 2;

void Display::drawBoard(WINDOW *win, int starty, int startx, int rows, int cols,
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

void Display::graphCell(WINDOW *win, int starty, int startx, double pressure) {
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

void Display::graphPressures(WINDOW *win, int starty, int startx,
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
}*/
/*
// This function will run in a different thread and control displaying to the screen and user input
void Display::displayEntryPoint(ProductManager* pm,Wam<DIMENSION>* wam) {
	ForceTorqueSensor* fts = NULL;
	if (pm.foundForceTorqueSensor()) {
		fts = pm.getForceTorqueSensor();
		fts->tare();
	}
    Hand = pm->getHand(); 
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
	jp_type jp;jv_type jv;jt_type jt;cp_type cp;Eigen::Quaterniond to;math::Matrix<6,DIMENSION> J;cf_type cf;ct_type ct;ca_type ca;Hand::jp_type hjp;
	while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
		line = wamY;
		jp = math::saturate(wam->getJointPositions(), 9.999);
		mvprintw(line++,wamX, "[%6.3f", jp[0]);
		for (size_t i = 1; i < DIMENSION; ++i) {
			printw(", %6.3f", jp[i]);}
		printw("]");
		jv = math::saturate(wam->getJointVelocities(), 9.999);
		mvprintw(line++,wamX, "[%6.3f", jv[0]);
		for (size_t i = 1; i < DIMENSION; ++i) {
			printw(", %6.3f", jv[i]);}
		printw("]");
		jt = math::saturate(wam->getJointTorques(), 99.99);
		mvprintw(line++,wamX, "[%6.2f", jt[0]);
		for (size_t i = 1; i < DIMENSION; ++i) {
			printw(", %6.2f", jt[i]);}
		printw("]");
		cp = math::saturate(wam->getToolPosition(), 9.999);
    	mvprintw(line++,wamX, "[%6.3f, %6.3f, %6.3f]", cp[0], cp[1], cp[2]);
		to = wam->getToolOrientation();  // We work only with unit quaternions. No saturation necessary.
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
		usleep(100000);}}*/
// Function to evaluate and move to the first pose in the trajectory
