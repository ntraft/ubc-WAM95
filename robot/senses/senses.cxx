#include "senses.h"
#include "stdheader.h"
#include "utils.h"
#include "utils-inl.h"
//#include "robot.h"

const int TACT_CELL_HEIGHT = 3;
const int TACT_CELL_WIDTH = 6;
const int TACT_BOARD_ROWS = 8;
const int TACT_BOARD_COLS = 3;
const int TACT_BOARD_STRIDE = TACT_BOARD_COLS * TACT_CELL_WIDTH + 2;


Senses::Senses(ProductManager* pm, Wam<DIMENSION>* wam){
    this->pm = pm;
    this->wam = wam;
	this->fts = NULL;
	if (pm->foundForceTorqueSensor())
        this->fts = pm->getForceTorqueSensor();
	this->hand = NULL;
	if (pm->foundHand())
        this->hand = pm->getHand();

    module_name = "Senses";

    std::cout << "Senses instantiated!" << std::endl;
}

//MAINLINE
void Senses::validate_args(){
    std::cout << "Robot validate args" << std::endl;
}
void Senses::run(){
    bool back = false;
    while (!back){//robot->get_pm()->getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
        step();
        switch (line[0]) {
#define X(aa, bb, cc, dd, ee) \
            case bb: cc; break;
#include "senses_table.h"
#undef X
            default: help(); break;
        }
    }
    exit();
}
void Senses::help(){
    printf("\n");
#define X(aa, bb, cc, dd, ee) \
    printf("     bb dd\n");
#include "senses_table.h"
#undef X
}

//ACCESSORS
ProductManager* Senses::get_pm(){ return pm; }
Wam<DIMENSION>* Senses::get_wam(){ return wam; }
ForceTorqueSensor* Senses::get_fts(){ return fts; }
Hand* Senses::get_hand(){ return hand; }
bool Senses::has_hand(){ return hand != NULL; }
cf_type Senses::get_force(){
    //fts->update();
    return fts->getForce() - tare_value_cf;
}
ct_type Senses::get_torque(){
    //fts->update();
    return fts->getTorque() - tare_value_ct;
}
ca_type Senses::get_accel(){
    //fts->updateAccel();
    return fts->getAccel();
}
jp_type Senses::get_tool_pose(){
    jp_type jp; 
    cp_type cp = wam->getToolPosition(); 
    co_type co = get_tool_orientation();
    int count = 0;
    for(int i = 0 ; i < cp.size(); i++){
        jp[count++] = cp[i];
    }
    for(int i = 0 ; i < co.size(); i++){
        jp[count++] = co[i];
    }
    return jp;
}
co_type Senses::get_tool_orientation(){
    Eigen::Quaterniond q = get_tool_orientation_q();
    return qd2co(&q);
}
Eigen::Quaterniond Senses::get_tool_orientation_q(){
    return wam->getToolOrientation();
}
Hand::jp_type Senses::get_tool_orientation_m(){
    Eigen::Quaterniond q = get_tool_orientation_q();
    return qd2co(&q);
}
Hand::jp_type Senses::get_fingertip_torques(){
    Hand::jp_type torques;
    vector<int> vtorques = hand->getFingertipTorque();
    for(int i = 0; i < 4; i++){ //for each fingertip torque sensor
        torques[i] = vtorques[i];
    }
    return torques - tare_value_ft;
}
Hand::jp_type Senses::get_fingertip_torques(bool realtime){
    hand->update(Hand::S_FINGERTIP_TORQUE,realtime);
    Hand::jp_type torques;
    vector<int> vtorques = hand->getFingertipTorque();
    for(int i = 0; i < 4; i++){ //for each fingertip torque sensor
        torques[i] = vtorques[i];
    }
    return torques - tare_value_ft;
}
int Senses::get_fingertip_torque_value(int finger_num){
    std::vector<int> fingertip_torque = hand->getFingertipTorque();
    return fingertip_torque[finger_num] - tare_value_ft[finger_num];
}
int Senses::get_fingertip_torque_value(int finger_num,bool realtime){
    hand->update(Hand::S_FINGERTIP_TORQUE,realtime);
    std::vector<int> fingertip_torque = hand->getFingertipTorque();
    return fingertip_torque[finger_num] - tare_value_ft[finger_num];
}
Hand::jp_type Senses::get_tactile_sums(){
    Hand::jp_type tact_sums;
    std::vector<TactilePuck*> tps;
    tps = hand->getTactilePucks();
    for(int j = 0; j < 4; j++){ //for each tactile pad
        tact_array_type finger_tact = tps[j]->getFullData();
        tact_sums[j] = 0; 
        for(int i = 0; i < finger_tact.size(); i++){
            tact_sums[j] += finger_tact[i];
        }
    }
    return tact_sums;
}
tv_type Senses::get_tactile_vector(){
    tv_type tact_vec;
    std::vector<TactilePuck*> tps;
    tps = hand->getTactilePucks();
    int count = 0;
    for(int j = 0; j < 2; j++){ //for finger 1 and 2 tactile pads ONLY
        tact_array_type finger_tact = tps[j]->getFullData();
        for(int i = 0; i < finger_tact.size(); i++){
            tact_vec[count++] = finger_tact[i];
        }
    }
    return tact_vec - tare_value_tv;
}
bool Senses::check_tactile_contact(int finger_num){
    //std::cout << "check_tactile_contact!" << std::endl;
    hand->update(Hand::S_TACT_FULL, true);
    std::vector<TactilePuck*> tps;
    tps = hand->getTactilePucks();
    tact_array_type finger_tact = tps[finger_num]->getFullData();
    for(int i = 0; i < finger_tact.size(); i++){
            //std::cout << finger_tact[i] << " > " << sensor_vars[TACT_BASE_VAL](finger_num) << "?" << std::endl;
            if(finger_tact[i] > sensor_vars[TACT_BASE_VAL][finger_num]){
                    return true;
            }
    }
    return false;
}
bool Senses::check_tactile_contact(int finger_num, float threshold){
        //std::cout << "check_tactile_contact!" << std::endl;
        hand->update(Hand::S_TACT_FULL, true);
        std::vector<TactilePuck*> tps;
        tps = hand->getTactilePucks();
        tact_array_type finger_tact = tps[finger_num]->getFullData();
        for(int i = 0; i < finger_tact.size(); i++){
                if(finger_tact[i] > threshold){
                        std::cout << finger_tact[i] << " > " << threshold << std::endl;
                        return true;
                }
        }
        return false;
}
bool Senses::check_tactile_contact(){
        return (check_tactile_contact(0) || check_tactile_contact(1) || check_tactile_contact(2));
}
bool Senses::check_fingertip_torque_contact(int finger_num, int fingertip_torque_thresh){
        hand->update(Hand::S_FINGERTIP_TORQUE,true);
        std::vector<int> fingertip_torque = hand->getFingertipTorque();
        if(fingertip_torque[finger_num] > fingertip_torque_thresh){
                std::cout << fingertip_torque[finger_num] << ">" << fingertip_torque_thresh << std::endl;
                return true;
        }
        else{
                //std::cout << fingertip_torque[finger_num] << ">" << fingertip_torque_thresh << std::endl;
                return false;
        }
}
bool Senses::check_fingertip_torque_contact(int fingertip_torque_thresh){
        return check_fingertip_torque_contact(0, fingertip_torque_thresh)
                || check_fingertip_torque_contact(1, fingertip_torque_thresh)
                || check_fingertip_torque_contact(2, fingertip_torque_thresh);
}
bool Senses::check_fingertip_torque_contact(){
        return check_fingertip_torque_contact(0, sensor_vars[FT_TORQUE_BASE_VAL][0])
                || check_fingertip_torque_contact(1, sensor_vars[FT_TORQUE_BASE_VAL][1])
                || check_fingertip_torque_contact(2, sensor_vars[FT_TORQUE_BASE_VAL][2]);
}
void Senses::tare_all(){
    Senses* senses = this;
#define X(aa, bb, cc, dd, ee) \
        zero_matrix(&tare_value_##cc); \
        tare_value_##cc = dd; \
        cout << "tare_value_" << aa << " = " << tare_value_##cc << endl;
#include "input_type_table.h"
#undef X
}
void Senses::tare_tactile(){
        //std::cout << "check_tactile_contact!" << std::endl;
        hand->update(Hand::S_TACT_FULL, true);
        std::vector<TactilePuck*> tps;
        tps = hand->getTactilePucks();
        std::cout << "tare-value for tactile pad on: " << std::endl;
        for(unsigned int finger_num = 0; finger_num < tps.size(); finger_num++){
                tact_array_type finger_tact = tps[finger_num]->getFullData();
                float max = -1;
                for(int i = 0; i < finger_tact.size(); i++){
                        if(finger_tact[i] > max){
                                max = finger_tact[i];
                        }
                }
                std::cout << "    F" << finger_num+1 << ": " << max << std::endl;
                sensor_vars[TACT_BASE_VAL][finger_num] = max;
        }
}
//reset zero-value of fingertip torque sensors
void Senses::tare_fingertip_torque(){
        hand->update(Hand::S_FINGERTIP_TORQUE,true);
        std::vector<int> fingertip_torque = hand->getFingertipTorque();
        std::cout << "tare-value for fingertip_torque: " << std::endl;
        for(unsigned int finger_num = 0; finger_num < fingertip_torque.size(); finger_num++){
                sensor_vars[FT_TORQUE_BASE_VAL][finger_num] = fingertip_torque[finger_num];
                 std::cout << "    F" << finger_num+1 << ": " << fingertip_torque[finger_num] << std::endl;
        }
}
void Senses::display(){
    
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DIMENSION);
    /*
    // Is an FTS attached?
	ForceTorqueSensor* fts = NULL;
	if (pm->foundForceTorqueSensor()) {
		fts = pm->getForceTorqueSensor();
		fts->tare();
	}

	// Is a Hand attached?
	Hand* hand = NULL;
	if (pm->foundHand()) {
		hand = pm->get_hand();

		printf(">>> Press [Enter] to initialize Hand. (Make sure it has room!)");
		waitForEnter();
		hand->initialize();
	}
    */
	std::vector<TactilePuck*> tps;
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
void Senses::drawBoard(WINDOW *win, int starty, int startx, int rows, int cols,
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

void Senses::graphCell(WINDOW *win, int starty, int startx, double pressure) {
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

void Senses::graphPressures(WINDOW *win, int starty, int startx,
		const TactilePuck::v_type& pressures) {
	for (int i = 0; i < pressures.size(); ++i) {
		graphCell(win,
				starty + 1 + TACT_CELL_HEIGHT *
						(TACT_BOARD_ROWS - 1 - (i / 3 /* integer division */)),
				startx + 1 + TACT_CELL_WIDTH * (i % TACT_BOARD_COLS),
				pressures[i]);
	}
}
