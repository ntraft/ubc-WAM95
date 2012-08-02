/*
 * master_master.cpp
 *
 *  Created on: Apr 22, 2012
 *      Author: Daniel Troniak
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>  // For strtod()
#include <algorithm>	//for min/max
#include <unistd.h> // For usleep()
#include <math.h>	//for sin/cos

#include <boost/thread/thread.hpp>
#include <boost/thread.hpp>

#include <boost/tuple/tuple.hpp>
#include <barrett/log.h>

#include <barrett/detail/ca_macro.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/products/hand.h>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

#include "main.h"

// The ncurses library allows us to write text to any location on the screen
#include <curses.h>
#include <barrett/math.h>  // For barrett::math::saturate()

#define DIMENSION 7u
#define STRAIN2TORQUE_RATIO 118.0	//convert hand strain to N-m
#define FINGER_JOINT_LIMIT 2.4435	//=140 degrees
#define ZERO_STRAIN_THRESHOLD 2000	//required threshold to be considered non-noise reading
#define ZERO_TACTILE_THRESHOLD 0.5	//required threshold to be considered non-noise reading

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
enum ACTION_PHASES{
	APPROACH = -1,
	PRELOAD,
	LOADING,
	TRANSITIONAL,
	STATIC,
	REPLACEMENT,
	UNLOADING,
	NUM_ACTION_PHASES
} ActionPhase;

using namespace barrett;
using systems::connect;
using detail::waitForEnter;

typedef math::Vector<24>::type v_type;

//thread management
bool expsemastop = true;
bool datasemastop = true;
bool backdrivesemastop = true;
bool graspsemastop = true;
bool showsensorsemastop = true;

// Functions that help display data from the Hand's (optional) tactile sensors.
// Note that the palm tactile sensor has a unique cell layout that these
// functions do not print  correctly.
const int TACT_CELL_HEIGHT = 3;
const int TACT_CELL_WIDTH = 6;
const int TACT_BOARD_ROWS = 8;
const int TACT_BOARD_COLS = 3;
const int TACT_BOARD_STRIDE = TACT_BOARD_COLS * TACT_CELL_WIDTH + 2;
void drawBoard(WINDOW *win, int starty, int startx, int rows, int cols,
		int tileHeight, int tileWidth);
void graphPressures(WINDOW *win, int starty, int startx,
		const TactilePuck::v_type& pressures);

//constants for hand sensors
static const unsigned int S_POSITION          = 1 << 0;
static const unsigned int S_FINGER_TIP_TORQUE = 1 << 1;
static const unsigned int S_TACT_FULL         = 1 << 2;
static const unsigned int S_ALL = S_POSITION | S_FINGER_TIP_TORQUE | S_TACT_FULL;
		
//for data logging
char tmpFile[14] = "/tmp/btXXXXXX";
char outFile[14] = "data/data.csv";
void dataCollect(Hand* hand, ForceTorqueSensor* fts, void* wamin, ProductManager* pm, 
					enum EXPERIMENT_KEYS expnum, enum EXPERIMENT_SHAPES expshape);
void runExperiment(	Hand* hand, ForceTorqueSensor* fts, void* wamin, ProductManager* pm,
					enum EXPERIMENT_KEYS expnum);
void backDriveHand(Hand* hand, ForceTorqueSensor* fts, void* wamin, ProductManager* pm);
void graspObject(Hand* hand);
void stop_thread(bool* semaphore);

//experiment variables
bool realtime = true;
int prev_state = -1;
int num_runs = 1;
std::string experiment_keys[NUM_EXPERIMENTS] = {
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
};
std::string experiment_shapes[NUM_SHAPES] = {
	"circle",
	"square",
	"triangle"
};
systems::Wam<DIMENSION>::jp_type wamBottom;
systems::Wam<DIMENSION>::jp_type wamTop;
systems::Wam<DIMENSION>::cp_type wamBottomC;
systems::Wam<DIMENSION>::cp_type wamBottomC_ip;	//intermediate point
systems::Wam<DIMENSION>::cp_type wamTopC;
Hand::jp_type wamBottomO;
Hand::jp_type wamTopO;
Eigen::Quaterniond wamTopQ;
Eigen::Quaterniond wamBottomQ;
Hand::jv_type handPregrasp;
Hand::jv_type handGrasp;
Hand::jv_type handUnGrasp;
Hand::jv_type tact_base_val;
Hand::jv_type strain_base_val;
Hand::ct_type torque_epsilon;
systems::Wam<DIMENSION>::jp_type joint_tolerance;
systems::Wam<DIMENSION>::jp_type temp;
systems::Wam<DIMENSION>::jp_type misc_parms;	//miscellaneous parameters
Hand::jp_type finger_contacts;	//entries are 0 if no contact, 1 if contact
enum EXPERIMENT_SHAPES expshape;

//experiment-specific flags
bool move_and_lift;
bool land_and_stroke;

//data collection vectors
std::vector< std::vector<int> > hstrain;	//hand strain measure
std::vector<double> min_hstrain(4,9999999);//running minimum values
std::vector<Hand::ct_type> ct;	//cartesian torque
Hand::ct_type min_ct;	//running minimum values
bool collectData = false;

//returns a random float between a and b
float randomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}
std::string num2str(int number){
   std::stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}
std::string num2str(float number){
   std::stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}
std::string num2str(double number){
   std::stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}
Eigen::Quaterniond hjp2quaternion(Hand::jp_type* p){
	return Eigen::Quaterniond((*p)[0], (*p)[1], (*p)[2], (*p)[3]);
}
Hand::jp_type quaternion2hjp(Eigen::Quaterniond* q){
	return Hand::jp_type(q->w(), q->x(), q->y(), q->z());
}
bool validate_args(int argc, char** argv) {
	/*if (argc != 2  &&  argc != 3) {
		printf("Usage: %s <remoteHost> [--auto]\n", argv[0]);
		printf("  --auto : Automatically link WAMs and start Hand or Gimbals Hand Controller thread\n");

		return false;
	}*/
	return true;
}
bool check_tactile_contact(Hand* hand, int finger_num){
	//std::cout << "check_tactile_contact!" << std::endl;
	hand->update(S_TACT_FULL, true);
	std::vector<TactilePuck*> tps;
	tps = hand->getTactilePucks();
	v_type finger_tact = tps[finger_num]->getFullData();
	for(int i = 0; i < finger_tact.size(); i++){
		//std::cout << finger_tact[i] << " > " << tact_base_val(finger_num) << "?" << std::endl;
		if(finger_tact[i] > tact_base_val[finger_num]){
			return true;
		}
	}
	return false;
}
bool check_tactile_contact(Hand* hand, int finger_num, float threshold){
	//std::cout << "check_tactile_contact!" << std::endl;
	hand->update(S_TACT_FULL, true);
	std::vector<TactilePuck*> tps;
	tps = hand->getTactilePucks();
	v_type finger_tact = tps[finger_num]->getFullData();
	for(int i = 0; i < finger_tact.size(); i++){
		if(finger_tact[i] > threshold){
			std::cout << finger_tact[i] << " > " << threshold << std::endl;
			return true;
		}
	}
	return false;
}
bool check_tactile_contact(Hand* hand){
	return (check_tactile_contact(hand, 0) || check_tactile_contact(hand, 1) || check_tactile_contact(hand, 2));
}
bool check_strain_contact(Hand* hand, int finger_num, int strain_thresh){
	hand->update(Hand::S_FINGERTIP_TORQUE,true);
	std::vector<int> strain = hand->getFingertipTorque();
	if(strain[finger_num] > strain_thresh){
		std::cout << strain[finger_num] << ">" << strain_thresh << std::endl;
		return true;
	}
	else{
		//std::cout << strain[finger_num] << ">" << strain_thresh << std::endl;
		return false;
	}
}
int get_strain_value(Hand* hand, int finger_num){
	hand->update(Hand::S_FINGERTIP_TORQUE,true);
	std::vector<int> strain = hand->getFingertipTorque();
	return strain[finger_num];
}
bool check_strain_contact(Hand* hand, int strain_thresh){
	return check_strain_contact(hand, 0, strain_thresh) 
		|| check_strain_contact(hand, 1, strain_thresh) 
		|| check_strain_contact(hand, 2, strain_thresh);
}
bool check_strain_contact(Hand* hand){
	return check_strain_contact(hand, 0, strain_base_val[0]) 
		|| check_strain_contact(hand, 1, strain_base_val[1]) 
		|| check_strain_contact(hand, 2, strain_base_val[2]);
}
void tare_tactile(Hand* hand){
	//std::cout << "check_tactile_contact!" << std::endl;
	hand->update(S_TACT_FULL, true);
	std::vector<TactilePuck*> tps;
	tps = hand->getTactilePucks();
	std::cout << "tare-value for tactile pad on: " << std::endl;
	for(unsigned int finger_num = 0; finger_num < tps.size(); finger_num++){
		v_type finger_tact = tps[finger_num]->getFullData();
		float max = -1;
		for(int i = 0; i < finger_tact.size(); i++){
			if(finger_tact[i] > max){
				max = finger_tact[i];
			}
		}
		std::cout << "    F" << finger_num+1 << ": " << max << std::endl;
		tact_base_val[finger_num] = max;
	}
}
void tare_strain(Hand* hand){
	hand->update(Hand::S_FINGERTIP_TORQUE,true);
	std::vector<int> strain = hand->getFingertipTorque();
	std::cout << "tare-value for strain gage: " << std::endl;
	for(unsigned int finger_num = 0; finger_num < strain.size(); finger_num++){
		strain_base_val[finger_num] = strain[finger_num];
		 std::cout << "    F" << finger_num+1 << ": " << strain[finger_num] << std::endl;
	}
}
// This function template will accept a math::Matrix with any number of rows,
// any number of columns, and any units. In other words: it will accept any
// barrett::units type.
template<int R, int C, typename Units>
bool parseDoubles(math::Matrix<R,C, Units>* dest, const std::string& str) {
	const char* cur = str.c_str();
	const char* next = cur;

	for (int i = 0; i < dest->size(); ++i) {
		(*dest)[i] = strtod(cur, (char**) &next);
		if (cur == next) {
			return false;
		} else {
			cur = next;
		}
	}

	// Make sure there are no extra numbers in the string.
	double ignore = strtod(cur, (char**) &next);
	(void)ignore;  // Prevent unused variable warnings

	if (cur != next) {
		return false;
	}

	return true;
}

// This function template will accept a math::Matrix with any number of rows,
// any number of columns, and any units. In other words: it will accept any
// barrett::units type.
template<int R, int C, typename Units>
std::string toString(math::Matrix<R,C, Units>* src) {
	std::string str;
	int i;
	for (i = 0; i < src->size(); ++i) {
		char buff[50];
		sprintf(buff, "%f",(*src)[i]);
		str.append(buff);
		if(i < src->size()-1)
			str.append(" ");
		else
			str.append("\n");
	}
	return str;
}
//first converts quaternion to hand joint position vector (same size = 4)
std::string toString(Eigen::Quaterniond* src) {
	Hand::jp_type temp = quaternion2hjp(src); 
	return toString(&temp);
}
//copy the value v to up to n (0 for all) element s of vector dest
//negative values for n indicate all except last n elements
template<int R, int C, typename Units>
void setVectorValues(math::Matrix<R,C, Units>* dest, float v, int n){
	if(n <= 0)
		n = dest->size()+n;
	for(int i = 0; i < n; i++){
		(*dest)[i] = v;
	}
}
//add the value v to up to n (0 for all) elements of vector dest
//negative values for n indicate all except last n elements
template<int R, int C, typename Units>
void addVectorValues(math::Matrix<R,C, Units>* dest, float v, int n){
	if(n <= 0)
		n = dest->size()+n;
	for(int i = 0; i < n; i++){
		(*dest)[i] += v;
	}
}
//add two vectors dest and v and store result in dest
template<int R, int C, typename Units>
void addVectorValues(math::Matrix<R,C, Units>* dest, math::Matrix<R,C, Units>* v){
	for(int i = 0; i < v->size(); i++){
		(*dest)[i] += (*v)[i];
	}
}
//multiply elements of two vectors dest and v and store result in dest
template<int R, int C, typename Units>
void multVectorValues(math::Matrix<R,C, Units>* dest, math::Matrix<R,C, Units>* v){
	for(int i = 0; i < v->size(); i++){
		(*dest)[i] *= (*v)[i];
	}
}
//obtain magnitude of each element-wise step from one vector to another
template<int R, int C, typename Units>
void getInterpolatingSteps(	math::Matrix<R,C, Units>* step, 
							math::Matrix<R,C, Units>* from, 
							math::Matrix<R,C, Units>* to,
							float num_steps){
	for(int i = 0; i < step->size(); i++){
		(*step)[i] = ((*to)[i] - (*from)[i]) / num_steps;
	}
}
//Close all fingers
void closeHand(Hand* hand){
	Hand::jv_type velocities;
	setVectorValues(&velocities, 3.0, -1);
	hand->velocityMove(velocities);
}
//Open all fingers
void openHand(Hand* hand){
	Hand::jv_type velocities;
	setVectorValues(&velocities, -3.0, 0);
	hand->velocityMove(velocities);
}
//Close all fingers until contacts detected
void graspObject(Hand* hand){
	if(backdrivesemastop){
		std::cout << "WARNING: Requires hand backdrivability to be set" << std::endl;
		return;
	}
	Hand::jv_type velocities;
	graspsemastop = false;
	bool done = false;
	while(!done && !graspsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(5));
		done = true;
		for(int i = 0; i < finger_contacts.size()-1; i++){
			//std::cout << finger_contacts[i] << std::endl;
			if(!finger_contacts[i]){
				velocities[i] = 3.0;
				done = false;
			}
		}
		hand->velocityMove(velocities);
	}
}
//open all fingers and reset finger contact flags
void ungraspObject(Hand* hand){
	graspsemastop = true;
	setVectorValues(&finger_contacts, 0, 0);
	openHand(hand);
}
void loadExpVariables(){
	std::string wamBottomStr;
	std::string wamTopStr;
	std::string wamBottomCStr;
	std::string wamTopCStr;
	std::string wamBottomOStr;
	std::string wamTopOStr;
	std::string handPregraspStr;
	std::string handGraspStr;
	std::string handUnGraspStr;
	std::string tact_base_valStr;
	std::string torque_epsilonStr;
	std::string joint_toleranceStr;
	std::string misc_parmsStr;
	
	//read parameters from file
	std::ifstream myfile ("in.txt");
	if (!myfile.is_open()){
		std::cout << "Unable to open file"; 
		exit(1);
	}
	std::getline (myfile,wamBottomStr);
	std::getline (myfile,wamTopStr);
	std::getline (myfile,wamBottomCStr);
	std::getline (myfile,wamTopCStr);
	std::getline (myfile,wamBottomOStr);
	std::getline (myfile,wamTopOStr);
	std::getline (myfile,handPregraspStr);
	std::getline (myfile,handGraspStr);
	std::getline (myfile,handUnGraspStr);
	std::getline (myfile,tact_base_valStr);
	std::getline (myfile,torque_epsilonStr);
	std::getline (myfile,joint_toleranceStr);
	std::getline (myfile,misc_parmsStr);
	
	//parse paramater vectors
	parseDoubles(&wamBottom, wamBottomStr);
	parseDoubles(&wamTop, wamTopStr);
	parseDoubles(&wamBottomC, wamBottomCStr);
	parseDoubles(&wamTopC, wamTopCStr);
	parseDoubles(&wamBottomO, wamBottomOStr); wamBottomQ = hjp2quaternion(&wamBottomO);
	parseDoubles(&wamTopO, wamTopOStr); wamTopQ = hjp2quaternion(&wamTopO);
	parseDoubles(&handPregrasp, handPregraspStr);
	parseDoubles(&handGrasp, handGraspStr);
	parseDoubles(&handUnGrasp, handUnGraspStr);
	parseDoubles(&tact_base_val, tact_base_valStr);
	parseDoubles(&torque_epsilon, torque_epsilonStr);
	parseDoubles(&joint_tolerance, joint_toleranceStr);
	parseDoubles(&misc_parms, misc_parmsStr);
}
void saveExpVariables(){	
	//read parameters from file
	std::ofstream myfile ("in.txt");
	if (!myfile.is_open()){
		std::cout << "Unable to open file" << std::endl; 
		exit(1);
	}

	//save paramater vectors
	myfile << toString(&wamBottom);
	myfile << toString(&wamTop);
	myfile << toString(&wamBottomC);
	myfile << toString(&wamTopC);
	wamBottomO = quaternion2hjp(&wamBottomQ);
	myfile << toString(&wamBottomO);
	wamTopO = quaternion2hjp(&wamTopQ);
	myfile << toString(&wamTopO);
	myfile << toString(&handPregrasp);
	myfile << toString(&handGrasp);
	myfile << toString(&handUnGrasp);
	myfile << toString(&tact_base_val);
	myfile << toString(&torque_epsilon);
	myfile << toString(&joint_tolerance);
	myfile << toString(&misc_parms);

#if 0
	std::cout << "saving wamBottom      as " << toString(&wamBottom) 	  << std::endl;
	std::cout << "saving wamTop         as " << toString(&wamTop) 		  << std::endl;
	std::cout << "saving wamBottomC     as " << toString(&wamBottomC) 	  << std::endl;
	std::cout << "saving wamTopC        as " << toString(&wamTopC) 		  << std::endl;
	std::cout << "saving wamBottomO     as " << toString(&wamBottomO) 	  << std::endl;
	std::cout << "saving wamTopO        as " << toString(&wamTopO) 		  << std::endl;
	std::cout << "saving handPregrasp   as " << toString(&handPregrasp)   << std::endl;
	std::cout << "saving handGrasp      as " << toString(&handGrasp) 	  << std::endl;
	std::cout << "saving handUnGrasp    as " << toString(&handUnGrasp) 	  << std::endl;
	std::cout << "saving tact_base_val  as " << toString(&tact_base_val)  << std::endl;
	std::cout << "saving torque_epsilon as " << toString(&torque_epsilon) << std::endl;
	std::cout << "saving joint_toleranceas " << toString(&joint_tolerance)<< std::endl;
#endif
}
/*ActionPhase0*/
template<size_t DOF>
void runActionPhaseExperiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottom, false, 3.0);
	std::cout << "start!" << std::endl;
	long timer = 0;
	while(prev_state < NUM_ACTION_PHASES && !expsemastop){
		timer++;
		int curr_state = prev_state;
		bool transition = false;
		//transition from prev_state according to sensorimotor event
		switch(prev_state){
			//first contact detected via palm tactile sensor
			case APPROACH:{
				transition = check_tactile_contact(hand, 3);
				break;
			}
			//contact with finger detected
			case PRELOAD:{
				transition = check_tactile_contact(hand, 2);
				break;
			}
			//load force peaked
			case LOADING:{
				fts->update(true);
				Hand::ct_type torques = fts->getTorque();
				if(ct.size() >= 100){ //more than torque_epsilon different than 1 second ago
					//std::cout << "prev_torque: " << ct[ct.size()-100][1] << std::endl;
					//std::cout << "torque: " << torques[1] << std::endl;
					//std::cout << "prev_torque-torque: " << ct[ct.size()-100][1]-torques[1] << std::endl;
					transition = 
						ct[ct.size()-100][1]-torques[1] > torque_epsilon[1] || 
						ct[ct.size()-100][1]-torques[1] < -1*torque_epsilon[1];
				}
				break;
			}
			//WAM reached wamTop
			case TRANSITIONAL:{
				//std::cout << "Wam done? " << wam.moveIsDone() << std::endl;
				transition = wam.moveIsDone();
				break;
			}
			//lift timer expired
			case STATIC:{
				transition = timer > 100;
				break;
			}
			//object contact with table (grip/load ratio event)
			case REPLACEMENT:{
				hand->update(S_ALL, true);
				Hand::ct_type torques = fts->getTorque();
				std::vector<int> istrain = hand->getFingertipTorque();
				std::vector<double> strain(4,-1);
				//standardize strain measurement and convert to N-m
				//std::cout << "strain (" << STRAIN2TORQUE_RATIO << "): ";
				for(size_t i = 0; i < istrain.size(); ++i){
					//std::cout << (double)istrain[i] << "->";
					strain[i] = ((double)istrain[i]) / STRAIN2TORQUE_RATIO;
					//std::cout << strain[i] << "->";
					strain[i] -= min_hstrain[i];
					//std::cout << strain[i] << ", ";
				}
				
				
				//std::cout << std::endl;
				//standardize torque measurement
				//std::cout << "torques: ";
				for(int i = 0; i < (int)torques.size(); i++){
					//std::cout << torques[i] << "->";
					torques[i] -= min_ct[i];
					//std::cout << torques[i] << ", ";
				}
				//std::cout << std::endl;
				
				//detect an interesting event in the grip ratio
				if(ct.size() >= 100 && hstrain.size() >= 100){
					double prev_strain = hstrain[hstrain.size()-100][2] 
						/ STRAIN2TORQUE_RATIO - min_hstrain[2];
					double prev_torque = ct[ct.size()-100][1] 
						- min_ct[1];
					//calculate grip ratio
					double grip_ratio;
					double prev_grip_ratio;
					prev_grip_ratio = prev_strain / prev_torque;
					grip_ratio = strain[2] / torques[1];
					//std::cout << "prev_grip_ratio: " << prev_grip_ratio << std::endl;
					//std::cout << "grip_ratio: " << grip_ratio << std::endl;
					
					//more than torque_epsilon different than previously
					transition = 
						prev_grip_ratio - grip_ratio > torque_epsilon[1] ||
						prev_grip_ratio - grip_ratio < -1*torque_epsilon[1];
				}
				break;
			}
			//zero lifting force
			case UNLOADING:{
				transition = true;
				break;
			}
			//done!
			default: {break;}
		}
		
		boost::this_thread::sleep(boost::posix_time::milliseconds(100)); 
		
		if(transition){
			++curr_state;
		}
		else //nothing to do
			continue;
		
		//action
		switch(curr_state){
		
			//stop approach & start finger close
			case PRELOAD:{
				std::cout << "Performing PRELOAD action-phase!" << std::endl;
				hand->velocityMove(handGrasp);
				break;
			}
			
			//slowly lift WAM to wamTop
			case LOADING:{
				std::cout << "Performing LOADING action-phase!" << std::endl;
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamTop, false, 3.0);
				break;
			}
			
			//none
			case TRANSITIONAL:{
				std::cout << "Performing TRANSITIONAL action-phase!" << std::endl;
				break;
			}
			
			//start timer
			case STATIC:{
				std::cout << "Performing STATIC action-phase!" << std::endl;
				timer = 0;
				break;
			}
			
			//slowly lower WAM to wamBottom
			case REPLACEMENT:{
				std::cout << "Performing REPLACEMENT action-phase!" << std::endl;
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottom, false, 3.0);
				break;
			}
			
			//slowly open finger
			case UNLOADING:{
				std::cout << "Performing UNLOADING action-phase!" << std::endl;
				hand->velocityMove(handUnGrasp);
				break;
			}
			
			//done!
			default: {
				std::cout << "Action COMPLETE!" << std::endl;
				break;
			}
		}
		
		//hand->velocityMove(handPregrasp);	
		//while(!hand->doneMoving(true));		
		prev_state = curr_state;
	}
}
/*ActiveSensing1*/
template<size_t DOF>
void runActiveSensingExperiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottom, false, 3.0, 3.0);
	std::cout << "start!" << std::endl;
	int curr_state = 0;
	//num_runs = 2;
	 
	while(curr_state < num_runs*2 && !expsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		
		if(wam.moveIsDone()){++curr_state;}else{continue;}
		
		if(curr_state % 2){ //odd
			if(num_runs % 2){	//odd
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamTop, false, 3.0, 0.5);
			}
			else{	//even
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamTopC, false, 3.0, 0.5);
			}
		}
		else{ //even
			if(num_runs % 2){	//odd
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottom, false, 3.0, 0.5);
			}
			else{	//even
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottomC, false, 3.0, 0.5);
			}
		}
	}
}
/*WAMVelocity2*/
template<size_t DOF>
void runWAMVelocityExperiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottom, false, 3.0);
	std::cout << "start!" << std::endl;
	int curr_state = 0;
	//int num_runs = 30;
	float velocity = 0.25;
	float accel = 0.25;
	
	printf("/%02d   vel   accel\n",num_runs);
	fflush(stdout);
	 
	while(curr_state < num_runs && !expsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		
		if(wam.moveIsDone()){++curr_state;}else{continue;}
		
		printf(" %02d  %-1.3f  %-1.3f\n",curr_state,velocity,accel);
		fflush(stdout);
		
		if(curr_state % 2){ //odd
			(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamTop, false, velocity, accel);
		}
		else{ //even
			(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottom, false, velocity, accel);
		}
		velocity+= 0.25;
		accel	+= 0.25;
		
		//hand->velocityMove(handPregrasp);	
		//while(!hand->doneMoving(true));
		
	}
}
/*WAMJointPos3*/
template<size_t DOF>
void runWAMJointPosExperiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottom, false, 3.0);
	std::cout << "start!" << std::endl;
	int curr_state = 0;
	//int num_runs = 5;
	systems::Wam<DIMENSION>::jp_type steps;
	getInterpolatingSteps(&steps, &wamBottom, &wamTop, num_runs);
	
	printf("/%02d   pos\n",num_runs);
	fflush(stdout);
	 
	while(curr_state < num_runs && !expsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		
		if(wam.moveIsDone()){++curr_state;}else{continue;}
		
		addVectorValues(&wamBottom,&steps);
		printf(" %02d ",curr_state);
		fflush(stdout);
		std::cout << toString(&wamBottom) << std::endl;
		(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottom, false, 3.0);
	}
}
/*WAMCartesianPos4*/
template<size_t DOF>
void runWAMCartesianPosExperiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	//Eigen::Quaterniond wamBottomQ = hjp2quaternion(&wamBottomO);
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottomC, false, 3.0);
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottomQ, true, 0.3, 0.25);
	
	//causes wam to hold its tool at the desiredOrientation
	systems::ExposedOutput<Eigen::Quaterniond> toSetpoint(wamBottomQ);
	{
		//std::cout << "maintaining " << toString(&wamBottomO) << std::endl;
		BARRETT_SCOPED_LOCK(pm->getExecutionManager()->getMutex());

		wam.idle();
		forceConnect(toSetpoint.output, wam.toController.referenceInput);
		forceConnect(wam.tt2jt.output, wam.input);
	}
	std::cout << "start!" << std::endl;
	int curr_state = 0;
	//int num_runs = 5;
	systems::Wam<DIMENSION>::cp_type steps;
	getInterpolatingSteps(&steps, &wamBottomC, &wamTopC, num_runs);
	
	printf("/%02d   pos\n",num_runs);
	fflush(stdout);
	 
	while(curr_state < num_runs && !expsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		
		if(wam.moveIsDone()){++curr_state;}else{continue;}
		
		addVectorValues(&wamBottomC,&steps);
		printf(" %02d ",curr_state);
		fflush(stdout);
		std::cout << toString(&wamBottomC) << std::endl;
		(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottomC, false, 3.0);
	}
}
/*WAMJointTorque5*/
template<size_t DOF>
void runWAMJointTorqueExperiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottom, false, 3.0);
	std::cout << "start!" << std::endl;
	int curr_state = 0;
	//int num_runs = 15;
	float step = 0.2;	//radians
	float position = 0.0;	
	Hand::jp_type joint_torques;
	
	printf("/%02d   pos\n",num_runs);
	fflush(stdout);
	 
	while(curr_state < num_runs && !expsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		
		if(wam.moveIsDone()){++curr_state;}else{continue;}
		
		printf(" %02d  %-1.3f\n",curr_state,position);
		fflush(stdout);
		
		if(curr_state % 2){ //odd
			setVectorValues(&joint_torques,position, -1);
		}
		else{ //even
			setVectorValues(&joint_torques,FINGER_JOINT_LIMIT-position, -1);
		}
		hand->setPositionCommand(joint_torques);
		position+= step;
	}
}
/*BHVelocity6*/
template<size_t DOF>
void runBHVelocityExperiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottom, false, 3.0);
	std::cout << "start!" << std::endl;
	int curr_state = 0;
	//int num_runs = 15;
	float velocity = 0.25;	//radians per second
	Hand::jv_type finger_velocities;
	
	printf("/%02d   vel   finger_velocities\n",num_runs);
	fflush(stdout);
	 
	while(curr_state < num_runs && !expsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		
		if(hand->doneMoving(true)&&wam.moveIsDone()){++curr_state;}else{continue;}
		
		if(curr_state % 2){ //odd
			setVectorValues(&finger_velocities,velocity, -1);
		}
		else{ //even
			setVectorValues(&finger_velocities,-1*velocity, -1);
		}
		
		printf(" %02d  %-1.3f: ",curr_state,velocity);
		std::cout << toString(&finger_velocities) << std::endl;
		fflush(stdout);
		
		hand->velocityMove(finger_velocities);
		velocity+= 0.25;
	}
}
/*BHPosition7*/
template<size_t DOF>
void runBHPositionExperiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottom, false, 3.0);
	std::cout << "start!" << std::endl;
	int curr_state = 0;
	//int num_runs = 15;
	float step = 0.2;//floor(FINGER_JOINT_LIMIT/num_runs*10.0)/10.0;	//radians
	float position = 0.0;	
	Hand::jp_type finger_positions;
	
	printf("/%02d   pos   finger_positions\n",num_runs);
	fflush(stdout);
	 
	while(curr_state < num_runs && !expsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		
		if(hand->doneMoving(true)&&wam.moveIsDone()){++curr_state;}else{continue;}
		/*
		if(curr_state % 2){ //odd
			setVectorValues(&finger_positions,position, -1);
		}
		else{ //even
			setVectorValues(&finger_positions,FINGER_JOINT_LIMIT-position, -1);
		}
		*/
		setVectorValues(&finger_positions,position, -1);
		
		printf(" %02d  %-1.3f: ",curr_state,position);
		std::cout << toString(&finger_positions) << std::endl;
		fflush(stdout);
		
		hand->setPositionMode();
		hand->setPositionCommand(finger_positions);
		position+= step;
	}
}
/*BHTorque8*/
template<size_t DOF>
void runBHTorqueExperiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottom, false, 3.0);
	std::cout << "start!" << std::endl;
	int curr_state = 0;
	//int num_runs = 1;
	float step = 0.1;	//N-m
	float torque = 0.0;	
	Hand::jt_type finger_torques;
	
	printf("/%02d   torque  finger_torques\n",num_runs);
	fflush(stdout);
	
	hand->setTorqueMode();
	 
	while(curr_state < num_runs && !expsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		
		if(hand->doneMoving(true)&&wam.moveIsDone()){++curr_state;}else{continue;}

		setVectorValues(&finger_torques,torque,-1);
		
		printf(" %02d  %-1.3f: ",curr_state,torque);
		std::cout << toString(&finger_torques) << std::endl;
		fflush(stdout);
		hand->setTorqueMode();
		hand->setTorqueCommand(finger_torques);
		torque+= step;	
	}
}
/*BHTrapezoidal9*/
template<size_t DOF>
void runBHTrapezoidalExperiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottom, false, 3.0);
	std::cout << "start!" << std::endl;
	int curr_state = 0;
	float step = FINGER_JOINT_LIMIT/num_runs;	//radians
	float position = 0.0;	
	Hand::jp_type finger_positions;
	
	printf("/%02d   pos   finger_positions\n",num_runs);
	fflush(stdout);
	 
	while(curr_state < num_runs && !expsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		
		if(hand->doneMoving(true)&&wam.moveIsDone()){++curr_state;}else{continue;}
		
		if(curr_state % 2){ //odd
			setVectorValues(&finger_positions,position, -1);
		}
		else{ //even
			setVectorValues(&finger_positions,FINGER_JOINT_LIMIT-position, -1);
		}
		
		printf(" %02d  %-1.3f: ",curr_state,position);
		std::cout << toString(&finger_positions) << std::endl;
		fflush(stdout);
		
		hand->trapezoidalMove(finger_positions, false);
		position+= step;
	}
}
/*SimpleShapes10*/
template<size_t DOF>
void runSimpleShapesExperiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottom, false, 3.0, 5.0);
	std::cout << "start!" << std::endl;
	int curr_state = 0;
	//num_runs = 3;
	Hand::jp_type digits;
	setVectorValues(&digits,0,0);	//init
	setVectorValues(&digits,1,-1);	//activate all except spread
	 
	while(curr_state < num_runs*4 && !expsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		
		if(hand->doneMoving(true)&&wam.moveIsDone()){++curr_state;}else{continue;}
		
		switch(curr_state%4){
			case 1 : {
				closeHand(hand);
				break;
			}
			case 2 : {
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamTop, false, 3.0, 5.0);
				break;
			}
			case 3 : {
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottom, false, 3.0, 5.0);
				break;
			}
			case 0 : {
				openHand(hand);
				break;
			}
		}
	}
}
/*ActiveProbing11*/
/*
	two phases of motion: (1) moving up and (0) moving down
	switches phases upon completing its up-movement and down-movement
	if contact occurs during phase 0, switch to phase 1 immediately
*/
template<size_t DOF>
void runActiveProbingExperiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottom, false, 0.3, 0.25);
	std::cout << "start!" << std::endl;
	int curr_state = 0;
	int num_states = 2;
	Hand::jp_type digits;
	setVectorValues(&digits,0,0);	//init
	setVectorValues(&digits,FINGER_JOINT_LIMIT/2,-1);	//activate all except spread
	
	bool strainContact = false;
	bool tactileContact = false;
	bool handAboveTable = false;
	bool objectDetected = false;
	
	//wamBottomQ = hjp2quaternion(&wamBottomO);
	//wamTopQ = hjp2quaternion(&wamTopO);
	
	float minX, maxX, minY, maxY, minZ, maxZ;
	
	minX = wamBottomC[0];
	minY = wamBottomC[1];
	minZ = wamBottomC[2];
	maxX = wamTopC[0];
	maxY = wamTopC[1];
	maxZ = 0.25;
	
	wamTopC[0] = (maxX+minX)/2;
	wamTopC[1] = (maxY+minY)/2;
	wamTopC[2] = maxZ;
	
	wamBottomC_ip[2] = wamBottomC[2] + 0.1;
	
	//hand->trapezoidalMove(digits, false);
	 
	while(curr_state < num_runs*num_states && !expsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		
		//record phase of motion
		int phase = curr_state % num_states;
		//std::cout << phase << std::endl;
		
		//check tactile reading and stop if object collision above the table
		strainContact = check_strain_contact(hand, ZERO_STRAIN_THRESHOLD);
		tactileContact = check_tactile_contact(hand);
		bool contact = strainContact || tactileContact;
		handAboveTable = true;//wam.getToolPosition()[2] > (minZ + 0.02);	//2 cm off the table
		
		if(contact && phase == 0 && !objectDetected){
			//boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
			std::cout << "object detected!" << std::endl;
			objectDetected = true;
			++curr_state; //want to be in phase 0 in next timestep
		}
		
		//move forward when wam and hand are finished (or preempted by object detection)
		if((hand->doneMoving(true)&&wam.moveIsDone())||objectDetected){
			++curr_state;
			std::cout << "switching to phase " << curr_state%2 << std::endl;
		}
		else{
			continue;
		}
		
		//std::cout << "min/maxX: " << minX << "/" << maxX << std::endl;
		//std::cout << "min/maxY: " << minY << "/" << maxY << std::endl;
		
		switch(phase){
			case 0 : {
				//std::cout << "moving to wtc @ " << toString(&wamTopC);
				std::cout << "ActiveProbing take " << curr_state/num_states << std::endl;
				std::cout << "Moving to " << toString(&wamTopC) << std::endl;
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamTopC, false, 0.3, 0.25);
				break;
			}
			case 1 : {
				objectDetected = false;
				wamBottomC[0] = randomFloat(minX, maxX);
				wamBottomC[1] = randomFloat(minY, maxY);
				wamBottomC_ip[0] = wamBottomC[0];
				wamBottomC_ip[1] = wamBottomC[1];
				//std::cout << "wbc0: " << wamBottomC[0] << std::endl;
				//std::cout << "wbc1: " << wamBottomC[1] << std::endl;
				//std::cout << "moving to wbc @ " << toString(&wamBottomC);
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottomC_ip, true, 0.3, 0.25);
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottomQ, true, 0.3, 0.25);
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottomC, false, 0.3, 0.25);
				break;
			}
		}
	}
}
/*ActiveProbing12*/
/*
	see above + keep tool orientation stable through cartesian move
*/
template<size_t DOF>
void runActiveProbing2Experiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottom, false, 0.3, 0.25);
	std::cout << "start!" << std::endl;
	int curr_state = 0;
	int num_states = 2;
	Hand::jp_type digits;
	setVectorValues(&digits,0,0);	//init
	setVectorValues(&digits,FINGER_JOINT_LIMIT/2,-1);	//activate all except spread
	
	bool strainContact = false;
	bool tactileContact = false;
	bool handAboveTable = false;
	bool objectDetected = false;
	
	//wamBottomQ = hjp2quaternion(&wamBottomO);
	//wamTopQ = hjp2quaternion(&wamTopO);
	
	float minX, maxX, minY, maxY, minZ, maxZ;
	
	minX = wamBottomC[0];
	minY = wamBottomC[1];
	minZ = wamBottomC[2];
	maxX = wamTopC[0];
	maxY = wamTopC[1];
	maxZ = 0.25;
	
	wamTopC[0] = (maxX+minX)/2;
	wamTopC[1] = (maxY+minY)/2;
	wamTopC[2] = maxZ;
	
	wamBottomC_ip[2] = wamBottomC[2] + 0.1;
	
	//hand->trapezoidalMove(digits, false);
	
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamTopC, true, 0.3, 0.25);
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottomQ, true, 0.3, 0.25);
	
	//causes wam to hold its tool at the desiredOrientation
	systems::ExposedOutput<Eigen::Quaterniond> toSetpoint(wamBottomQ);
	//systems::disconnect(toSetpoint.output);
	//systems::disconnect(wam.tt2jt.output);
	
	{
		std::cout << "maintaining " << toString(&wamBottomQ) << std::endl;
		BARRETT_SCOPED_LOCK(pm->getExecutionManager()->getMutex());

		wam.idle();
		forceConnect(toSetpoint.output, wam.toController.referenceInput);
		forceConnect(wam.tt2jt.output, wam.input);
		/*
		input.output = &output;
		output.inputs.push_back(input);

		input.pushExecutionManager();*/
	}
	 
	while(curr_state < num_runs*num_states && !expsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		
		//record phase of motion
		int phase = curr_state % num_states;
		//std::cout << phase << std::endl;
		
		//check tactile reading and stop if object collision above the table
		strainContact = check_strain_contact(hand);
		tactileContact = check_tactile_contact(hand);
		bool contact = strainContact || tactileContact;
		handAboveTable = true;//wam.getToolPosition()[2] > (minZ + 0.02);	//2 cm off the table
		
		if(contact && phase == 0 && !objectDetected){
			//boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
			std::cout << "object detected!" << std::endl;
			objectDetected = true;
			++curr_state; //want to be in phase 0 in next timestep
		}
		
		//move forward when wam and hand are finished (or preempted by object detection)
		if((hand->doneMoving(true)&&wam.moveIsDone())||objectDetected){
			++curr_state;
		}
		else{
			continue;
		}
		
		//std::cout << "min/maxX: " << minX << "/" << maxX << std::endl;
		//std::cout << "min/maxY: " << minY << "/" << maxY << std::endl;
		
		switch(phase){
			case 0 : {
				//std::cout << "moving to wtc @ " << toString(&wamTopC);
				if(!objectDetected && move_and_lift){
					++curr_state; //try moving randomly again until we hit something
				}
				else{
					std::cout << "ActiveProbing take " << curr_state/num_states << std::endl;
					std::cout << "Moving to " << toString(&wamTopC) << std::endl;
					(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamTopC, false, 0.3, 0.25);
				}
				break;
			}
			case 1 : {
				objectDetected = false;
				wamBottomC[0] = randomFloat(minX, maxX);
				wamBottomC[1] = randomFloat(minY, maxY);
				wamBottomC_ip[0] = wamBottomC[0];
				wamBottomC_ip[1] = wamBottomC[1];
				//std::cout << "wbc0: " << wamBottomC[0] << std::endl;
				//std::cout << "wbc1: " << wamBottomC[1] << std::endl;
				//std::cout << "moving to wbc @ " << toString(&wamBottomC);
				//(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottomC_ip, true, 0.3, 0.25);
				//(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottomQ, true, 0.3, 0.25);
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottomC, false, 0.3, 0.25);
				break;
			}
		}
	}
}
/*TactileTestExperiment*/
/*
	test tactile sensor by moving the finger in a repeatable raster motion
	spherical objects (marbles/ball bearings) are then to be placed beneath
	the tactile surface. the purpose is to determine if edges can be reliably
	detected by the sensors or if significant aliasing occurs
*/
/*WAMCartesianPos4*/
template<size_t DOF>
void runCartesianRasterExperiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	//Eigen::Quaterniond wamBottomQ = hjp2quaternion(&wamBottomO);
	systems::Wam<DIMENSION>::cp_type curr_pos;
	
	//start experiment: move WAM to first goal
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottomC, true, 3.0);
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottomQ, true, 0.3, 0.25);
	
	//causes wam to hold its tool at the desiredOrientation
	systems::ExposedOutput<Eigen::Quaterniond> toSetpoint(wamBottomQ);
	{
		//std::cout << "maintaining " << toString(&wamBottomO) << std::endl;
		BARRETT_SCOPED_LOCK(pm->getExecutionManager()->getMutex());

		wam.idle();
		forceConnect(toSetpoint.output, wam.toController.referenceInput);
		forceConnect(wam.tt2jt.output, wam.input);
	}
	std::cout << "start!" << std::endl;
	int curr_state = -1;
	//int num_runs = 5;

	//printf("/%02d   x_pos\n",num_runs);
	//fflush(stdout);
	std::cout << "x_pos" << std::endl;
	
	float min_x = wamBottomC[0];
	float min_y = wamBottomC[1];
	float max_x = wamTopC[0];
	float max_y = wamTopC[1];
	
	float step = 0.003;	//3mm steps
	curr_pos[0] = min_x;
	curr_pos[1] = min_y;
	curr_pos[2] = wamBottomC[2];	//want tool to remain in the same horizontal plane
	
	float x, y;		//temp coordinates of locations on tactile finger pad edges
	float x0, y0;	//coordinates of centre point of tactile finger pad (point of rotation)
	x = min_x;
	x0 = max_x - min_x;
	y0 = max_y - min_y;
	float theta = 0.523;	//30 degree rotation
	 
	while((curr_pos[0] < max_x || curr_pos[1] < max_y) && !expsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		
		if(wam.moveIsDone()){++curr_state;}else{continue;}
		
		switch(expshape){
			case CIRCLE:{
				if(curr_state%2 == 0){	//even
					//swap y values
					if(curr_pos[1] == max_y)
						curr_pos[1] = min_y;
					else
						curr_pos[1] = max_y;
				}
				else{	//odd
					curr_pos[0] += step;
				}
				break;
			}
			case SQUARE:{
				if(curr_state%2 == 0){	//even
					//swap x values
					if(curr_pos[0] == max_x)
						curr_pos[0] = min_x;
					else
						curr_pos[0] = max_x;
				}
				else{	//odd
					curr_pos[1] += step;
				}
				break;
			}
			case TRIANGLE:{
				if(curr_state%4 == 0){	//first of 4 steps
					y = min_y;
				}
				if(curr_state%4 == 2){	//third of 4 steps
					y = max_y;
				}
				else{	//second or fourth of 4 steps
					x += step;
				}
				//rotate about tactile pad centre point by theta radians
				curr_pos[0] = x0+(x-x0)*cos(theta)+(y-y0)*sin(theta);
				curr_pos[1] = y0-(x-x0)*sin(theta)+(y-y0)*cos(theta);
				break;
			}
		}
		
		//printf(" %02d ",curr_x);
		//fflush(stdout);
		std::cout << toString(&curr_pos) << std::endl;
		(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(curr_pos, false, 3.0);
	}
}
void runExperiment(	Hand* hand, ForceTorqueSensor* fts, void* wamin, ProductManager* pm, 
					enum EXPERIMENT_KEYS expnum){
	systems::Wam<DIMENSION>* wam = (systems::Wam<DIMENSION>*)wamin;
	
	datasemastop = false;
	boost::thread* dataCollectionThread = NULL;
	if(collectData){
		dataCollectionThread = new boost::thread(dataCollect, hand, fts, wam, pm, expnum, expshape);
	}
	
	std::cout << "Running " << experiment_keys[int(expnum)] << " Experiment..." << std::endl;
	switch(expnum){
		case ACTIONPHASE:{
			runActionPhaseExperiment(*wam, hand, fts, pm);
			break;
		}
		case ACTIVESENSING:{
			runActiveSensingExperiment(*wam, hand, fts, pm);
			break;
		}
		case WAMVELOCITY:{
			runWAMVelocityExperiment(*wam, hand, fts, pm);
			break;
		}
		case WAMJOINTPOS:{
			runWAMJointPosExperiment(*wam, hand, fts, pm);
			break;
		}
		case WAMCARTESIANPOS:{
			runWAMCartesianPosExperiment(*wam, hand, fts, pm);
			break;
		}
		case WAMJOINTTORQUE:{
			runWAMJointTorqueExperiment(*wam, hand, fts, pm);
			break;
		}
		case BHVELOCITY:{
			runBHVelocityExperiment(*wam, hand, fts, pm);
			break;
		}
		case BHPOSITION:{
			runBHPositionExperiment(*wam, hand, fts, pm);
			break;
		}
		case BHTORQUE:{
			runBHTorqueExperiment(*wam, hand, fts, pm);
			break;
		}
		case BHTRAPEZOIDAL:{
			runBHTrapezoidalExperiment(*wam, hand, fts, pm);
			break;
		}
		case SIMPLESHAPES:{
			runSimpleShapesExperiment(*wam, hand, fts, pm);
			break;
		}
		case ACTIVEPROBING:{
			runActiveProbing2Experiment(*wam, hand, fts, pm);
			break;
		}
		case CARTESIANRASTER:{
			runCartesianRasterExperiment(*wam, hand, fts, pm);
			break;
		}
		default:{ 
		}
	}
	
	std::cout << "Experiment " << experiment_keys[int(expnum)];
	
	if(!expsemastop){
		std::cout << " Completed Successfully!" << std::endl;
		datasemastop = true;
		if(collectData)
			dataCollectionThread->join();
		std::cout << "Press [Enter] to continue." << std::endl;
	}
	else{
		std::cout << " Was Interrupted!" << std::endl;
		datasemastop = true;
		if(collectData)
			dataCollectionThread->join();
	}
	
}
template<size_t DOF>
void output_log(barrett::ProductManager& pm, void* loggerin){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	/*
	typedef boost::tuple<double, jp_type, jv_type, jt_type, cp_type, Eigen::Quaterniond> tuple_type;
	
	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);


	//logger.closeLog();
	((systems::PeriodicDataLogger<tuple_type>*)loggerin)->closeLog();
	printf("Logging stopped.\n");

	log::Reader<tuple_type> lr(tmpFile);
	lr.exportCSV(outFile);
	printf("Output written to %s.\n", outFile);
	std::remove(tmpFile);
	exit(0);
	*/
	/*
	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	
	typedef boost::tuple<double, jp_type, jv_type, jt_type, cp_type, Eigen::Quaterniond> tuple_type;

	((systems::PeriodicDataLogger<tuple_type>*)loggerin)->closeLog();
	printf("Logging stopped.\n");
	
	log::Reader<tuple_type> lr(tmpFile);
	lr.exportCSV(outFile);
	printf("Output written to %s.\n", outFile);
	std::remove(tmpFile);
	exit(0);
	*/
}

template<size_t DOF, int R, int C, typename Units>
void moveToStr(systems::Wam<DOF>& wam, math::Matrix<R,C, Units>* dest,
		const std::string& description, const std::string& str)
{
	if (parseDoubles(dest, str)) {
		std::cout << "Moving to " << description << ": " << *dest << std::endl;
		wam.moveTo(*dest);
	} else {
		printf("ERROR: Please enter exactly %d numbers separated by "
				"whitespace.\n", dest->size());
	}
}
void moveToStr(Hand* hand, Hand::jp_type* dest,
		const std::string& description, const std::string& str)
{
	if (parseDoubles(dest, str)) {
		std::cout << "Moving Hand to " << description << ": " << *dest << std::endl;
		hand->trapezoidalMove(*dest);
	} else {
		printf("ERROR: Please enter exactly 4 numbers separated by whitespace.\n");
	}
}

double velCommand(bool open, bool close, double speed = 1.25) {
	if (open  &&  !close) {
		return -speed;
	} else if (close  &&  !open) {
		return speed;
	} else {
		return 0.0;
	}
}

void handCommand(Hand* hand, unsigned char data){
	Hand::jv_type hjv;
	hjv[0] = velCommand(data & (1<<2), data & (1<<3));  // Middle
	hjv[1] = velCommand(data & (1<<0), data & (1<<1));  // Pointer
	hjv[2] = velCommand(data & (1<<4), data & (1<<5));  // Thumb
	hjv[3] = velCommand(data & (1<<6), data & (1<<7));  // Rocker
	hand->velocityMove(hjv);
	std::cout << "Velocity: " << hjv << std::endl;
}


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	const jp_type SYNC_POS(0.0);  // the position each WAM should move to before linking
	
	// These vectors are fixed sized, stack allocated, and zero-initialized.
	Hand::jp_type hjp; //hjp is a 4x1 column vector of hand joint positions
	jp_type jp;  // jp is a DOFx1 column vector of joint positions
	cp_type cp;  // cp is a 3x1 vector representing a Cartesian position
	
	setVectorValues(&finger_contacts, 0, 0);

	//MasterMaster<DOF> mm(pm.getExecutionManager(), argv[1]);
	//systems::connect(wam.jpOutput, mm.input);


	wam.gravityCompensate();

	Hand* hand = NULL;
	std::vector<TactilePuck*> tps;

	std::vector<std::string> autoCmds;
	
	std::string line;
	v_type gainTmp;
	
	// Is an FTS attached?
	ForceTorqueSensor* fts = NULL;
	if (pm.foundForceTorqueSensor()) {
		fts = pm.getForceTorqueSensor();
		fts->tare();
	}
	// Is a Hand attached?

	if (pm.foundHand()) {
		hand = pm.getHand();
		if (argc == 2) {  // auto init
			printf(">>> Press [Enter] to initialize Hand. (Make sure it has room!)");
			waitForEnter();
			hand->initialize();
		}
	}
	
	
	//data logging
	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}
	
	/*
	systems::Ramp time(pm.getExecutionManager(), 1.0);

	systems::TupleGrouper<double, jp_type, jv_type, jt_type, cp_type, Eigen::Quaterniond> tg;
	connect(time.output, tg.template getInput<0>());
	connect(wam.jpOutput, tg.template getInput<1>());
	connect(wam.jvOutput, tg.template getInput<2>());
	connect(wam.jtSum.output, tg.template getInput<3>());
	connect(wam.toolPosition.output, tg.template getInput<4>());
	connect(wam.toolOrientation.output, tg.template getInput<5>());

	typedef boost::tuple<double, jp_type, jv_type, jt_type, cp_type, Eigen::Quaterniond> tuple_type;
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(
			pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);

	time.start();
	connect(tg.output, logger.input);
	printf("Logging started.\n");
	*/
	/*
	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}

	systems::Ramp time(pm.getExecutionManager(), 1.0);

	systems::TupleGrouper<double, jp_type, jv_type, jt_type, cp_type, Eigen::Quaterniond> tg;
	connect(time.output, tg.template getInput<0>());
	connect(wam.jpOutput, tg.template getInput<1>());
	connect(wam.jvOutput, tg.template getInput<2>());
	connect(wam.jtSum.output, tg.template getInput<3>());
	connect(wam.toolPosition.output, tg.template getInput<4>());
	connect(wam.toolOrientation.output, tg.template getInput<5>());

	typedef boost::tuple<double, jp_type, jv_type, jt_type, cp_type, Eigen::Quaterniond> tuple_type;
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(
			pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);
	time.start();
	connect(tg.output, logger.input);
	printf("Logging started.\n");
	bool collectData = false;
	if(collectData){		
		dataCollect(hand, fts, wam, pm, (void*)&logger);
		//dataCollectionThread = new boost::thread(dataCollect, hand, fts, &wam, &pm);
	}
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	
	typedef boost::tuple<double, jp_type, jv_type, jt_type, cp_type, Eigen::Quaterniond> tuple_type;

	((systems::PeriodicDataLogger<tuple_type>*)loggerin)->closeLog();
	printf("Logging stopped.\n");
	
	log::Reader<tuple_type> lr(tmpFile);
	lr.exportCSV(outFile);
	printf("Output written to %s.\n", outFile);
	std::remove(tmpFile);
	exit(0);
	*/
	
	while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
		if (autoCmds.empty()) {
			printf(">>> ");
			std::getline(std::cin, line);
		} else {
			line = autoCmds.back();
			autoCmds.pop_back();
		}
		switch (line[0]) {
		/*
		case 'l':
			if (mm.isLinked()) {
				mm.unlink();
			} else {
				wam.moveTo(SYNC_POS);

				printf("Press [Enter] to link with the other WAM.");
				waitForEnter();
				mm.tryLink();
				wam.trackReferenceSignal(mm.output);

				usleep(100000);  // wait an execution cycle or two
				if (mm.isLinked()) {
					printf("Linked with remote WAM.\n");
				} else {
					printf("WARNING: Linking was unsuccessful.\n");
				}
			}

			break;
			*/
		case 's':
		{
			/*
			//start a thread that allows user to tear down the ncurses display upon typing <Enter>
			showsensorsemastop = false;
			boost::thread* stopThread;
			stopThread = new boost::thread(stop_thread, &showsensorsemastop);
			
			
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
			line++;
			
			if (fts != NULL) {
				mvprintw(line++,0, "F/T Sensor");
				mvprintw(line++,0, "     Force (N): ");
				getyx(stdscr, ftsY, ftsX);
				mvprintw(line++,0, "  Torque (N*m): ");
				line++;
			}

			if (hand != NULL) {
				mvprintw(line++,0, "Hand");
				mvprintw(line++,0, "  Inner Position (rad): ");
				getyx(stdscr, handY, handX);
				mvprintw(line++,0, "  Outer Position (rad): ");
				mvprintw(line++,0, "  Strain-gauge sensors: ");
				if ( !hand->hasFingertipTorqueSensors() ) {
					printw(" n/a");
				}
				mvprintw(line++,0, "       Tactile sensors: ");
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
			cf_type cf;
			ct_type ct;
			Hand::jp_type hjp;
			
			// Fall out of the loop once the user Shift-idles
			while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
				// WAM
				//jp = math::saturate(wam.getJointPositions(), 9.9999);
				//jv = math::saturate(wam.getJointVelocities(), 9.9999);
				//jt = math::saturate(wam.getJointTorques(), 99.999);
				line = wamY;

				// math::saturate() prevents the absolute value of the joint positions
				// from exceeding 9.9999. This puts an upper limit on the length of the
				// string that gets printed to the screen below. We do this to make sure
				// that the string will fit properly on the screen.
				jp = math::saturate(wam.getJointPositions(), 9.9999);
				mvprintw(line++,wamX, "[%7.4f", jp[0]);
				for (size_t i = 1; i < DOF; ++i) {
					printw(", %7.4f", jp[i]);
				}
				printw("]");

				jv = math::saturate(wam.getJointVelocities(), 9.9999);
				mvprintw(line++,wamX, "[%7.4f", jv[0]);
				for (size_t i = 1; i < DOF; ++i) {
					printw(", %7.4f", jv[i]);
				}
				printw("]");

				jt = math::saturate(wam.getJointTorques(), 99.999);
				mvprintw(line++,wamX, "[%7.3f", jt[0]);
				for (size_t i = 1; i < DOF; ++i) {
					printw(", %7.3f", jt[i]);
				}
				printw("]");


				// FTS
				if (fts != NULL) {
					line = ftsY;

					fts->update();
					cf = math::saturate(fts->getForce(), 99.999);
					mvprintw(line++,ftsX, "[%7.3f, %7.3f, %7.3f]", cf[0], cf[1], cf[2]);
					ct = math::saturate(fts->getTorque(), 9.9999);
					mvprintw(line++,ftsX, "[%7.4f, %7.4f, %7.4f]", ct[0], ct[1], ct[2]);
				}

				// Hand
				if (hand != NULL) {
					line = handY;

					hand->updatePosition();
					hjp = math::saturate(hand->getInnerLinkPosition(), 9.9999);
					mvprintw(line++,handX, "[%7.4f, %7.4f, %7.4f, %7.4f]",
							hjp[0], hjp[1], hjp[2], hjp[3]);
					hjp = math::saturate(hand->getOuterLinkPosition(), 9.9999);
					mvprintw(line++,handX, "[%7.4f, %7.4f, %7.4f, %7.4f]",
							hjp[0], hjp[1], hjp[2], hjp[3]);

					if (hand->hasFingertipTorqueSensors()) {
						hand->updateStrain();
						mvprintw(line,handX, "[%4d, %4d, %4d, %4d]",
								hand->getFingertipTorque()[0], hand->getFingertipTorque()[1],
								hand->getFingertipTorque()[2], hand->getFingertipTorque()[3]);
					}

					line += 2;
					if (hand->hasTactSensors()) {
						hand->updateTactFull();

						for (size_t i = 0; i < tps.size(); ++i) {
							graphPressures(stdscr, line, i * TACT_BOARD_STRIDE,
									tps[i]->getFullData());
						}
					}
				}
				refresh();  // Ask ncurses to display the new text
				usleep(200000);  // Slow the loop rate down to roughly 5 Hz
			}*/
			break;
		}
		case 'j':
			moveToStr(wam, &jp, "joint positions", line.substr(1));
			break;
		case 'p':
			moveToStr(wam, &cp, "tool position", line.substr(1));
			break;
		case 'i':
			printf("WAM & Hand idled.\n");
			wam.idle();
			hand->idle();
			/*
			if(toSetpoint != NULL){
				systems::disconnect(toSetpoint->output);
				delete toSetpoint;
				toSetpoint = NULL;
			}*/
			systems::disconnect(wam.tt2jt.output);
			systems::disconnect(wam.toController.referenceInput);
			systems::disconnect(wam.input);
			break;
		case 'h':
			std::cout << "Moving to home position: "
					<< wam.getHomePosition() << std::endl;
			wam.moveHome();
			break;
		case 'g':{
			boost::thread* graspObjectThread;
			graspObjectThread = new boost::thread(graspObject, hand);
			break;
		}
		case 'u':{
			ungraspObject(hand);
			break;
		}
		case 'w':{
			moveToStr(hand, &hjp, "joint positions", line.substr(1));
			break;
		}
		case 'b':{
			if(backdrivesemastop){
				boost::thread* backDriveHandThread;
				backdrivesemastop = false;
				backDriveHandThread = new boost::thread(backDriveHand, hand, fts, &wam, &pm);
			}
			else{
				backdrivesemastop = true;
			}
			break;
		}
		case 'r':{
			std::string expnumstr = "";
			std::string expshapestr = "";
			std::string sub = "";
			int found_w = int(line.find(" "));
			int found_s = int(line.find("-s"));
			int found_n = int(line.find("-n"));
			int found_l = int(line.find("-l"));
			int found_t = int(line.find("-t"));
			//arg -s: shape of object to grasp
			if (found_s!=int(std::string::npos)){
				//find next whitespace or newline
				int found_tmp = int(line.find(" ",found_s+3));
				if(found_tmp==int(std::string::npos)){
					found_tmp = int(line.find("\n",found_s+3));
				}
							
				sub = line.substr(found_s+3,found_tmp-found_s+3);
				expshapestr = sub;
			}
			if (found_n!=int(std::string::npos)){
				//find next whitespace or newline
				int found_tmp = int(line.find(" ",found_n+3));
				if(found_tmp==int(std::string::npos)){
					found_tmp = int(line.find("\n",found_n+3));
				}
				
				sub = line.substr(found_n+3,found_tmp-found_n+3);
				num_runs = atoi(sub.c_str());
			}
			if(found_w==int(std::string::npos)){
				found_w = int(line.find("\n"));
			}
			expnumstr = line.substr(1,found_w-1);
			
			if(expnumstr != ""){
				int expnum 	= atoi(expnumstr.c_str());
				expshape = EXPERIMENT_SHAPES(atoi(expshapestr.c_str()));
				
				boost::thread* experimentThread;
				expsemastop = false;
				experimentThread = new boost::thread(
					runExperiment, hand, fts, &wam, &pm, EXPERIMENT_KEYS(expnum));
				waitForEnter();
				expsemastop = true;
			}
			else{
				std::cout << "please enter r[exp#] -s [shape#] -n [num_runs]" << std::endl;
				std::cout << "possible exp#:" << std::endl;
				for(int i = 0; i < NUM_EXPERIMENTS; i++){
					std::cout << "\t" << i << ": " << experiment_keys[i] << std::endl;
				}
				std::cout << "possible shape# (default:0):" << std::endl;
				for(int i = 0; i < NUM_SHAPES; i++){
					std::cout << "\t" << i << ": " << experiment_shapes[i] << std::endl;
				}
				std::cout << "num_runs (default: 1): how many times to repeat the experiment" << std::endl;
			}
			break;
		}
		case 'e':
		{
			/*output_log<DIMENSION>(pm, (void*)&logger);
			//output data to log file
			// Wait for the user to press Shift-idle
			pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
			logger.closeLog();
			//((systems::PeriodicDataLogger<tuple_type>*)loggerin)->closeLog();
			printf("Logging stopped.\n");
			
			typedef boost::tuple<double, jp_type, jv_type, jt_type, cp_type, Eigen::Quaterniond> tuple_type;
			
			log::Reader<tuple_type> lr(tmpFile);
			lr.exportCSV(outFile);
			printf("Output written to %s.\n", outFile);
			std::remove(tmpFile);
			*/
			exit(0);
			break;
		}
		case 't':
		{
			int window_size = 1;
			std::string sub = "";
			int found_w = int(line.find(" "));
			int found_s = int(line.find("-s"));
			//arg -s: shape of object to grasp
			if (found_s!=int(std::string::npos)){
				//find next whitespace or newline
				int found_tmp = int(line.find(" ",found_s+3));
				if(found_tmp==int(std::string::npos)){
					found_tmp = int(line.find("\n",found_s+3));
				}
				sub = line.substr(found_s+3,found_tmp-found_s+3);
				window_size = atoi(sub.c_str());
			}
			tare_strain(hand);
			tare_tactile(hand);
			break;
		}	
		case 'o':
		{
			std::cout << "not yet implemented" << std::endl;
			/*
			Eigen::Quaterniond orientation = wam.getToolOrientation();
			std::cout 	<< "maintaining " 
						<< toString(&quaternion2hjp(&orientation)) 
						<< std::endl;			
			wam.moveTo(orientation, true, 0.3, 0.25);
			systems::ExposedOutput<Eigen::Quaterniond> toSetpoint(orientation);
			//causes wam to hold its tool at the desiredOrientation
			{
				BARRETT_SCOPED_LOCK(pm.getExecutionManager()->getMutex());
				wam.idle();
				forceConnect(toSetpoint.output, wam.toController.referenceInput);
				forceConnect(wam.tt2jt.output, wam.input);
			}*/
			break;
		}
		case 'd':
		{
			collectData = !collectData;
			std::cout << "Data collection toggled: " << collectData << std::endl;
			break;
		}
		case '1':
		{
			loadExpVariables();
			//cast wam to 7DOF first
			wamBottom	= (*((systems::Wam<DIMENSION>*)(&wam))).getJointPositions();
			wamBottomC 	= (*((systems::Wam<DIMENSION>*)(&wam))).getToolPosition();
			wamBottomQ 	= (*((systems::Wam<DIMENSION>*)(&wam))).getToolOrientation();
			wamBottomO	= quaternion2hjp(&wamBottomQ);
			std::cout << "Setting wamBottom to " << toString(&wamBottom) << std::endl;
			std::cout << "Setting wamBottomC to " << toString(&wamBottomC) << std::endl;
			std::cout << "Setting wamBottomQ to " << toString(&wamBottomO) << std::endl;
			saveExpVariables();
			break;
		}
		case '2':
		{
			loadExpVariables();
			//cast wam to 7DOF first
			wamTop 	= (*((systems::Wam<DIMENSION>*)(&wam))).getJointPositions();
			wamTopC = (*((systems::Wam<DIMENSION>*)(&wam))).getToolPosition();
			wamTopQ = (*((systems::Wam<DIMENSION>*)(&wam))).getToolOrientation();
			wamTopO = quaternion2hjp(&wamTopQ);
			std::cout << "Setting wamTop to " << toString(&wamTop) << std::endl;
			std::cout << "Setting wamTopC to " << toString(&wamTopC) << std::endl;
			std::cout << "Setting wamTopQ to " << toString(&wamTopO) << std::endl;
			saveExpVariables();
			break;
		}
		default:
			Hand* hand = pm.getHand();
			unsigned char in = atoi(line.c_str());
			handCommand(hand, in);
			
			//parseDoubles(&temp,line.c_str());
			//std::cout << toString(temp) << std::endl;
			
			printf("\n");
			printf("    'j' go to joint position\n");
			printf("    'p' go to tool  position\n");
			printf("    'w' go to hand  position\n");
			printf("    'g' grasp object\n");
			printf("    'u' ungrasp object\n");
			printf("    'b' toggle hand backdrivability\n");
			printf("    'i' to idle wam\n");
			printf("    'h' return to home position\n");
			printf("    's' to show formatted output\n");
			printf("    'r' to run experiment\n");
			printf("    'e' to end program and output to log\n");
			printf("    't' to tare the tactile and strain sensors\n");
			printf("    'o' to cause WAM to hold its current orientation\n");
			printf("	'd' to toggle data collection on/off (default off)\n");
			printf("    '1' to record WAM bottom joint angles\n");
			printf("    '2' to record WAM top joint angles\n");
			break;
		}
	}
	return 0;
}

int openSocket(const char* remoteHost, int port = 3333) {
	int sock;

	int err;
	long flags;
	struct sockaddr_in bind_addr;
	struct sockaddr_in their_addr;

	/* Create socket */
	sock = socket(PF_INET, SOCK_DGRAM, 0);
	if (sock == -1)
	{
		syslog(LOG_ERR,"%s: Could not create socket.",__func__);
		throw std::runtime_error("openSocket(): Failed. Check /var/log/syslog.");
	}

	/* Set socket to non-blocking, set flag associated with open file */
	flags = fcntl(sock, F_GETFL, 0);
	if (flags < 0)
	{
		syslog(LOG_ERR,"%s: Could not get socket flags.",__func__);
		throw std::runtime_error("openSocket(): Failed. Check /var/log/syslog.");
	}
	flags |= O_NONBLOCK;
	err = fcntl(sock, F_SETFL, flags);
	if (err < 0)
	{
		syslog(LOG_ERR,"%s: Could not set socket flags.",__func__);
		throw std::runtime_error("openSocket(): Failed. Check /var/log/syslog.");
	}

	/* Set up the bind address */
	bind_addr.sin_family = AF_INET;
	bind_addr.sin_port = htons(port);
	bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	err = bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
	if (err == -1)
	{
		syslog(LOG_ERR,"%s: Could not bind to socket on port %d.",__func__,port);
		throw std::runtime_error("openSocket(): Failed. Check /var/log/syslog.");
	}

	/* Set up the other guy's address */
	their_addr.sin_family = AF_INET;
	their_addr.sin_port = htons(port);
	err = ! inet_pton(AF_INET, remoteHost, &their_addr.sin_addr);
	if (err)
	{
		syslog(LOG_ERR,"%s: Bad IP argument '%s'.",__func__,remoteHost);
		throw std::runtime_error("openSocket(): Failed. Check /var/log/syslog.");
	}

	/* Call "connect" to set datagram destination */
	err = connect(sock, (struct sockaddr *)&their_addr, sizeof(struct sockaddr));
	if (err)
	{
		syslog(LOG_ERR,"%s: Could not set datagram destination.",__func__);
		throw std::runtime_error("openSocket(): Failed. Check /var/log/syslog.");
	}

	return sock;
}
void backDriveHand(Hand* hand, ForceTorqueSensor* fts, void* wamin, ProductManager* pm){
	std::cout << "Hand Backdrivability ON" << std::endl;
	double finger_joint_step = misc_parms[0];
	double tact_stop = misc_parms[1];
	double tact_push = misc_parms[2];
	double strain_stop = misc_parms[3];
	double strain_push = misc_parms[4];
	double strain_pull = misc_parms[5];
	double temp = misc_parms[6];
	std::cout << "parameters: ";
	std::cout 	<< finger_joint_step << ", "
				<< tact_stop << ", "
				<< tact_push << ", "
				<< strain_stop << ", "
				<< strain_push << ", "
				<< strain_pull << ", " << std::endl;
	std::cout << "tact_base_vals: ";
	std::cout 	<< tact_base_val[0] << ", "
				<< tact_base_val[1] << ", "
				<< tact_base_val[2] << std::endl;
	std::cout << "strain_base_vals: ";
	std::cout 	<< strain_base_val[0] << ", "
				<< strain_base_val[1] << ", "
				<< strain_base_val[2] << ", "
				<< strain_base_val[3] << std::endl;
	//hand->setPositionMode();
	while(!backdrivesemastop){
		//boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		int num_fingers = 3;
		hand->update(S_POSITION, true);
		Hand::jp_type hjp = hand->getInnerLinkPosition();
		//std::cout << "before: " << toString(&hjp) << std::endl;
		for(int i = 0; i < num_fingers; i++){
			//F > zero_thresh
			if(check_tactile_contact(hand, i, tact_base_val[i]*tact_stop)||
				check_strain_contact(hand, i, strain_base_val[i]*strain_stop)){	//F1,2,3
				finger_contacts[i] = 1;
				//F > push_thresh
				if(check_tactile_contact(hand, i, tact_base_val[i]*tact_push)||
					check_strain_contact(hand, i, strain_base_val[i]*strain_push)){
					//Hand::jv_type hjv;
					//setVectorValues(&hjv, 0, 0);
					//std::cout << "Move F" << i+1 << " back slightly" << std::endl;
					//hjv[i] = -0.1;	//radians/sec
					//hand->velocityMove(hjv);
					//hand->trapezoidalMove(hjp, true);
					//std::cout << "changing for F" << i << std::endl;
					//std::cout << toString(&hjp) << std::endl;

					hjp[i] /*-= finger_joint_step;*/ = std::max(hjp[i]-finger_joint_step, finger_joint_step);	//radians
					hand->setPositionMode();
					hand->setPositionCommand(hjp);
				}
			}
			//F < pull_thresh
			if(i != 2 && get_strain_value(hand, i) < strain_base_val[i]*strain_pull){	//F1,2,3
				//Hand::jv_type hjv;
				//setVectorValues(&hjv, 0, 0);
				//std::cout << "Move F" << i+1 << " back slightly" << std::endl;
				//hjv[i] = 0.1;	//radians/s
				//hand->velocityMove(hjv);
				//hand->trapezoidalMove(hjp, true);
				//std::cout << toString(&hjp) << std::endl;
				
				std::cout << get_strain_value(hand, i) << " < " << strain_base_val[i]*strain_pull << std::endl;
				
				hjp[i] /*+= finger_joint_step;*/ = std::min(hjp[i]+finger_joint_step, FINGER_JOINT_LIMIT-finger_joint_step);	//radians
				hand->setPositionMode();
				hand->setPositionCommand(hjp);
			}
			if(i == 2 && get_strain_value(hand, i) < strain_base_val[i]*strain_pull*1.1){	//F1,2,3
				//Hand::jv_type hjv;
				//setVectorValues(&hjv, 0, 0);
				//std::cout << "Move F" << i+1 << " back slightly" << std::endl;
				//hjv[i] = 0.1;	//radians/s
				//hand->velocityMove(hjv);
				//hand->trapezoidalMove(hjp, true);
				//std::cout << toString(&hjp) << std::endl;
				
				std::cout << get_strain_value(hand, i) << " < " << strain_base_val[i]*strain_pull << std::endl;
				
				hjp[i] /*+= finger_joint_step;*/ = std::min(hjp[i]+finger_joint_step, FINGER_JOINT_LIMIT-finger_joint_step);	//radians
				hand->setPositionMode();
				hand->setPositionCommand(hjp);
			}
		}
		//std::cout << "after: " << toString(&hjp) << std::endl;
		//if(check_tactile_contact(hand, 3, ZERO_TACTILE_THRESHOLD*3)){	//PALM
			//cp_type
			//std::cout << "Move hand back slightly" << std::endl;
		//}
	}
	std::cout << "Hand Backdrivability OFF" << std::endl;
}
void dataCollect(Hand* hand, ForceTorqueSensor* fts, void* wamin, ProductManager* pm, 
					enum EXPERIMENT_KEYS expnum, enum EXPERIMENT_SHAPES expshape){
	//BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	systems::Wam<DIMENSION>* wam = (systems::Wam<DIMENSION>*)wamin;
	std::vector<systems::Wam<DIMENSION>::jp_type> jp; 	std::vector<systems::Wam<DIMENSION>::jp_type>::iterator jpit;
	std::vector<systems::Wam<DIMENSION>::jv_type> jv; 	std::vector<systems::Wam<DIMENSION>::jv_type>::iterator jvit;
	std::vector<systems::Wam<DIMENSION>::jt_type> jt; 	std::vector<systems::Wam<DIMENSION>::jt_type>::iterator jtit;
	std::vector<systems::Wam<DIMENSION>::cp_type> cp; 	std::vector<systems::Wam<DIMENSION>::cp_type>::iterator cpit;
	std::vector< std::vector<double> > to; 		std::vector< std::vector<double> >::iterator toit;  std::vector<double>::iterator eigscait;
	std::vector<Hand::cf_type> cf; 				std::vector<Hand::cf_type>::iterator cfit;
	ct.clear();	/*defined globally*/			std::vector<Hand::ct_type>::iterator ctit;
	std::vector<Hand::jp_type> hjp_in; 			std::vector<Hand::jp_type>::iterator hjpit; //can use for both in and out
	std::vector<Hand::jp_type> hjp_out;
	hstrain.clear(); /*defined globally*/		std::vector< std::vector<int> >::iterator hsit;		std::vector<int>::iterator intit;
	std::vector< std::vector<v_type> > htact;	std::vector< std::vector<v_type> >::iterator htit;	std::vector<v_type>::iterator vtit;
	
	std::vector<TactilePuck*> tps;
	tps = hand->getTactilePucks();
	
	std::vector<v_type> temp;
	
	std::cout << "data collection thread started!" << std::endl;
	
	//systems::Wam<DIMENSION>::jp_type wamBottom;
	//parseDoubles(&wamBottom, "-0.0800 -1.8072 -0.0199 0.9068 0.5583 -0.4459 0.0");
	//wam->moveTo(wamBottom, false, 1.0);
	
	while(/*pm->getSafetyModule()->getMode() == SafetyModule::ACTIVE && */!datasemastop){
		// WAM
		jp.push_back(wam->getJointPositions());
		jv.push_back(wam->getJointVelocities());
		jt.push_back(wam->getJointTorques());
		
		//WAM end-link Position (wrist)
		cp.push_back(wam->getToolPosition());
		Eigen::Quaterniond q = wam->getToolOrientation();
		std::vector<double> q_Scalars;
		q_Scalars.push_back(q.w());
		q_Scalars.push_back(q.x());
		q_Scalars.push_back(q.y());
		q_Scalars.push_back(q.z());
		to.push_back(q_Scalars);
		
		// FTS
		fts->update(true);
		cf.push_back(fts->getForce());
		
		Hand::ct_type _ct = fts->getTorque();
		ct.push_back(_ct);
		for(int i = 0; i < (int)_ct.size(); ++i){
			if(_ct[i] < min_ct[i])
				min_ct[i] = _ct[i];
		}
		
		// Hand
		hand->update(S_ALL, true);
		hjp_in.push_back(hand->getInnerLinkPosition());
		hjp_out.push_back(hand->getOuterLinkPosition());
		
		std::vector<int> strain = hand->getFingertipTorque();
		hstrain.push_back(strain);
		for (size_t i = 0; i < strain.size(); ++i){
			double torque = strain[i]/STRAIN2TORQUE_RATIO;
			if(torque < min_hstrain[i])
				min_hstrain[i] = torque;
		}
		for (size_t i = 0; i < tps.size(); ++i){
			temp.push_back(tps[i]->getFullData());
		}
		htact.push_back(temp);
		temp.clear();
		
		usleep(10000);	//record data @ 100 Hz
	}
	
	bool dump_data = true;
	if(dump_data){
		std::cout << "dumping data to file...";
		fflush(stdout);
		//dump data to file
		std::ofstream WAMJP;
		std::ofstream WAMJV;
		std::ofstream WAMJT;
		std::ofstream WAMCP;
		std::ofstream WAMTO;
		std::ofstream FTSF;
		std::ofstream FTST;
		std::ofstream BHIN;
		std::ofstream BHOUT;
		std::ofstream HSTRAIN;
		std::ofstream HTACT;
		
		std::string prefix = 	"data/" 						+ 
								experiment_keys[  int(expnum  )]+ "/" +
								experiment_shapes[int(expshape)]+ "/" ;
		std::string suffix = ".dat";
		
		std::cout << "open..." << prefix << "...";
		
		WAMJP.open	((prefix + "WAMJP" 	+ suffix).c_str(),	std::ios::trunc);
		WAMJV.open	((prefix + "WAMJV" 	+ suffix).c_str(),	std::ios::trunc);
		WAMJT.open	((prefix + "WAMJT" 	+ suffix).c_str(),	std::ios::trunc);
		WAMCP.open	((prefix + "WAMCP" 	+ suffix).c_str(),	std::ios::trunc);
		WAMTO.open	((prefix + "WAMTO" 	+ suffix).c_str(),	std::ios::trunc);
		FTSF.open	((prefix + "FTSF" 	+ suffix).c_str(), 	std::ios::trunc);
		FTST.open	((prefix + "FTST" 	+ suffix).c_str(), 	std::ios::trunc);
		BHIN.open	((prefix + "BHIN" 	+ suffix).c_str(), 	std::ios::trunc);
		BHOUT.open	((prefix + "BHOUT" 	+ suffix).c_str(), 	std::ios::trunc);
		HSTRAIN.open((prefix + "HSTRAIN"+ suffix).c_str(),	std::ios::trunc);
		HTACT.open	((prefix + "HTACT" 	+ suffix).c_str(), 	std::ios::trunc);
		
		std::cout << "for...";
		
		for (jpit=jp.begin()	  ; jpit < jp.end()		 ; jpit++)	{WAMJP << *jpit << std::endl;}
		for (jvit=jv.begin()	  ; jvit < jv.end()		 ; jvit++)	{WAMJV << *jvit << std::endl;}
		for (jtit=jt.begin()	  ; jtit < jt.end()		 ; jtit++)	{WAMJT << *jtit << std::endl;}
		for (cpit=cp.begin()	  ; cpit < cp.end()		 ; cpit++)	{WAMCP << *cpit << std::endl;}
		for (toit=to.begin()	  ; toit < to.end()		 ; toit++)	{
			std::vector<double> tovec = *toit;
			WAMTO << '[';
			for(eigscait=tovec.begin(); eigscait<tovec.end()-1; eigscait++){WAMTO << *eigscait << ", ";}
			WAMTO << *eigscait << ']' << std::endl;
		}
		for (cfit=cf.begin()	  ; cfit < cf.end()		 ; cfit++)	{FTSF << *cfit << std::endl; }
		for (ctit=ct.begin()	  ; ctit < ct.end()      ; ctit++)	{FTST << *ctit << std::endl; }
		for (hjpit=hjp_in.begin() ; hjpit < hjp_in.end() ; hjpit++) {BHIN << *hjpit << std::endl;}
		for (hjpit=hjp_out.begin(); hjpit < hjp_out.end(); hjpit++)	{BHOUT << *hjpit << std::endl;}
		for (hsit=hstrain.begin() ; hsit < hstrain.end() ; hsit++){
			std::vector<int> hsvec = *hsit;
			HSTRAIN << '[';
			for(intit=hsvec.begin(); intit<hsvec.end()-1; intit++){HSTRAIN << *intit << ", ";}
			HSTRAIN << *intit << ']' << std::endl;
		}
		for (htit=htact.begin()   ; htit < htact.end()   ; htit++){
			std::vector<v_type> htvec = *htit;
			HTACT << '[';
			for(vtit=htvec.begin(); vtit<htvec.end(); vtit++){HTACT << *vtit;}
			HTACT << ']' << std::endl;
		}
		
		std::cout << "close...";
		
		WAMJP.close();
		WAMJV.close();
		WAMJT.close();
		WAMCP.close();
		WAMTO.close();
		FTSF.close();
		FTST.close();
		BHIN.close();
		BHOUT.close();
		std::cout << "done" << std::endl;
		fflush(stdout);
	}
}

void stop_thread(bool* semaphore){
	waitForEnter();
	*semaphore = !*semaphore;
}

/*
void handEntryPoint(Hand* hand, const char* remoteHost) {
	static const int SIZE_OF_MSG = sizeof(unsigned char);

	int sock = openSocket(remoteHost);
	unsigned char data = 0, data_1 = 0;
	Hand::jv_type hjv;

	const int MAX_MISSED = 50;
	int numMissed = MAX_MISSED;

	while (!boost::this_thread::interruption_requested()) {
		// Get any packets in the buffer
		if (numMissed <= MAX_MISSED) {  // Keep numMissed from wrapping.
			++numMissed;
		}
		while (recv(sock, &data, SIZE_OF_MSG, 0) == SIZE_OF_MSG) {
			numMissed = 0;
		}
		printf("%x\n",data);

		// If things havn't changed, just wait and loop
		if (numMissed  ||  data == data_1) {
			// If we havn't seen a message in a while, stop the Hand.
			if (numMissed == MAX_MISSED) {
				printf("Sending stop command to hand.\n");
				hand->idle();
			}
		} else {
			hjv[0] = velCommand(data & (1<<2), data & (1<<3));  // Middle
			hjv[1] = velCommand(data & (1<<0), data & (1<<1));  // Pointer
			hjv[2] = velCommand(data & (1<<4), data & (1<<5));  // Thumb
			hjv[3] = velCommand(data & (1<<6), data & (1<<7));  // Rocker
			hand->velocityMove(hjv);
			std::cout << "Velocity: " << hjv << std::endl;

			data_1 = data;
		}

		usleep(10000);
	}

	close(sock);
}
*/
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
