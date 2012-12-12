#include "experiment.h"
#include "action.h"
#include "utils.h"
#include "control.h"
#include "senses.h"
#include "robot.h"

Action::Action(Robot* robot)
: Experiment(robot){
}
ActionPhase::ActionPhase(Robot* robot)
: Action(robot){
}
SimpleShapes::SimpleShapes(Robot* robot)
: Action(robot){
}
void Action::run(){
}
void ActionPhase::run(){
	//BARRETT_UNITS_TEMPLATE_TYPEDEFS(DIMENSION);
	/*
	load_exp_variables();
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM], false, 3.0);
	std::cout << "start!" << std::endl;
	long timer = 0;
	while(prev_state < NUM_ACTION_PHASES){// && !expsemastop){
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
				if(ct.size() >= 100){ //more than exp_vars[TORQUE_EPSILON] different than 1 second ago
					//std::cout << "prev_torque: " << ct[ct.size()-100][1] << std::endl;
					//std::cout << "torque: " << torques[1] << std::endl;
					//std::cout << "prev_torque-torque: " << ct[ct.size()-100][1]-torques[1] << std::endl;
					transition = 
						ct[ct.size()-100][1]-torques[1] > exp_vars[TORQUE_EPSILON][1] || 
						ct[ct.size()-100][1]-torques[1] < -1*exp_vars[TORQUE_EPSILON][1];
				}
				break;
			}
			//WAM reached exp_vars[WAM_TOP]
			case TRANSITIONAL:{
				//std::cout << "Wam done? " << wam->moveIsDone() << std::endl;
				transition = wam->moveIsDone();
				break;
			}
			//lift timer expired
			case STATIC:{
				transition = timer > 100;
				break;
			}
			//object contact with table (grip/load ratio event)
			case REPLACEMENT:{
				hand->update(Hand::S_ALL, true);
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
					
					//more than exp_vars[TORQUE_EPSILON] different than previously
					transition = 
						prev_grip_ratio - grip_ratio > exp_vars[TORQUE_EPSILON][1] ||
						prev_grip_ratio - grip_ratio < -1*exp_vars[TORQUE_EPSILON][1];
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
				hand->setVelocity(exp_vars[HAND_GRASP]);
				break;
			}
			
			//slowly lift WAM to exp_vars[WAM_TOP]
			case LOADING:{
				std::cout << "Performing LOADING action-phase!" << std::endl;
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_TOP], false, 3.0);
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
			
			//slowly lower WAM to exp_vars[WAM_BOTTOM]
			case REPLACEMENT:{
				std::cout << "Performing REPLACEMENT action-phase!" << std::endl;
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM], false, 3.0);
				break;
			}
			
			//slowly open finger
			case UNLOADING:{
				std::cout << "Performing UNLOADING action-phase!" << std::endl;
				hand->setVelocity(exp_vars[HAND_UNGRASP]);
				break;
			}
			
			//done!
			default: {
				std::cout << "Action COMPLETE!" << std::endl;
				break;
			}
		}
		
		//hand->setVelocity(exp_vars[HAND_PREGRASP]);	
		//while(!hand->doneMoving(true));		
		prev_state = curr_state;
	}*/
}


void SimpleShapes::run(){
/*SimpleShapes10*/
//template<size_t DIMENSION>
//void runSimpleShapesExperiment(systems::Wam<DIMENSION>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	//BARRETT_UNITS_TEMPLATE_TYPEDEFS(DIMENSION);
	
	//start experiment: move WAM to first goal (no blocking)
	/*(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM], false, 3.0, 5.0);
	std::cout << "start!" << std::endl;
	int curr_state = 0;
	//num_runs = 3;
	Hand::jp_type digits;
	set_vector_values(&digits,0,0);	//init
	set_vector_values(&digits,1,-1);	//activate all except spread
	 
	while(curr_state < num_runs*4){// && !expsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		
		if(hand->doneMoving(true)&&wam->moveIsDone()){++curr_state;}else{continue;}
		
		switch(curr_state%4){
			case 1 : {
				closeHand(hand);
				break;
			}
			case 2 : {
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_TOP], false, 3.0, 5.0);
				break;
			}
			case 3 : {
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM], false, 3.0, 5.0);
				break;
			}
			case 0 : {
				openHand(hand);
				break;
			}
		}
	}*/
}

