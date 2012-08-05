#include "bh.h"
//BHVelocity6
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
