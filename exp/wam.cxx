#include "wam.h"

/*WAMVelocity2*/
template<size_t DOF>
void runWAMVelocityExperiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM], false, 3.0);
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
			(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_TOP], false, velocity, accel);
		}
		else{ //even
			(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM], false, velocity, accel);
		}
		velocity+= 0.25;
		accel	+= 0.25;
		
		//hand->setVelocity(exp_vars[HAND_PREGRASP]);	
		//while(!hand->doneMoving(true));
		
	}
}
/*WAMJointPos3*/
template<size_t DOF>
void runWAMJointPosExperiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM], false, 3.0);
	std::cout << "start!" << std::endl;
	int curr_state = 0;
	//int num_runs = 5;
	systems::Wam<DIMENSION>::jp_type steps;
	get_interpolating_steps(&steps, &exp_vars[WAM_BOTTOM], &exp_vars[WAM_TOP], num_runs);
	
	printf("/%02d   pos\n",num_runs);
	fflush(stdout);
	 
	while(curr_state < num_runs && !expsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		
		if(wam.moveIsDone()){++curr_state;}else{continue;}
		
		add_vector_values(&exp_vars[WAM_BOTTOM],&steps);
		printf(" %02d ",curr_state);
		fflush(stdout);
		std::cout << to_string(&exp_vars[WAM_BOTTOM]) << std::endl;
		(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM], false, 3.0);
	}
}
/*WAMCartesianPos4*/
template<size_t DOF>
void runWAMCartesianPosExperiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	Eigen::Quaterniond exp_vars[WAM_BOTTOM]Q = hjp2quaternion(&exp_vars[WAM_BOTTOM_O]);
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM_C], false, 3.0);
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM]Q, true, 0.3, 0.25);
	
	//causes wam to hold its tool at the desiredOrientation
	systems::ExposedOutput<Eigen::Quaterniond> toSetpoint(exp_vars[WAM_BOTTOM]Q);
	{
		//std::cout << "maintaining " << to_string(&exp_vars[WAM_BOTTOM_O]) << std::endl;
		BARRETT_SCOPED_LOCK(pm->getExecutionManager()->getMutex());

}
/*ActiveSensing1*/
template<size_t DOF>
void runActiveSensingExperiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM], false, 3.0, 3.0);
	std::cout << "start!" << std::endl;
	int curr_state = 0;
	//num_runs = 2;
	 
	while(curr_state < num_runs*2 && !expsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		
		if(wam.moveIsDone()){++curr_state;}else{continue;}
		
		if(curr_state % 2){ //odd
			if(num_runs % 2){	//odd
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_TOP], false, 3.0, 0.5);
			}
			else{	//even
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_TOP_C], false, 3.0, 0.5);
			}
		}
		else{ //even
			if(num_runs % 2){	//odd
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM], false, 3.0, 0.5);
			}
			else{	//even
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM_C], false, 3.0, 0.5);
			}
		}
	}
}
//WAMJointTorque5
template<size_t DOF>
void runWAMJointTorqueExperiment(systems::Wam<DOF>& wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	loadExpVariables();
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM], false, 3.0);
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
			set_vector_values(&joint_torques,position, -1);
		}
		else{ //even
			set_vector_values(&joint_torques,FINGER_JOINT_LIMIT-position, -1);
		}
		hand->setPositionCommand(joint_torques);
		position+= step;
	}
}
