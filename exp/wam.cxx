#include "wam.h"

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
		
		//hand->setVelocity(handPregrasp);	
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
	
	Eigen::Quaterniond wamBottomQ = hjp2quaternion(&wamBottomO);
	
	//start experiment: move WAM to first goal (no blocking)
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottomC, false, 3.0);
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(wamBottomQ, true, 0.3, 0.25);
	
	//causes wam to hold its tool at the desiredOrientation
	systems::ExposedOutput<Eigen::Quaterniond> toSetpoint(wamBottomQ);
	{
		//std::cout << "maintaining " << toString(&wamBottomO) << std::endl;
		BARRETT_SCOPED_LOCK(pm->getExecutionManager()->getMutex());

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
//WAMJointTorque5
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
