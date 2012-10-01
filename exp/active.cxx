#include "active.h"

CartesianRaster::CartesianRaster(Controller* controller, Senses* senses):
    Active(controller, senses){
}

//MISTAKE: NOT CARTESIAN RASTER CODE!!
/*ActiveSensing1*/
void CartesianRaster::run(){
	//BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	//loadExpVariables();
	
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
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM], false, 0.3, 0.25);
	std::cout << "start!" << std::endl;
	int curr_state = 0;
	int num_states = 2;
	Hand::jp_type digits;
	set_vector_values(&digits,0,0);	//init
	set_vector_values(&digits,FINGER_JOINT_LIMIT/2,-1);	//activate all except spread
	
	bool fingertip_torqueContact = false;
	bool tactileContact = false;
	bool handAboveTable = false;
	bool objectDetected = false;
	
	//exp_vars[WAM_BOTTOM]Q = hjp2quaternion(&exp_vars[WAM_BOTTOM_O]);
	//exp_vars[WAM_TOP]Q = hjp2quaternion(&exp_vars[WAM_TOP_O]);
	
	float minX, maxX, minY, maxY, minZ, maxZ;
	
	minX = exp_vars[WAM_BOTTOM_C][0];
	minY = exp_vars[WAM_BOTTOM_C][1];
	minZ = exp_vars[WAM_BOTTOM_C][2];
	maxX = exp_vars[WAM_TOP_C][0];
	maxY = exp_vars[WAM_TOP_C][1];
	maxZ = 0.25;
	
	exp_vars[WAM_TOP_C][0] = (maxX+minX)/2;
	exp_vars[WAM_TOP_C][1] = (maxY+minY)/2;
	exp_vars[WAM_TOP_C][2] = maxZ;
	
	exp_vars[WAM_BOTTOM_C]_ip[2] = exp_vars[WAM_BOTTOM_C][2] + 0.1;
	
	//hand->trapezoidalMove(digits, false);
	 
	while(curr_state < num_runs*num_states && !expsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		
		//record phase of motion
		int phase = curr_state % num_states;
		//std::cout << phase << std::endl;
		
		//check tactile reading and stop if object collision above the table
		fingertip_torqueContact = check_fingertip_torque_contact(hand, ZERO_FINGERTIP_TORQUE_THRESHOLD);
		tactileContact = check_tactile_contact(hand);
		bool contact = fingertip_torqueContact || tactileContact;
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
				//std::cout << "moving to wtc @ " << to_string(&exp_vars[WAM_TOP_C]);
				std::cout << "ActiveProbing take " << curr_state/num_states << std::endl;
				std::cout << "Moving to " << to_string(&exp_vars[WAM_TOP_C]) << std::endl;
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_TOP_C], false, 0.3, 0.25);
				break;
			}
			case 1 : {
				objectDetected = false;
				exp_vars[WAM_BOTTOM_C][0] = random_float(minX, maxX);
				exp_vars[WAM_BOTTOM_C][1] = random_float(minY, maxY);
				exp_vars[WAM_BOTTOM_C]_ip[0] = exp_vars[WAM_BOTTOM_C][0];
				exp_vars[WAM_BOTTOM_C]_ip[1] = exp_vars[WAM_BOTTOM_C][1];
				//std::cout << "wbc0: " << exp_vars[WAM_BOTTOM_C][0] << std::endl;
				//std::cout << "wbc1: " << exp_vars[WAM_BOTTOM_C][1] << std::endl;
				//std::cout << "moving to wbc @ " << to_string(&exp_vars[WAM_BOTTOM_C]);
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM_C]_ip, true, 0.3, 0.25);
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM]Q, true, 0.3, 0.25);
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM_C], false, 0.3, 0.25);
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
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM], false, 0.3, 0.25);
	std::cout << "start!" << std::endl;
	int curr_state = 0;
	int num_states = 2;
	Hand::jp_type digits;
	set_vector_values(&digits,0,0);	//init
	set_vector_values(&digits,FINGER_JOINT_LIMIT/2,-1);	//activate all except spread
	
	bool fingertip_torqueContact = false;
	bool tactileContact = false;
	bool handAboveTable = false;
	bool objectDetected = false;
	
	//exp_vars[WAM_BOTTOM]Q = hjp2quaternion(&exp_vars[WAM_BOTTOM_O]);
	//exp_vars[WAM_TOP]Q = hjp2quaternion(&exp_vars[WAM_TOP_O]);
	
	float minX, maxX, minY, maxY, minZ, maxZ;
	
	minX = exp_vars[WAM_BOTTOM_C][0];
	minY = exp_vars[WAM_BOTTOM_C][1];
	minZ = exp_vars[WAM_BOTTOM_C][2];
	maxX = exp_vars[WAM_TOP_C][0];
	maxY = exp_vars[WAM_TOP_C][1];
	maxZ = 0.25;
	
	exp_vars[WAM_TOP_C][0] = (maxX+minX)/2;
	exp_vars[WAM_TOP_C][1] = (maxY+minY)/2;
	exp_vars[WAM_TOP_C][2] = maxZ;
	
	exp_vars[WAM_BOTTOM_C]_ip[2] = exp_vars[WAM_BOTTOM_C][2] + 0.1;
	
	//hand->trapezoidalMove(digits, false);
	
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_TOP_C], true, 0.3, 0.25);
	(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM]Q, true, 0.3, 0.25);
	
	//causes wam to hold its tool at the desiredOrientation
	systems::ExposedOutput<Eigen::Quaterniond> toSetpoint(exp_vars[WAM_BOTTOM]Q);
	//systems::disconnect(toSetpoint.output);
	//systems::disconnect(wam.tt2jt.output);
	
	{
		std::cout << "maintaining " << to_string(&exp_vars[WAM_BOTTOM]Q) << std::endl;
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
		fingertip_torqueContact = check_fingertip_torque_contact(hand);
		tactileContact = check_tactile_contact(hand);
		bool contact = fingertip_torqueContact || tactileContact;
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
				//std::cout << "moving to wtc @ " << to_string(&exp_vars[WAM_TOP_C]);
				if(!objectDetected && move_and_lift){
					++curr_state; //try moving randomly again until we hit something
				}
				else{
					std::cout << "ActiveProbing take " << curr_state/num_states << std::endl;
					std::cout << "Moving to " << to_string(&exp_vars[WAM_TOP_C]) << std::endl;
					(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_TOP_C], false, 0.3, 0.25);
				}
				break;
			}
			case 1 : {
				objectDetected = false;
				exp_vars[WAM_BOTTOM_C][0] = random_float(minX, maxX);
				exp_vars[WAM_BOTTOM_C][1] = random_float(minY, maxY);
				exp_vars[WAM_BOTTOM_C]_ip[0] = exp_vars[WAM_BOTTOM_C][0];
				exp_vars[WAM_BOTTOM_C]_ip[1] = exp_vars[WAM_BOTTOM_C][1];
				//std::cout << "wbc0: " << exp_vars[WAM_BOTTOM_C][0] << std::endl;
				//std::cout << "wbc1: " << exp_vars[WAM_BOTTOM_C][1] << std::endl;
				//std::cout << "moving to wbc @ " << to_string(&exp_vars[WAM_BOTTOM_C]);
				//(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM_C]_ip, true, 0.3, 0.25);
				//(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM]Q, true, 0.3, 0.25);
				(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(exp_vars[WAM_BOTTOM_C], false, 0.3, 0.25);
				break;
			}
		}
	}
}
