#include "active.h"

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
	
	bool fingertip_torqueContact = false;
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
	
	bool fingertip_torqueContact = false;
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
