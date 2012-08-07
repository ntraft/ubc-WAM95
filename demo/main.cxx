/*
 * master_master.cpp
 *
 *  Created on: Apr 22, 2012
 *      Author: Daniel Troniak
 */

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h> //wam_main function
#include "stdheader.h"  //check here for all standard includes and definitions

#include "utils.h"
#include "senses.h"
#include "control.h"

#include "experiment.h"
#include "action.h"

Experiment* experiment;

//thread management
/*bool expsemastop = true;
bool datasemastop = true;
bool backdrivesemastop = true;
bool graspsemastop = true;
bool showsensorsemastop = true;*/


bool validate_args(int argc, char** argv) {
	/*if (argc != 2  &&  argc != 3) {
		printf("Usage: %s <remoteHost> [--auto]\n", argv[0]);
		printf("  --auto : Automatically link WAMs and start Hand or Gimbals Hand Controller thread\n");

		return false;
	}*/
	return true;
}

void runExperiment(	Hand* hand, ForceTorqueSensor* fts, void* wamin, ProductManager* pm, 
					enum EXPERIMENT_KEYS expnum){
	systems::Wam<DIMENSION>* wam = (systems::Wam<DIMENSION>*)wamin;
	
	//datasemastop = false;
	boost::thread* dataCollectionThread = NULL;
	/*if(collectData){
		dataCollectionThread = new boost::thread(dataCollect, hand, fts, wam, pm, expnum, expshape);
	}*/

    Experiment* exp;
	
	//std::cout << "Running " << experiment_keys[int(expnum)] << " Experiment..." << std::endl;
	switch(expnum){
		case ACTIONPHASE:{
//			runActionPhaseExperiment(*wam, hand, fts, pm);
			exp = new ActionPhase(wam, hand, fts, pm);
            break;
		}
		case ACTIVESENSING:{
//			runActiveSensingExperiment(*wam, hand, fts, pm);
			break;
		}
		case WAMVELOCITY:{
//			runWAMVelocityExperiment(*wam, hand, fts, pm);
			break;
		}
		case WAMJOINTPOS:{
//			runWAMJointPosExperiment(*wam, hand, fts, pm);
			break;
		}
		case WAMCARTESIANPOS:{
//			runWAMCartesianPosExperiment(*wam, hand, fts, pm);
			break;
		}
		case WAMJOINTTORQUE:{
//			runWAMJointTorqueExperiment(*wam, hand, fts, pm);
			break;
		}
		case BHVELOCITY:{
//			runBHVelocityExperiment(*wam, hand, fts, pm);
			break;
		}
		case BHPOSITION:{
//			runBHPositionExperiment(*wam, hand, fts, pm);
			break;
		}
		case BHTORQUE:{
//			runBHTorqueExperiment(*wam, hand, fts, pm);
			break;
		}
		case BHTRAPEZOIDAL:{
//			runBHTrapezoidalExperiment(*wam, hand, fts, pm);
			break;
		}
		case SIMPLESHAPES:{
//			runSimpleShapesExperiment(*wam, hand, fts, pm);
			break;
		}
		case ACTIVEPROBING:{
			break;
		}
		case CARTESIANRASTER:{
//			runCartesianRasterExperiment(*wam, hand, fts, pm);
			break;
		}
		default:{ 
		}
	}
	
	//std::cout << "Experiment " << experiment_keys[int(expnum)];
	
	/*if(!expsemastop){
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
	}*/	
}



template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	const jp_type SYNC_POS(0.0);  // the position each WAM should move to before linking
	
	// These vectors are fixed sized, stack allocated, and zero-initialized.
	Hand::jp_type hjp; //hjp is a 4x1 column vector of hand joint positions
	jp_type jp;  // jp is a DOFx1 column vector of joint positions
	cp_type cp;  // cp is a 3x1 vector representing a Cartesian position
	
	//setVectorValues(&finger_contacts, 0, 0);

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
			/*if(backdrivesemastop){
				boost::thread* backDriveHandThread;
				backdrivesemastop = false;
				backDriveHandThread = new boost::thread(backDriveHand, hand, fts, &wam, &pm);
			}
			else{
				backdrivesemastop = true;
			}*/
			break;
		}
		case 'r':{
            experiment->init(line);
            experiment->run();
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
			//int found_w = int(line.find(" "));
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
			tare_fingertip_torque(hand);
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
			//collectData = !collectData;
			//std::cout << "Data collection toggled: " << collectData << std::endl;
			break;
		}
		case '1':
		{
            /*
			loadExpVariables();
			//cast wam to 7DOF first
			wamBottom	= (*((systems::Wam<DIMENSION>*)(&wam))).getJointPositions();
			wamBottomC 	= (*((systems::Wam<DIMENSION>*)(&wam))).getToolPosition();
			wamBottomQ 	= (*((systems::Wam<DIMENSION>*)(&wam))).getToolOrientation();
			wamBottomO	= quaternion2hjp(&wamBottomQ);
			std::cout << "Setting wamBottom to " << toString(&wamBottom) << std::endl;
			std::cout << "Setting wamBottomC to " << toString(&wamBottomC) << std::endl;
			std::cout << "Setting wamBottomQ to " << toString(&wamBottomO) << std::endl;
			saveExpVariables();*/
			break;
		}
		case '2':
		{
			/*
            loadExpVariables();
			//cast wam to 7DOF first
			wamTop 	= (*((systems::Wam<DIMENSION>*)(&wam))).getJointPositions();
			wamTopC = (*((systems::Wam<DIMENSION>*)(&wam))).getToolPosition();
			wamTopQ = (*((systems::Wam<DIMENSION>*)(&wam))).getToolOrientation();
			wamTopO = quaternion2hjp(&wamTopQ);
			std::cout << "Setting wamTop to " << toString(&wamTop) << std::endl;
			std::cout << "Setting wamTopC to " << toString(&wamTopC) << std::endl;
			std::cout << "Setting wamTopQ to " << toString(&wamTopO) << std::endl;
			saveExpVariables();*/
			break;
		}
		default:
			Hand* hand = pm.getHand();
			unsigned char in = atoi(line.c_str());
			handCommand(hand, in);

			//hello();
			
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
			printf("    't' to tare the tactile and fingertip_torque sensors\n");
			printf("    'o' to cause WAM to hold its current orientation\n");
			printf("	'd' to toggle data collection on/off (default off)\n");
			printf("    '1' to record WAM bottom joint angles\n");
			printf("    '2' to record WAM top joint angles\n");
			break;
		}
	}
	return 0;
}

void dataCollect(Hand* hand, ForceTorqueSensor* fts, void* wamin, ProductManager* pm, 
					enum EXPERIMENT_KEYS expnum, enum EXPERIMENT_SHAPES expshape){
/*	
//BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	systems::Wam<DIMENSION>* wam = (systems::Wam<DIMENSION>*)wamin;
	std::vector<systems::Wam<DIMENSION>::jp_type> jp; 	std::vector<systems::Wam<DIMENSION>::jp_type>::iterator jpit;
	std::vector<systems::Wam<DIMENSION>::jv_type> jv; 	std::vector<systems::Wam<DIMENSION>::jv_type>::iterator jvit;
	std::vector<systems::Wam<DIMENSION>::jt_type> jt; 	std::vector<systems::Wam<DIMENSION>::jt_type>::iterator jtit;
	std::vector<systems::Wam<DIMENSION>::cp_type> cp; 	std::vector<systems::Wam<DIMENSION>::cp_type>::iterator cpit;
	std::vector< std::vector<double> > to; 		std::vector< std::vector<double> >::iterator toit;  std::vector<double>::iterator eigscait;
	std::vector<Hand::cf_type> cf; 				std::vector<Hand::cf_type>::iterator cfit;
	ct.clear();						std::vector<Hand::ct_type>::iterator ctit;
	std::vector<Hand::jp_type> hjp_in; 			std::vector<Hand::jp_type>::iterator hjpit; //can use for both in and out
	std::vector<Hand::jp_type> hjp_out;
	hfingertip_torque.clear(); 				std::vector< std::vector<int> >::iterator hsit;		std::vector<int>::iterator intit;
	std::vector< std::vector<v_type> > htact;	std::vector< std::vector<v_type> >::iterator htit;	std::vector<v_type>::iterator vtit;
	
	std::vector<TactilePuck*> tps;
	tps = hand->getTactilePucks();
	
	std::vector<v_type> temp;
	
	std::cout << "data collection thread started!" << std::endl;
	
	//systems::Wam<DIMENSION>::jp_type wamBottom;
	//parseDoubles(&wamBottom, "-0.0800 -1.8072 -0.0199 0.9068 0.5583 -0.4459 0.0");
	//wam->moveTo(wamBottom, false, 1.0);
	
	while(datasemastop){
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
		hand->update(Hand::S_ALL, true);
		hjp_in.push_back(hand->getInnerLinkPosition());
		hjp_out.push_back(hand->getOuterLinkPosition());
		
		std::vector<int> fingertip_torque = hand->getFingertipTorque();
		hfingertip_torque.push_back(fingertip_torque);
		for (size_t i = 0; i < fingertip_torque.size(); ++i){
			double torque = fingertip_torque[i]/FINGERTIP_TORQUE2TORQUE_RATIO;
			if(torque < min_hfingertip_torque[i])
				min_hfingertip_torque[i] = torque;
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
		std::ofstream HFINGERTIP_TORQUE;
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
		HFINGERTIP_TORQUE.open((prefix + "HFINGERTIP_TORQUE"+ suffix).c_str(),	std::ios::trunc);
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
		for (hsit=hfingertip_torque.begin() ; hsit < hfingertip_torque.end() ; hsit++){
			std::vector<int> hsvec = *hsit;
			HFINGERTIP_TORQUE << '[';
			for(intit=hsvec.begin(); intit<hsvec.end()-1; intit++){HFINGERTIP_TORQUE << *intit << ", ";}
			HFINGERTIP_TORQUE << *intit << ']' << std::endl;
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
	}*/
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
