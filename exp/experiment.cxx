#include "experiment.h"
#include "utils.h"
#include "action.h"
#include "bh.h"
#include "control.h"
#include "senses.h"

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
std::string var_keys_7[NUM_EXP_VARS_7] = {
    "WamBottom",
    "WamTop",
    "JointTolerance",
    "MiscParms"
};
std::string var_keys_4[NUM_EXP_VARS_4] = {
    "WamBottomO",
    "WamTopO",
    "HandPregrasp",
    "HandGrasp",
    "HandUngrasp"
};
std::string var_keys_3[NUM_EXP_VARS_3] = {
    "WamBottomC",
    "WamTopC"
};
std::vector<double> min_hfingertip_torque(4,9999999);//running minimum values
Experiment::Experiment(Controller* controller, Senses* senses){ 
    this->controller = controller;
    this->senses = senses;
    wam = senses->getWAM();
    hand = senses->getHand();
}

void Experiment::toggle_collect_data(){
    flag_collect_data = !flag_collect_data;
    std::cout << "Data collection toggled: " << flag_collect_data << std::endl;
}

Experiment* Experiment::get_experiment(){
	std::cout << "Running " << experiment_keys[int(exp_id)] << " Experiment..." << std::endl;
	switch(exp_id){
		case ACTIONPHASE:{      return new ActionPhase(controller, senses);}
		case ACTIVESENSING:{    return new BHTorque(controller, senses);}
		case WAMVELOCITY:{      return new BHTorque(controller, senses);}
		case WAMJOINTPOS:{      return new BHTorque(controller, senses);}
		case WAMCARTESIANPOS:{  return new BHTorque(controller, senses);}
		case WAMJOINTTORQUE:{   return new BHTorque(controller, senses);}
		case BHVELOCITY:{       return new BHTorque(controller, senses);}
		case BHPOSITION:{       return new BHTorque(controller, senses);}
		case BHTORQUE:{         return new BHTorque(controller, senses);}
		case BHTRAPEZOIDAL:{    return new BHTorque(controller, senses);}
		case SIMPLESHAPES:{     return new BHTorque(controller, senses);}
		case ACTIVEPROBING:{    return new BHTorque(controller, senses);}
		case CARTESIANRASTER:{  return new BHTorque(controller, senses);}
		default:{}
	}
}

void Experiment::teach_pose(int seqnum){
    load_exp_variables();
	if(seqnum == 0){		
        //cast wam to 7DOF first
        
        copy_matrix(&exp_vars_7[WAM_BOTTOM], wam->getJointPositions());
        copy_matrix(&exp_vars_3[WAM_BOTTOM_C], wam->getToolPosition());
        Eigen::Quaterniond wam_bottom_q =  wam->getToolOrientation();
        copy_matrix(&exp_vars_4[WAM_BOTTOM_O], quaternion2hjp(&wam_bottom_q));
        /*
        std::cout << "Setting exp_vars[WAM_BOTTOM] to " << to_string(&exp_vars[WAM_BOTTOM]) << std::endl;
        std::cout << "Setting exp_vars[WAM_BOTTOM_C] to " << to_string(&exp_vars[WAM_BOTTOM_C]) << std::endl;
        std::cout << "Setting exp_vars[WAM_BOTTOM]Q to " << to_string(&exp_vars[WAM_BOTTOM_O]) << std::endl;
        */
    }
    else if(seqnum == 1){	
        //cast wam to 7DOF first
        
        copy_matrix(&exp_vars_7[WAM_TOP], wam->getJointPositions());
        copy_matrix(&exp_vars_3[WAM_TOP_C], wam->getToolPosition());
        Eigen::Quaterniond wam_top_q =  wam->getToolOrientation();
        copy_matrix(&exp_vars_4[WAM_TOP_O], quaternion2hjp(&wam_top_q));
        /*
        std::cout << "Setting exp_vars[WAM_TOP] to " << to_string(&exp_vars[WAM_TOP]) << std::endl;
        std::cout << "Setting exp_vars[WAM_TOP_C] to " << to_string(&exp_vars[WAM_TOP_C]) << std::endl;
        std::cout << "Setting exp_vars[WAM_TOP]Q to " << to_string(&exp_vars[WAM_TOP_O]) << std::endl;
        */
    }
    save_exp_variables();
}

void Experiment::run(){
	//datasemastop = false;
	//boost::thread* dataCollectionThread = NULL;
	/*if(flag_collect_data){
		dataCollectionThread = new boost::thread(dataCollect, hand, fts, wam, pm, exp_id, expshape);
	}
    if(!is_initialized){
        std::cout << "experiment not yet initialized...aborting" << std::endl;
        return;
    }
    else{*/
        get_experiment()->run();
        //run!!
        //boost::thread* experimentThread;
        //expsemastop = false;
        //experimentThread = new boost::thread(
        //    runExperiment, EXPERIMENT_KEYS(exp_id));
        //waitForEnter();
        //expsemastop = true;
    //}
    ////`std::cout << "Experiment " << experiment_keys[int(exp_id)];
	
	/*if(!expsemastop){
		std::cout << " Completed Successfully!" << std::endl;
		datasemastop = true;
		if(flag_collect_data)
			dataCollectionThread->join();
		std::cout << "Press [Enter] to continue." << std::endl;
	}
	else{
		std::cout << " Was Interrupted!" << std::endl;
		datasemastop = true;
		if(flag_collect_data)
			dataCollectionThread->join();
	}*/	

}

void Experiment::init(std::string args){
    bool is_initialized = true;
    std::string exp_idstr = "";
    std::string exp_shapestr = "";
    std::string sub = "";
    int found_w = int(args.find(" "));
    int found_s = int(args.find("-s"));
    int found_n = int(args.find("-n"));
    int found_l = int(args.find("-l"));
    int found_t = int(args.find("-t"));
    //arg -s: shape of object to grasp
    if (found_s!=int(std::string::npos)){
        //find next whitespace or newline
        int found_tmp = int(args.find(" ",found_s+3));
        if(found_tmp==int(std::string::npos)){
            found_tmp = int(args.find("\n",found_s+3));
        }
                    
        sub = args.substr(found_s+3,found_tmp-found_s+3);
        exp_shapestr = sub;
    }
    if (found_n!=int(std::string::npos)){
        //find next whitespace or newline
        int found_tmp = int(args.find(" ",found_n+3));
        if(found_tmp==int(std::string::npos)){
            found_tmp = int(args.find("\n",found_n+3));
        }
        
        sub = args.substr(found_n+3,found_tmp-found_n+3);
        num_runs = atoi(sub.c_str());
    }
    if(found_w==int(std::string::npos)){
        found_w = int(args.find("\n"));
    }
    exp_idstr = args.substr(1,found_w-1);
    
    if(exp_idstr != ""){
        exp_id = EXPERIMENT_KEYS(atoi(exp_idstr.c_str()));
        exp_shape = EXPERIMENT_SHAPES(atoi(exp_shapestr.c_str()));
    }
    else{
        help();
    }
}

void Experiment::help(){
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

void Experiment::load_exp_variables(){

    std::cout << "loading experiment variables" << std::endl;
    
    load_exp_variables_3();
    load_exp_variables_4();
    load_exp_variables_7();
}
void Experiment::load_exp_variables_7(){

    //read parameters from file
	std::ifstream myfile ("data/exp7.dat");
	if (!myfile.is_open()){
		std::cout << "Unable to open file" << std::endl; 
		exit(1);
	}
    
    for(int i = 0; i < NUM_EXP_VARS_7; i++){
        std::string temp;
        std::getline(myfile,temp);
        parseDoubles(&exp_vars_7[i], temp);
    }
    //wam_bottom_q = hjp2quaternion(&exp_vars[WAM_BOTTOM_O]);
    //wam_top_q    = hjp2quaternion(&exp_vars[WAM_TOP_O]);
}
void Experiment::load_exp_variables_4(){

    //read parameters from file
	std::ifstream myfile ("data/exp4.dat");
	if (!myfile.is_open()){
		std::cout << "Unable to open file" << std::endl; 
		exit(1);
	}
    
    for(int i = 0; i < NUM_EXP_VARS_4; i++){
        std::string temp;
        std::getline(myfile,temp);
        parseDoubles(&exp_vars_4[i], temp);
    }
    //wam_bottom_q = hjp2quaternion(&exp_vars[WAM_BOTTOM_O]);
    //wam_top_q    = hjp2quaternion(&exp_vars[WAM_TOP_O]);
}
void Experiment::load_exp_variables_3(){

    //read parameters from file
	std::ifstream myfile ("data/exp3.dat");
	if (!myfile.is_open()){
		std::cout << "Unable to open file" << std::endl; 
		exit(1);
	}
    
    for(int i = 0; i < NUM_EXP_VARS_3; i++){
        std::string temp;
        std::getline(myfile,temp);
        parseDoubles(&exp_vars_3[i], temp);
    }
    //wam_bottom_q = hjp2quaternion(&exp_vars[WAM_BOTTOM_O]);
    //wam_top_q    = hjp2quaternion(&exp_vars[WAM_TOP_O]);
}

void Experiment::save_exp_variables(){	

    std::cout << "saving experiment variables" << std::endl;

    save_exp_variables_3();
    save_exp_variables_4();
    save_exp_variables_7();
	
}
void Experiment::save_exp_variables_7(){
    //read parameters from file
	std::ofstream myfile ("data/exp7.dat");
	if (!myfile.is_open()){
		std::cout << "Unable to open file" << std::endl; 
		exit(1);
	}

	//exp_vars[WAM_BOTTOM_O] = quaternion2hjp(&wam_bottom_q);
    //exp_vars[WAM_TOP_O] = quaternion2hjp(&wam_top_q);
	//save paramater vectors
    for(int i = 0; i < NUM_EXP_VARS_7; i++){
        myfile << to_string(&exp_vars_7[i]);
        //std::cout << "saving " << var_keys[i] << " as " << to_string(&exp_vars[i]) << std::endl;
    }
}
void Experiment::save_exp_variables_4(){
    //read parameters from file
	std::ofstream myfile ("data/exp4.dat");
	if (!myfile.is_open()){
		std::cout << "Unable to open file" << std::endl; 
		exit(1);
	}

	//exp_vars[WAM_BOTTOM_O] = quaternion2hjp(&wam_bottom_q);
    //exp_vars[WAM_TOP_O] = quaternion2hjp(&wam_top_q);
	//save paramater vectors
    for(int i = 0; i < NUM_EXP_VARS_4; i++){
        myfile << to_string(&exp_vars_4[i]);
        //std::cout << "saving " << var_keys[i] << " as " << to_string(&exp_vars[i]) << std::endl;
    }
}
void Experiment::save_exp_variables_3(){
    //read parameters from file
	std::ofstream myfile ("data/exp3.dat");
	if (!myfile.is_open()){
		std::cout << "Unable to open file" << std::endl; 
		exit(1);
	}

	//exp_vars[WAM_BOTTOM_O] = quaternion2hjp(&wam_bottom_q);
    //exp_vars[WAM_TOP_O] = quaternion2hjp(&wam_top_q);
	//save paramater vectors
    for(int i = 0; i < NUM_EXP_VARS_3; i++){
        myfile << to_string(&exp_vars_3[i]);
        //std::cout << "saving " << var_keys[i] << " as " << to_string(&exp_vars[i]) << std::endl;
    }
}

void Experiment::data_collect(){
    /*
	std::vector<jp_type> jp; 	std::vector<jp_type>::iterator jpit;
	std::vector<jv_type> jv; 	std::vector<jv_type>::iterator jvit;
	std::vector<jt_type> jt; 	std::vector<jt_type>::iterator jtit;
	std::vector<systems::Wam<DIMENSION>::cp_type> cp; 	std::vector<systems::Wam<DIMENSION>::cp_type>::iterator cpit;
	std::vector< std::vector<double> > to; 		std::vector< std::vector<double> >::iterator toit;  std::vector<double>::iterator eigscait;
	std::vector<Hand::cf_type> cf; 				std::vector<Hand::cf_type>::iterator cfit;
	ct_vector.clear();						std::vector<Hand::ct_type>::iterator ctit;
	std::vector<Hand::jp_type> hjp_in; 			std::vector<Hand::jp_type>::iterator hjpit; //can use for both in and out
	std::vector<Hand::jp_type> hjp_out;
	hfingertip_torque.clear(); 				std::vector< std::vector<int> >::iterator hsit;		std::vector<int>::iterator intit;
	std::vector< std::vector<TactilePuck::v_type> > htact;	std::vector< std::vector<TactilePuck::v_type> >::iterator htit;	std::vector<TactilePuck::v_type>::iterator vtit;
	
	std::vector<TactilePuck*> tps;
	tps = hand->getTactilePucks();
	
	std::vector<TactilePuck::v_type> temp;
	
	std::cout << "data collection thread started!" << std::endl;
	
	//systems::Wam<DIMENSION>::jp_type exp_vars[WAM_BOTTOM];
	//parseDoubles(&exp_vars[WAM_BOTTOM], "-0.0800 -1.8072 -0.0199 0.9068 0.5583 -0.4459 0.0");
	//wam->moveTo(exp_vars[WAM_BOTTOM], false, 1.0);
	
	while(pm->getSafetyModule()->getMode() == SafetyModule::ACTIVE){//datasemastop){
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
		ct_vector.push_back(_ct);
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
								experiment_keys[  int(exp_id  )]+ "/" +
								experiment_shapes[int(exp_shape)]+ "/" ;
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
		for (ctit=ct_vector.begin()	  ; ctit < ct_vector.end()      ; ctit++)	{FTST << *ctit << std::endl; }
		for (hjpit=hjp_in.begin() ; hjpit < hjp_in.end() ; hjpit++) {BHIN << *hjpit << std::endl;}
		for (hjpit=hjp_out.begin(); hjpit < hjp_out.end(); hjpit++)	{BHOUT << *hjpit << std::endl;}
		for (hsit=hfingertip_torque.begin() ; hsit < hfingertip_torque.end() ; hsit++){
			std::vector<int> hsvec = *hsit;
			HFINGERTIP_TORQUE << '[';
			for(intit=hsvec.begin(); intit<hsvec.end()-1; intit++){HFINGERTIP_TORQUE << *intit << ", ";}
			HFINGERTIP_TORQUE << *intit << ']' << std::endl;
		}
		for (htit=htact.begin()   ; htit < htact.end()   ; htit++){
			std::vector<TactilePuck::v_type> htvec = *htit;
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
	}

    */
}

