#include "memory.h"
/*
//data collection vectors
std::vector< std::vector<int> > hstrain;      //hand strain measure
std::vector<double> min_hstrain(4,9999999);//running minimum values
std::vector<Hand::ct_type> ct_vector;        //cartesian torque
Hand::ct_type min_ct; //running minimum values
//bool collectData = false;

typedef math::Vector<24>::type v_type;

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
*/

//experiment data loading is still too tightly coupled with main.cpp, will migrate it here at a later time
/*
void loadExpVariables(){
	std::string exp_vars[WAM_BOTTOM]Str;
	std::string exp_vars[WAM_TOP]Str;
	std::string exp_vars[WAM_BOTTOM_C]Str;
	std::string exp_vars[WAM_TOP_C]Str;
	std::string exp_vars[WAM_BOTTOM_O]Str;
	std::string exp_vars[WAM_TOP_O]Str;
	std::string exp_vars[HAND_PREGRASP]Str;
	std::string exp_vars[HAND_GRASP]Str;
	std::string exp_vars[HAND_UNGRASP]Str;
	std::string exp_vars[TACT_BASE_VAL]Str;
	std::string exp_vars[TORQUE_EPSILON]Str;
	std::string exp_vars[JOINT_TOLERANCE]Str;
	std::string exp_vars[MISC_PARMS]Str;
	
	//read parameters from file
	std::ifstream myfile ("in.txt");
	if (!myfile.is_open()){
		std::cout << "Unable to open file"; 
		exit(1);
	}
	std::getline (myfile,exp_vars[WAM_BOTTOM]Str);
	std::getline (myfile,exp_vars[WAM_TOP]Str);
	std::getline (myfile,exp_vars[WAM_BOTTOM_C]Str);
	std::getline (myfile,exp_vars[WAM_TOP_C]Str);
	std::getline (myfile,exp_vars[WAM_BOTTOM_O]Str);
	std::getline (myfile,exp_vars[WAM_TOP_O]Str);
	std::getline (myfile,exp_vars[HAND_PREGRASP]Str);
	std::getline (myfile,exp_vars[HAND_GRASP]Str);
	std::getline (myfile,exp_vars[HAND_UNGRASP]Str);
	std::getline (myfile,exp_vars[TACT_BASE_VAL]Str);
	std::getline (myfile,exp_vars[TORQUE_EPSILON]Str);
	std::getline (myfile,exp_vars[JOINT_TOLERANCE]Str);
	std::getline (myfile,exp_vars[MISC_PARMS]Str);
	
	//parse paramater vectors
	parseDoubles(&exp_vars[WAM_BOTTOM], exp_vars[WAM_BOTTOM]Str);
	parseDoubles(&exp_vars[WAM_TOP], exp_vars[WAM_TOP]Str);
	parseDoubles(&exp_vars[WAM_BOTTOM_C], exp_vars[WAM_BOTTOM_C]Str);
	parseDoubles(&exp_vars[WAM_TOP_C], exp_vars[WAM_TOP_C]Str);
	parseDoubles(&exp_vars[WAM_BOTTOM_O], exp_vars[WAM_BOTTOM_O]Str); exp_vars[WAM_BOTTOM]Q = hjp2quaternion(&exp_vars[WAM_BOTTOM_O]);
	parseDoubles(&exp_vars[WAM_TOP_O], exp_vars[WAM_TOP_O]Str); exp_vars[WAM_TOP]Q = hjp2quaternion(&exp_vars[WAM_TOP_O]);
	parseDoubles(&exp_vars[HAND_PREGRASP], exp_vars[HAND_PREGRASP]Str);
	parseDoubles(&exp_vars[HAND_GRASP], exp_vars[HAND_GRASP]Str);
	parseDoubles(&exp_vars[HAND_UNGRASP], exp_vars[HAND_UNGRASP]Str);
	parseDoubles(&exp_vars[TACT_BASE_VAL], exp_vars[TACT_BASE_VAL]Str);
	parseDoubles(&exp_vars[TORQUE_EPSILON], exp_vars[TORQUE_EPSILON]Str);
	parseDoubles(&exp_vars[JOINT_TOLERANCE], exp_vars[JOINT_TOLERANCE]Str);
	parseDoubles(&exp_vars[MISC_PARMS], exp_vars[MISC_PARMS]Str);
}
void saveExpVariables(){	
	//read parameters from file
	std::ofstream myfile ("in.txt");
	if (!myfile.is_open()){
		std::cout << "Unable to open file" << std::endl; 
		exit(1);
	}

	//save paramater vectors
	myfile << to_string(&exp_vars[WAM_BOTTOM]);
	myfile << to_string(&exp_vars[WAM_TOP]);
	myfile << to_string(&exp_vars[WAM_BOTTOM_C]);
	myfile << to_string(&exp_vars[WAM_TOP_C]);
	exp_vars[WAM_BOTTOM_O] = quaternion2hjp(&exp_vars[WAM_BOTTOM]Q);
	myfile << to_string(&exp_vars[WAM_BOTTOM_O]);
	exp_vars[WAM_TOP_O] = quaternion2hjp(&exp_vars[WAM_TOP]Q);
	myfile << to_string(&exp_vars[WAM_TOP_O]);
	myfile << to_string(&exp_vars[HAND_PREGRASP]);
	myfile << to_string(&exp_vars[HAND_GRASP]);
	myfile << to_string(&exp_vars[HAND_UNGRASP]);
	myfile << to_string(&exp_vars[TACT_BASE_VAL]);
	myfile << to_string(&exp_vars[TORQUE_EPSILON]);
	myfile << to_string(&exp_vars[JOINT_TOLERANCE]);
	myfile << to_string(&exp_vars[MISC_PARMS]);

#if 0
	std::cout << "saving exp_vars[WAM_BOTTOM]      as " << to_string(&exp_vars[WAM_BOTTOM]) 	  << std::endl;
	std::cout << "saving exp_vars[WAM_TOP]         as " << to_string(&exp_vars[WAM_TOP]) 		  << std::endl;
	std::cout << "saving exp_vars[WAM_BOTTOM_C]     as " << to_string(&exp_vars[WAM_BOTTOM_C]) 	  << std::endl;
	std::cout << "saving exp_vars[WAM_TOP_C]        as " << to_string(&exp_vars[WAM_TOP_C]) 		  << std::endl;
	std::cout << "saving exp_vars[WAM_BOTTOM_O]     as " << to_string(&exp_vars[WAM_BOTTOM_O]) 	  << std::endl;
	std::cout << "saving exp_vars[WAM_TOP_O]        as " << to_string(&exp_vars[WAM_TOP_O]) 		  << std::endl;
	std::cout << "saving exp_vars[HAND_PREGRASP]   as " << to_string(&exp_vars[HAND_PREGRASP])   << std::endl;
	std::cout << "saving exp_vars[HAND_GRASP]      as " << to_string(&exp_vars[HAND_GRASP]) 	  << std::endl;
	std::cout << "saving exp_vars[HAND_UNGRASP]    as " << to_string(&exp_vars[HAND_UNGRASP]) 	  << std::endl;
	std::cout << "saving exp_vars[TACT_BASE_VAL]  as " << to_string(&exp_vars[TACT_BASE_VAL])  << std::endl;
	std::cout << "saving exp_vars[TORQUE_EPSILON] as " << to_string(&exp_vars[TORQUE_EPSILON]) << std::endl;
	std::cout << "saving exp_vars[JOINT_TOLERANCE]as " << to_string(&exp_vars[JOINT_TOLERANCE])<< std::endl;
#endif
}
*/
/*
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
	ct_vector.clear();	std::vector<Hand::ct_type>::iterator ctit;
	std::vector<Hand::jp_type> hjp_in; 			std::vector<Hand::jp_type>::iterator hjpit; //can use for both in and out
	std::vector<Hand::jp_type> hjp_out;
	hstrain.clear(); std::vector< std::vector<int> >::iterator hsit;		std::vector<int>::iterator intit;
	std::vector< std::vector<v_type> > htact;	std::vector< std::vector<v_type> >::iterator htit;	std::vector<v_type>::iterator vtit;
	
	std::vector<TactilePuck*> tps;
	tps = hand->getTactilePucks();
	
	std::vector<v_type> temp;
	
	std::cout << "data collection thread started!" << std::endl;
	
	//systems::Wam<DIMENSION>::jp_type exp_vars[WAM_BOTTOM];
	//parseDoubles(&exp_vars[WAM_BOTTOM], "-0.0800 -1.8072 -0.0199 0.9068 0.5583 -0.4459 0.0");
	//wam->moveTo(exp_vars[WAM_BOTTOM], false, 1.0);
	
	while(pm->getSafetyModule()->getMode() == SafetyModule::ACTIVE){
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
		for (ctit=ct_vector.begin()	  ; ctit < ct_vector.end()      ; ctit++)	{FTST << *ctit << std::endl; }
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
}*/
