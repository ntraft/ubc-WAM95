#include "io.h"

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


//experiment data loading is still too tightly coupled with main.cpp, will migrate it here at a later time
/*
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
*/
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
	ct_vector.clear();	/*defined globally*/			std::vector<Hand::ct_type>::iterator ctit;
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
}
