#include "data_stream.h"

//data collection vectors
std::vector< std::vector<int> > hstrain;      //hand strain measure
std::vector<std::vector< std::vector<int> > > hstrain_vec;      //hand strain measure

std::vector<double> min_hstrain(4,9999999);//running minimum values
std::vector<Hand::ct_type> ct_vector;        //cartesian torque
std::vector<std::vector<Hand::ct_type> > ct_vector_vec;        //cartesian torque
Hand::ct_type min_ct; //running minimum values
//bool collectData = false;


typedef math::Vector<24>::type v_type;

enum DATA_TYPES{
    WAMJP,
    WAMJV,
    WAMJT,
    WAMCP,
    WAMTO,
    FTSF,
    FTST,
    BHIN,
    BHOUT,
    HSTRAIN,
    HTACT,
    NUM_DATA_TYPES
};

std::string data_keys[NUM_DATA_TYPES] = {
    "WAMJP", 
    "WAMJV",
    "WAMJT",
    "WAMCP",
    "WAMTO",
    "FTSF",
    "FTST",
    "BHIN",
    "BHOUT",
    "HSTRAIN",
    "HTACT"
};


/*void DataStream::DataStream(Hand* hand_, ForceTorqueSensor* fts_, void* wamin_, ProductManager* pm_)
    hand(hand_), fts(fts_), wam((systems::Wam<DIMENSION>*)wamin_), pm(pm_){
}

void DataStream::init_stream(std::string trajectory_name)

void DataStream::loop_stream(){
}*/

std::string itoa(int number)
{
    std::stringstream ss;//create a stringstream
    ss << number;//add number to the stream
    return ss.str();//return a string with the contents of the stream
}


void data_collect(Hand* hand, ForceTorqueSensor* fts, void* wamin, ProductManager* pm, int* loop_count){
	
    systems::Wam<DIMENSION>* wam = (systems::Wam<DIMENSION>*)wamin;
	
    std::vector<systems::Wam<DIMENSION>::jp_type> jp; 	
    std::vector<std::vector<systems::Wam<DIMENSION>::jp_type> > jp_vec;
        std::vector<systems::Wam<DIMENSION>::jp_type>::iterator jpit;

    std::vector<systems::Wam<DIMENSION>::jv_type> jv; 	
    std::vector<std::vector<systems::Wam<DIMENSION>::jv_type> > jv_vec;     
        std::vector<systems::Wam<DIMENSION>::jv_type>::iterator jvit;
    
    std::vector<systems::Wam<DIMENSION>::jt_type> jt; 	
    std::vector<std::vector<systems::Wam<DIMENSION>::jt_type> > jt_vec;
        std::vector<systems::Wam<DIMENSION>::jt_type>::iterator jtit;
    
    std::vector<systems::Wam<DIMENSION>::cp_type> cp; 	
    std::vector<std::vector<systems::Wam<DIMENSION>::cp_type> > cp_vec;
        std::vector<systems::Wam<DIMENSION>::cp_type>::iterator cpit;
	
    std::vector< std::vector<double> > to; 		
    std::vector<std::vector< std::vector<double> > > to_vec; 		
        std::vector< std::vector<double> >::iterator toit;  
        std::vector<double>::iterator eigscait;
	
    std::vector<Hand::cf_type> cf; 				
    std::vector<std::vector<Hand::cf_type> > cf_vec;
        std::vector<Hand::cf_type>::iterator cfit;
	
    ct_vector.clear();	/*defined globally*/			
        std::vector<Hand::ct_type>::iterator ctit;

    std::vector<Hand::jp_type> hjp_in;	
    std::vector<std::vector<Hand::jp_type> > hjp_in_vec;	
    std::vector<Hand::jp_type> hjp_out; 
    std::vector<std::vector<Hand::jp_type> > hjp_out_vec; 
        std::vector<Hand::jp_type>::iterator hjpit; //can use for both in and out
	
    hstrain.clear(); /*defined globally*/		
        std::vector< std::vector<int> >::iterator hsit;		
        std::vector<int>::iterator intit;
	
    std::vector< std::vector<v_type> > htact;	
    std::vector<std::vector< std::vector<v_type> > > htact_vec;	
        std::vector< std::vector<v_type> >::iterator htit;	
        std::vector<v_type>::iterator vtit;
	
	std::vector<TactilePuck*> tps;
	tps = hand->getTactilePucks();
	
	std::vector<v_type> temp;
	
	std::cout << "data collection thread started!" << std::endl;

	
	//systems::Wam<DIMENSION>::jp_type exp_vars[WAM_BOTTOM];
	//parseDoubles(&exp_vars[WAM_BOTTOM], "-0.0800 -1.8072 -0.0199 0.9068 0.5583 -0.4459 0.0");
	//wam->moveTo(exp_vars[WAM_BOTTOM], false, 1.0);
    
    int curr_loop = *loop_count;
	
	while(pm->getSafetyModule()->getMode() == SafetyModule::ACTIVE){
        if(curr_loop != *loop_count){ //loop_counter was modified
            jp_vec.push_back(jp);
            jv_vec.push_back(jv);
            jt_vec.push_back(jt);
            cp_vec.push_back(cp);
            to_vec.push_back(to);
            cf_vec.push_back(cf);
            ct_vector_vec.push_back(ct_vector);
            hstrain_vec.push_back(hstrain);
            hjp_in_vec.push_back(hjp_in);
            hjp_out_vec.push_back(hjp_out);
            htact_vec.push_back(htact);

            jp.clear();
            jv.clear();
            jt.clear();
            cp.clear();
            to.clear();
            cf.clear();
            ct_vector.clear();
            hstrain.clear();
            hjp_in.clear();
            hjp_out.clear();
            htact.clear();

            std::cout << "Loop# " << *loop_count << std::endl;

            curr_loop = *loop_count;
        }

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
		std::cout << "creating " << (int)jp_vec.size() << " data files..." << std::endl; fflush(stdout);
        
        for(int i = 0; i < (int)jp_vec.size(); i++){
            //dump data to file
            std::ofstream outfiles[NUM_DATA_TYPES];
            
            std::string prefix = "data_streams/test_traj/"; 
            std::string suffix = ".m";
            
            std::cout << "opening" << prefix << "*_" << i << "..."; fflush(stdout);
            for(int j = 0; j < NUM_DATA_TYPES; j++){
                outfiles[j].open	((prefix + data_keys[j] + "_" +itoa(i) 	+ suffix).c_str(),	std::ios::trunc);
                jp=jp_vec[i];
                jv=jv_vec[i];
                jt=jt_vec[i];
                cp=cp_vec[i];
                to=to_vec[i];
                cf=cf_vec[i];
                ct_vector=ct_vector_vec[i];
                hjp_in=hjp_in_vec[i];
                hjp_out=hjp_out_vec[i];
                hstrain=hstrain_vec[i];
                htact=htact_vec[i];

                if(j==WAMJP){    
                    outfiles[j] << "data=" << "[";
                    for (jpit=jp.begin(); jpit < jp.end(); jpit++)	{outfiles[j] << *jpit << ";" << std::endl;}
                    outfiles[j] << "];";
                }
                else if(j==WAMJV){
                    outfiles[j] << "data=" << "[";
                    for (jvit=jv.begin(); jvit < jv.end(); jvit++)	{outfiles[j] << *jvit << ";" << std::endl;}
                    outfiles[j] << "];";
                }
                else if(j==WAMJT){
                    outfiles[j] << "data=" << "[";
                    for (jtit=jt.begin(); jtit < jt.end(); jtit++)	{outfiles[j] << *jtit << ";" << std::endl;}
                    outfiles[j] << "];";
                }
                else if(j==WAMCP){
                    outfiles[j] << "data=" << "[";
                    for (cpit=cp.begin(); cpit < cp.end(); cpit++)	{outfiles[j] << *cpit << ";" << std::endl;}
                    outfiles[j] << "];";
                }
                else if(j==WAMTO){ 
                    outfiles[j] << "data=[";
                    for (toit=to.begin()	  ; toit < to.end()		 ; toit++)	{
                        std::vector<double> tovec = *toit;
                        outfiles[j] << "[";
                        for(eigscait=tovec.begin(); eigscait<tovec.end()-1; eigscait++){outfiles[j] << *eigscait;}
                        outfiles[j] << *eigscait << "];" << std::endl;
                    }
                    outfiles[j] << "];";
                }
                else if(j==FTSF){
                    outfiles[j] << "data=[";
                    for (cfit=cf.begin()	  ; cfit < cf.end()		 ; cfit++)	{outfiles[j] << *cfit << ";" << std::endl; }
                    outfiles[j] << "];";
                }
                else if(j==FTST){
                    outfiles[j] << "data=[";
                    for (ctit=ct_vector.begin()	  ; ctit < ct_vector.end()      ; ctit++)	{outfiles[j] << *ctit << ";" << std::endl; }
                    outfiles[j] << "];";
                    }
                else if(j==BHIN){
                    outfiles[j] << "data=[";
                    for (hjpit=hjp_in.begin() ; hjpit < hjp_in.end() ; hjpit++) {outfiles[j] << *hjpit << ";" << std::endl;}
                    outfiles[j] << "];";
                }
                else if(j==BHOUT){
                    outfiles[j] << "data=[";
                    for (hjpit=hjp_out.begin(); hjpit < hjp_out.end(); hjpit++)	{outfiles[j] << *hjpit << ";" << std::endl;}
                    outfiles[j] << "];";
                }
                else if(j==HSTRAIN){
                    outfiles[j] << "data=[";
                    for (hsit=hstrain.begin() ; hsit < hstrain.end() ; hsit++){
                        std::vector<int> hsvec = *hsit;
                        outfiles[j] << "[";
                        for(intit=hsvec.begin(); intit<hsvec.end()-1; intit++){outfiles[j] << *intit << ", ";}
                        outfiles[j] << *intit << "];" << std::endl;
                    } 
                    outfiles[j] << "];";
                }
                else if(j==HTACT){
                        //***octave declare 3D array??***
                        outfiles[j] << "data=[";
                    for (htit=htact.begin()   ; htit < htact.end()   ; htit++){
                        std::vector<v_type> htvec = *htit;
                        outfiles[j] << "[";
                        //distill each tactile matrix into one number (element summation)
                        for(vtit=htvec.begin(); vtit<htvec.end(); vtit++){
                            v_type vec = *vtit;
                            outfiles[j] << vec.sum() << ", ";
                        }
                        outfiles[j] << "];" << std::endl;
                    }	
                    outfiles[j] << "];";
                } 
                outfiles[j].close();
            }
            std::cout << "done" << std::endl; fflush(stdout);
        }    
        std::cout << "All data written successfully" << std::endl;
    }
}
