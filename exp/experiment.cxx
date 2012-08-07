#include "experiment.h"
#include "utils.h"

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

Experiment::Experiment(systems::Wam<DIMENSION>* wam, Hand* hand, ForceTorqueSensor* fts, ProductManager* pm){
    this->wam = wam;
    this->hand = hand;
    this->fts = fts;
    this->pm = pm;
}
void Experiment::run(){
    if(!is_initialized){
        std::cout << "experiment not yet initialized...aborting" << std::endl;
        return;
    }
    else{
        //run!!
        //boost::thread* experimentThread;
        //expsemastop = false;
        //experimentThread = new boost::thread(
        //    runExperiment, EXPERIMENT_KEYS(exp_id));
        //waitForEnter();
        //expsemastop = true;
    }

}

void Experiment::init(std::string args){
    bool is_initialized = true;
    std::string exp_idstr = "";
    std::string expshapestr = "";
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
        expshapestr = sub;
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
        exp_shape = EXPERIMENT_SHAPES(atoi(expshapestr.c_str()));     
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
	//std::getline (myfile,tact_base_valStr);
	//std::getline (myfile,torque_epsilonStr);
	std::getline (myfile,joint_toleranceStr);
	std::getline (myfile,misc_parmsStr);
	
	//parse paramater vectors
	parseDoubles(&wamBottom, wamBottomStr);
	parseDoubles(&wamTop, wamTopStr);
	parseDoubles(&wamBottomC, wamBottomCStr);
	parseDoubles(&wamTopC, wamTopCStr);
	parseDoubles(&wamBottomO, wamBottomOStr); 
        wamBottomQ = hjp2quaternion(&wamBottomO);
	parseDoubles(&wamTopO, wamTopOStr); 
        wamTopQ = hjp2quaternion(&wamTopO);
	parseDoubles(&handPregrasp, handPregraspStr);
	parseDoubles(&handGrasp, handGraspStr);
	parseDoubles(&handUnGrasp, handUnGraspStr);
	//parseDoubles(&tact_base_val, tact_base_valStr);
	//parseDoubles(&torque_epsilon, torque_epsilonStr);
	parseDoubles(&joint_tolerance, joint_toleranceStr);
	parseDoubles(&misc_parms, misc_parmsStr);
}
void Experiment::save_exp_variables(){	
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
	//myfile << toString(&tact_base_val);
	//myfile << toString(&torque_epsilon);
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
