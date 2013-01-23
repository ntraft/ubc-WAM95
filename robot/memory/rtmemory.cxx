#include "rtmemory.h"
#include "memory.h"
#include "senses.h"
#include "sensor_stream_system.h"
#include "rtcontrol.h"
#include "hand_system.cxx"
#include "wam_system.cxx"
#include "utils.h"

int recordType = 0;
enum SENSORS{
    enum_time,
#define X(aa, bb, cc, dd, ee) enum_##cc,
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
    NUM_SENSORS
};

RTMemory::RTMemory(ProductManager* _pm, Wam<DIMENSION>* _wam, Memory* _memory, Senses* _senses, RobotController* _control) :
			//pm(_pm), wam(_wam), senses(_senses),
            playName("recorded/test.csv"), inputType(0), saveName("test"),
            //cpVec(NULL), qVec(NULL), 
            jpSpline( NULL), cpSpline(NULL), qSpline(NULL), 
            jpTrajectory(NULL), cpTrajectory( NULL), qTrajectory(NULL), 
            time(_pm->getExecutionManager()), dataSize( 0), loop(false), 
            tmpStr("/tmp/btXXXXXX")
{ 
        pm = _pm;
        wam = _wam;
        memory = _memory;
        senses = _senses;
        control= _control;
        log_prefix = "./data_streams/";
        data_log_headers = 
            "time"
            ",joint_pos_0,joint_pos_1,joint_pos_2,joint_pos_3,joint_pos_4,joint_pos_5,joint_pos_6,joint_pos_7"
            ",joint_vel_0,joint_vel_1,joint_vel_2,joint_vel_3,joint_vel_4,joint_vel_5,joint_vel_6,joint_vel_7"
            ",joint_tor_0,joint_tor_1,joint_tor_2,joint_tor_3,joint_tor_4,joint_tor_5,joint_tor_6,joint_tor_7"
            ",cart_pos_0,cart_pos_1,cart_pos_2,cart_pos_3"
            ",cart_ori_0,cart_ori_1,cart_ori_2,cart_ori_3"
            ",ft_torque_0,ft_torque_1,ft_torque_2,ft_torque_3"
            ;
        
        jpVec = new std::vector<input_jp_type, Eigen::aligned_allocator<input_jp_type> >();
        jpSample= new input_jp_type();
#define X(aa, bb, cc, dd, ee) vec_##cc = new std::vector<input_type_##cc, Eigen::aligned_allocator<input_type_##cc> >();
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) sample_##cc = new input_type_##cc();
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
//#define X(aa, bb, cc, dd, ee) trajectory_##cc = new systems::Callback<double,bb>(boost::ref(*spline_##cc));
//    #include "wam_type_table.h"
//    #include "tool_type_table.h"
//#undef X
        cout << "RTMemory instantiated!" << endl;
}
void RTMemory::init(){
    //hand system deals with realtime sensor reading
    //hand_system = new HandSystem(hand,&problem,&hand_debug);
    wam_system = new WamSystem(wam);
    sss = new SensorStreamSystem(senses);
    rtc = new RTControl(&rtc_debug, 
#define X(aa, bb, cc, dd, ee) &problem_count_##cc,
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X 
            memory, senses, control);
	
    pm->getExecutionManager()->startManaging(time); //starting time management
    cout << "RTMemory initialized!" << endl;
}

//*********TEACH*********
bool RTMemory::prepare_log_file(){
	tmpFile = new char[tmpStr.length() + 1];
	strcpy(tmpFile, tmpStr.c_str());
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return false;
	}
	if (recordType == 0){
		jpLogger = new systems::PeriodicDataLogger<jp_sample_type>(
				pm->getExecutionManager(),
				new barrett::log::RealTimeWriter<jp_sample_type>(tmpFile,
						pm->getExecutionManager()->getPeriod()), 1);
    }
	else{
		poseLogger = new systems::PeriodicDataLogger<pose_sample_type>(
				pm->getExecutionManager(),
				new barrett::log::RealTimeWriter<pose_sample_type>(tmpFile,
						pm->getExecutionManager()->getPeriod()), 1);
    }
	return true;
}
void RTMemory::record(){
	BARRETT_SCOPED_LOCK(pm->getExecutionManager()->getMutex());
    cout << "recording trajectory as ";
	if (recordType == 0) {
        cout << "jp_type";
		connect(time.output, jpLogTg.getInput<0>());
		connect(wam->jpOutput, jpLogTg.getInput<1>());
		connect(jpLogTg.output, jpLogger->input);
	} else {
        cout << "pose_type";
		connect(time.output, poseLogTg.getInput<0>());
		connect(wam->toolPose.output, poseLogTg.getInput<1>());
		connect(poseLogTg.output, poseLogger->input);
	}
    cout << endl;
	time.start();
}
void RTMemory::create_spline(){
	saveName = "recorded/" + saveName;
	if (recordType == 0) {
		jpLogger->closeLog();
		disconnect(jpLogger->input);
		// Build spline between recorded points
		log::Reader<jp_sample_type> lr(tmpFile);
		lr.exportCSV(saveName.c_str());
	}
	else{
		poseLogger->closeLog();
		disconnect(poseLogger->input);
		log::Reader<pose_sample_type> pr(tmpFile);
		pr.exportCSV(saveName.c_str());
	}
	// Adding our datatype as the first line of the recorded trajectory
	fileOut = saveName + ".csv";
	std::ifstream in(saveName.c_str());
	std::ofstream out(fileOut.c_str());
	if (recordType == 0)
		out << "jp_type\n";
	else
		out << "pose_type\n";
	out << in.rdbuf();
	out.close();
	in.close();
	remove(saveName.c_str());
	printf("Trajectory saved to the location: %s \n\n ", fileOut.c_str());
}
//**********************
//*********PLAY*********
bool RTMemory::load_trajectory(){
    
   // std::string filename(argv[1]);
    // Load our vc_calibration file.
    /*
	libconfig::Config config;
	std::string calibration_file;
    calibration_file = "calibration7.conf";
	config.readFile(calibration_file.c_str());
	config.getRoot();
    */
    //Create stream from input file
	std::ifstream fs(playName.c_str());
	std::string line;
	// Check to see the data type specified on the first line (jp_type or pose_type)
	// this will inform us if we are tracking 4DIMENSION WAM Joint Angles, 7DIMENSION WAM Joint Angles, or WAM Poses
	std::getline(fs, line);
	// Using a boost tokenizer to parse the data of the file into our vector.
	boost::char_separator<char> sep(",");
	typedef boost::tokenizer<boost::char_separator<char> > t_tokenizer;
	t_tokenizer tok(line, sep);
	if (strcmp(line.c_str(), "pose_type") == 0) {
        /*
		// Create our spline and trajectory if the first line of the parsed file informs us of a pose_type
		inputType = 1;
		float fLine[8];

		//input_cp_type *sample_cp;
		//input_qd_type *sample_qd;
		while (true) {
			std::getline(fs, line);
			if (!fs.good())
				break;
			t_tokenizer tok(line, sep);
			int j = 0;
			for (t_tokenizer::iterator beg = tok.begin(); beg != tok.end();
					++beg) {
				fLine[j] = boost::lexical_cast<float>(*beg);
				j++;
			}
			boost::get<0>(*sample_cp) = fLine[0];
			boost::get<0>(*sample_qd) = boost::get<0>(*sample_cp);

			boost::get<1>(*sample_cp) << fLine[1], fLine[2], fLine[3];
			boost::get<1>(*sample_qd) = Eigen::Quaterniond(fLine[4], fLine[5],
					fLine[6], fLine[7]);
			boost::get<1>(*sample_qd).normalize();
			vec_cp->push_back(*sample_cp);
			vec_qd->push_back(*sample_qd);
		}
		// Make sure the vectors created are the same size
		assert(vec_cp->size() == vec_qd->size());
		// Create our splines between points
		cpSpline = new math::Spline<cp_type>(*vec_cp);
		qSpline = new math::Spline<Eigen::Quaterniond>(*vec_qd);
		// Create trajectories from the splines
		cpTrajectory = new systems::Callback<double, cp_type>(
				boost::ref(*cpSpline));
		qTrajectory = new systems::Callback<double, Eigen::Quaterniond>(
				boost::ref(*qSpline));*/
	} else if (strcmp(line.c_str(), "jp_type") == 0) {
		// Create our spline and trajectory if the first line of the parsed file informs us of a jp_type
		//std::vector<input_jp_type, Eigen::aligned_allocator<input_jp_type> > vec_jp;
		float fLine[8];
		//input_type_jp samp;
		while (true) {
			std::getline(fs, line);
			if (!fs.good())
				break;
			t_tokenizer tok(line, sep);
			int j = 0;
			for (t_tokenizer::iterator beg = tok.begin(); beg != tok.end();
					++beg) {
				fLine[j] = boost::lexical_cast<float>(*beg);
				j++;
			}
			boost::get<0>(*jpSample) = fLine[0];
			// To handle the different WAM configurations
			if (j == 5)
				boost::get<1>(*jpSample) << fLine[1], fLine[2], fLine[3], fLine[4];
			else
				boost::get<1>(*jpSample) << fLine[1], fLine[2], fLine[3], fLine[4], fLine[5], fLine[6], fLine[7];
			jpVec->push_back(*jpSample);
		}
		// Create our splines between points
		jpSpline= new math::Spline<systems::Wam<DIMENSION>::jp_type>(*jpVec);
		// Create our trajectory
		jpTrajectory= new systems::Callback<double, systems::Wam<DIMENSION>::jp_type>(boost::ref(*jpSpline));
	} else {
		// The first line does not contain "jp_type or pose_type" return false and exit.
		printf(
				"EXITING: First line of file must specify jp_type or pose_type data, found %s\n",line.c_str());
		btsleep(1.5);
		return false;
	}
	
    //Close the file
	fs.close();


	printf("\nFile Contains data in the form of: %s\n\n",
			inputType == 0 ? "jp_type" : "pose_type");
    return true;
}

//gets called at the start of each loop
 void RTMemory::init_data_logger(){
	char* ptmpFile = new char[tmpStr.length() + 1];
	strcpy(ptmpFile, tmpStr.c_str());
	if (mkstemp(ptmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
	}
    //set up realtime data logging
    char tmp_filename_template[] = "/tmp/btXXXXXX";
    int tmp_file_descriptor;
    
	if ((tmp_file_descriptor = mkstemp(tmp_filename_template)) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
	}
    
    //get temporary filename created by system
    char tmp_filename_buf[14];
    std::string read_str = "/proc/self/fd/" + boost::lexical_cast<std::string>(tmp_file_descriptor); 
    readlink(read_str.c_str(),tmp_filename_buf,14);
    std::string tmp_filename(tmp_filename_buf); 
	tmp_filenames.push_back(tmp_filename);

    //if(loop_count > 0)
    //    logger->closeLog();
    const size_t PERIOD_MULTIPLIER = 1;
    logger = new systems::PeriodicDataLogger<input_stream_type> (
			pm->getExecutionManager(),
			new log::RealTimeWriter<input_stream_type>(
                (char*)tmp_filename.c_str(), PERIOD_MULTIPLIER * pm->getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);
    /*plogger = new systems::PeriodicDataLogger<pinput_stream_type> (
			pm->getExecutionManager(),
			new log::RealTimeWriter<pinput_stream_type>(
                ptmpFile, PERIOD_MULTIPLIER * pm->getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);*/
    cout << "Data logger initialized!" << endl;
}

//export to csv files
void RTMemory::output_data_stream(){
    logger->closeLog(); //clost outstanding log
    //plogger->closeLog(); //clost outstanding log
    //save headers for data log of entire trajectory
    std::string log_name = playName.substr(9,playName.length()-4-9); //strip recorded/ and .csv from playName
    std::string header_filename = log_prefix+log_name+".h";
    std::ofstream out(header_filename.c_str());
    out << data_log_headers << std::endl;
    out.close();
    int counter = 0;
    std::vector<std::string>::iterator tmp_filename_it;	
    for (tmp_filename_it=tmp_filenames.begin(); tmp_filename_it < tmp_filenames.end(); tmp_filename_it++){
        std::string tmp_filename = *tmp_filename_it;
        std::string out_filename = log_prefix+itoa(counter++)+log_name+".csv";
        log::Reader<input_stream_type> lr((tmp_filename).c_str());
        lr.exportCSV(out_filename.c_str()); 
        std::remove((tmp_filename).c_str());
        printf("Data log saved to the location: %s \n", out_filename.c_str());
    }
    std::cout <<  "All data logs saved successfully!" << std::endl;
}

void RTMemory::load_data_stream(bool mean){
    //Create stream from input file
    std::ifstream* fs;
    if(mean){
        fs = new std::ifstream("data_streams/mean_test.csv");
        cout << "loading data mean" << endl;
    }
    else{
        fs = new std::ifstream("data_streams/std_test.csv");
        cout << "loading data std " << endl;
    }
	std::string line;
	typedef boost::tokenizer<boost::char_separator<char> > t_tokenizer;
	boost::char_separator<char> sep(",");
    
    float fLine[STREAM_SIZE];
    //float count = 0.002;
    while (true) {
        std::getline(*fs, line);
        if (!fs->good())
            break;
        int fLine_i = 1; //skip time column
        
        t_tokenizer tok(line, sep);
        int j = 0;
        for (t_tokenizer::iterator beg = tok.begin(); beg != tok.end(); ++beg) {
            fLine[j] = boost::lexical_cast<float>(*beg);
            j++;
        }
#define X(aa, bb, cc, dd, ee) \
        boost::get<0>(*sample_##cc) = fLine[0];\
        bb cc; \
        for(int i = 0; i < cc.size(); i++){\
            cc[i] = fLine[fLine_i++];\
        } \
        boost::get<1>(*sample_##cc)=cc;\
        //cout<<"got "<<aa<<" at time "<<fLine[0]<<endl;fflush(stdout); 
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) vec_##cc->push_back(*sample_##cc); 
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
    }
    //Create our splines between points
#define X(aa, bb, cc, dd, ee) spline_##cc = new math::Spline<bb>(*vec_##cc); 
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X

#define X(aa, bb, cc, dd, ee) vec_##cc->clear();
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
    cout << "\tSpline created!" << endl;
    
    if(mean){
#define X(aa, bb, cc, dd, ee) mean_trajectory_##cc = new systems::Callback<double, bb>(boost::ref(*spline_##cc)); 
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
    }
    else{
#define X(aa, bb, cc, dd, ee) std_trajectory_##cc = new systems::Callback<double, bb>(boost::ref(*spline_##cc)); 
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
    }
    cout << "\tTrajectory created!" << endl;
	fs->close();
}
void RTMemory::start_playback() {time.start();}
void RTMemory::pause_playback() {time.stop();}
bool RTMemory::playback_active(){
    bool active;
	if (inputType == 0){
		active = (jpTrajectory->input.getValue() < jpSpline->finalS());
    }
	else {
		active = (jpTrajectory->input.getValue() < cpSpline->finalS());
	}
    if(!active)
        cout << "playback active: " << active << endl;
    return active;
}
void RTMemory::disconnect_systems(){
	disconnect(wam->input);
    disconnect(logger->input);
    //disconnect(plogger->input);
    disconnect(sss->time_input);
    disconnect(tg.getInput<0>());
#define X(aa, bb, cc, dd, ee) disconnect(tg.getInput<ee>());
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
    disconnect(tg.getInput<NUM_SENSORS>());
	disconnect(rtc->input_time);
#define X(aa, bb, cc, dd, ee) disconnect(rtc->mean_input_##cc);
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) disconnect(rtc->std_input_##cc);
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) disconnect(rtc->actual_input_##cc);
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
    /*
*/
   /* 
    disconnect(tgp.getInput<0>());
#define X(aa, bb, cc, dd, ee) disconnect(tgp.getInput<ee>());
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
    disconnect(tgp.getInput<NUM_SENSORS>());
*/
    wam->idle();
	time.stop();
	time.setOutput(0.0);
    cout << "Systems disconnected! " << endl;
}
void RTMemory::reconnect_systems(){
    wam_system->init();

	if (inputType == 0) {
		//systems::forceConnect(time.output, jpTrajectory->input);
		//systems::forceConnect(jpTrajectory->output, wam_system->input);
	} else {
		systems::forceConnect(time.output, cpTrajectory->input);
		systems::forceConnect(time.output, qTrajectory->input);
		systems::forceConnect(cpTrajectory->output, poseTg.getInput<0>());
		systems::forceConnect(qTrajectory->output, poseTg.getInput<1>());
		wam->trackReferenceSignal(poseTg.output);
	}
    systems::forceConnect(time.output, sss->time_input);
	systems::forceConnect(time.output, rtc->input_time);
#define X(aa, bb, cc, dd, ee) systems::forceConnect(time.output, mean_trajectory_##cc->input);
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) systems::forceConnect(time.output, std_trajectory_##cc->input);
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) systems::forceConnect(mean_trajectory_##cc->output, rtc->mean_input_##cc);
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) systems::forceConnect(std_trajectory_##cc->output, rtc->std_input_##cc);
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) systems::forceConnect(sss->output_##cc, rtc->actual_input_##cc);
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
/*
*/
	systems::forceConnect(time.output,                tg.getInput<0>());
	//systems::forceConnect(rtc->output_time,             tg.getInput<0>());
#define X(aa, bb, cc, dd, ee) systems::forceConnect(sss->output_##cc, tg.getInput<ee>());
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
	//systems::forceConnect(time.output,                tg.getInput<NUM_SENSORS>());
	systems::forceConnect(rtc->output_jp,             tg.getInput<NUM_SENSORS>());
    wam->trackReferenceSignal(rtc->output_jp);
    //cout << "NUM_SENSORS: " << NUM_SENSORS << endl;
/*    systems::forceConnect(time.output,                tgp.getInput<0>());
#define X(aa, bb, cc, dd, ee) systems::forceConnect(rtc->output_##cc, tgp.getInput<ee>());
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
	systems::forceConnect(rtc->output_time,                tgp.getInput<NUM_SENSORS>());
*/
    systems::forceConnect(tg.output, logger->input);
    //systems::forceConnect(tgp.output, plogger->input); 
    //hand_system->time_count = 0;
	time.start();
	printf("Logging started.\n");
}
jp_type RTMemory::get_initial_jp(){
    return jpSpline->eval(jpSpline->initialS());
}
pose_type RTMemory::get_initial_tp(){
    return boost::make_tuple(cpSpline->eval(cpSpline->initialS()),
            qSpline->eval(qSpline->initialS()));
}
//checks all entries in src and dest and adds 1 to each entry where mat1 is less than mat2 
template<int R, int C, typename Units>
void check_greater_than_reset(math::Matrix<R,C, Units>* to_check, double limit, string output_string){
    for (int i = 0; i < to_check->size(); ++i) {
        if((*to_check)[i] > limit){
            (*to_check)[i] = 0;
        }
    }
}
//checks all entries in src and dest and adds 1 to each entry where mat1 is less than mat2 
template<int R, int C, typename Units>
void check_greater_than_reset_print(math::Matrix<R,C, Units>* to_check, double limit, string output_string){
    for (int i = 0; i < to_check->size(); ++i) {
        if((*to_check)[i] > limit){
            cout << output_string << "[" << i << "]" << endl;
            (*to_check)[i] = 0;
        }
    }
}
void RTMemory::check_for_problems(){
    if(strcmp(rtc_debug.str().c_str(),"")!=0)
        cout << "rtcdbg: " << endl << rtc_debug.str() << endl;
    float threshold = memory->get_float("problem_count_threshold");
#define X(aa, bb, cc, dd, ee) check_greater_than_reset_print(&problem_count_##cc,threshold,aa);
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X   
/*    
    cout << "pcft0: " << problem_count_ft[0] << endl;
    if(problem_count_ft[0] > 40){
        cout << "uh-oh" << endl;
    }
#define X(aa, bb, cc, dd, ee) check_greater_than_reset(&problem_count_##cc,40,aa);
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X   
*/
}
void RTMemory::set_control_strategy(ControlStrategy* strategy){
    rtc->set_control_strategy(strategy);
}
