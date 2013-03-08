#include "rtmemory.h"
#include <cfloat>
#include "memory.h"
#include "senses.h"
#include "sensor_stream_system.h"
#include "qd2co_system.h"
#include "co2qd_system.h"
#include "cp_system.h"
#include "rtcontrol.h"
#include "naive_bayes_system.h"
#include "hand_system.cxx"
#include "wam_system.cxx"
#include "utils.h"
#include "control_strategy.cxx"

enum SENSORS{
    enum_time,
#define X(aa, bb, cc, dd, ee) \
    enum_##cc,
#include "input_type_table.h"
#undef X
    NUM_SENSORS
};

RTMemory::RTMemory(ProductManager* _pm, Wam<DIMENSION>* _wam, 
        Memory* _memory, Senses* _senses, RobotController* _control) :
			pm(_pm), wam(_wam), memory(_memory), senses(_senses), control(_control),
            inputType(_memory->get_float("trajectory_type")), 
            jpSpline(NULL), cpSpline(NULL), qdSpline(NULL), coSpline(NULL), 
            jpTrajectory(NULL), cpTrajectory(NULL), qdTrajectory(NULL), coTrajectory(NULL), 
            time(_pm->getExecutionManager()), dataSize(0), loop(false), 
            tmpStr("/tmp/btXXXXXX")
{ 
        log_prefix = "./data_streams/";
#define X(aa, bb, cc, dd, ee) \
        bb cc;
#include "input_type_table.h"
#undef X
        
        data_log_headers = 
            string("TIME,1;") +
#define X(aa, bb, cc, dd, ee) \
            aa + "," + num2str(cc.size()) + ";" +
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
#define P(aa, bb, cc, dd, ee) \
            aa + "," + num2str(1) + ";" +
#include "parameter_table.h"
#undef P 
            "";
         
        STREAM_SIZE = 
            1 +
#define X(aa, bb, cc, dd, ee) \
            cc.size() +
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
            1;
        
        PERIOD_MULTIPLIER = memory->get_float("period_multiplier");
        cout << "Control loop running at " << 1.0 / PERIOD_MULTIPLIER * pm->getExecutionManager()->getPeriod() << endl;

        cpVec = new std::vector<input_cp_type, Eigen::aligned_allocator<input_cp_type> >();
        qdVec = new std::vector<input_qd_type, Eigen::aligned_allocator<input_qd_type> >();
        coVec = new std::vector<input_co_type, Eigen::aligned_allocator<input_co_type> >();
        jpVec = new std::vector<input_jp_type, Eigen::aligned_allocator<input_jp_type> >();
        jpSample = new input_jp_type();
#define X(aa, bb, cc, dd, ee) \
        vec_##cc = new std::vector<input_type_##cc, Eigen::aligned_allocator<input_type_##cc> >();\
        sample_##cc = new input_type_##cc(); \
        //trajectory_##cc = new systems::Callback<double,bb>(boost::ref(*spline_##cc));
        #include "input_type_table.h"
        #include "tool_type_table.h"
#undef X
        for(int l = 0; l < NUM_PARAMETERS; l++){
#define X(aa, bb, cc, dd, ee) \
            mean_trajectory_vec_##cc.push_back(NULL); \
            std_trajectory_vec_##cc.push_back(NULL);
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
        }
        cout << "RTMemory instantiated!" << endl;
}
void RTMemory::init(){
    output_counter = 0;
    wam_system = new WamSystem(wam);
    sss = new SensorStreamSystem(memory, senses);
    qd2co_system = new Qd2CoSystem();
    co2qd_system = new Co2QdSystem(memory);
    cp_system = new CpSystem(memory);
    for(int l = 0; l < NUM_PARAMETERS; l++){
        NaiveBayesSystem* nbs = new NaiveBayesSystem(pm->getExecutionManager(), memory);
        nbs_vec.push_back(nbs);
    }
    rtc = new RTControl(&rtc_debug, 
#define X(aa, bb, cc, dd, ee) \
            &problem_count_##cc,
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X 
            memory, senses, control);
    //param_estimator = new ParameterEstimator();
    //param_output_system = new Parameter<pv_type>();
    
#define P(aa, bb, cc, dd, ee) \
	//param_outputter_##cc = new PrintToStream<bb>(pm->getExecutionManager(), aa, param_ostream_##cc);
#include "parameter_table.h"
#undef P

    pm->getExecutionManager()->startManaging(time); //starting time management
    cout << "RTMemory initialized!" << endl;
}


//*********TEACH*********
void RTMemory::set_teach_name(string _teachName){
    saveName = _teachName;
}
bool RTMemory::prepare_log_file(){
	tmpFile = new char[tmpStr.length() + 1];
	strcpy(tmpFile, tmpStr.c_str());
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return false;
	}
	if (inputType == 0){
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
	if (inputType == 0) {
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
	if (inputType == 0) {
		jpLogger->closeLog();
		disconnect(jpLogger->input);
		// Build spline between recorded points
		log::Reader<jp_sample_type> lr(tmpFile);
		lr.exportCSV(saveName.c_str());
        disconnect(jpLogTg.getInput<0>());
        disconnect(jpLogTg.getInput<1>());
        disconnect(jpLogger->input);
	}
	else{
		poseLogger->closeLog();
		disconnect(poseLogger->input);
		log::Reader<pose_sample_type> pr(tmpFile);
		pr.exportCSV(saveName.c_str());
        disconnect(poseLogTg.getInput<0>());
        disconnect(poseLogTg.getInput<1>());
        disconnect(poseLogger->input);
	}
	// Adding our datatype as the first line of the recorded trajectory
	fileOut = saveName + ".csv";
	std::ifstream in(saveName.c_str());
	std::ofstream out(fileOut.c_str());
	if (inputType == 0)
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
void RTMemory::set_play_name(string _playName){
    playName = _playName;
}
void RTMemory::reset_output_counter(int base){
    output_counter = base;
}
void RTMemory::record_zero_values(){
#define X(aa, bb, cc, dd, ee) \
    senses->tare_all();
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
}
bool RTMemory::load_trajectory(){
    //Create stream from input file
    string play_filename = string("recorded/") + playName + string(".csv");
	std::ifstream fs(play_filename.c_str());
	std::string line;
	// Check to see the data type specified on the first line (jp_type or pose_type)
	// this will inform us if we are tracking 4DIMENSION WAM Joint Angles, 7DIMENSION WAM Joint Angles, or WAM Poses
	std::getline(fs, line);
	// Using a boost tokenizer to parse the data of the file into our vector.
	boost::char_separator<char> sep(",");
	typedef boost::tokenizer<boost::char_separator<char> > t_tokenizer;
	t_tokenizer tok(line, sep);
	if (strcmp(line.c_str(), "pose_type") == 0) {
        
		// Create our spline and trajectory if the first line of the parsed file informs us of a pose_type
		inputType = 1;
		float fLine[8];

		input_cp_type cpSample;
		input_qd_type qdSample;
		input_co_type coSample;
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
			boost::get<0>(cpSample) = fLine[0];
			boost::get<0>(qdSample) = boost::get<0>(cpSample);
			boost::get<0>(coSample) = boost::get<0>(cpSample);

			boost::get<1>(cpSample) << fLine[1], fLine[2], fLine[3];
			boost::get<1>(qdSample) = Eigen::Quaterniond(fLine[4], fLine[5], fLine[6], fLine[7]);
			boost::get<1>(qdSample).normalize();
			boost::get<1>(coSample) << fLine[4], fLine[5], fLine[6], fLine[7];
			cpVec->push_back(cpSample);
			qdVec->push_back(qdSample);
			coVec->push_back(coSample);
		}
		// Make sure the vectors created are the same size
		assert(cpVec->size() == qdVec->size());
		assert(cpVec->size() == coVec->size());
		// Create our splines between points
		cpSpline = new math::Spline<cp_type>(*cpVec);
		qdSpline = new math::Spline<Eigen::Quaterniond>(*qdVec);
		coSpline = new math::Spline<co_type>(*coVec);
        //cout << "created splines" << endl;
		// Create trajectories from the splines
		cpTrajectory = new systems::Callback<double, cp_type>(boost::ref(*cpSpline));
        //causes Eigen unaligned assertion failure...
		//qdTrajectory = new systems::Callback<double, Eigen::Quaterniond>(boost::ref(*qdSpline));
		coTrajectory = new systems::Callback<double, co_type>(boost::ref(*coSpline));
        //cout << "created trajectories" << endl;
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
    logger = new systems::PeriodicDataLogger<input_stream_type> (
			pm->getExecutionManager(),
			new log::RealTimeWriter<input_stream_type>(
                (char*)tmp_filename.c_str(), PERIOD_MULTIPLIER * pm->getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);
    cout << "Data logger initialized!" << endl;
}
//gets called at the start of each loop
 void RTMemory::init_param_logger(){
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
	//tmp_filenames.push_back(tmp_filename);

    //if(loop_count > 0)
    //    logger->closeLog();
    param_logger = new systems::PeriodicDataLogger<parameter_tuple_type> (
			pm->getExecutionManager(),
			new log::RealTimeWriter<parameter_tuple_type>(
                (char*)tmp_filename.c_str(), PERIOD_MULTIPLIER * pm->getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);
    cout << "Parameter logger initialized!" << endl;
}

// converts a vector of doubles into a string of sep_str-separated doubles with optional prefix/suffix
template<int R, int C, typename Units>
std::string to_string(math::Matrix<R,C, Units>* src, string sep_str, string prefix = "", string suffix = ""){
    for (int i = 0; i < src->size(); ++i) {
        char buff[50];
        sprintf(buff, "%f",(*src)[i]);
        prefix.append(buff);
        if(i < src->size()-1)
            prefix.append(sep_str);
        else
            prefix.append(suffix);
    }
    return prefix;
}
//export to separate csv files
void RTMemory::output_data_stream(){
    logger->closeLog(); //clost outstanding log
    //save headers for data log of entire trajectory
    //std::string log_name = playName.substr(9,playName.length()-4-9); //strip recorded/ and .csv from playName
    std::string log_name = playName;
    std::string header_filename = log_prefix+log_name+".h";
    std::ofstream out(header_filename.c_str());
    out << data_log_headers << std::endl;
    /*
    out << "0," <<
#define X(aa, bb, cc, dd, ee) \
        to_string(&zero_value_##cc,",","",",") << 
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
        "0," << "0," <<
        std::endl;*/
    out.close();
    std::vector<std::string>::iterator tmp_filename_it;	
    for (tmp_filename_it=tmp_filenames.begin(); tmp_filename_it < tmp_filenames.end(); tmp_filename_it++){
        std::string tmp_filename = *tmp_filename_it;
        std::string out_filename = log_prefix+itoa(output_counter++)+log_name+".csv";
        log::Reader<input_stream_type> lr((tmp_filename).c_str());
        lr.exportCSV(out_filename.c_str()); 
        std::remove((tmp_filename).c_str());
        printf("Data log saved to the location: %s \n", out_filename.c_str());
    }
    tmp_filenames.clear();
    std::cout <<  "All data logs saved successfully!" << std::endl;
}

void append_to_file(string app_filename, string in_filename){
    ofstream appfile;
    ifstream infile;

    appfile.open(app_filename.c_str(), ios_base::app);
    infile.open(in_filename.c_str(), ios_base::in);
    appfile << infile.rdbuf(); 
    appfile.close();
    infile.close();
}
void append_to_filebuf(string app_filename, string in_filename){
    std::filebuf appfile, infile;

    appfile.open(app_filename.c_str(), ios_base::app | ios_base::binary);
    infile.open(in_filename.c_str(), ios_base::in | ios_base::binary);

    appfile.pubsetbuf(NULL, 1024 * 1024 * 1024);
    infile.pubsetbuf(NULL, 5 * 1024 * 1024);

    boost::iostreams::copy(appfile, infile);
}

//append to one large csv file
void RTMemory::append_data_stream(){
    logger->closeLog(); //clost outstanding log
    //save headers for data log of entire trajectory
    //std::string log_name = playName.substr(9,playName.length()-4-9); //strip recorded/ and .csv from playName
    string log_name = playName;
    string header_filename = log_prefix+log_name+".h";

    //output data header (do NOT OVERWRITE if exists)
    ifstream header_exists(header_filename.c_str());
    if(!header_exists){
        ofstream header_file(header_filename.c_str());
        header_file << data_log_headers << std::endl;
        /*
        header_file << "0," <<
#define X(aa, bb, cc, dd, ee) \
            to_string(&zero_value_##cc,",","",",") << 
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
            "0," << "0," <<
            std::endl;*/
        header_file.close();
    }
    std::vector<std::string>::iterator tmp_filename_it;	
    for (tmp_filename_it=tmp_filenames.begin(); tmp_filename_it < tmp_filenames.end()-1; tmp_filename_it++){
        std::string tmp_filename = *tmp_filename_it;
        std::string out_filename = log_prefix+itoa(output_counter++)+log_name+".csv";
        std::string app_filename = log_prefix+log_name+".csv";
        log::Reader<input_stream_type> lr((tmp_filename).c_str());
        lr.exportCSV(out_filename.c_str()); 
        append_to_file(app_filename, out_filename);
        std::remove((tmp_filename).c_str());
        printf("Data log saved to the location: %s and appended to %s\n", out_filename.c_str(), app_filename.c_str());
    }
    tmp_filenames.clear();
    std::cout <<  "All data logs saved successfully!" << std::endl;
}
bool RTMemory::load_data_stream(bool mean, enum parameters parameter){
    //Create stream from input file
    string stream_prefix = "";
    int param_i = (int)parameter;
    cout << "loading data stream " << param_i << endl;
    if(mean){stream_prefix = string("mean_") + num2str(param_i) + "_";}
    else{stream_prefix = string("std_") + num2str(param_i) + "_";}
    string stream_filename = log_prefix + stream_prefix + playName + ".csv";
    std::ifstream* fs = new std::ifstream(stream_filename.c_str());

    if(!fs->is_open()){
        cout << "WARNING: Data stream does not yet exist for trajectory " << playName << " with parameter set " << parameter << ". Toggling Realtime Learning off." << endl;
        memory->set_float("realtime_learning",0);
        return false;
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
        boost::get<0>(*sample_##cc) = fLine[0]; \
        bb cc; \
        for(int i = 0; i < cc.size(); i++){ \
            cc[i] = fLine[fLine_i++]; \
        } \
        boost::get<1>(*sample_##cc)=cc; \
        vec_##cc->push_back(*sample_##cc); \
        //cout<<"got "<<aa<<" at time "<<fLine[0]<<endl;fflush(stdout); 
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
    }
    //Create our splines between points
#define X(aa, bb, cc, dd, ee) \
        spline_##cc = new math::Spline<bb>(*vec_##cc); \
        vec_##cc->clear();
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
    //cout << "\tSpline created!" << endl;
    
    if(mean){
#define X(aa, bb, cc, dd, ee) \
        mean_trajectory_vec_##cc[param_i] = new systems::Callback<double, bb>(boost::ref(*spline_##cc)); 
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
    }
    else{
#define X(aa, bb, cc, dd, ee) \
        std_trajectory_vec_##cc[param_i] = new systems::Callback<double, bb>(boost::ref(*spline_##cc));
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
    }
    //cout << "\tTrajectory created!" << endl;
	fs->close();
    return true;
}
bool RTMemory::load_data_stream(bool mean){
    //Create stream from input file
    string stream_prefix = "";
    if(mean){stream_prefix = "mean_";}
    else{stream_prefix = "std_";}
    string stream_filename = log_prefix + stream_prefix + playName + ".csv";
    std::ifstream* fs = new std::ifstream(stream_filename.c_str());

    if(!fs->is_open()){
        cout << "WARNING: Data stream does not yet exist for trajectory " << playName << ". Toggling Realtime Learning off." << endl;
        memory->set_float("realtime_learning",0);
        return false;
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
        boost::get<0>(*sample_##cc) = fLine[0]; \
        bb cc; \
        for(int i = 0; i < cc.size(); i++){ \
            cc[i] = fLine[fLine_i++]; \
        } \
        boost::get<1>(*sample_##cc)=cc; \
        vec_##cc->push_back(*sample_##cc); 
        //cout<<"got "<<aa<<" at time "<<fLine[0]<<endl;fflush(stdout); 
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
    }
    //Create our splines between points
#define X(aa, bb, cc, dd, ee) \
        spline_##cc = new math::Spline<bb>(*vec_##cc); \
        vec_##cc->clear();
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
    //cout << "\tSpline created!" << endl;
    
    if(mean){
#define X(aa, bb, cc, dd, ee) \
        mean_trajectory_##cc = new systems::Callback<double, bb>(boost::ref(*spline_##cc)); 
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
    }
    else{
#define X(aa, bb, cc, dd, ee) \
        std_trajectory_##cc = new systems::Callback<double, bb>(boost::ref(*spline_##cc)); 
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
    }
    //cout << "\tTrajectory created!" << endl;
	fs->close();
    return true;
}
void RTMemory::start_playback() {
    time.start();
	cout << "Playback Started!" << endl; fflush(stdout);
}
void RTMemory::pause_playback() {time.stop();}
bool RTMemory::playback_active(){
    bool active;
    //cout << "playback active?" << endl; fflush(stdout);
	if (inputType == 0){
		active = (jpTrajectory->input.getValue() < jpSpline->finalS());
    }
	else {
		active = (cpTrajectory->input.getValue() < cpSpline->finalS());
	}
    //if(!active)
    //    cout << "playback active: " << active << endl; fflush(stdout);
    return active;
}
void RTMemory::disconnect_systems(){
    cout << "Systems disconnecting... "; fflush(stdout);
    if(inputType == 0)
        disconnect(jpTrajectory->input);
    else{
        disconnect(cpTrajectory->input);
        disconnect(coTrajectory->input);
    }
    //cout << "1";
	disconnect(wam->input);
    if(memory->get_float("data_stream_out"))
        disconnect(logger->input);
    //disconnect(param_logger->input);
    disconnect(sss->time_input);
	disconnect(rtc->input_time);
    disconnect(tg.getInput<0>());
    //cout << "2";
#define X(aa, bb, cc, dd, ee) \
    disconnect(rtc->mean_input_##cc); \
    disconnect(rtc->std_input_##cc); \
    disconnect(rtc->actual_input_##cc);
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
#define P(aa, bb, cc, dd, ee) \
    disconnect(tg.getInput<ee>()); 
#include "parameter_table.h"
#undef P 
    disconnect(tg.getInput<NUM_PARAMETERS>());
    for(int l = 0; l < NUM_PARAMETERS; l++){
        disconnect(nbs_vec[l]->input_time); 
#define X(aa, bb, cc, dd, ee) \
        disconnect(nbs_vec[l]->mean_input_##cc); \
        disconnect(nbs_vec[l]->std_input_##cc); \
        disconnect(nbs_vec[l]->actual_input_##cc);
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
    }
    //cout << "3";
    /*
#define P(aa, bb, cc, dd, ee) \
    disconnect(param_outputter_##cc->input);
    //disconnect(pg.getInput<ee>()); \
    //disconnect(param_estimator->input_##cc); \
    //disconnect(param_estimator.getInput<ee>());
#include "parameter_table.h"
#undef P*/
    //cout << "4";
    //disconnect(pg.getInput<NUM_PARAMETERS>());
	time.stop();
	time.setOutput(0.0);
    cout << "Systems disconnected! " << endl; fflush(stdout);
}
void RTMemory::reconnect_systems(){
    cout << "Systems reconnecting... "; fflush(stdout);
    wam_system->init();
    rtc->init();
    cp_system->init();
    co2qd_system->init();
    for(int i = 0; i < NUM_PARAMETERS; i++){ nbs_vec[i]->init(); }

	if (inputType == 0) {
		systems::forceConnect(time.output, jpTrajectory->input); //important!!
		systems::forceConnect(jpTrajectory->output, wam_system->input);
	} else {
		systems::forceConnect(time.output, cpTrajectory->input);
		//systems::forceConnect(time.output, qdTrajectory->input);
		systems::forceConnect(time.output, coTrajectory->input);
	}
    systems::forceConnect(time.output, sss->time_input);
    //cout << "rtl: " << memory->get_float("realtime_learning") << endl;
    if(memory->get_float("realtime_learning")){
        systems::forceConnect(time.output, rtc->input_time);
#define X(aa, bb, cc, dd, ee) \
        //systems::forceConnect(sss->output_##cc, rtc->actual_input_##cc); \
        //systems::forceConnect(time.output, mean_trajectory_##cc->input); \
        //systems::forceConnect(time.output, std_trajectory_##cc->input); \
        //systems::forceConnect(mean_trajectory_##cc->output, rtc->mean_input_##cc); \
        //systems::forceConnect(std_trajectory_##cc->output, rtc->std_input_##cc);
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
        cout << "NBS "; fflush(stdout);
        for(int l = 0; l < NUM_PARAMETERS; l++){
            cout << "l: " << l << endl; fflush(stdout);
            systems::forceConnect(time.output, nbs_vec[l]->input_time);
#define X(aa, bb, cc, dd, ee) \
            systems::forceConnect(time.output, mean_trajectory_vec_##cc[l]->input); \
            systems::forceConnect(time.output, std_trajectory_vec_##cc[l]->input); \
            systems::forceConnect(sss->output_##cc, nbs_vec[l]->actual_input_##cc);\
            systems::forceConnect(mean_trajectory_vec_##cc[l]->output, nbs_vec[l]->mean_input_##cc); \
            systems::forceConnect(std_trajectory_vec_##cc[l]->output, nbs_vec[l]->std_input_##cc); \
            //systems::forceConnect(mean_trajectory_##cc->output, nbs_vec[l]->mean_input_##cc); \
            //systems::forceConnect(std_trajectory_##cc->output, nbs_vec[l]->std_input_##cc);
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
        }
#define P(aa, bb, cc, dd, ee) \
        //systems::forceConnect(nbs_vec[ee]->output_probability, param_outputter_##cc->input); \
        //systems::forceConnect(nbs_vec[ee]->output_probability, tg.getInput<NUM_SENSORS>()); \
        //systems::forceConnect(nbs_vec[ee]->output_probability, param_estimator->input_##cc); \
        systems::forceConnect(param_splitter.getOutput<ee>(), param_outputter_##cc->input);
#include "parameter_table.h"
#undef P
        //systems::forceConnect(time.output, param_estimator.getInput<NUM_PARAMETERS>());
        //systems::forceConnect(param_estimator.output_time, tg.getInput<NUM_SENSORS>());
    }
    else{
    }
    //cout << "transform_qd_x: " << memory->get_float("transform_qd_x") << endl;	
    //cout << "transform_cp_z: " << memory->get_float("transform_cp_z") << endl;	
    cout << "tg "; fflush(stdout);
    if(memory->get_float("data_stream_out")){
        systems::forceConnect(time.output, tg.getInput<0>());
#define X(aa, bb, cc, dd, ee) \
        systems::forceConnect(sss->output_##cc, tg.getInput<ee>());
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
        systems::forceConnect(sss->output_param, tg.getInput<NUM_SENSORS>());
    }
    /*else{
        systems::forceConnect(time.output, tg.getInput<0>());
#define P(aa, bb, cc, dd, ee) \
        systems::forceConnect(nbs_vec[ee]->output_probability, tg.getInput<ee>());
#include "parameter_table.h"
#undef P 
        systems::forceConnect(time.output, tg.getInput<NUM_PARAMETERS>());
    }*/
    
/*
#define P(aa, bb, cc, dd, ee) \
    systems::forceConnect(nbs_vec[ee]->output_probability, pg.getInput<ee>());
#include "parameter_table.h"
#undef P 
	systems::forceConnect(time.output, pg.getInput<NUM_PARAMETERS>());*/
    if(memory->get_float("realtime_learning") && memory->get_float("track_rtc_jp") && inputType == 0)
    {
        //systems::forceConnect(jpTrajectory->output,        rtc->actual_input_jp);
        //systems::forceConnect(rtc->output_jp,                 tg.getInput<NUM_SENSORS>());
        wam->trackReferenceSignal(rtc->output_jp);
    }
    else if(memory->get_float("realtime_learning") && memory->get_float("track_rtc_c") && inputType == 1)
    {
        //systems::forceConnect(sss->output_jp, tg.getInput<NUM_SENSORS>());
		//systems::forceConnect(cpTrajectory->output, rtc->actual_input_cp);
		//systems::forceConnect(coTrajectory->output, rtc->actual_input_co);
		systems::forceConnect(rtc->output_cp, poseTg.getInput<0>());
		systems::forceConnect(rtc->output_qd, poseTg.getInput<1>());
		//systems::forceConnect(cpTrajectory->output, poseTg.getInput<0>());
		//systems::forceConnect(qdTrajectory->output, poseTg.getInput<1>());
		wam->trackReferenceSignal(poseTg.output);
    }
    else if(inputType == 0)
    {
        //systems::forceConnect(jpTrajectory->output, tg.getInput<NUM_SENSORS>());
        wam->trackReferenceSignal(jpTrajectory->output);
    }
    else
    {
        cout << "pose tracking " << endl; fflush(stdout);
        //systems::forceConnect(sss->output_jp, tg.getInput<NUM_SENSORS>());
        //if(memory->get_float("track_rtc_c")){
            systems::forceConnect(cpTrajectory->output, cp_system->input);
            systems::forceConnect(cp_system->output, poseTg.getInput<0>());
            systems::forceConnect(coTrajectory->output, co2qd_system->input);
            systems::forceConnect(co2qd_system->output, poseTg.getInput<1>());
            wam->trackReferenceSignal(poseTg.output);
       //}
    }
    //cout << "NUM_SENSORS: " << NUM_SENSORS << endl;
    //if(memory->get_float("data_stream_out")){
        cout << "logger input" << endl; fflush(stdout);
        systems::forceConnect(tg.output, logger->input);
    //}
    //systems::forceConnect(pg.output, param_logger->input);
	cout << "Systems reconnected!" << endl; fflush(stdout);
	time.start();
}
jp_type RTMemory::get_initial_jp(){
    if(jpSpline != NULL)
        return jpSpline->eval(jpSpline->initialS());
    else{
        return memory->get_initial_jp();
    }
}
pose_type RTMemory::get_initial_tp(){
    cp_type init_cp = memory->get_transform_cp() + cpSpline->eval(cpSpline->initialS());
    co_type init_co = coSpline->eval(coSpline->initialS());
    qd_type init_qd = memory->get_transform_qd() * co2qd(&init_co);
    //cout << "initial_cp: " << init_cp << endl;
    //cout << "initial_qd: " << init_qd.w() << ", " << init_qd.x() << ", " << init_qd.y() << ", " << init_qd.z() << endl;
    return boost::make_tuple(init_cp, init_qd);
}
//checks all entries in src and dest and adds 1 to each entry where mat1 is less than mat2 
template<int R, int C, typename Units>
void check_greater_than_reset(math::Matrix<R,C, Units>* to_check, double limit, string output_string){
    for (int i = 0; i < to_check->size(); ++i) {
        if((*to_check)[i] > limit){
            (*to_check)[i] = 0; } } }
//checks all entries in src and dest and adds 1 to each entry where mat1 is less than mat2 
template<int R, int C, typename Units>
void check_greater_than_reset_print(math::Matrix<R,C, Units>* to_check, double limit, string output_string){
    for (int i = 0; i < to_check->size(); ++i) {
        if((*to_check)[i] > limit){
            cout << output_string << "[" << i << "]" << endl;
            (*to_check)[i] = 0; } } }
double RTMemory::get_probability_non_normalized(enum parameters parameter){
   return nbs_vec[(int)parameter]->get_probability();
}
double RTMemory::get_probability(enum parameters parameter){
    double sum = 0;
    double max = FLT_MIN;
#define P(aa, bb, cc, dd, ee) \
    double pnn_##cc = get_probability_non_normalized(enum_##cc); \
    max = pnn_##cc > max ? pnn_##cc : max;
#include "parameter_table.h"
#undef P
#define P(aa, bb, cc, dd, ee) \
    sum += max + exp(pnn_##cc-max);
#include "parameter_table.h"
#undef P
    return get_probability_non_normalized(parameter) - std::log(sum);
}
void RTMemory::check_for_problems(){
    if(strcmp(rtc_debug.str().c_str(),"")!=0)
        cout << "rtcdbg: " << endl << rtc_debug.str() << endl;
#define P(aa, bb, cc, dd, ee) \
        cout << aa << ": " << get_probability(enum_##cc) << endl; fflush(stdout); \
        //cout << "l: " << ee << endl; fflush(stdout);
#include "parameter_table.h"
#undef P
}
void RTMemory::set_control_strategy(ControlStrategy* strategy){
    rtc->set_control_strategy(strategy);
}
void RTMemory::set_environment_param(pv_type param){
    param_output_system->setValue(param);
}
