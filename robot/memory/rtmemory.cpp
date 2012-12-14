string data_log_headers = 
    "time"
    ",joint_pos_0,joint_pos_1,joint_pos_2,joint_pos_3,joint_pos_4,joint_pos_5,joint_pos_6,joint_pos_7"
    ",joint_vel_0,joint_vel_1,joint_vel_2,joint_vel_3,joint_vel_4,joint_vel_5,joint_vel_6,joint_vel_7"
    ",joint_tor_0,joint_tor_1,joint_tor_2,joint_tor_3,joint_tor_4,joint_tor_5,joint_tor_6,joint_tor_7"
    ",cart_pos_0,cart_pos_1,cart_pos_2,cart_pos_3"
    ",cart_ori_0,cart_ori_1,cart_ori_2,cart_ori_3"
    ",ft_torque_0,ft_torque_1,ft_torque_2,ft_torque_3"
    ;
string log_prefix = "./data_streams/";
std::string playName;
int inputType;
const libconfig::Setting& setting;
libconfig::Config config;

std::string tmpStr, saveName, fileOut;

ControlModeSwitcher<DIMENSION>* cms;
std::vector<input_cp_type, Eigen::aligned_allocator<input_cp_type> >* cpVec;
std::vector<input_qd_type, Eigen::aligned_allocator<input_qd_type> >* qVec;

math::Spline<systems::Wam<DIMENSION>::jp_type>* jpSpline;
math::Spline<cp_type>* cpSpline;
math::Spline<Eigen::Quaterniond>* qSpline;

systems::Callback<double, systems::Wam<DIMENSION>::jp_type>* jpTrajectory;
//jpTrajectory = new systems::Callback<double, systems::Wam<DIMENSION>::jp_type>(boost::ref(*jpSpline));
systems::Callback<double, cp_type>* cpTrajectory;
systems::Callback<double, Eigen::Quaterniond>* qTrajectory;

systems::Ramp time;
systems::TupleGrouper<cp_type, Eigen::Quaterniond> poseTg;
    
//realtime data logging
//typedef boost::tuple<double, jp_type, jv_type, jt_type, cp_type, Eigen::Quaterniond> input_stream_type;
//systems::TupleGrouper<double, jp_type, jv_type, jt_type, cp_type, Eigen::Quaterniond> tg;
//systems::Callback<double, systems::Wam<DIMENSION>::jp_type>* jpTrajectory;
//	jpTrajectory = new systems::Callback<double, systems::Wam<DIMENSION>::jp_type>(boost::ref(*jpSpline));

typedef boost::tuple<double, 
        systems::Wam<DIMENSION>::jp_type, 
        systems::Wam<DIMENSION>::jv_type, 
        systems::Wam<DIMENSION>::jt_type, 
        cp_type, 
        Eigen::Quaterniond, 
        Hand::jp_type> input_stream_type;

systems::TupleGrouper<double, 
    systems::Wam<DIMENSION>::jp_type, 
    systems::Wam<DIMENSION>::jv_type, 
    systems::Wam<DIMENSION>::jt_type, 
    cp_type, 
    Eigen::Quaterniond, 
    Hand::jp_type> tg;

const static int STREAM_SIZE = 1+7+7+7+3+4+4;

std::string data_log_headers;
systems::PeriodicDataLogger<input_stream_type>* logger;
std::vector<std::string> tmp_filenames;
std::string log_prefix;

math::Spline<Wam<DIMENSION>::jp_type>* stream_jp_spline;
math::Spline<Wam<DIMENSION>::jv_type>* stream_jv_spline;
math::Spline<Wam<DIMENSION>::jt_type>* stream_jt_spline;
math::Spline<cp_type>*                 stream_cp_spline;
math::Spline<Eigen::Quaterniond>*      stream_qd_spline;
math::Spline<Hand::jp_type>*           stream_ft_spline;
math::Spline<cp_type>*                 stream_cf_spline;
math::Spline<cp_type>*                 stream_ct_spline;

systems::Callback<double, Wam<DIMENSION>::jp_type>* stream_jp_mean_trajectory;
systems::Callback<double, Wam<DIMENSION>::jv_type>* stream_jv_mean_trajectory;
systems::Callback<double, Wam<DIMENSION>::jt_type>* stream_jt_mean_trajectory;
systems::Callback<double, cp_type>*                 stream_cp_mean_trajectory;
systems::Callback<double, Eigen::Quaterniond>*      stream_qd_mean_trajectory;
systems::Callback<double, Hand::jp_type>*           stream_ft_mean_trajectory;
systems::Callback<double, cp_type>*                 stream_cf_mean_trajectory;
systems::Callback<double, cp_type>*                 stream_ct_mean_trajectory;

systems::Callback<double, Wam<DIMENSION>::jp_type>* stream_jp_std_trajectory;
systems::Callback<double, Wam<DIMENSION>::jv_type>* stream_jv_std_trajectory;
systems::Callback<double, Wam<DIMENSION>::jt_type>* stream_jt_std_trajectory;
systems::Callback<double, cp_type>*                 stream_cp_std_trajectory;
systems::Callback<double, Eigen::Quaterniond>*      stream_qd_std_trajectory;
systems::Callback<double, Hand::jp_type>*           stream_ft_std_trajectory;
systems::Callback<double, cp_type>*                 stream_cf_std_trajectory;
systems::Callback<double, cp_type>*                 stream_ct_std_trajectory;

//RTMemory Class
class RTMemory {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DIMENSION);
protected:
	systems::Wam<DIMENSION>* wam;
	Hand* hand;
    HandSystem* hand_system; //for realtime data logging of hand sensors
    WamSystem* wam_system; //for realtime manipulation of wam trajectory
	ProductManager& pm;
	
public:
	int dataSize;
	bool loop;
    bool problem;
    stringstream hand_debug;
	RTMemory(systems::Wam<DIMENSION>* wam_, ProductManager& pm_, std::string filename_,
			const libconfig::Setting& setting_) :
			wam(wam_), hand(NULL), pm(pm_), playName(filename_), inputType(0), setting(
					setting_), cms(NULL), cpVec(NULL), qVec(NULL), jpSpline(
					NULL), cpSpline(NULL), qSpline(NULL), jpTrajectory(NULL), cpTrajectory(
					NULL), qTrajectory(NULL), time(pm.getExecutionManager()), dataSize(
					0), loop(false) {
    }
	bool init();
	void init_data_logger();
    void output_data_stream();
    void load_data_stream(bool);
private:
	DISALLOW_COPY_AND_ASSIGN(RTMemory);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

init(){
    
    std::string filename(argv[1]);
    // Load our vc_calibration file.
	libconfig::Config config;
	std::string calibration_file;
	if (DIMENSION == 4)
		calibration_file = "calibration4.conf";
	else
		calibration_file = "calibration7.conf";
	config.readFile(calibration_file.c_str());
	config.getRoot();
    
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
		// Create our spline and trajectory if the first line of the parsed file informs us of a pose_type
		inputType = 1;
		cpVec = new std::vector<input_cp_type,
				Eigen::aligned_allocator<input_cp_type> >();
		qVec = new std::vector<input_qd_type,
				Eigen::aligned_allocator<input_qd_type> >();
		float fLine[8];
		input_cp_type cpSamp;
		input_qd_type qSamp;
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
			boost::get<0>(cpSamp) = fLine[0];
			boost::get<0>(qSamp) = boost::get<0>(cpSamp);

			boost::get<1>(cpSamp) << fLine[1], fLine[2], fLine[3];
			boost::get<1>(qSamp) = Eigen::Quaterniond(fLine[4], fLine[5],
					fLine[6], fLine[7]);
			boost::get<1>(qSamp).normalize();
			cpVec->push_back(cpSamp);
			qVec->push_back(qSamp);
		}
		// Make sure the vectors created are the same size
		assert(cpVec->size() == qVec->size());
		// Create our splines between points
		cpSpline = new math::Spline<cp_type>(*cpVec);
		qSpline = new math::Spline<Eigen::Quaterniond>(*qVec);
		// Create trajectories from the splines
		cpTrajectory = new systems::Callback<double, cp_type>(
				boost::ref(*cpSpline));
		qTrajectory = new systems::Callback<double, Eigen::Quaterniond>(
				boost::ref(*qSpline));
	} else if (strcmp(line.c_str(), "jp_type") == 0) {
		// Create our spline and trajectory if the first line of the parsed file informs us of a jp_type
		std::vector<input_jp_type, Eigen::aligned_allocator<input_jp_type> > jp_vec;
		float fLine[8];
		input_jp_type samp;
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
			boost::get<0>(samp) = fLine[0];
			// To handle the different WAM configurations
			if (j == 5)
				boost::get<1>(samp) << fLine[1], fLine[2], fLine[3], fLine[4];
			else
				boost::get<1>(samp) << fLine[1], fLine[2], fLine[3], fLine[4], fLine[5], fLine[6], fLine[7];
			jp_vec.push_back(samp);
		}
		// Create our splines between points
		jpSpline = new math::Spline<systems::Wam<DIMENSION>::jp_type>(jp_vec);
		// Create our trajectory
		jpTrajectory = new systems::Callback<double, systems::Wam<DIMENSION>::jp_type>(boost::ref(*jpSpline));
	} else {
		// The first line does not contain "jp_type or pose_type" return false and exit.
		printf(
				"EXITING: First line of file must specify jp_type or pose_type data.");
		btsleep(1.5);
		return false;
	}
	
    //Close the file
	fs.close();

    load_data_stream(true);
    load_data_stream(false);

	printf("\nFile Contains data in the form of: %s\n\n",
			inputType == 0 ? "jp_type" : "pose_type");

}

//gets called at the start of each loop
 void RTMemory::init_data_logger(){
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

    if(loop_count > 0)
        logger->closeLog();
    const size_t PERIOD_MULTIPLIER = 1;
    logger = new systems::PeriodicDataLogger<input_stream_type> (
			pm.getExecutionManager(),
			new log::RealTimeWriter<input_stream_type>((char*)tmp_filename.c_str(), PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);
}

//export to csv files

void RTMemory::output_data_stream(){
    logger->closeLog(); //clost outstanding log
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
/*typedef boost::tuple<//double, 
            systems::Wam<DIMENSION>::jp_type, 
            systems::Wam<DIMENSION>::jv_type, 
            systems::Wam<DIMENSION>::jt_type, 
            cp_type, 
            Eigen::Quaterniond, 
            Hand::jp_type > input_stream_type;
*/;

void RTMemory::load_data_stream(bool mean){
    
    //Create stream from input file
    std::ifstream* fs;
    if(mean)
        fs = new std::ifstream("data_streams/mean_flip_0412.csv");
    else
        fs = new std::ifstream("data_streams/std_flip_0412.csv");
	std::string line;
	typedef boost::tokenizer<boost::char_separator<char> > t_tokenizer;
	boost::char_separator<char> sep(",");
    
    // Create our spline and trajectory if the first line of the parsed file informs us of a jp_type
    std::vector<input_jp_type, Eigen::aligned_allocator<input_jp_type> > stream_jp_vec;
    std::vector<input_jv_type, Eigen::aligned_allocator<input_jv_type> > stream_jv_vec;
    std::vector<input_jt_type, Eigen::aligned_allocator<input_jt_type> > stream_jt_vec;
    std::vector<input_cp_type, Eigen::aligned_allocator<input_cp_type> > stream_cp_vec;
    std::vector<input_qd_type, Eigen::aligned_allocator<input_qd_type> > stream_qd_vec;
    qVec = new std::vector<input_qd_type,Eigen::aligned_allocator<input_qd_type> >();
    std::vector<input_ft_type, Eigen::aligned_allocator<input_ft_type> > stream_ft_vec;
    
    float fLine[STREAM_SIZE];
    input_jp_type jp_sample;
    input_jv_type jv_sample;
    input_jt_type jt_sample;
    input_cp_type cp_sample;
    input_qd_type qd_sample;
    input_ft_type ft_sample;
    float count = 0.002;
    while (true) {
        std::getline(*fs, line);
        if (!fs->good())
            break;
        int fLine_i = 0;
        
        t_tokenizer tok(line, sep);
        int j = 0;
        for (t_tokenizer::iterator beg = tok.begin(); beg != tok.end();
                ++beg) {
            if(j >= STREAM_SIZE-4) //only look at trailing values
                fLine[j] = boost::lexical_cast<float>(*beg);
            j++;
        }
        fLine[0] = count;
        count += 0.002;

        boost::get<0>(jp_sample) = fLine[fLine_i];
        boost::get<0>(jv_sample) = fLine[fLine_i];
        boost::get<0>(jt_sample) = fLine[fLine_i];
        boost::get<0>(cp_sample) = fLine[fLine_i];
        boost::get<0>(qd_sample) = fLine[fLine_i];
        boost::get<0>(ft_sample) = fLine[fLine_i++];
        
        //cout << "got time " << fLine_i << endl;
        //get joint pos
        /*Wam<DIMENSION>::jp_type jp;
        for(int i = 0; i < jp.size(); i++){
            jp[i] = fLine[fLine_i++];
        }
        boost::get<1>(jp_sample) = jp;*/
        //boost::get<1>(jp_sample) << fLine[fLine_i+0], fLine[fLine_i+1], fLine[fLine_i+2], fLine[fLine_i+3],
        //                            fLine[fLine_i+4], fLine[fLine_i+5], fLine[fLine_i+6];
        fLine_i+=7;

        //cout << "got jp "<< fLine_i <<endl;//: " << to_string(&jp) << endl;
        //get joint vel
        /*Wam<DIMENSION>::jv_type jv;
        for(int i = 0; i < jv.size(); i++){
            jv[i] = fLine[fLine_i++];
        }
        boost::get<1>(jv_sample) = jv;*/
        //boost::get<1>(jv_sample) << fLine[fLine_i+0], fLine[fLine_i+1], fLine[fLine_i+2], fLine[fLine_i+3],
        //                            fLine[fLine_i+4], fLine[fLine_i+5], fLine[fLine_i+6];
        fLine_i+=7;

        //cout << "got jv " << fLine_i <<endl;//: " << to_string(&jp) << endl;
        //get joint tor
        /*Wam<DIMENSION>::jt_type jt;
        for(int i = 0; i < jt.size(); i++){
            jt[i] = fLine[fLine_i++];
        }
        boost::get<1>(jt_sample) = jt;*/
        //boost::get<1>(jt_sample) << fLine[fLine_i+0], fLine[fLine_i+1], fLine[fLine_i+2], fLine[fLine_i+3],
        //                            fLine[fLine_i+4], fLine[fLine_i+5], fLine[fLine_i+6];
        fLine_i+=7;
        //cout << "got jt "<< fLine_i <<endl;//: " << to_string(&jp) << endl;
        //get cart pos
        /*cp_type cp; 
        for(int i = 0; i < cp.size(); i++){
            cp[i] = fLine[fLine_i++];
        }
        boost::get<1>(cp_sample) = cp;*/
		//boost::get<1>(cp_sample) << fLine[fLine_i+0], fLine[fLine_i+1], fLine[fLine_i+2];
        fLine_i+=3;
        //cout << "got cp " << fLine_i <<endl;//: " << to_string(&jp) << endl;
        //get cart ori
        /*Eigen::Quaterniond qd;(
                fLine[fLine_i++],
                fLine[fLine_i++],
                fLine[fLine_i++],
                fLine[fLine_i++]
                ); */
		//boost::get<1>(qd_sample) << fLine[fLine_i+0], fLine[fLine_i+1], fLine[fLine_i+2], fLine[fLine_i+3];
        fLine_i+=4;
        //cout << "got qd " << fLine_i <<endl;//: " << to_string(&jp) << endl;
        //fLine_i+= STREAM_SIZE-1-1-4;
        //get finger torque
		boost::get<1>(ft_sample) << fLine[fLine_i+0], fLine[fLine_i+1], fLine[fLine_i+2], fLine[fLine_i+3];
        //if(mean && fLine[0] > 3.1 && fLine[0] < 4.5)
        //    cout << fLine[0] << ": " << fLine[fLine_i+0] << ", "<< fLine[fLine_i+1] << endl;
        fLine_i+=4;
        /*
        for(int i = 0; i < ft.size(); i++){
            ft[i] = fLine[fLine_i++];
        }
        boost::get<1>(ft_sample) = ft;*/
        //cout << "got ft"<< fLine_i << endl;//: " << to_string(&jp) << endl;
        //boost::get<0>(sample) = fLine[j];
        //

        stream_jp_vec.push_back(jp_sample);
        stream_jv_vec.push_back(jv_sample);
        stream_jt_vec.push_back(jt_sample);
        stream_cp_vec.push_back(cp_sample);
        stream_qd_vec.push_back(qd_sample);
        qVec->push_back(qd_sample);
        stream_ft_vec.push_back(ft_sample);
    }
    //Create our splines between points
    stream_jp_spline = new math::Spline<Wam<DIMENSION>::jp_type>(stream_jp_vec);
    stream_jv_spline = new math::Spline<Wam<DIMENSION>::jv_type>(stream_jv_vec);
    stream_jt_spline = new math::Spline<Wam<DIMENSION>::jt_type>(stream_jt_vec);
    stream_cp_spline = new math::Spline<cp_type>                (stream_cp_vec);
    //stream_qd_spline = new math::Spline<Eigen::Quaterniond>     (*qVec)
    //stream_qd_spline = new math::Spline<Eigen::Quaterniond>     (stream_qd_vec);
    stream_ft_spline = new math::Spline<Hand::jp_type>          (stream_ft_vec);
    //cout << "got spline" << endl;
    if(mean){
        // Create our trajectory
        stream_jp_mean_trajectory = new systems::Callback<double, Wam<DIMENSION>::jp_type>(boost::ref(*stream_jp_spline));
        //cout << "got jp_traj" << endl;
        stream_jv_mean_trajectory = new systems::Callback<double, Wam<DIMENSION>::jv_type>(boost::ref(*stream_jv_spline));
        //cout << "got jv_traj" << endl;
        stream_jt_mean_trajectory = new systems::Callback<double, Wam<DIMENSION>::jt_type>(boost::ref(*stream_jt_spline));
        //cout << "got jt_traj" << endl;
        stream_cp_mean_trajectory = new systems::Callback<double, cp_type>                (boost::ref(*stream_cp_spline));
        //cout << "got cp_traj" << endl;
        //stream_qd_mean_trajectory = new systems::Callback<double, Eigen::Quaterniond>     (boost::ref(*stream_qd_spline));
        //cout << "got qd_traj" << endl;
        stream_ft_mean_trajectory = new systems::Callback<double, Hand::jp_type>          (boost::ref(*stream_ft_spline));
        //cout << "got ft_traj" << endl;
    }
    else{
        // Create our trajectory
        stream_jp_std_trajectory = new systems::Callback<double, Wam<DIMENSION>::jp_type>(boost::ref(*stream_jp_spline));
        //cout << "got jp_traj" << endl;
        stream_jv_std_trajectory = new systems::Callback<double, Wam<DIMENSION>::jv_type>(boost::ref(*stream_jv_spline));
        //cout << "got jv_traj" << endl;
        stream_jt_std_trajectory = new systems::Callback<double, Wam<DIMENSION>::jt_type>(boost::ref(*stream_jt_spline));
        //cout << "got jt_traj" << endl;
        stream_cp_std_trajectory = new systems::Callback<double, cp_type>                (boost::ref(*stream_cp_spline));
        //cout << "got cp_traj" << endl;
        //stream_qd_std_trajectory = new systems::Callback<double, Eigen::Quaterniond>     (boost::ref(*stream_qd_spline));
        //cout << "got qd_traj" << endl;
        stream_ft_std_trajectory = new systems::Callback<double, Hand::jp_type>          (boost::ref(*stream_ft_spline));
        //cout << "got ft_traj" << endl;
    }
	fs->close();
}

