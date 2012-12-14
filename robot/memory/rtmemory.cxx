#include <iostream>  // For std::cin
#include <string>  // For std::string and std::getline()
#include <cstdlib>  // For std::atexit()
#include <unistd.h>  // For usleep() & readlink
#include <string>


#include "stdheader.h"
#include "macros.h"
#include "control_mode_switcher.h"
#include "robot.h"

std::string itoa(int i){std::string a = boost::lexical_cast<std::string>(i);return a;}

//#include "data_stream.h" //for sensor data stream io
//#include "hand_system.cxx" 
//#include "wam_system.cxx" 
//#include "display.cxx" 
//#include <barrett/standard_main_function.h>

string log_prefix = "./data_streams/";

//RTMemory Class
class RTMemory {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DIMENSION);
string data_log_headers;
std::string playName;
int inputType;
const libconfig::Setting& setting;
libconfig::Config config;

std::string tmpStr, saveName, fileOut;

ControlModeSwitcher<DIMENSION>* cms;

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
#define X(aa, bb, cc) bb,
    #include "type_table.h"
#undef X
        double> input_stream_type;

systems::TupleGrouper<double, 
#define X(aa, bb, cc) bb,
    #include "type_table.h"
#undef X
        double> tg;

const static int STREAM_SIZE = 1+7+7+7+3+4+4;

systems::PeriodicDataLogger<input_stream_type>* logger;
std::vector<std::string> tmp_filenames;
std::string log_prefix;

//realtime logfile reading:
//  1. vectors (input_type_xx) of data points are built from data samples (lines of an input file)
//  2. splines are built from these data points
//  3. sensorimotor trajectories are built from the splines
//  note: see lib/type_table.h
#define X(aa, bb, cc) typedef boost::tuple<double, bb> input_type_##cc;
    #include "type_table.h"
#undef X
#define X(aa, bb, cc) input_type_##cc* sample_##cc;
    #include "type_table.h"
#undef X
#define X(aa, bb, cc) std::vector<input_type_##cc, Eigen::aligned_allocator<input_type_##cc> >* vec_##cc;
    #include "type_table.h"
#undef X
#define X(aa, bb, cc) math::Spline<bb>* spline_##cc;
    #include "type_table.h"
#undef X
#define X(aa, bb, cc) systems::Callback<double, bb>* trajectory_##cc;
    #include "type_table.h"
#undef X
#define X(aa, bb, cc) systems::Callback<double, bb>* mean_trajectory_##cc;
    #include "type_table.h"
#undef X
#define X(aa, bb, cc) systems::Callback<double, bb>* std_trajectory_##cc;
    #include "type_table.h"
#undef X

protected:
    Robot* robot;
	systems::Wam<DIMENSION>* wam;
	Hand* hand;
    //HandSystem* hand_system; //for realtime data logging of hand sensors
    //WamSystem* wam_system; //for realtime manipulation of wam trajectory
	
public:
	int dataSize;
	bool loop;
    bool problem;
    stringstream hand_debug;
	RTMemory(Robot* _robot, std::string filename_, const libconfig::Setting& setting_) :
			robot(_robot), playName(filename_), inputType(0), setting(setting_), cms(NULL), 
            //cpVec(NULL), qVec(NULL), 
            jpSpline( NULL), cpSpline(NULL), qSpline(NULL), 
            jpTrajectory(NULL), cpTrajectory( NULL), qTrajectory(NULL), 
            time(robot->getPM()->getExecutionManager()), dataSize( 0), loop(false) 
    { 
        data_log_headers = 
    "time"
    ",joint_pos_0,joint_pos_1,joint_pos_2,joint_pos_3,joint_pos_4,joint_pos_5,joint_pos_6,joint_pos_7"
    ",joint_vel_0,joint_vel_1,joint_vel_2,joint_vel_3,joint_vel_4,joint_vel_5,joint_vel_6,joint_vel_7"
    ",joint_tor_0,joint_tor_1,joint_tor_2,joint_tor_3,joint_tor_4,joint_tor_5,joint_tor_6,joint_tor_7"
    ",cart_pos_0,cart_pos_1,cart_pos_2,cart_pos_3"
    ",cart_ori_0,cart_ori_1,cart_ori_2,cart_ori_3"
    ",ft_torque_0,ft_torque_1,ft_torque_2,ft_torque_3"
    ;
#define X(aa, bb, cc) vec_##cc = new std::vector<input_type_##cc, Eigen::aligned_allocator<input_type_##cc> >();
        #include "type_table.h"
#undef X
#define X(aa, bb, cc) sample_##cc = new input_type_##cc();
        #include "type_table.h"
#undef X
//#define X(aa, bb, cc) trajectory_##cc = new systems::Callback<double,bb>(boost::ref(*spline_##cc));
//    #include "type_table.h"
//#undef X
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

bool RTMemory::init(){
    
   // std::string filename(argv[1]);
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
				boost::ref(*qSpline));
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
			boost::get<0>(*sample_jp) = fLine[0];
			// To handle the different WAM configurations
			if (j == 5)
				boost::get<1>(*sample_jp) << fLine[1], fLine[2], fLine[3], fLine[4];
			else
				boost::get<1>(*sample_jp) << fLine[1], fLine[2], fLine[3], fLine[4], fLine[5], fLine[6], fLine[7];
			vec_jp->push_back(*sample_jp);
		}
		// Create our splines between points
		jpSpline= new math::Spline<systems::Wam<DIMENSION>::jp_type>(*vec_jp);
		// Create our trajectory
		jpTrajectory= new systems::Callback<double, systems::Wam<DIMENSION>::jp_type>(boost::ref(*jpSpline));
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

    //if(loop_count > 0)
    //    logger->closeLog();
    const size_t PERIOD_MULTIPLIER = 1;
    logger = new systems::PeriodicDataLogger<input_stream_type> (
			robot->getPM()->getExecutionManager(),
			new log::RealTimeWriter<input_stream_type>(
                (char*)tmp_filename.c_str(), PERIOD_MULTIPLIER * robot->getPM()->getExecutionManager()->getPeriod()),
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
//#define X(aa, bb, cc) std::vector<input_type_##cc, = new input_type_##cc();
 //       #include "type_table.h"
//#undef X
    /*std::vector<input_jp_type, Eigen::aligned_allocator<input_jp_type> > stream_jp_vec;
    std::vector<input_jv_type, Eigen::aligned_allocator<input_jv_type> > stream_jv_vec;
    std::vector<input_jt_type, Eigen::aligned_allocator<input_jt_type> > stream_jt_vec;
    std::vector<input_cp_type, Eigen::aligned_allocator<input_cp_type> > stream_cp_vec;
    std::vector<input_qd_type, Eigen::aligned_allocator<input_qd_type> > stream_qd_vec;
    vec_qd = new std::vector<input_qd_type,Eigen::aligned_allocator<input_qd_type> >();
    std::vector<input_ft_type, Eigen::aligned_allocator<input_ft_type> > stream_ft_vec;*/
    
    float fLine[STREAM_SIZE];
    /*input_jp_type jp_sample;
    input_jv_type jv_sample;
    input_jt_type jt_sample;
    input_cp_type cp_sample;
    input_qd_type qd_sample;
    input_ft_type ft_sample;*/
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

#define X(aa, bb, cc) boost::get<0>(*sample_##cc) = fLine[fLine_i];
        #include "type_table.h"
#undef X
        fLine_i++;
        
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
		boost::get<1>(*sample_ft) << fLine[fLine_i+0], fLine[fLine_i+1], fLine[fLine_i+2], fLine[fLine_i+3];
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

#define X(aa, bb, cc) vec_##cc->push_back(*sample_##cc); 
    #include "type_table.h"
#undef X
    }
    //Create our splines between points
#define X(aa, bb, cc) spline_##cc = new math::Spline<bb>(*vec_##cc); 
    #include "type_table.h"
#undef X
    //stream_ft_spline = new math::Spline<Hand::jp_type>          (stream_ft_vec);
    if(mean){
#define X(aa, bb, cc) mean_trajectory_##cc = new systems::Callback<double, bb>(boost::ref(*spline_##cc)); 
    #include "type_table.h"
#undef X
        //stream_ft_std_trajectory = new systems::Callback<double, Hand::jp_type>          (boost::ref(*stream_ft_spline));
    }
    else{
#define X(aa, bb, cc) std_trajectory_##cc = new systems::Callback<double, bb>(boost::ref(*spline_##cc)); 
    #include "type_table.h"
#undef X
        //stream_ft_std_trajectory = new systems::Callback<double, Hand::jp_type>          (boost::ref(*stream_ft_spline));
    }
	fs->close();
}

