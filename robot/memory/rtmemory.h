#ifndef RTMEMORY_H_
#define RTMEMORY_H_

#include "stdheader.h"
#include <boost/iostreams/copy.hpp>
#include "control_mode_switcher.h"
class Robot;
class Senses;
class RobotController;
class WamSystem;
class HandSystem;
class SensorStreamSystem;
class Qd2CoSystem;
class Co2QdSystem;
class CpSystem;
class RTControl;
class NaiveBayesSystem;
class RTControl2;
class Memory;
class ControlStrategy;
class PartialLeastSquaresSystem;
#include "parameter_estimator.h"



//RTMemory Class
class RTMemory {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DIMENSION);

//file I/O
std::string tmpStr, saveName, fileOut;
char* tmpFile;
std::vector<std::string> tmp_filenames;
string pose_tmp_filename;
std::string log_prefix;

    public:
//TaP Trajectory
std::string playName;
int inputType;
math::Spline<systems::Wam<DIMENSION>::jp_type>* jpSpline;
math::Spline<cp_type>* cpSpline;
math::Spline<Eigen::Quaterniond>* qdSpline;
math::Spline<co_type>* coSpline;
systems::Callback<double, systems::Wam<DIMENSION>::jp_type>* jpTrajectory;
systems::Callback<double, cp_type>* cpTrajectory;
systems::Callback<double, Eigen::Quaterniond>* qdTrajectory;
systems::Callback<double, co_type>* coTrajectory;

//Teach related
typedef boost::tuple<double, jp_type> input_jp_type;
typedef boost::tuple<double, cp_type> input_cp_type;
typedef boost::tuple<double, qd_type> input_qd_type;
typedef boost::tuple<double, co_type> input_co_type;
systems::TupleGrouper<double, jp_type> jpLogTg;
systems::TupleGrouper<double, pose_type> poseLogTg;
typedef boost::tuple<double, jp_type> jp_sample_type;
typedef boost::tuple<double, pose_type> pose_sample_type;
systems::PeriodicDataLogger<jp_sample_type>* jpLogger;
systems::PeriodicDataLogger<pose_sample_type>* poseLogger;

//Play related
systems::TupleGrouper<cp_type, Eigen::Quaterniond> poseTg;
std::vector<input_cp_type, Eigen::aligned_allocator<input_cp_type> >* cpVec;
std::vector<input_qd_type, Eigen::aligned_allocator<input_qd_type> >* qdVec;
std::vector<input_co_type, Eigen::aligned_allocator<input_co_type> >* coVec;
std::vector<input_jp_type, Eigen::aligned_allocator<input_jp_type> >* jpVec;
input_jp_type* jpSample;

    public:
systems::Ramp time;
    
//realtime data logging
string data_log_headers;
/**/
//input_streams' type definitions
typedef boost::tuple<double, pv_type,
#define X(aa, bb, cc, dd, ee) bb,
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
        double> input_stream_type;
typedef boost::tuple<double, 
#define X(aa, bb, cc, dd, ee) bb,
#include "input_type_table.h"
#undef X
        double> input_stream_wam_type;
typedef boost::tuple<double, 
#define X(aa, bb, cc, dd, ee) bb,
#include "tool_type_table.h"
#undef X
        double> input_stream_tool_type;
typedef boost::tuple<double, 
#define X(aa, bb, cc, dd, ee) bb,
#include "hand_type_table.h"
#undef X
        double> input_stream_hand_type;
//tuple groupers
systems::TupleGrouper<double, 
        pv_type,
#define X(aa, bb, cc, dd, ee) bb,
#include "input_type_table.h"
#include "tool_type_table.h"
#undef X
        double> tg;
systems::TupleGrouper<double, 
#define X(aa, bb, cc, dd, ee) bb,
#include "input_type_table.h"
#undef X
        double> wam_tg;
systems::TupleGrouper<double, 
#define X(aa, bb, cc, dd, ee) bb,
#include "tool_type_table.h"
#undef X
        double> tool_tg;
systems::TupleGrouper<double, 
#define X(aa, bb, cc, dd, ee) bb,
#include "hand_type_table.h"
#undef X
        double> hand_tg;

/*
typedef boost::tuple<double, 
#define P(aa, bb, cc, dd, ee) \
        bb,
#include "parameter_table.h"
#undef P
        double> input_stream_type;

systems::TupleGrouper<double, 
#define P(aa, bb, cc, dd, ee) \
        bb,
#include "parameter_table.h"
#undef P
        double> tg;
*/
int STREAM_SIZE;
int PERIOD_MULTIPLIER;
int output_counter;

    private:
systems::PeriodicDataLogger<input_stream_type>* logger;
pv_type param_vec;

//realtime logfile reading:
//  1. vectors (input_type_xx) of data points are built from data samples (lines of an input file)
//  2. splines are built from these data points
//  3. sensorimotor trajectories are built from the splines
//  note: see lib/input_type_table.h & lib/tool_type_table.h
#define X(aa, bb, cc, dd, ee) \
    typedef boost::tuple<double, bb> input_type_##cc; \
    input_type_##cc* sample_##cc; \
    std::vector<input_type_##cc, Eigen::aligned_allocator<input_type_##cc> >* vec_##cc; \
    math::Spline<bb>* spline_##cc;
    #include "input_type_table.h"
    #include "tool_type_table.h"
#undef X
#define P(aa, bb, cc, dd, ee) \
    typedef boost::tuple<double, in_type> input_type_##cc; \
    input_type_##cc* sample_##cc; \
    std::vector<input_type_##cc, Eigen::aligned_allocator<input_type_##cc> >* vec_##cc; \
    math::Spline<in_type>* spline_##cc;
    #include "parameter_table.h"
#undef P
    public:
#define X(aa, bb, cc, dd, ee) \
    systems::Callback<double, bb>* trajectory_##cc; \
    systems::Callback<double, bb>* mean_trajectory_##cc; \
    systems::Callback<double, bb>* std_trajectory_##cc; \
    bb problem_count_##cc;
    #include "input_type_table.h"
    #include "tool_type_table.h"
#undef X
#define P(aa, bb, cc, dd, ee) \
    systems::Callback<double, in_type>* beta_trajectory_##cc; 
    #include "parameter_table.h"
#undef P
#define X(aa, bb, cc, dd, ee) \
    vector<systems::Callback<double, bb>* > mean_trajectory_vec_##cc; \
    vector<systems::Callback<double, bb>* > std_trajectory_vec_##cc;
    #include "input_type_table.h"
    #include "tool_type_table.h"
#undef X


protected:
	Robot* robot;
    ProductManager* pm;
    Memory* memory;
    Senses* senses;
    RobotController* control;
	systems::Wam<DIMENSION>* wam;
	Hand* hand;
    HandSystem* hand_system; //for realtime data logging of hand sensors
    WamSystem* wam_system; //for realtime manipulation of wam trajectory
    SensorStreamSystem* sss; //for realtime logging all sensor data readings 
    Qd2CoSystem* qd2co_system; //for realtime conversions between Quaternion and Matrix
    Co2QdSystem* co2qd_system; //for realtime conversions between Quaternion and Matrix
    CpSystem* cp_system; //for realtime cartesian position control
    RTControl* rtc; //for realtime manipulation of robot 
    ExposedOutput<pv_type>* param_output_system; //to record environment parameters
    PartialLeastSquaresSystem* pls_system; //to record environment parameters
    jp_type initial_jp;
    //bt_type pls_beta;

//parameter estimation
    vector<NaiveBayesSystem*> nbs_vec;
    //ParameterEstimator* param_estimator;
    /*typedef ParameterEstimator< 
#define P(aa, bb, cc, dd, ee) \
        bb,
#include "parameter_table.h"
#undef P 
        double> pe_type;
    pe_type param_estimator;
    systems::TupleSplitter< 
#define P(aa, bb, cc, dd, ee) \
        bb,
#include "parameter_table.h"
#undef P 
        double> param_splitter;
    typedef typename pe_type::tuple_type tuple_type;
    */
typedef boost::tuple<
#define P(aa, bb, cc, dd, ee) \
        bb,
#include "parameter_table.h"
#undef X
        double> parameter_tuple_type;
systems::TupleGrouper<
#define P(aa, bb, cc, dd, ee) \
        bb,
#include "parameter_table.h"
#undef P 
        double> pg;
#define P(aa, bb, cc, dd, ee) \
    ostringstream param_ostream_##cc;\
	PrintToStream<bb>* param_outputter_##cc;
#include "parameter_table.h"
#undef P
systems::PeriodicDataLogger<pose_sample_type>* pose_logger;


#define X(aa, bb, cc, dd, ee) \
    bb zero_value_##cc;
    #include "input_type_table.h"
    #include "tool_type_table.h"
#undef X

public:
	int dataSize;
	bool loop;
    bool problem;
    stringstream hand_debug;
    stringstream rtc_debug;
    stringstream nbs_debug;
	RTMemory(ProductManager* _pm, Wam<DIMENSION>* _wam, Memory* _memory, Senses* _senses, RobotController* _control); 
    void init();
    //teach
    void set_teach_name(string saveName);
    void set_initial_jp();
    bool prepare_log_file();
    void record();
    void create_spline(string suffix = "");
    //play
    void set_play_name(string playName);
    void reset_output_counter(int base = 0);
    void record_zero_values();
    bool load_trajectory();
	void init_data_logger();
	void init_pose_logger();
    void output_data_stream();
    void output_pose_stream();
    void append_data_stream();
    bool load_data_stream(bool);
    bool load_beta_stream(bool);
    bool load_data_stream(bool, enum parameters);
    void reset_time();
    void start_playback();
    void pause_playback();
    bool playback_active();
    void disconnect_systems();
    void reconnect_systems();
    string get_log_directory();
    double get_probability_non_normalized(enum parameters parameter);
    double get_probability(enum parameters parameter);
    void check_for_problems();
    void set_control_strategy(ControlStrategy* strategy);
    void set_environment_param(pv_type param);
    void set_data_stream_index(int index);
    //accessor
    jp_type get_initial_jp();
    pose_type get_initial_tp();

private:
	DISALLOW_COPY_AND_ASSIGN(RTMemory);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
