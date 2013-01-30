#ifndef RTMEMORY_H_
#define RTMEMORY_H_

#include "stdheader.h"
#include "control_mode_switcher.h"
class Robot;
class Senses;
class RobotController;
class WamSystem;
class HandSystem;
class SensorStreamSystem;
class Qd2CoSystem;
class RTControl;
class RTControl2;
class Memory;
class ControlStrategy;
//class co_type;

//RTMemory Class
class RTMemory {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DIMENSION);

//file I/O
std::string tmpStr, saveName, fileOut;
char* tmpFile;
std::vector<std::string> tmp_filenames;
std::string log_prefix;

    public:
//TaP Trajectory
std::string playName;
int inputType;
math::Spline<systems::Wam<DIMENSION>::jp_type>* jpSpline;
math::Spline<cp_type>* cpSpline;
math::Spline<Eigen::Quaterniond>* qdSpline;
systems::Callback<double, systems::Wam<DIMENSION>::jp_type>* jpTrajectory;
systems::Callback<double, cp_type>* cpTrajectory;
systems::Callback<double, Eigen::Quaterniond>* qdTrajectory;

//Teach related
typedef boost::tuple<double, jp_type> input_jp_type;
typedef boost::tuple<double, cp_type> input_cp_type;
typedef boost::tuple<double, qd_type> input_qd_type;
systems::TupleGrouper<double, jp_type> jpLogTg;
systems::TupleGrouper<double, pose_type> poseLogTg;
typedef boost::tuple<double, jp_type> jp_sample_type;
typedef boost::tuple<double, pose_type> pose_sample_type;
systems::PeriodicDataLogger<jp_sample_type>* jpLogger;
systems::PeriodicDataLogger<pose_sample_type>* poseLogger;

//Play related
systems::TupleGrouper<cp_type, Eigen::Quaterniond> poseTg;
std::vector<input_cp_type, Eigen::aligned_allocator<input_cp_type> >* cpVec;
//input_cp_type* cpSample;
std::vector<input_qd_type, Eigen::aligned_allocator<input_qd_type> >* qdVec;
//input_qd_type* qdSample;
std::vector<input_jp_type, Eigen::aligned_allocator<input_jp_type> >* jpVec;
input_jp_type* jpSample;

    public:
systems::Ramp time;
    
//realtime data logging
string data_log_headers;
typedef boost::tuple<double, 
#define X(aa, bb, cc, dd, ee) bb,
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
        jp_type> input_stream_type;

systems::TupleGrouper<double, 
#define X(aa, bb, cc, dd, ee) bb,
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
        jp_type> tg;

const static int STREAM_SIZE = 1+7+7+7+3+4+4+1;

    private:
systems::PeriodicDataLogger<input_stream_type>* logger;

//realtime logfile reading:
//  1. vectors (input_type_xx) of data points are built from data samples (lines of an input file)
//  2. splines are built from these data points
//  3. sensorimotor trajectories are built from the splines
//  note: see lib/wam_type_table.h
#define X(aa, bb, cc, dd, ee) typedef boost::tuple<double, bb> input_type_##cc;
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) input_type_##cc* sample_##cc;
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) std::vector<input_type_##cc, Eigen::aligned_allocator<input_type_##cc> >* vec_##cc;
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) math::Spline<bb>* spline_##cc;
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X

    public:
#define X(aa, bb, cc, dd, ee) systems::Callback<double, bb>* trajectory_##cc;
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) systems::Callback<double, bb>* mean_trajectory_##cc;
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) systems::Callback<double, bb>* std_trajectory_##cc;
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) bb problem_count_##cc;
        #include "wam_type_table.h"
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
    RTControl* rtc; //for realtime manipulation of robot 
	
public:
	int dataSize;
	bool loop;
    bool problem;
    stringstream hand_debug;
    stringstream rtc_debug;
	RTMemory(ProductManager* _pm, Wam<DIMENSION>* _wam, Memory* _memory, Senses* _senses, RobotController* _control); 
    void init();
    //teach
    void set_teach_name(string saveName);
    bool prepare_log_file();
    void record();
    void create_spline();
    //play
    void set_play_name(string playName);
    bool load_trajectory();
	void init_data_logger();
    void output_data_stream();
    bool load_data_stream(bool);
    void start_playback();
    void pause_playback();
    bool playback_active();
    void disconnect_systems();
    void reconnect_systems();
    void check_for_problems();
    void set_control_strategy(ControlStrategy* strategy);
    //accessor
    jp_type get_initial_jp();
    pose_type get_initial_tp();

private:
	DISALLOW_COPY_AND_ASSIGN(RTMemory);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
