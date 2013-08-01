#include "stdheader.h"
#include "data_stream.h" //for sensor data stream io
#include "hand_system.cxx" 
#include "wam_system.cxx" 
#include "sensor_stream_system.h" 
#include "display.h" 
#include "robot.h"
#include "play.h"
#include "rtmemory.h"
#include "utils.h"
#include "control_strategy.h"
#include "SaS.h"
#include "SaA.h"
//#include "co_type.h"

enum STATE {
#define X(aa, bb, cc, dd) aa,
    #include "play_table.h"
#undef X
} curr_state = STOPPED, last_state = STOPPED;
enum STRATEGY{
#define X(aa, bb, cc, dd) aa,
    #include "ctrl_table.h"
#undef X
};
enum VAR_TOGGLE{
#define X(aa, bb, cc, dd) aa,
    #include "var_table.h"
#undef X
};
enum MISC{
#define X(aa, bb, cc, dd) aa,
    #include "misc_table.h"
#undef X
};

static int loop_count = 0;

Play::Play(Robot* robot) : inputType(robot->get_memory()->get_float("trajectory_type")), time(robot->get_pm()->getExecutionManager()), is_init(false), loop_flag(false), playing(false){ 
    //cout << "Play instantiating...";
    this->robot = robot; 
    pm = robot->get_pm();
    wam = robot->get_wam();
    hand = robot->get_hand();
    instantiate_control_strategies();
    playName = robot->get_memory()->get_string("traj_name");
    zero_matrix(&qd_increase_count);
    zero_matrix(&cp_increase_count);
    cout << "Play instantiated!" << endl;
}
void Play::instantiate_control_strategies(){
/*#define X(aa, bb, cc, dd) control_strategies.push_back(new cc(robot->get_memory(),robot->get_controller()));
    #include "ctrl_table.h"
#undef X*/
    //cout << "Control strategies instantiated!" << endl;
}
// Initialization - Gravity compensating, setting safety limits, parsing input file and creating trajectories
bool Play::init() {
    robot->get_rtmemory()->set_play_name(playName);
    robot->get_rtmemory()->reset_output_counter();
    robot->get_rtmemory()->load_trajectory();
    if(robot->get_memory()->get_float("realtime_learning")){
        bool success = true;
        if(robot->get_memory()->get_float("pls_learning")) 
            success = robot->get_rtmemory()->load_beta_stream(true);
        if(robot->get_memory()->get_float("nb_learning")){
#define P(aa, bb, cc, dd, ee) \
            if(success) success = robot->get_rtmemory()->load_data_stream(true,enum_##cc); \
            if(success) robot->get_rtmemory()->load_data_stream(false,enum_##cc);
#include "parameter_table.h"
#undef P 
        }
    }
    //set_control_strategy(robot->get_memory()->get_float("default_control_strategy"));
    cout << "Play initialized!" << endl;
	return true;
}
void Play::move_to_start() {
    //cout << "moving to start..."; fflush(stdout);
	if (inputType == 0) {
		wam->moveTo( robot->get_rtmemory()->get_initial_jp() , true);
	} else{
        if(0 != playName.find("home"))//{ 
            wam->moveTo( robot->get_rtmemory()->get_initial_jp() , true);
        //}
		wam->moveTo( robot->get_rtmemory()->get_initial_tp() , true, 0.5, 0.5);
    }
    //wam->moveTo( robot->get_rtmemory()->get_initial_tp());//, true, 0.5, 0.5);
    //cout << "playback started!" << endl; fflush(stdout);
}
void Play::toggle_var(string name){
    robot->get_memory()->toggle_float(name);
}
void Play::set_control_strategy(int type){
    //cout << "setting cs " << type << endl;
    //strategy = control_strategies[type];
    //robot->get_rtmemory()->set_control_strategy(strategy);
}
void Play::loop(){
    loop_count++;
    cout << "loop# " << loop_count << endl;
    robot->get_rtmemory()->disconnect_systems();
    //robot->get_memory()->reload_vars();
    last_state = STOPPED;
    curr_state = PLAYING;
}
void Play::output_data_stream(){
    robot->get_rtmemory()->append_data_stream();
}
// This function will run in a different thread and control displaying to the screen and user input
void Play::user_control() {
// Some hand variables allow for switching between open and close positions
	Hand::jp_type currentPos(0.0);
	Hand::jp_type nextPos(M_PI);
	nextPos[3] = 0;
    
    //printf("Please press [Enter] to tare sensors: ");
    //barrett::detail::waitForEnter();
    //robot->get_rtmemory()->record_zero_values();

    char last_command;


// Instructions displayed to screen.
	printf("\n");
	printf("Commands:\n");
#define X(aa, bb, cc, dd) cout << bb << ": " << dd << endl;
    #include "play_table.h"
    #include "ctrl_table.h"
    #include "var_table.h"
    #include "misc_table.h"
#undef X
	//printf("  At any time, press [Enter] to open or close the Hand.\n");
	printf("\n");


	std::string line;
	while (true) {
		// Continuously parse our line input
		printf(">> ");
		std::getline(std::cin, line);
        if(!playing) break;
		if (line.size() == 0) { // Enter press recognized without any input
            line[0] = last_command;
            //hand->trapezoidalMove(nextPos, false);
            //std::swap(currentPos, nextPos);
		} else { // User input - Set State
			switch (line[0]) {
#define X(aa, bb, cc, dd) case bb: loop_flag = cc; curr_state = aa; break;
            #include "play_table.h"
#undef X
#define X(aa, bb, cc, dd) case bb: set_control_strategy((int)aa);break;
            #include "ctrl_table.h"
#undef X
#define X(aa, bb, cc, dd) case bb: toggle_var(cc);break;
            #include "var_table.h"
#undef X
#define X(aa, bb, cc, dd) case bb: cc;last_command = bb;break;
            #include "misc_table.h"
#undef X
			default:
				break;
			}
		}
	}
}

//cause robot to make a real-time step toward ultimate transformation goal
void Play::transform_cp(){
    //cout << "transform cp" << endl;
    cp_type cp = robot->get_memory()->get_transform_cp();
    for(int i = 0; i < cp_increase_count.size(); i++){
        //cout << "in for: " << i << endl;
        if(cp_increase_count[i] > 0){
            //cout << cp_increase_count[i] << ">0" << endl;
            cp[i] += robot->get_memory()->get_float("cp_rt_step");
            cp_increase_count[i]--;
        }
        else if(cp_increase_count[i] < 0){
            //cout << cp_increase_count[i] << "<0" << endl;
            cp[i] -= robot->get_memory()->get_float("cp_rt_step");
            cp_increase_count[i]++;
        }
    }
    robot->get_memory()->set_transform_cp(&cp);
}
void Play::transform_qd(){
    cp_type qd_euler = robot->get_memory()->get_transform_qd_euler();
    for(int i = 0; i < qd_increase_count.size(); i++){
        if(qd_increase_count[i] > 0){
            qd_euler[i] += robot->get_memory()->get_float("qd_rt_step");
            qd_increase_count[i]--;
        }
        else if(qd_increase_count[i] < 0){
            qd_euler[i] -= robot->get_memory()->get_float("qd_rt_step");
            qd_increase_count[i]++;
        }
    }
    robot->get_memory()->set_transform_qd(&qd_euler);
}
//ind: index of cartesian vector (0=x,1=y,2=z)
void Play::increase_cp(int ind){
    int step_count = robot->get_memory()->get_float("cp_step") / robot->get_memory()->get_float("cp_rt_step");
    cout << "increase cp " << ind << " by " << step_count << endl;
    cp_increase_count[ind] += step_count; 
}
void Play::increase_qd(int ind){
    int step_count = robot->get_memory()->get_float("qd_step") / robot->get_memory()->get_float("qd_rt_step");
    qd_increase_count[ind] += step_count; 
}
void Play::decrease_cp(int ind){
    int step_count = robot->get_memory()->get_float("cp_step") / robot->get_memory()->get_float("cp_rt_step");
    cp_increase_count[ind] -= step_count; 
}
void Play::decrease_qd(int ind){
    int step_count = robot->get_memory()->get_float("qd_step") / robot->get_memory()->get_float("qd_rt_step");
    qd_increase_count[ind] -= step_count; 
}

void Play::run(){
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DIMENSION);
	std::string line;
    printf("Please type name of trajectory to be played (press [Enter] for no change): ");
    std::getline(std::cin, line);
    if (line.size() == 0){ 
        playName = robot->get_memory()->get_string("traj_name");
    }
    else{
        //is_init = (playName.compare(line) == 0); playName = line;
        playName = line;
        robot->get_memory()->set_string("traj_name",playName);
    }
    
    //is_init = line.compare(playName);
    
    cout << "Play name set to: " << playName << endl;

    if(!is_init){
        is_init = init();
    }

    boost::thread displayThread(&Play::user_control, this);
        
    //cout << "tcp: " << robot->get_memory()->get_string(string("transform_cp_"+playName)) << endl;
    //cout << "tqd: " << robot->get_memory()->get_string(string("transform_qd_"+playName)) << endl;
    curr_state = STOPPED;
    float sleep_s = 0.002;
    playing = true;
    while (playing) {
        //cout << "updating sensors..." << endl;fflush(stdout);
        robot->update_sensors(); //important!!
        //perform any outstanding modifications to realtime robot pose
        transform_qd();
        transform_cp();
        //cout << "playing" << endl;fflush(stdout);
        if(robot->get_memory()->get_float("reload_vars")){
            robot->get_memory()->reload_vars();
            //toggle_var("reload_vars");
            robot->get_memory()->set_float("reload_vars", 0);
            robot->get_rtmemory()->reset_output_counter(robot->get_memory()->get_float("data_stream_index"));
        }
        switch (curr_state) {
        case QUIT:
            playing = false;
            break;
        case PLAYING:
            switch (last_state) {
            case STOPPED:
                cout << "STOPPED -> PLAYING" << endl; fflush(stdout);
                move_to_start();
                //if(robot->get_memory()->get_float("data_stream_out"))
                    robot->get_rtmemory()->init_data_logger();
                //robot->get_rtmemory()->init_param_logger();
                robot->get_rtmemory()->reconnect_systems();
                if(robot->get_memory()->get_float("record_pose")){
                    robot->get_rtmemory()->set_teach_name(playName);
                    robot->get_rtmemory()->prepare_log_file();
                    robot->get_rtmemory()->record();
                }
                robot->get_rtmemory()->start_playback();
                cout << "started playback" << endl; fflush(stdout);
                if(loop_count >= robot->get_memory()->get_float("loop_max")){
                    curr_state = STOPPED;
                    playing = false;
                }
                last_state = PLAYING;
                break;
            case PAUSED:
                //cout << "PAUSED -> PLAYING" << endl;fflush(stdout);
                robot->get_rtmemory()->start_playback();
                last_state = PLAYING;
                break;
            case PLAYING:
                //cout << "PLAYING -> PLAYING" << endl;fflush(stdout);
                if (robot->get_rtmemory()->playback_active()) {
                    btsleep(sleep_s);
                    //cout << "sleep complete" << endl;fflush(stdout);
                    //robot->update_sensors();
                    if(robot->get_memory()->get_float("realtime_learning")){
                        robot->get_rtmemory()->check_for_problems();
                    }
                    if(robot->get_memory()->get_float("realtime_control_break")){
                        robot->get_memory()->set_float("realtime_control_break",0);
                        loop();
                    }
                    break;
                } else if (loop_flag) {
                    //cout << "PLAYING -> PLAYING (loop)" << endl;fflush(stdout);
                    loop();
                    break;
                } else {
                    curr_state = STOPPED;
                    break;
                }
                default:
                break;
            }
            break;
        case PAUSED:
            switch (last_state) {
            case PLAYING:
                robot->get_rtmemory()->pause_playback();
                last_state = PAUSED;
                break;
            case PAUSED:
                btsleep(sleep_s);
                break;
            case STOPPED:
                break;
            default:
                break;
            }
            break;
        case STOPPED:
            switch (last_state) {
            case PLAYING:
                robot->get_rtmemory()->disconnect_systems();
                if(robot->get_memory()->get_float("record_pose")){
                    //cout << "mod: " << playName.find("mod") ;
                    if(0 != playName.find("mod")){ 
                        robot->get_rtmemory()->create_spline("_mod");
                    }
                    else{
                        robot->get_rtmemory()->create_spline("");
                    }
                }
                last_state = STOPPED;
                break;
            case PAUSED:
                robot->get_rtmemory()->disconnect_systems();
                //if(robot->get_memory()->get_float("record_pose"))
                    //robot->get_rtmemory()->create_spline();
                last_state = STOPPED;
                break;
            case STOPPED:
                btsleep(sleep_s);
                break;
            default:
                break;
            }
            break;
        case LOOPING:
            curr_state = PLAYING;
            break;
        }
    }
    robot->get_rtmemory()->disconnect_systems();
    if(robot->get_memory()->get_float("data_stream_out")){
        output_data_stream();
        //robot->get_rtmemory()->output_data_stream();
    }
    else
        cout << "NOT writing out data streams" << endl;
}
