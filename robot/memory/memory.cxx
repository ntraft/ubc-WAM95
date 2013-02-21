#include "memory.h" 
#include "var_server.cxx"
#include "utils.h"

Memory::Memory(){
    string_server = new VarServer<std::string>("conf/strings.yml");
    float_server = new VarServer<float>("conf/floats.yml");
    cout << "Memory instantiated!" << endl;
}
void Memory::reload_vars(){
    string_server->load_vars();
    float_server->load_vars();
}
void Memory::writeout_vars(){
    string_server->save_vars();
    float_server->save_vars();
}
std::string Memory::get_string(std::string name){
    return string_server->get_value(name);
}
float Memory::get_float(std::string name){
    return float_server->get_value(name);
}
void Memory::set_string(std::string name, std::string value){
    return string_server->set_value(name, value);
}
void Memory::set_float(std::string name, float value){
    return float_server->set_value(name, value);
}
void Memory::toggle_float(std::string name){
    //cout << name << " is " << get_float(name) << endl;
    if(get_float(name)){
        set_float(name, 0);
    }
    else{
        set_float(name, 1);
    }
    //cout << "set " << name << " to " << get_float(name) << endl;
}

void Memory::set_transform_qd(float x, float y, float z){
    set_float("transform_qd_x", x);
    set_float("transform_qd_y", y);
    set_float("transform_qd_z", z);
}
qd_type Memory::get_transform_qd(){
    transform_qd =
            AngleAxisd(get_float("transform_qd_x"),Vector3d::UnitX()) *
            AngleAxisd(get_float("transform_qd_y"),Vector3d::UnitY()) *
            AngleAxisd(get_float("transform_qd_z"),Vector3d::UnitZ()) ;
    return transform_qd;
}
void Memory::set_transform_cp(float x, float y, float z){
    set_float("transform_cp_x", x);
    set_float("transform_cp_y", y);
    set_float("transform_cp_z", z);
}
cp_type Memory::get_transform_cp(){
    transform_cp = cp_type(
            get_float("transform_cp_x") ,
            get_float("transform_cp_y") , 
            get_float("transform_cp_z"));
    return transform_cp;
}
jp_type Memory::get_initial_jp(){
    jp_type init_jp;
    parseDoubles(&init_jp, get_string("initial_jp"));
    return init_jp;
}
