#include "memory.h" 
#include "var_server.cxx"

Memory::Memory(){
    string_server = new VarServer<string>("conf/strings.yml");
    float_server = new VarServer<float>("conf/floats.yml");
}
void Memory::reload_vars(){
    string_server->load_vars();
    float_server->load_vars();
}
string Memory::get_string(string name){
    return string_server->get_value(name);
}
float Memory::get_float(string name){
    return float_server->get_value(name);
}
void Memory::set_string(string name, string value){
    return string_server->set_value(name, value);
}
void Memory::set_float(string name, float value){
    return float_server->set_value(name, value);
}
void Memory::toggle_float(string name){
    bool b = (bool)get_float(name);
    float f = (float)(!b);
    cout << name << " set to " << f << "." << endl;
    set_float(name, f);
}

void Memory::set_qd_transform(float x, float y, float z){
    set_float("qd_transform_x", x);
    set_float("qd_transform_y", y);
    set_float("qd_transform_z", z);
}
qd_type Memory::get_qd_transform(){
    transform_qd =
            AngleAxisd(get_float("qd_transform_x"),Vector3d::UnitX()) *
            AngleAxisd(get_float("qd_transform_y"),Vector3d::UnitY()) *
            AngleAxisd(get_float("qd_transform_z"),Vector3d::UnitZ()) ;
    return transform_qd;
}
