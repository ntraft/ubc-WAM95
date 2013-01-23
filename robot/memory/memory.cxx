#include "memory.h"
#include "var_server.cxx"

Memory::Memory(){
    string_server = new VarServer<string>("conf/strings.yml");
    float_server = new VarServer<float>("conf/floats.yml");
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
