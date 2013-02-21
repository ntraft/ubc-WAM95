#include "robot.h"
#include "rtmemory.h"
#include <iostream>
#include <assert.h>

using namespace std;

bool memory_test(){
    Memory* memory = new Memory();

    string var_name = "memory_test";

    cout << "Testing set_string + get_string methods" << endl;
    string str_test = "Hello";
    memory->set_string(var_name, str_test);
    assert(memory->get_string(var_name).compare(str_test));

    cout << "Testing set_float + get_float methods" << endl;
    float test = 12.345;
    memory->set_float(var_name, test);
    memory->get_float(var_name, test);
    assert(memory->get_float(var_name) == test);

    cout << "Testing toggle_float" << endl;
    memory->toggle_float(var_name, test);
    assert(memory->get_float(var_name) != test);

    cout << "Testing set_ & get_transform_qd methods" << endl;
    memory->set_transform_qd_x(test);
    memory->set_transform_qd_y(test);
    memory->set_transform_qd_z(test);
    assert( memory->get_transform_qd_x() == test && 
            memory->get_transform_qd_y() == test && 
            memory->get_transform_qd_z() == test);
    
    cout << "Testing set_ & get_transform_cp methods" << endl;
    memory->set_transform_cp_x(test);
    memory->set_transform_cp_y(test);
    memory->set_transform_cp_z(test);
    assert( memory->get_transform_cp_x() == test && 
            memory->get_transform_cp_y() == test && 
            memory->get_transform_cp_z() == test);


    cout << "Testing writeout_ & reload_vars methods" << endl;
    string str_test2 = "Hello World!";
    memory->set_string(var_name, str_test);
    cout << "Writing out variables..."
    writeout_vars();
    cout << "...done!" << endl;
    memory->set_string(var_name, str_test2);
    cout << "Reloading Variables..."
    reload_vars();
    cout << "...done!" << endl;
    assert(!memory->get_string(var_name).compare(str_test2));

}

int main(){
    RTMemory* rtmemory = new RTMemory();
    memory_test();
}

