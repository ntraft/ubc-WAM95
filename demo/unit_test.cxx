#include "stdheader.h"
#include "robot.h"
#include "memory.h"
#include "rtmemory.h"
#include <iostream>
#include <assert.h>

using namespace std;

bool memory_test(){
    cout << "Testing Memory System" << endl;
    Memory* memory = new Memory();

    string var_name = "memory_test";

    cout << "Testing set_string + get_string methods" << endl;
    string str_test = "Hello";
    memory->set_string(var_name, str_test);
    assert(memory->get_string(var_name).compare(str_test) == 0);

    cout << "Testing set_float + get_float methods" << endl;
    float test = 12.345;
    memory->set_float(var_name, test);
    assert(memory->get_float(var_name) == test);

    cout << "Testing toggle_float" << endl;
    memory->toggle_float(var_name);
    assert(memory->get_float(var_name) != test);

    cout << "Testing set_ & get_transform_qd methods" << endl;
    memory->set_transform_qd(test, test, test);
    qd_type qd_test = memory->get_transform_qd();
    qd_type qd_set = 
                AngleAxisd(test, Vector3d::UnitX()) *
                AngleAxisd(test, Vector3d::UnitY()) *
                AngleAxisd(test, Vector3d::UnitZ()) ;
    assert( qd_test.w() == qd_set.w() && 
            qd_test.x() == qd_set.x() && 
            qd_test.y() == qd_set.y() && 
            qd_test.z() == qd_set.z() ); 
    
    cout << "Testing set_ & get_transform_cp methods" << endl;
    memory->set_transform_cp(test, test, test);
    cp_type cp_test = memory->get_transform_cp();
    assert( cp_test[0] == test && 
            cp_test[1] == test && 
            cp_test[2] == test);


    cout << "Testing writeout_ & reload_vars methods" << endl;
    string str_test2 = "Hello World!";
    memory->set_string(var_name, str_test);
    cout << "Writing out variables...";
    memory->writeout_vars();
    cout << "...done!" << endl;
    memory->set_string(var_name, str_test2);
    cout << "Reloading Variables...";
    memory->reload_vars();
    cout << "...done!" << endl;
    assert(memory->get_string(var_name).compare(str_test2) != 0);

    cout << "Memory Test Complete!" << endl;

    return true;
}

int main(){
    memory_test();
}

