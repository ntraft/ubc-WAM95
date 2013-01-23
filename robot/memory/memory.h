#ifndef IO_H_
#define IO_H_
#include "stdheader.h"

using namespace std;

template<class T>class VarServer;

class Memory{
private:
    VarServer<float>* float_server;
    VarServer<string>* string_server;
public:
    Memory();
    string get_string(string name);
    float get_float(string name);
    void set_string(string name, string value);
    void set_float(string name, float value);
};

#endif /* IO_H_ */
