#include "stdheader.h"
#include "senses.h"
#include "sensor_stream_system.h"
#include "utils-inl.h"

void SensorStreamSystem::operate() {
    //ostringstream sensor_stringstream;
#define X(aa, bb, cc, dd, ee) \
    readings_##cc = dd; \
    output_value_##cc->setData(&readings_##cc); \
    //sensor_stringstream << to_string(&readings_##cc," ") << " ";
    #include "input_type_table.h"
    #include "tool_type_table.h"
#undef X
#define P(aa, bb, cc, dd, ee) \
    params[ee] = memory->get_float(dd);
#include "parameter_table.h"
#undef P 
    //parseDoubles(&sensor_vec, sensor_stringstream.str());
    output_value_param->setData(&params);
    //output_value_sensor_vec->setData(&sensor_vec);
    //robot->get_rtmemory()->set_environment_param(params);
}

