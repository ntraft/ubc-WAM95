#include "stdheader.h"
#include "senses.h"
#include "sensor_stream_system.h"

void SensorStreamSystem::operate() {
#define X(aa, bb, cc, dd, ee) \
    readings_##cc = dd; \
    output_value_##cc->setData(&readings_##cc);
    #include "input_type_table.h"
    #include "tool_type_table.h"
#undef X
#define P(aa, bb, cc, dd, ee) \
    params[ee] = memory->get_float(dd);
#include "parameter_table.h"
#undef P 
    output_value_param->setData(&params);
    //robot->get_rtmemory()->set_environment_param(params);
}

