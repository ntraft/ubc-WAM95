#include "stdheader.h"
#include "senses.h"
#include "sensor_stream_system.h"

void SensorStreamSystem::operate() {
#define X(aa, bb, cc, dd, ee) \
    readings_##cc = dd; \
    output_value_##cc->setData(&readings_##cc);
    #include "wam_type_table.h"
    #include "tool_type_table.h"
#undef X
}

