#include "stdheader.h"
#include "senses.h"
#include "sensor_stream_system.h"

void SensorStreamSystem::operate() {
        //senses->get_hand()->update(Hand::S_FINGERTIP_TORQUE,true); //update hand sensors
        //senses->get_hand()->update(Hand::S_POSITION,true); //update hand sensors
        //senses->get_hand()->update(Hand::S_ALL,true); //tactile sensors must be updated out of realtime
#define X(aa, bb, cc, dd, ee) readings_##cc = dd;
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) output_value_##cc->setData(&readings_##cc);
        #include "wam_type_table.h"
        #include "tool_type_table.h"
#undef X
}

