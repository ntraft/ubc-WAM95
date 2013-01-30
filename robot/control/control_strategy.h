#ifndef _CONTROL_STRATEGY_H
#define _CONTROL_STRATEGY_H

#include "stdheader.h"
class Memory;
class RobotController;

class ControlStrategy{
protected:
    Memory* memory;
    RobotController* controller;
public:
    ControlStrategy(Memory* _memory, RobotController* _controller);
    virtual void invoke(jp_type* output_jp, 
            const Hand::jp_type* expected_mean_ft, 
            const Hand::jp_type* actual_ft, 
            Hand::jp_type* problem_count_ft);
    virtual void invoke(
            cp_type* offsets_cp, 
            qd_type* transform_qd, 
            const ct_type* expected_mean_ct, 
            const ct_type* actual_ct, 
            ct_type* problem_count_ct);
};

#endif
