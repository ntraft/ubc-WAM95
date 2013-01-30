#ifndef _SAS_h
#define _SAS_h

#include "stdheader.h"
#include "control_strategy.h"

class RobotController;
class Memory;

class SaS : public ControlStrategy{
public:
    SaS(Memory* _memory, RobotController* _controller);
    virtual void invoke(jp_type* output_jp, 
            const Hand::jp_type* expected_mean_ft, 
            const Hand::jp_type* actual_ft, 
            Hand::jp_type* problem_count_ft);
};

#endif
