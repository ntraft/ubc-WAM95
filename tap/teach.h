#ifndef _TEACH_H
#define _TEACH_H
#include "stdheader.h"

class Robot;

//Teach Class

class Teach {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DIMENSION);
protected:
    Robot* robot;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
	Teach(Robot* _robot);
	bool init();
    void run();
	void display();
}
;
#endif
