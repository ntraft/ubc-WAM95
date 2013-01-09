#ifndef TAP_H_
#define TAP_H_

#include "stdheader.h"
#include "mainline.h"

class Robot;
class Teach;

class TeachAndPlay: public MainLine{
private:

protected:
    Robot* robot;
    Teach* teach;
    //Play* play;
    //Loop* loop;
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    TeachAndPlay(Robot* robot);

    void init(std::string args);
    void help();
	virtual void run();
};

#endif
