#ifndef TAP_H_
#define TAP_H_

#include "stdheader.h"
#include "mainline.h"

class Robot;
class Teach;
class Play;

class TeachAndPlay: public MainLine{
private:

protected:
    Robot* robot;
    Teach* teach;
    Play* play;
    //Loop* loop;
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    TeachAndPlay(Robot* robot);

    void init_tap();
    void init_play();
    void init_teach();
    void help();
	virtual void run();
};

#endif
