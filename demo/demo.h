#ifndef DEMO_H_
#define DEMO_H_

#include "stdheader.h"

class Demo: public MainLine{
private:
public:
    Demo();

    virtual void help();
    virtual void validate_args();
    virtual void run();
};
