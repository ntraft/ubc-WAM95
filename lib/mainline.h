#ifndef MLINE_H_
#define MLINE_H_

#include "stdheader.h"

class MainLine{
protected:
    std::string module_name;
    std::vector<std::string> autoCmds;
    std::string line;

public:
    MainLine();

protected:
    virtual void help();
    virtual void validate_args();
    virtual void run();
    void exit();
    void step();
};
#endif
