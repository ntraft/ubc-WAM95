#include "mainline.h"

MainLine::MainLine(){
}
void MainLine::help(){
}
void MainLine::validate_args(){
}
void MainLine::run(){
}
void MainLine::step(){
    if (autoCmds.empty()) {
        printf(">>> ");
        std::getline(std::cin, line);
    } else {
        line = autoCmds.back();
        autoCmds.pop_back();
    }
}
void MainLine::exit(){
    std::cout << module_name << "Exited Normally" << std::endl;
}
