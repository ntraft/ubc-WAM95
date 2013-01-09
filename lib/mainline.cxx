#include "mainline.h"

vector<MainLine*>* MainLine::command_line_stack = new vector<MainLine*>(); 

MainLine::MainLine(){
}
void MainLine::help(){
}
void MainLine::validate_args(){
}
void MainLine::run(){
    push(this);
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
    command_line_stack->pop_back();
    if(command_line_stack->empty())
        std::cout << module_name << "Exited Normally" << std::endl;
    else{
        command_line_stack->back()->run();
    }
}
void MainLine::push(MainLine* to_push){
    command_line_stack->push_back(to_push);
}
