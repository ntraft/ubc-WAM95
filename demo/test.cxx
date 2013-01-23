#include "robot.h"
#include <iostream>

using namespace std;

int main(){
   Robot robot;
   cout << robot.get_memory()->get_float("test") << endl; 
}
