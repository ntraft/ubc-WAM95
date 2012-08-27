#include "tactile.h"
/*TactileTestExperiment*/
/*
	test tactile sensor by moving the finger in a repeatable raster motion
	spherical objects (marbles/ball bearings) are then to be placed beneath
	the tactile surface. the purpose is to determine if edges can be reliably
	detected by the sensors or if significant aliasing occurs
*/
/*WAMCartesianPos4*/
void CartesianRaster::run(){
	load_exp_variables();
	
	//Eigen::Quaterniond exp_vars[WAM_BOTTOM]Q = hjp2quaternion(&exp_vars[WAM_BOTTOM_O]);
	systems::Wam<DIMENSION>::cp_type curr_pos;
	
	//start experiment: move WAM to first goal
    wam->moveTo(exp_vars[WAM_BOTTOM_C], true, 3.0);
	wam->moveTo(exp_vars[WAM_BOTTOM]Q, true, 0.3, 0.25);
	
	std::cout << "start!" << std::endl;
	int curr_state = -1;
	//int num_runs = 5;

	//printf("/%02d   x_pos\n",num_runs);
	//fflush(stdout);
	std::cout << "x_pos" << std::endl;
	
	float min_x = exp_vars[WAM_BOTTOM_C][0];
	float min_y = exp_vars[WAM_BOTTOM_C][1];
	float max_x = exp_vars[WAM_TOP_C][0];
	float max_y = exp_vars[WAM_TOP_C][1];
	
	float step = 0.003;	//3mm steps
	curr_pos[0] = min_x;
	curr_pos[1] = min_y;
	curr_pos[2] = exp_vars[WAM_BOTTOM_C][2];	//want tool to remain in the same horizontal plane
	
	float x, y;		//temp coordinates of locations on tactile finger pad edges
	float x0, y0;	//coordinates of centre point of tactile finger pad (point of rotation)
	x = min_x;
	x0 = max_x - min_x;
	y0 = max_y - min_y;
	float theta = 0.523;	//30 degree rotation
	 
	while((curr_pos[0] < max_x || curr_pos[1] < max_y) && !expsemastop){
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		
		if(wam.moveIsDone()){++curr_state;}else{continue;}
		
		switch(expshape){
			case CIRCLE:{
				if(curr_state%2 == 0){	//even
					//swap y values
					if(curr_pos[1] == max_y)
						curr_pos[1] = min_y;
					else
						curr_pos[1] = max_y;
				}
				else{	//odd
					curr_pos[0] += step;
				}
				break;
			}
			case SQUARE:{
				if(curr_state%2 == 0){	//even
					//swap x values
					if(curr_pos[0] == max_x)
						curr_pos[0] = min_x;
					else
						curr_pos[0] = max_x;
				}
				else{	//odd
					curr_pos[1] += step;
				}
				break;
			}
			case TRIANGLE:{
				if(curr_state%4 == 0){	//first of 4 steps
					y = min_y;
				}
				if(curr_state%4 == 2){	//third of 4 steps
					y = max_y;
				}
				else{	//second or fourth of 4 steps
					x += step;
				}
				//rotate about tactile pad centre point by theta radians
				curr_pos[0] = x0+(x-x0)*cos(theta)+(y-y0)*sin(theta);
				curr_pos[1] = y0-(x-x0)*sin(theta)+(y-y0)*cos(theta);
				break;
			}
		}
		
		//printf(" %02d ",curr_x);
		//fflush(stdout);
		std::cout << to_string(&curr_pos) << std::endl;
		(*((systems::Wam<DIMENSION>*)(&wam))).moveTo(curr_pos, false, 3.0);
	}
}
