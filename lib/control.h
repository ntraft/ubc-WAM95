#ifndef CONTROL_H_
#define CONTROL_H_

#include "stdheader.h"

//Close all fingers
void closeHand(Hand* hand);
//Open all fingers
void openHand(Hand* hand);
//Close all fingers until contacts detected
void graspObject(Hand* hand);
//open all fingers and reset finger contact flags
void ungraspObject(Hand* hand);
void backDriveHand(Hand* hand, ForceTorqueSensor* fts, void* wamin, ProductManager* pm);
template<size_t DOF, int R, int C, typename Units>
void moveToStr(systems::Wam<DOF>& wam, math::Matrix<R,C, Units>* dest,
		const std::string& description, const std::string& str)
{
	if (parseDoubles(dest, str)) {
		std::cout << "Moving to " << description << ": " << *dest << std::endl;
		wam.moveTo(*dest);
	} else {
		printf("ERROR: Please enter exactly %d numbers separated by "
				"whitespace.\n", dest->size());
	}
}
void moveToStr(Hand* hand, Hand::jp_type* dest,
		const std::string& description, const std::string& str);
double velCommand(bool open, bool close, double speed = 1.25);
void handCommand(Hand* hand, unsigned char data);

#endif
