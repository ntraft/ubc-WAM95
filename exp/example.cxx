#include "experiment.h"
#include "action.h"
#include "utils.h"
#include "control.h"
#include "senses.h"

Example::Example(Controller* controller, Senses* senses)
: Experiment(controller, senses){
}
HoldPosition::HoldPosition(Controller* controller, Senses* senses)
: Example(controller, senses){
}
SystemsIntro::SystemsIntro(Controller* controller, Senses* senses)
: Example(controller, senses){
}
RealtimeMove::RealtimeMove(Controller* controller, Senses* senses)
: Example(controller, senses){
}
TeachAndPlay::TeachAndPlay(Controller* controller, Senses* senses)
: Example(controller, senses){
}
TorqueControl::TorqueControl(Controller* controller, Senses* senses)
: Example(controller, senses){
}
Haptics::Haptics(Controller* controller, Senses* senses)
: Example(controller, senses){
}

void HoldPosition::print_menu() {
	printf("Commands:\n");
	printf("  j  Hold joint positions\n");
	printf("  p  Hold tool position (in Cartesian space)\n");
	printf("  o  Hold tool orientation\n");
	printf("  b  Hold both tool position and orientation\n");
	printf("  i  Idle (release position/orientation constraints)\n");
	printf("  q  Quit\n");
}

void HoldPosition::run(){
	wam->gravityCompensate();
	print_menu();

	std::string line;
	bool going = true;
	while (going) {
		printf(">>> ");
		std::getline(std::cin, line);

		switch (line[0]) {
		case 'j':
			printf("Holding joint positions.\n");
			wam->moveTo(wam->getJointPositions());
            print_menu();
			break;
		case 'p':
			printf("Holding tool position.\n");
			wam->moveTo(wam->getToolPosition());
			print_menu();
            break;

		case 'o':
			printf("Holding tool orientation.\n");
			wam->moveTo(wam->getToolOrientation());
			print_menu();
            break;

		case 'b':
			printf("Holding both tool position and orientation.\n");
			wam->moveTo(wam->getToolPose());
			print_menu();
            break;

		case 'i':
			printf("WAM idled.\n");

			// Note that this use of the word "idle" does not mean "Shift-idle".
			// Calling Wam::idle() will disable any of the controllers that may
			// be connected (joint position, tool position, tool orientation,
			// etc.) leaving only gravity compensation. (More specifically,
			// Wam::idle() disconnects any inputs that were connected using
			// Wam::trackReferenceSignal().)
			wam->idle();
			  print_menu();
		break;

		case 'q':
		case 'x':
			printf("Quitting.\n");
			going = false;
			print_menu();
		break;

		default:
			if (line.size() != 0) {
				printf("Unrecognized option.\n");
				print_menu();
			}
		    break;
		}
	}

	// Release the WAM if we're holding. This is convenient because it allows
	// users to move the WAM back to some collapsed position before exiting, if
	// they want.
	wam->idle();

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
