//Submenus
X(T, 't', tap->run(), "run teach and play interface", "N/A")
//WAM
X(G, 'g', robot->get_wam()->gravityCompensate(), "compensate for Gravity with WAM", "N/A")
X(J, 'j', robot->get_controller()->move_wam_to_str(new jp_type(), "joint positions", line.substr(1)), "go to joint position", "7 #s separated by spaces")
X(I, 'i', robot->get_controller()->idle(), "idle wam", "N/A")
X(H, 'h', robot->get_controller()->home(), "return to home position", "N/A")
//Hand
X(A, 'a', robot->get_controller()->init_hand(), "initialize hand", "N/A")
X(B, 'b', robot->get_controller()->backdrive_hand(), "toggle hand backdrivability", "N/A")
//Miscellaneous
X(S, 's', robot->get_senses()->display(), "show realtime sensor output (q to quit)", "N/A")
X(Q, 'q', quit = true, "quit program", "N/A")
X(R, 'r', run_pls_regress(), "run pls regress", "N/A")

#ifdef DEPRECATED_MENU_ITEMS 
X(P, 'p', move_wam_to_str(&cp, "tool position", line.substr(1)), "go to tool  position", "4 #s separated by spaces")
X(W, 'w', move_hand_to_str(&hjp, "joint positions", line.substr(1)), "go to hand position", "4 #s separated by spaces")
X(G, 'g', controller->grasp(), "grasp object", "N/A")
X(U, 'u', controller->ungrasp(), "ungrasp object", "N/A")
X(R, 'r', experiment->run(line), "to run experiment", "N/A")
X(T, 't', senses->tare(), "to tare the tactile and fingertip_torque sensors", "N/A")
X(O, 'o', controller->hold_orientation(), "to cause WAM to hold its current orientation", "N/A")
X(D, 'd', experiment->toggle_collect_data(), "to toggle data collection on/off (default off)", "N/A")
X(ONE, '1', experiment->teach_pose(0), "to record WAM bottom joint angles", "N/A")
X(TWO, '2', experiment->teach_pose(1), "to record WAM top joint angles", "N/A")
#endif
