//follow the template to add new input vectors to the sensorimotor data stream
//X("<NAME_OF_SENSOR>", <vector_data_type>, <unique_variable_name>, <function_returning_new_vector_reading>, <count_starts_from_2_max_8>)
X("JOINT_POSITIONS",    Wam<DIMENSION>::jp_type,  jp, senses->get_wam()->getJointPositions(), 2)
X("JOINT_VELOCITIES",   Wam<DIMENSION>::jv_type,  jv, senses->get_wam()->getJointVelocities(), 3)
X("JOINT_TORQUES",      Wam<DIMENSION>::jt_type,  jt, senses->get_wam()->getJointTorques(), 4)
X("CARTESIAN_POSE",     Wam<DIMENSION>::jp_type,  cp, senses->get_tool_pose(),5) 
X("CARTESIAN_FORCE",          cf_type,            cf, senses->get_force(), 6)
X("CARTESIAN_TORQUE",         ct_type,            ct, senses->get_torque(), 7)
//X("CARTESIAN_ACCELERATION",   ca_type,            ca, senses->get_accel(), 7)
//X("FINGERTIP_TORQUE",         Hand::jp_type,      ft, senses->get_fingertip_torques(), 7)
//X("TACTILE_VEC",              tv_type,            tv, senses->get_tactile_vector(), 8)
