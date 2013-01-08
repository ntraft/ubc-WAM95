X(JOINT_POSITIONS,    Wam<DIMENSION>::jp_type,  jp, robot->get_wam()->getJointPositions())   
X(JOINT_VELOCITIES,   Wam<DIMENSION>::jv_type,  jv, robot->get_wam()->getJointVelocities())   
#ifdef JT 
X(JOINT_TORQUES,      Wam<DIMENSION>::jt_type,  jt, robot->get_wam()->getJointTorques())   
#endif 
X(CARTESIAN_POSITION, cp_type,                  cp, robot->get_wam()->getToolPosition())   
#ifdef CO 
X(CARTESIAN_ORIENTATION,Hand::jp_type,          co, )   
#endif 
X(CARTESIAN_ORIENTATION_QD, Eigen::Quaterniond, qd, robot->get_wam()->getToolOrientation())   
X(CARTESIAN_FORCE,    cf_type,                  cf, robot->get_fts()->getForce())   
X(CARTESIAN_TORQUE,   ct_type,                  ct, robot->get_fts()->getTorque())   
#ifdef CA 
X(CARTESIAN_ACCELERATION,ca_type,               ca, robot->get_fts()->getAccel())   
#endif 
X(FINGERTIP_TORQUE,   Hand::jp_type,            ft, robot->get_senses()->get_fingertip_torques())   
X(TACTILE_SUM,        Hand::jp_type,            ts, robot->get_senses()->get_tactile_sums())   
