#ifndef STDHEADER_H_
#define STDHEADER_H_

//system includes
#include <stdexcept>
#include <syslog.h>
#include <unistd.h> /* for close() */
#include <sys/socket.h> /* For sockets */
#include <fcntl.h>      /* To change socket to nonblocking mode */
#include <arpa/inet.h>  /* For inet_pton() */
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>  // For strtod()
#include <algorithm>    //for min/max
#include <unistd.h> // For usleep()
#include <math.h>       //for sin/co
#include <stdio.h>
#include <stdlib.h>
#include <time.h>


//boost includes
#include <boost/thread/thread.hpp>
#include <boost/thread.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/filesystem.hpp>  //for create_directories
#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

//barrett includes
#include <barrett/log.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/thread/disable_secondary_mode_warning.h>
#include <barrett/units.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/products/hand.h>
#include <barrett/math.h>  // For barrett::math::saturate()
#include <barrett/exception.h>

//eigen includes
#include <Eigen/Core>
#include <Eigen/StdVector>
//#define EIGEN_DONT_ALIGN_STATICALLY
#define EIGEN_DONT_ALIGN 1
//#define EIGEN_DONT_VECTORIZE
//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

//reading config files
//#include <libconfig.h++>
//#include "control_mode_switcher.h"

// The ncurses library allows us to write text to any location on the screen
#include <curses.h>


#define DIMENSION 7u
#define FINGERTIP_TORQUE2TORQUE_RATIO 118.0     //convert hand fingertip_torque to N-m
#define FINGER_JOINT_LIMIT 2.4435       //=140 degrees
#define ZERO_FINGERTIP_TORQUE_THRESHOLD 2000    //required threshold to be considered non-noise reading
#define ZERO_TACTILE_THRESHOLD 0.5      //required threshold to be considered non-noise reading
#define BARRETT_SMF_VALIDATE_ARGS


//#define EIGEN_DONT_VECTORIZE 
//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

using namespace barrett;
using namespace systems;
using namespace std;
using namespace Eigen;
using systems::connect;
using barrett::detail::waitForEnter;

BARRETT_UNITS_TYPEDEFS(DIMENSION);

typedef Quaterniond qd_type;
typedef Hand::jp_type co_type;
typedef TactilePuck::v_type tact_array_type;
//typedef systems::Wam<DIMENSION>::jv_type jv_type;
//typedef systems::Wam<DIMENSION>::jp_type jp_type;
//typedef systems::Wam<DIMENSION>::jt_type jt_type;


//Function defns

#endif
