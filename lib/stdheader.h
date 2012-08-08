#ifndef STDINCLUDES_H_
#define STDINCLUCES_H_


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

//boost includes
#include <boost/thread/thread.hpp>
#include <boost/thread.hpp>
#include <boost/tuple/tuple.hpp>

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


// The ncurses library allows us to write text to any location on the screen
#include <curses.h>

#define DIMENSION 7u
#define FINGERTIP_TORQUE2TORQUE_RATIO 118.0     //convert hand fingertip_torque to N-m
#define FINGER_JOINT_LIMIT 2.4435       //=140 degrees
#define ZERO_FINGERTIP_TORQUE_THRESHOLD 2000    //required threshold to be considered non-noise reading
#define ZERO_TACTILE_THRESHOLD 0.5      //required threshold to be considered non-noise reading

using namespace barrett;
using systems::connect;
using detail::waitForEnter;

typedef TactilePuck::v_type v_type;
typedef systems::Wam<DIMENSION>::jv_type jv_type;
typedef systems::Wam<DIMENSION>::jp_type jp_type;
typedef systems::Wam<DIMENSION>::jt_type jt_type;


//Function defns

#endif
