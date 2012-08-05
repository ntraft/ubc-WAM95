#ifndef IO_H_
#define IO_H_


#include <stdexcept>

#include <syslog.h>
#include <unistd.h> /* for close() */
#include <sys/socket.h> /* For sockets */
#include <fcntl.h>      /* To change socket to nonblocking mode */
#include <arpa/inet.h>  /* For inet_pton() */

#include <barrett/detail/ca_macro.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/thread/disable_secondary_mode_warning.h>
#include <barrett/units.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/products/hand.h>

#define DIMENSION 7u
#define STRAIN2TORQUE_RATIO 118.0     //convert hand strain to N-m

//typedef math::Vector<24>::type v_type;

enum EXPERIMENT_KEYS{
        ACTIONPHASE,
        ACTIVESENSING,
        WAMVELOCITY,
        WAMJOINTPOS,
        WAMCARTESIANPOS,
        WAMJOINTTORQUE,
        BHVELOCITY,
        BHPOSITION,
        BHTORQUE,
        BHTRAPEZOIDAL,
        SIMPLESHAPES,
        ACTIVEPROBING,
        CARTESIANRASTER,
        NUM_EXPERIMENTS
};
enum EXPERIMENT_SHAPES{
        CIRCLE,
        SQUARE,
        TRIANGLE,
        NUM_SHAPES
};

using namespace barrett;
using systems::connect;
using detail::waitForEnter;

//Function defns

void dataCollect(Hand* hand, ForceTorqueSensor* fts, void* wamin, ProductManager* pm, enum EXPERIMENT_KEYS expnum, enum EXPERIMENT_SHAPES expshape);


#endif /* IO_H_ */
