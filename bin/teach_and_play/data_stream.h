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

#include <boost/filesystem.hpp>

#define DIMENSION 7u
#define STRAIN2TORQUE_RATIO 118.0     //convert hand strain to N-m

using namespace barrett;
using systems::connect;
using detail::waitForEnter;

static int loop_count = 0;

//Function defns

void data_collect(Hand* hand, ForceTorqueSensor* fts, void* wamin, ProductManager* pm);

/*void init_stream(std::string trajectory_name);
void loop_stream();*/

#endif /* IO_H_ */
