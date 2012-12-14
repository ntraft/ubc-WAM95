#ifndef IO_H_
#define IO_H_


#include <stdexcept>

#include <syslog.h>
#include <unistd.h> /* for close() */
#include <sys/socket.h> /* For sockets */
#include <fcntl.h>      /* To change socket to nonblocking mode */
#include <arpa/inet.h>  /* For inet_pton() */

#include "stdheader.h"

//Function defns

//void dataCollect(Hand* hand, ForceTorqueSensor* fts, void* wamin, ProductManager* pm, enum EXPERIMENT_KEYS expnum, enum EXPERIMENT_SHAPES expshape);


#endif /* IO_H_ */
