#if !defined(IMG_SAP_comms_socket_h_)
#define IMG_SAP_comms_socket_h_

#include "comms.h"

#if defined(__cplusplus)
extern "C" {
#endif

// port_number of 0 means assume we are running under inetd
SAP_Comms* SAP_CommsCreateSocket(int port_number);

#if defined(__cplusplus)
}
#endif

#endif
