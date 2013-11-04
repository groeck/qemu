#include "sap.h"
#include "comms.h"
#include "comms_shared.h"
#include "comms_socket.h"
#include "sap_api.h"

#include <string.h>


SAP_Comms* SAP_CommsCreate(int comms_type)
{
	if (comms_type == SAP_COMMS_SHARED)
	{
		return SAP_CommsCreateShared();
	}
	else if (comms_type == SAP_COMMS_INETD ||
			(comms_type >= SAP_COMMS_SOCKET_FIRST && comms_type <= SAP_COMMS_SOCKET_LAST))
	{
		return SAP_CommsCreateSocket(comms_type);
	}
	return 0;
}
