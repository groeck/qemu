#if !defined(IMG_SAP_comms_shared_h_)
#define IMG_SAP_comms_shared_h_

#include "comms.h"

#if defined(__cplusplus)
extern "C" {
#endif


#include "sap_api.h"

typedef struct SAP_SharedCommsUserDataTag
{
	SAP_SharedInit		handles;
	void *				pShared;
} SAP_SharedCommsUserData;



void SAP_CommsSharedInit(SAP_Comms* comms);
void *SAP_CommsSharedBuffer(SAP_Comms* comms);
int  SAP_CommsSharedRecv(SAP_Comms* comms, int milliseconds);
void SAP_CommsSharedSend(SAP_Comms* comms);
void SAP_CommsSharedCleanup(SAP_Comms* comms);

SAP_Comms* SAP_CommsCreateShared(void);

#if defined(__cplusplus)
}
#endif

#endif
