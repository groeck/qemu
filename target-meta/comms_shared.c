
#include "sap.h"
#include "comms_shared.h"
#include "malloc.h"

static void SAP_CommsSharedDestroy(SAP_Comms* comms)
{
	SAP_CommsSharedCleanup(comms);
	free(comms->UserData);
	free(comms);
}

SAP_Comms* SAP_CommsCreateShared(void)
{
	SAP_Comms * comms	=	(SAP_Comms*) malloc(sizeof(SAP_Comms));
	if (!comms)
	{
		SAP_SetError(SAP_ERROR_COMMS, "SAP_CommsCreateShared : Failed to malloc space for comms\n");
		return 0;
	}
	comms->Buffer		=	SAP_CommsSharedBuffer;
	comms->Send			=	SAP_CommsSharedSend;
	comms->Recv			=	SAP_CommsSharedRecv;
	comms->Destroy		=	SAP_CommsSharedDestroy;
	comms->UserData		=	malloc(sizeof(SAP_SharedCommsUserData));
	if (!comms->UserData)
	{
		free(comms);
		SAP_SetError(SAP_ERROR_COMMS, "SAP_CommsCreateShared : Failed to malloc space for comms->UserData\n");
		return 0;
	}	
	SAP_CommsSharedInit(comms);
	if (SAP_GetError())
	{
		free(comms->UserData);
		free(comms);
		return 0;
	}
	return comms;
}
