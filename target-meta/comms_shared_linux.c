
#include "sap.h"
#include "comms_shared.h"
#include <sys/shm.h>
#include <sys/select.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

static SAP_SharedCommsUserData* self(SAP_Comms* comms)
{
	return (SAP_SharedCommsUserData*) comms->UserData;
}

void * SAP_CommsSharedBuffer(SAP_Comms* comms)
{
	return self(comms)->pShared;
}

void SAP_CommsSharedInit(SAP_Comms* comms)
{
	int result = 0;

    memset(&self(comms)->handles, 0, sizeof(self(comms)->handles));

    result = read(0, &self(comms)->handles, sizeof(self(comms)->handles));
    if (result != sizeof(self(comms)->handles) || self(comms)->handles.MemoryMappedHandle == 0 || self(comms)->handles.RequestEvent == 0 || self(comms)->handles.ReplyEvent == 0)
    {
		SAP_SetError(SAP_ERROR_COMMS, "SAP_CommsSharedInit: Invalid handles received from parent %d\n", errno);
        return;
    }

	self(comms)->pShared = shmat((int) self(comms)->handles.MemoryMappedHandle, 0, 0);
    if (self(comms)->pShared == (void *)-1)
    {
		SAP_SetError(SAP_ERROR_COMMS, "SAP_CommsSharedInit: Could not map shared memory - %d\n", errno);
		return;
    }

    SAP_CommsSharedSend(comms);
}

int SAP_CommsSharedRecv(SAP_Comms* comms, int milliseconds)
{
	struct timeval tv = { milliseconds / 1000, (milliseconds % 1000) * 1000 };
	int res = 0;
	int handle = (int) self(comms)->handles.RequestEvent;
	fd_set set;

	FD_ZERO(&set);
	FD_SET(handle, &set);
	res = select(handle + 1, &set, 0, 0, &tv);
	if (res < 0)
	{
		SAP_SetError(SAP_ERROR_COMMS, "SAP_CommsSharedRecv: select failed - %d\n", errno);
		return 0;
	}
	else if (res > 0)
	{
		char c = 0;
		if (read(handle, &c, sizeof(c)) != sizeof(c))
		{
			SAP_SetError(SAP_ERROR_COMMS, "SAP_CommsSharedRecv: Could not read from pipe - %d\n", errno);
			return 0;
		}
		return 1;
	}
	return 0;
}
void SAP_CommsSharedSend(SAP_Comms* comms)
{
    char c = 1;
    if (write((int) self(comms)->handles.ReplyEvent, &c, sizeof(c)) != sizeof(c))
    {
		SAP_SetError(SAP_ERROR_COMMS,"SAP_CommsSharedSend: Could not set reply event - %d\n", errno);
		return;
    }
}

void SAP_CommsSharedCleanup(SAP_Comms* comms)
{
	shmctl((int) self(comms)->handles.MemoryMappedHandle, IPC_RMID, 0);
    close((int) self(comms)->handles.ReplyEvent);
    close((int) self(comms)->handles.RequestEvent);
}

