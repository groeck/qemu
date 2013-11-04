
#include "sap.h"
#include "comms_shared.h"
#pragma warning( disable : 4115 )
#include <windows.h>

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
	DWORD dw = 0;
	memset(&self(comms)->handles, 0, sizeof(self(comms)->handles));

	ReadFile(GetStdHandle(STD_INPUT_HANDLE), &self(comms)->handles.MemoryMappedHandle, sizeof(HANDLE), &dw, NULL);
	ReadFile(GetStdHandle(STD_INPUT_HANDLE), &self(comms)->handles.RequestEvent, sizeof(HANDLE), &dw, NULL);
	ReadFile(GetStdHandle(STD_INPUT_HANDLE), &self(comms)->handles.ReplyEvent, sizeof(HANDLE), &dw, NULL);

	if (self(comms)->handles.MemoryMappedHandle == 0 || 
		self(comms)->handles.RequestEvent == 0 || 
		self(comms)->handles.ReplyEvent == 0)
	{
		SAP_SetError(SAP_ERROR_COMMS, "SAP_CommsSharedInit: Invalid handles received from parent %d\n", GetLastError());
		return;
	}

	self(comms)->pShared = MapViewOfFile((HANDLE) self(comms)->handles.MemoryMappedHandle, FILE_MAP_WRITE, 0, 0, 0);
	if (self(comms)->pShared == 0)
	{
		SAP_SetError(SAP_ERROR_COMMS, "SAP_CommsSharedInit: Could not map shared memory - %d\n", GetLastError());
		return;
	}
	SAP_CommsSharedSend(comms);
}

int SAP_CommsSharedRecv(SAP_Comms* comms, int milliseconds)
{
	if (WaitForSingleObject((HANDLE) self(comms)->handles.RequestEvent, milliseconds) != WAIT_TIMEOUT)
	{
		ResetEvent((HANDLE) self(comms)->handles.RequestEvent);
		return 1;
	}
	return 0;
}
void SAP_CommsSharedSend(SAP_Comms* comms)
{
	SetEvent((HANDLE) self(comms)->handles.ReplyEvent);
}

void SAP_CommsSharedCleanup(SAP_Comms* comms)
{
	UnmapViewOfFile(self(comms)->pShared);
	CloseHandle((HANDLE) self(comms)->handles.MemoryMappedHandle);
	CloseHandle((HANDLE) self(comms)->handles.ReplyEvent);
	CloseHandle((HANDLE) self(comms)->handles.RequestEvent);
}

