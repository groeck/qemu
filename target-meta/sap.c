#include "sap.h"
#include "comms.h"
#include "sap_api.h"
#include <malloc.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#if defined(WIN32)
#pragma warning( disable : 4127 )
#define LOG_INFO	0
#define LOG_ERR		1
#else
#include <syslog.h>
#endif

static int			SAP_error				=	SAP_ERROR_NONE;
static int			SAP_finish				=	0;
static char *		SAP_logfilename			=	0;
FILE*				SAP_logfile				=	0;
static int			SAP_syslog				=	0;
static int			SAP_comms_type			=	SAP_COMMS_SHARED;
static const char *	SAP_application_name	=	"libsap";
static SAP_Target**	SAP_targetv				=	0;
static int			SAP_target_count		=	0;
static SAP_Comms*	SAP_comms				=	0;
static const int 	comms_check_frequency	=	50;
static const int 	idle_timeout			=	20;
static const int 	active_timeout			=	0;

#define SAP_DELETE(x)   do { if (x && x->Destroy) (x->Destroy)(x); x = 0; } while (0)


// checks that the size of the request command is ok, 	returns TRUE if it is
//														Sets error value and returns false if its not
static int SizeOK(SAP_Header *header, SAP_uint32 expected)
{
	if (header->Size < expected)
	{
		SAP_SetError(SAP_ERROR_PACKET, "SAP_HandleMessage: Packet size incorrect for cmd %d, expected %08x but got %08x\n", header->Command, expected, header->Size);
		return 0;
	}
	return 1;
}

static int GetTimeout(int any_target_active)
{
	return any_target_active ?  active_timeout : idle_timeout;
}

static SAP_uint32 GetReadWriteSize(SAP_uint32 Begin, SAP_uint32 End)
{
	if (End == 0)
	{
		End = 0xffffffff;
		--Begin;
	}	
	return End - Begin;
}

static void OpenLog(void)
{
	if (SAP_logfilename)
	{
		if (strcmp(SAP_logfilename, "stderr") == 0)
		{
			SAP_logfile = stderr;
		}
#if !defined(WIN32)
		else if (strcmp(SAP_logfilename, "syslog") == 0)
		{
			SAP_syslog = 1;
			openlog(SAP_application_name, LOG_PID, 0);
		}
#endif
		else
		{
			SAP_logfile = fopen(SAP_logfilename, "a");
		}

		free(SAP_logfilename);
		SAP_logfilename = 0;
	}
}

static void CloseLog(void)
{
	if (SAP_logfile !=0 && SAP_logfile != stderr)
	{
		fclose(SAP_logfile);
	}
#if !defined(WIN32)
	else if (SAP_syslog)
	{
		closelog();
	}
#endif
}

void SAP_SetFinish(void)
{
	SAP_finish = 1;
}

int SAP_GetFinish(void)
{
	return SAP_finish;
}

void SAP_MessageLoop(SAP_Target* targetv[])
{
	int			any_target_active	=	0;

	if (!SAP_Initialise(targetv))
	{
		return;
	}

	while (!SAP_finish)
	{
		SAP_HandleMessage(GetTimeout(any_target_active));
	
		{
			int n = 0;
			for ( any_target_active = 1 ; n < comms_check_frequency && any_target_active && !SAP_finish; ++n)
			{
				int targetn = 0;
				any_target_active = 0;
				for (; targetn != SAP_target_count; ++targetn)
				{
					SAP_Target * target = SAP_targetv[targetn];
					if (target->DoTimeSlice)
					{
						any_target_active |= (target->DoTimeSlice)(target);
					}
				}
			}
		}
	}
	SAP_Cleanup();
}


int SAP_Initialise(SAP_Target* targetv[])
{
	SAP_Target**		p = &targetv[0];
	for (; *p; ++p)
	{
		++SAP_target_count;
		if (!(*p)->GetProcessorFamily ||
			!(*p)->GetProcessorVersion || 
			!(*p)->Read || 
			!(*p)->Write)
		{
			SAP_Error("SAP_Initialise: targetv[%d] is missing a required function pointer.\n", SAP_target_count);
			SAP_target_count = 0;
			return 0;
		}
	}

	SAP_targetv			=	targetv;
	if (!SAP_comms)
	{
		SAP_Trace("No comms type specified, assuming shared memory.");
		SAP_SetCommsType(SAP_COMMS_SHARED);
	}

	if (!SAP_comms)
	{
		SAP_Error("Comms could not be created. Aborting.\n");
		CloseLog();
		return 0;
	}
	return 1;
}

void SAP_HandleMessage(int timeout_ms)
{
	if (!SAP_comms)
	{
		SAP_Error("SAP_HandleMessage without a successful call to SAP_Initialise.\n");
		return;
	}
	if ((SAP_comms->Recv)(SAP_comms, timeout_ms))
	{
		SAP_Header	*header 	= (SAP_Header*) (SAP_comms->Buffer)(SAP_comms);
		int target_num = SAP_ExtractTarget(header->TargetThread);
		int thread_num = SAP_ExtractThread(header->TargetThread);

		SAP_ClearError();

		if (target_num < SAP_target_count)
		{
			SAP_Target* target = SAP_targetv[target_num];
			switch (header->Command)
			{
				case SAP_DISCOVER_TARGETS :
				{
					if (SizeOK(header, 0))
					{
						SAP_DiscoverTargets*	reply 	=	(SAP_DiscoverTargets*) (header + 1);
						reply->TargetCount	=	SAP_target_count;
						reply->Family		=	(target->GetProcessorFamily)(target);
						reply->Version		=	(target->GetProcessorVersion)(target);
						header->Size		=	sizeof(*reply);
					}
					break;
				}

				case SAP_READ_MULTIPLE :
				{
					SAP_ReadMultiple*	request 	=	(SAP_ReadMultiple *) (header + 1);
					void*				buffer		=	(header + 1);

					if (SizeOK(header, sizeof(*request)))
					{
						SAP_uint32 size = GetReadWriteSize(request->Begin, request->End);
						(target->Read)(target, thread_num, request->Begin, size, request->Type, buffer);
						header->Size = size;
					}
					break;
				}

				case SAP_WRITE_MULTIPLE :
				{
					SAP_WriteMultiple	*request 	=	(SAP_WriteMultiple*) (header + 1);
					void*				buffer		=	request + 1;
					if (SizeOK(header, sizeof(*request)))
					{
						(target->Write)(target, thread_num, request->Begin, GetReadWriteSize(request->Begin, request->End), request->Type, buffer);
						header->Size = 0;
					}
					break;
				}

				case SAP_IOCTL :
                    if (target->IOCtl)
                    {
                        SAP_IOCtl	*request 	= (SAP_IOCtl*) (header + 1);
                        SAP_uint32	*pIOCTLData = (SAP_uint32*) (request + 1);
                        (target->IOCtl)(target, thread_num, request->IOCTL_Cmd, (header->Size - sizeof(*request) ) / sizeof(SAP_uint32),  pIOCTLData);
                        break;
                    }

				// SAP_READ and SAP_WRITE will only be sent by old virtual dashes, but keep it here for compatibility.
				case SAP_READ :
				{
					SAP_ReadRequest	* request	=	(SAP_ReadRequest*)(header + 1);
					SAP_ReadReply	* reply		=	(SAP_ReadReply*)(header + 1);
					if (SizeOK(header, sizeof(*request)))
					{
						(target->Read)(target, thread_num, request->Address, sizeof(reply->Data), 0, &reply->Data);
						header->Size = sizeof(*reply);
					}
					break;
				}

				case SAP_WRITE :
				{
					SAP_WriteRequest	* request	=	(SAP_WriteRequest*)(header + 1);
					if (SizeOK(header, sizeof(*request)))
					{
						(target->Write)(target, thread_num, request->Address, sizeof(request->Data), 0, &request->Data);
						header->Size = 0;
					}
					break;
				}

				case SAP_QUIET_READ :
				default :
					SAP_SetError(SAP_ERROR_COMMAND, "SAP_HandleMessage: Unsupported command number - %d\n", header->Command);
					break;
			}
		}
		else
		{
			SAP_SetError(SAP_ERROR_BAD_TARGET, "SAP_HandleMessage: Target identifier was out of range - %d\n", target_num);
		}

		if (SAP_GetError())
		{
			header->Command 			= SAP_ERROR | SAP_GetError();
			header->Size				= 0;
		}
		(SAP_comms->Send)(SAP_comms);
	}
}

void SAP_Cleanup(void)
{
    int targetn = 0;
    for (; targetn != SAP_target_count; ++targetn)
    {
        SAP_DELETE(SAP_targetv[targetn]);
    }    
    SAP_DELETE(SAP_comms);
	CloseLog();
}

int SAP_ParseCommandLine(int argc, char* argv[])
{
	int argn = 1;
	if (argc > 0)
	{
		SAP_application_name = argv[0];
	}
	for (argn = 1; argn < argc && argv[argn]; ++argn)
	{
		char * equals = strchr(argv[argn], '=');
		if (strncmp(argv[argn], "-sap-log", equals - argv[argn]) == 0)
		{
			char * arg = equals + 1;
			SAP_SetLogFile(arg);
			SAP_Trace("SAP_ParseCommandLine: Set log output to %s\n", arg);
		}
	}
	for (argn = 1; argn < argc && argv[argn]; ++argn)
	{
		char * equals = strchr(argv[argn], '=');
		if (strncmp(argv[argn], "-sap-comms", equals - argv[argn]) == 0)
		{
			char * arg = equals + 1;
			if (strcmp(arg, "inetd") == 0)
			{
				SAP_SetCommsType(SAP_COMMS_INETD);
			}
			else
			{
				int port = atoi(arg);
				if (port >= SAP_COMMS_SOCKET_FIRST && port <= SAP_COMMS_SOCKET_LAST)
				{
					SAP_SetCommsType(port);
				}
				else
				{
					SAP_Error("Invalid argument : %s", argv[argn]);
					return 1;
				}
			}
		}
	}
	return 0;
}

int SAP_GetCommsType(void)
{
	return SAP_comms_type;
}

void SAP_SetCommsType(int type_or_port)
{
	if (SAP_comms)
	{
		SAP_Error("Ignoring call to SAP_SetCommsType because the comms type has already been set.");
	}
	else
	{
		SAP_comms_type = type_or_port;
		SAP_comms			=	SAP_CommsCreate(type_or_port);
	}
}

void SAP_SetLogFile(const char * file)
{
	SAP_logfilename = realloc(SAP_logfilename, strlen(file) + 1);
	strcpy(SAP_logfilename, file);
}

static void message(int severity, const char * fmt, va_list args)
{
	OpenLog();
	if (SAP_logfile)
	{
		vfprintf(SAP_logfile, fmt, args);
		fflush (SAP_logfile);
		(void) severity;
	}
#if !defined(WIN32)
	else if (SAP_syslog)
	{
		vsyslog(severity, fmt, args);
	}
#endif
	else if (severity == LOG_ERR)
	{
		vfprintf(stderr, fmt, args);
		fflush (stderr);
	}
}


void SAP_Trace(const char * fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	message(LOG_INFO, fmt, args);
	va_end (args);
}

void SAP_Error(const char * fmt, ...)
{

	va_list args;
	va_start(args, fmt);
	message(LOG_ERR, fmt, args);
	va_end (args);
}

void SAP_SetError(int error, const char * fmt, ...)
{	
	va_list args;
	SAP_error = error;	
	va_start(args, fmt);
	message(LOG_ERR, fmt, args);
	va_end (args);
}

void SAP_ClearError(void)
{
	SAP_error = SAP_ERROR_NONE;
}

int SAP_GetError(void)
{
	return SAP_error;
}

