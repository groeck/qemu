#ifdef WIN32
	#pragma warning( disable : 4201 )
#endif

#include "sap.h"
#include "comms_socket.h"
#include "sap_api.h"

#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>

#if defined(WIN32)
	#pragma warning( disable : 4115 )
	#pragma warning( disable : 4127 )
	#include "winsock2.h"

	typedef int socklen_t;
	#define IMG_SOCKADDR_U32 S_un.S_addr

	static int socket_error()			{	return WSAGetLastError();	}


	static size_t sap_write(SOCKET fd, const void* buf, size_t count)
	{
		return send(fd, buf, count, 0);
	}

	static size_t sap_read(SOCKET fd, void* buf, size_t count)
	{
		return recv(fd, buf, count, 0);
	}

	void InitSockets()
	{
		const unsigned short WinsockVersion	=	0x101;
		WSADATA	wsaData;
		int startup_res = WSAStartup(WinsockVersion, &wsaData);
		if (startup_res  != 0)
		{
			SAP_SetError(SAP_ERROR_COMMS, "SAP_CommsCreateSocket: Could not initialise Winsock.dll - %d\n", socket_error());
			return;
		}
	}

	void CleanupSockets()
	{
		WSACleanup();
	}

#else
	#include <errno.h>
	#include <sys/socket.h>
	#include <unistd.h>
	#include <netdb.h>
	#include <sys/types.h>
	#include <sys/stat.h>
	#include <fcntl.h>
	#include <time.h>

	extern FILE* SAP_logfile; // this is a hack, sorry, but all of the redirecting of stdout/err for inetd is a hack!

	typedef int SOCKET;
	#define IMG_SOCKADDR_U32 s_addr
	#define INVALID_SOCKET -1
	#define SOCKET_ERROR -1

	static void closesocket(int socket)	{	close(socket);	}
	static int socket_error(void)		{	return errno;	}

	static size_t sap_write(SOCKET fd, const void* buf, size_t count)
	{
		return write(fd, buf, count);
	}

	static size_t sap_read(SOCKET fd, void* buf, size_t count)
	{
		return read(fd, buf, count);
	}

	static void InitSockets(void)
	{
	}

	static void CleanupSockets(void)
	{
	}

#endif

typedef struct SAP_SocketCommsUserDataTag
{
	SOCKET		listening;		// if we are in inetd mode, this socket is ALWAYS zero.
								// if not then this is the socket that listens on the specified port

	SOCKET		connected_in;	// if we are in inetd mode, this will be zero (ie stdout/stdin)
	SOCKET		connected_out;	// if not and a dash has connected to us, then this is a valid fd.
								// if not and a dash has not connected to us, then this is zero.
	void *		buffer;

	SAP_uint32	connected_ip;	//	the ip address of the currently connected dash

} SAP_SocketCommsUserData;

static SAP_SocketCommsUserData* self(SAP_Comms* comms)
{
	return (SAP_SocketCommsUserData*) comms->UserData;
}

static void* Buffer(SAP_Comms* comms)
{
	return self(comms)->buffer;
}

static void Init(SAP_Comms* comms, int port_number)
{
	InitSockets();
	if (SAP_GetError())
	{
		return;
	}
	self(comms)->buffer		=	malloc(SAP_MAXPACKETSIZE);
	if (port_number != SAP_COMMS_INETD)
	{
		struct sockaddr_in	addr;
		socklen_t			addrlen = sizeof(addr);
		memset(&addr, 0, sizeof(addr));
		addr.sin_family					=	AF_INET;
		addr.sin_port					=	htons((u_short)port_number);
		addr.sin_addr.IMG_SOCKADDR_U32	=	htonl(INADDR_ANY);            

		self(comms)->listening	=	socket(AF_INET, SOCK_STREAM, 0);
		if (!self(comms)->listening)
		{
			free(self(comms)->buffer);
			CleanupSockets();
			SAP_SetError(SAP_ERROR_COMMS, "SAP_CommsCreateSocket: Failed to create the listening socket - %d\n", socket_error());
			return;
		}

		if (bind(self(comms)->listening, (struct sockaddr*)&addr, addrlen) == SOCKET_ERROR)
		{
			free(self(comms)->buffer);
			closesocket(self(comms)->listening);
			CleanupSockets();
			SAP_SetError(SAP_ERROR_COMMS, "SAP_CommsCreateSocket: Failed to bind() on the listening socket - %d\n", socket_error());
			return;
		}

		if (listen(self(comms)->listening, 1) == SOCKET_ERROR)
		{
			free(self(comms)->buffer);
			closesocket(self(comms)->listening);
			CleanupSockets();
			SAP_SetError(SAP_ERROR_COMMS, "SAP_CommsCreateSocket: Failed to listen() on the listening socket - %d\n", socket_error());
			return;
		}
	}
	else
	{
#if defined(WIN32)
		SAP_SetError(SAP_ERROR_COMMS, "SAP_CommsCreateSocket: inetd mode is not supported on Win32\n");
		return;
#else
		// inetd ...
		int fd;
		struct sockaddr_in addr;
		socklen_t			addrlen = sizeof(addr);
		if (getpeername(0, (struct sockaddr*) &addr, &addrlen) != 0)
		{
			SAP_Trace("getpeername failed - %d(%s)\n"
					  "Maybe because output has already been sent to stdout?\n", errno, strerror(errno));
		}
		else
		{
			self(comms)->connected_ip = htonl(addr.sin_addr.IMG_SOCKADDR_U32);
			SAP_Trace("Starting in inetd mode, connection from : %d.%d.%d.%d\n", 
						(self(comms)->connected_ip >> 24) & 0xff, 
						(self(comms)->connected_ip >> 16) & 0xff, 
						(self(comms)->connected_ip >> 8) & 0xff, 
						(self(comms)->connected_ip) & 0xff
					);
		}
		/*
		If we are going to use inetd with the meta simulator we are going to have 
		problems because the simulator sends a lot of output to stdout/stderr which 
		will mess up inetd.  Redirect it to somewhere sensible.
		*/
		SAP_Trace("Redirecting all stdout/err because we are in inetd mode...\n");
		if (!SAP_logfile || (fd = fileno(SAP_logfile)) == -1)
		{
			SAP_Trace("Redirected stdout/err will only work when -sap-log=filename is used.\n"
	 				  "All stdout and stderr will be lost.");
			close(STDOUT_FILENO);
			close(STDERR_FILENO);
		}
		else
		{
			self(comms)->connected_in = dup(STDIN_FILENO);
			self(comms)->connected_out = dup(STDOUT_FILENO);
			if (dup2(fd, STDOUT_FILENO) == -1 ||	// assign stdout
				dup2(fd, STDERR_FILENO) == -1)		// stderr to our log file
			{
				SAP_Trace("Failed to dup2 the log file onto stdout/err all stdout/err will be lost.\n"
					      "errno = %d \"%s\"\n", errno, strerror(errno));
				close(STDOUT_FILENO);
				close(STDERR_FILENO);
			}
			else
			{
				SAP_Trace("Successfully redirected stdout/err to the file specified on -sap-log=filename\n");
			}
		}
#endif
	}
}

static void Send(SAP_Comms* comms)
{
	SAP_uint32 size = ((SAP_Header*) self(comms)->buffer)->Size;
	if (sap_write(self(comms)->connected_out, self(comms)->buffer, sizeof(SAP_Header) + size) == -1)
	{
		SAP_Trace("Send sap_write failed\n");
	}
}

static int Recv(SAP_Comms* comms, int milliseconds)
{
	struct timeval tv;
	SAP_Header* header = (SAP_Header*) self(comms)->buffer;
	size_t amount = 0;
	fd_set read_set;
	SOCKET	max_fd = 0;

	tv.tv_sec =   milliseconds / 1000;
	tv.tv_usec = (milliseconds % 1000) * 1000;

	if (self(comms)->listening && !self(comms)->connected_in)
	{
		struct sockaddr_in addr;
		socklen_t			addrlen = sizeof(addr);
		// we have to wait for a connection first
		FD_ZERO(&read_set);
		FD_SET(self(comms)->listening, &read_set);

		if (select(self(comms)->listening + 1, &read_set, 0, 0, &tv) == 0)
		{
			return 0;
		}

		self(comms)->connected_in = /* deliberate fall thru ... */
		self(comms)->connected_out = accept(self(comms)->listening, (struct sockaddr*)&addr, &addrlen);
		if (self(comms)->connected_in == INVALID_SOCKET)
		{
			SAP_SetError(SAP_ERROR_COMMS, "SAP_Comms::Socket::Recv: accept failed - %d\n", socket_error());
			return 0;
		}

		self(comms)->connected_ip = htonl(addr.sin_addr.IMG_SOCKADDR_U32);
		SAP_Trace("Connection from : %d.%d.%d.%d\n", 
					(self(comms)->connected_ip >> 24) & 0xff, 
					(self(comms)->connected_ip >> 16) & 0xff, 
					(self(comms)->connected_ip >> 8) & 0xff, 
					(self(comms)->connected_ip) & 0xff
				);
	}

	// now we are connected

	FD_ZERO(&read_set);
	FD_SET(self(comms)->connected_in, &read_set);
	max_fd = self(comms)->connected_in;
	if (self(comms)->listening)
	{
		// add the listening socket so we can tell other dashes that we are in use.
		FD_SET(self(comms)->listening, &read_set);
		max_fd = self(comms)->listening > max_fd ? self(comms)->listening : max_fd;
	}

	select(max_fd + 1, &read_set, 0, 0, &tv);

	if (FD_ISSET(self(comms)->listening, &read_set))
	{
		// a dash is trying to connect, but we are already in use. tell them!
		struct sockaddr_in addr;
		socklen_t			addrlen = sizeof(addr);
		int accepted = accept(self(comms)->listening, (struct sockaddr*)&addr, &addrlen);
		if (accepted != INVALID_SOCKET)
		{
			SAP_Header error;
			error.Command		= SAP_ERROR | SAP_ERROR_IN_USE;
			error.Size			= 0;
			error.TargetThread	= self(comms)->connected_ip;
			if (sap_write(accepted, &error, sizeof(error)) == -1)
			{
				SAP_Trace("Recv in use sap_write failed\n");
			}
			closesocket(accepted);
		}
	}

	if (!FD_ISSET(self(comms)->connected_in, &read_set))
	{
		return 0;
	}

	amount = sap_read(self(comms)->connected_in, header, sizeof(SAP_Header));
	if (amount == 0)
	{	// the dash has disconnected
		SAP_Trace("%d.%d.%d.%d has disconnected\n", 
						(self(comms)->connected_ip >> 24) & 0xff, 
						(self(comms)->connected_ip >> 16) & 0xff, 
						(self(comms)->connected_ip >> 8) & 0xff, 
						(self(comms)->connected_ip) & 0xff
					);

		closesocket(self(comms)->connected_in);
		self(comms)->connected_in = 0;
		self(comms)->connected_out = 0;
		self(comms)->connected_ip = 0;
		if (!self(comms)->listening)
		{
			// in inetd mode we only support one connection, and then we close the app
			SAP_SetFinish();
		}
		return 0;
	}
	if (amount != sizeof(SAP_Header))
	{
		SAP_SetError(SAP_ERROR_COMMS, "SAP_Comms::Socket::Recv: sap_read did not return a full SAP_Header - read = %d\n", amount);
		return 0;
	}

	if (header->Size)
	{
		amount = sap_read(self(comms)->connected_in, header + 1, header->Size);
		if (amount != header->Size)
		{
			SAP_SetError(SAP_ERROR_COMMS, "SAP_Comms::Socket::Recv: sap_read did not return the correct amount of data. header->Size = %d, read = %d\n", header->Size, amount);
			return 0;
		}
	}
	return 1;
}

static void Destroy(SAP_Comms* comms)
{
	if (self(comms)->listening)		closesocket(self(comms)->listening);
	if (self(comms)->connected_in)	closesocket(self(comms)->connected_in);
	free(self(comms)->buffer);
	free(comms->UserData);
	free(comms);
	CleanupSockets();
}


SAP_Comms* SAP_CommsCreateSocket(int port_number)
{
	SAP_Comms * comms	=	(SAP_Comms*) malloc(sizeof(SAP_Comms));
	if (!comms)
	{
		SAP_SetError(SAP_ERROR_COMMS, "SAP_CommsCreateSocket : Failed to malloc space for comms\n");
		return 0;
	}
	comms->Buffer		=	Buffer;
	comms->Send			=	Send;
	comms->Recv			=	Recv;
	comms->Destroy		=	Destroy;
	comms->UserData		=	malloc(sizeof(SAP_SocketCommsUserData));
	if (!comms->UserData)
	{
		free(comms);
		SAP_SetError(SAP_ERROR_COMMS, "SAP_CommsCreateSocket : Failed to malloc space for comms->UserData\n");
		return 0;
	}
	memset(comms->UserData, 0, sizeof(SAP_SocketCommsUserData));

	Init(comms, port_number);
	if (SAP_GetError())
	{
		free(comms->UserData);
		free(comms);
		return 0;
	}
	return comms;
}

