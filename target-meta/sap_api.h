//-----------------------------------------------------------------------------------------
// sap_api.h
//
//	See <Simulator Access Protocol.TRM.doc> for documentation on this file.
//

#if !defined(IMG_SAP_api_h_)
#define IMG_SAP_api_h_

#ifndef IMG_sap_h_
typedef unsigned long SAP_uint32;
#endif

#if defined(SAP_PACKED)
	#error SAP_PACKED is already defined.
#else
	#if defined (WIN32)
		#pragma pack(push, sap_sap_api_h)
		#pragma	pack(1)
		#pragma warning(disable: 4103)
		#define SAP_PACKED
	#else
		#define SAP_PACKED	__attribute__ ((packed))
	#endif
#endif


// initialization header, used to transfer handles in the shared memory interface
typedef struct  SAP_SharedInitTag
{
	SAP_uint32	MemoryMappedHandle	SAP_PACKED;
	SAP_uint32	RequestEvent		SAP_PACKED;
	SAP_uint32	ReplyEvent			SAP_PACKED;
} SAP_SharedInit;

//
// 	Every request and reply command starts with the command header
//
// --------------------------------------------------------
// |  32-bits - Command Number					          |
// --------------------------------------------------------
// |  32-bits - Most significant 16 bits - Target Number  |
// |            Lease significant 16 bits - Thread Number |
// --------------------------------------------------------
// |  32-bits - Command Size (excluding header)           |
// --------------------------------------------------------
//
// A reply sets the command number to the same as the request, unless an error occurs,
// in which case Command is the error code, see SAP_ERROR below

typedef struct SAP_HeaderTag
{
	SAP_uint32	Command			SAP_PACKED;
	SAP_uint32	TargetThread	SAP_PACKED;
	SAP_uint32	Size			SAP_PACKED;
} SAP_Header;

// maximum number of bytes any command may hold
#define SAP_MAXPACKETSIZE 	sizeof(SAP_Header) + sizeof(SAP_WriteMultiple) + SAP_IOCTL_MAX_DATA * sizeof(SAP_uint32)	

#define SAP_MakeTargetThread(target, thread)		\
			((thread & 0xffff) | (target << 16))	\
	/**/

#define SAP_ExtractTarget(target_thread)			\
		((target_thread >> 16) & 0xffff)			\
	/**/

#define SAP_ExtractThread(target_thread)			\
		((target_thread & 0xffff))					\
	/**/


// ----------------------------------------------------------
// ERROR
	// returned when an error occurs in the simulator

	#define SAP_ERROR	0xFFFF0000	// error codes are combined with the following error codes:

	#define SAP_ERROR_NONE				0x0000	// no error has occurred (never sent, but useful for internal state)
	#define SAP_ERROR_PACKET			0x0001	// packet size wrong for command
	#define SAP_ERROR_COMMAND			0x0002	// command number invalid
	#define SAP_ERROR_COMMS				0x0004	// communications error, check logs
	#define SAP_ERROR_BAD_TARGET		0x0003	// target number invalid
	#define SAP_ERROR_IN_USE			0x0005	// the target is in use by another dash (TargetThread contains the IP address of the other dash)
#if !defined(SAP_ERROR_TARGET_BASE)
	#define	SAP_ERROR_TARGET_BASE		0x8000	// start of simulator specific errors
#endif

	#define SAP_ERROR_UNSPECIFIED		0xFFFF	// an unspecified error occurred

// ----------------------------------------------------------
//	DISCOVER_TARGETS command

#define SAP_DISCOVER_TARGETS	0x00000000

	// request data : none

	// reply data : 
	typedef struct SAP_DiscoverTargetsTag 
	{
		SAP_uint32 	TargetCount		SAP_PACKED;	
		SAP_uint32 	Family			SAP_PACKED;	
		SAP_uint32	Version			SAP_PACKED;
	} SAP_DiscoverTargets;

// ----------------------------------------------------------
//  READ command
	// read memory

	#define SAP_READ	0x00000001

	// request data : 	one 32-bit value for the address
	typedef struct SAP_ReadRequestTag
	{
		SAP_uint32	Address		SAP_PACKED;
	} SAP_ReadRequest;


	// reply data : 	one 32-bit value for the data
	typedef struct SAP_ReadReplyTag
	{
		SAP_uint32 	Data		SAP_PACKED;	
	} SAP_ReadReply;

// ----------------------------------------------------------
//  WRITE command
	// write memory

	#define SAP_WRITE	0x00000002

	// request data : 	one 32-bit value for the address
	typedef struct SAP_WriteRequestTag
	{
		SAP_uint32 	Address 	SAP_PACKED;
		SAP_uint32 	Data		SAP_PACKED;
	} SAP_WriteRequest;

	// reply data : 	none


// ----------------------------------------------------------
// IOCTL command
	// extension to access simulator specific data,

	#define SAP_IOCTL	0x00000003

	// request data : 	one 32-bit value for the IOCTL command number
	//	(reply same)	any number of 32-bit values up to a maximum of :
	#define SAP_IOCTL_MAX_DATA	256
	//					values
	// (size determined by the header Size member)
	typedef struct SAP_IOCtlTag
	{
		SAP_uint32	IOCTL_Cmd	SAP_PACKED;
		// ... any number of SAP_uint32 values (up to SAP_IOCTL_MAX_DATA)
	} SAP_IOCtl;


// -----------------------------------------------------------
// QUIET_READ command
//	this command is not used.

	#define SAP_QUIET_READ	0x00000004


// ----------------------------------------------------------
//  READ_MULTIPLE command
	// read memory

	#define SAP_READ_MULTIPLE	0x00000005

	#define SAP_READ_MULTIPLE_MAX_DATA	256

	// request data : 	one 32-bit value for the address
	typedef struct SAP_ReadMultipleTag 
	{
		SAP_uint32	Begin		SAP_PACKED;
		SAP_uint32	End			SAP_PACKED;
		SAP_uint32	Type		SAP_PACKED;
	} SAP_ReadMultiple;

	// reply data :
	//..	SAP_uint32 	Data[(End - Begin) / 4]		SAP_PACKED;	

// ----------------------------------------------------------
//  WRITE command
	// write memory

	#define SAP_WRITE_MULTIPLE	0x00000006

	#define SAP_WRITE_MULTIPLE_MAX_DATA	256

	// request data : 	one 32-bit value for the begin address
	//					one 32-bit value for one past the last address to write
	//					(End - Begin) / 4 values to write
	typedef struct SAP_WriteMultipleTag 
	{
		SAP_uint32 	Begin 	SAP_PACKED;
		SAP_uint32 	End		SAP_PACKED;
		SAP_uint32	Type	SAP_PACKED;
	//..	SAP_uint32 	Data[(End - Begin) / 4]		SAP_PACKED;	
	} SAP_WriteMultiple;

	// reply data : 	none



#if !defined(SAP_PACKED)
	#error SAP_PACKED is not defined.
#else
	#if defined (WIN32)
		#pragma pack(pop, sap_sap_api_h)
	#endif
	#undef SAP_PACKED
#endif

    
#endif // include guard



