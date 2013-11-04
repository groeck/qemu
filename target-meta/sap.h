#if !defined(IMG_sap_h_)
#define IMG_sap_h_

//	See <Simulator Access Protocol.TRM.doc> for documentation on this file.

#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

typedef uint32_t SAP_uint32;



typedef struct SAP_TargetTag
{
	// Return the family of the target processor, this must be one of :
	#define SAP_PROCESSOR_FAMILY_HITACHI	1
	#define SAP_PROCESSOR_FAMILY_ARM		2
	#define SAP_PROCESSOR_FAMILY_IMG_META	3
	#define SAP_PROCESSOR_FAMILY_IMG_UCC	4
	#define SAP_PROCESSOR_FAMILY_VHDL_META	5
	SAP_uint32 (*GetProcessorFamily)(struct SAP_TargetTag* self);

	// the format of this is processor specific
	// on META this is METAC_ID
	SAP_uint32 (*GetProcessorVersion)(struct SAP_TargetTag* self);

	// Perform 'one clock' or timeslice of simulator time.
	// This function pointer can be null if :
	//	SAP_MessageLoop is not used, or 
	//	If the simulator is implemented in another thread/process
	// should return non-zero if the simulator is active, 
	//					 zero if the simulator is idle
	int (*DoTimeSlice)(struct SAP_TargetTag* self);	

	// Read memory from the given address and memory type.
	// size is the length of the memory operation in bytes
	void (*Read)(struct SAP_TargetTag* self, SAP_uint32 thread, 
					SAP_uint32 address, SAP_uint32 size, 
					SAP_uint32 type, void* data);

	// Write memory to the given address and memory type.
	// size is the length of the memory operation in bytes
	void (*Write)(struct SAP_TargetTag* self, SAP_uint32 thread, 
					SAP_uint32 address, SAP_uint32 size, 
					SAP_uint32 type, const void * data);

	// Perform an IO Control command.  The purpose of this is target 
	// specific.  This function may be null if IO Control functions
	// are not required.
	// count is the number of 32bit words
	void (*IOCtl)(struct SAP_TargetTag* self, SAP_uint32 thread, 
					SAP_uint32 ioctl_cmd, SAP_uint32 count,
					SAP_uint32 * pioctl); 

    // Perform any cleanup required, this may be null if desired
    void (*Destroy)(struct SAP_TargetTag* self);

	// The caller may use UserData for stateful data.
	void * UserData;

	// Internal structures no longer allocated statically
	void * pMETAC;
	void * pGbl;

} SAP_Target;



// Initialise SAP settings from command line options.  
//	Supported options are documented with SAP_SetLogFile and SAP_SetCommsType
int SAP_ParseCommandLine(int argc, char* argv[]);


// Set the log destination.  Can be 
//		"stderr", 
//		"syslog", 
// or a filename
//
// By default, no log will be generated, even for error conditions.
// This setting can also be changed using the command line option : 
//		-sap-log=stderr|syslog|logfilename
//
void SAP_SetLogFile(const char * file);
const char * SAP_GetLogFile(void);

// Set the comms type, can be one of the following constants.
#define SAP_COMMS_SHARED		-1
#define SAP_COMMS_INETD			0
// or a port number in the range SOCKET_FIRST >= port >= SOCKET_LAST
#define SAP_COMMS_SOCKET_FIRST	1
#define SAP_COMMS_SOCKET_LAST	65535
// By default the comms will be shared memory ( backwards compatibility)
// This setting can also be changed using the command line option : 
//		-sap-comms=shared|inetd|port-number
//
void SAP_SetCommsType(int type_or_port);
int SAP_GetCommsType(void);

// To Run the server, the caller can either use : 

	// Start the comms, 
	// Listens for messages, 
	// Periodically calls target->DoTimeSlice
	// Load balancing of the comms and the simulator is automatically handled
	void SAP_MessageLoop(SAP_Target* targetv[]);

// Or the caller can manually call the following functions

	// Initialise the SAP system, returns nonzero on success
	int SAP_Initialise(SAP_Target* targetv[]);

	// handle a single message, will wait for timeout_ms milliseconds if no 
	// message is available immediately.  The caller is responsible for 
	// managing the running of the simulator and load balancing
	void SAP_HandleMessage(int timeout_ms);

	// Call this if you manually called SAP_Initialise
	void SAP_Cleanup(void);


// non-zero if the application should finish
int SAP_GetFinish(void);

// call to request the application to close
void SAP_SetFinish(void);

// print a trace message
void SAP_Trace(const char* fmt, ...);

// print an error message
void SAP_Error(const char* fmt, ...);

// print message and set pending error state.  
// error should be SAP_ERROR_TARGET_BASE ORed with a caller defined constant
#define	SAP_ERROR_TARGET_BASE		0x8000
void SAP_SetError(int error, const char* fmt, ...);

// returns the current error state
int SAP_GetError(void);

// reset any pending error state
void SAP_ClearError(void);

#if defined(__cplusplus)
}
#endif

#endif

