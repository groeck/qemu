#if !defined(IMG_SAP_comms_h_)
#define IMG_SAP_comms_h_

#if defined(__cplusplus)
extern "C" {
#endif

typedef struct SAP_CommsTag
{
	void* (*Buffer)(struct SAP_CommsTag* self);

	void (*Send)(struct SAP_CommsTag* self);

	/// returns non-zero if a command has been received
	int (*Recv)(struct SAP_CommsTag* self, int milliseconds);

	void (*Destroy)(struct SAP_CommsTag* self);

	void * UserData;
} SAP_Comms;


SAP_Comms* SAP_CommsCreate(int comms_type);

#if defined(__cplusplus)
}
#endif

#endif

