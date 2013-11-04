#ifndef _TARGETMETA_DASIM_H_
#define _TARGETMETA_DASIM_H_

/* Communication with DA-Sim and CODESCAPE via SAP */

#include "core.h"

extern const char *sap_comms;

/* Handy */
int meta_dasim_rw(CPUArchState *env, target_ulong addr,
                  uint8_t *buf, int len, int is_write);
/* Start listening for DA-Sim connections */
int meta_dasim_setup(MetaCore *core);

#endif /* _TARGETMETA_DASIM_H_ */
