/* META */

/* this struct defines the way the registers are stored on the
   stack during a system call. */

struct target_pt_regs {
    abi_long dregs[2][16];
    abi_long aregs[2][8];
    abi_long pc[2];
    /* TXRPT */
    /* FLAGS */
    /* etc */
};

#define META_pc		pc[META_PC]

#define META_A0StP	aregs[0][0]
#define META_A1GbP	aregs[1][0]
#define META_A0FrP	aregs[0][1]
#define META_A1LbP	aregs[1][1]

#define META_D0Re0      dregs[0][0]
#define META_D1Re0      dregs[1][0]
#define META_D0Ar6      dregs[0][1]
#define META_D1Ar5      dregs[1][1]
#define META_D0Ar4      dregs[0][2]
#define META_D1Ar3      dregs[1][2]
#define META_D0Ar2      dregs[0][3]
#define META_D1Ar1      dregs[1][3]
#define META_D0FrT      dregs[0][4]
#define META_D1RtP      dregs[1][4]

#define UNAME_MACHINE "metag"
