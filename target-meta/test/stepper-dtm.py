###############################################################################
# Copyright (C) 2011 Imagination Technologies Ltd.
# Written by James Hogan <james.hogan@imgtec.com>
#
# This is a script for CODESCAPEDebugger 7 to step through a program, recording
# the register states. It generates two log files, one with a dump after every
# cycle, and another with a dump after every non-linear PC change.
#
# How does it work?
# 1) Switch on hardware single stepping
# 2) Set up a periodic DTM to repeatedly:
#    a) read each of the interesting registers
#    b) set bit 0 of TXENABLE to do another step
#
# The intention is for the output to be diffed against a trace from QEMU to
# find where the real and emulated behaviours diverged. The verbose log can
# then be used to see what should be happening.
###############################################################################
# Configuration variables

# It's handy to keep this zero so CODESCAPE doesn't autorun the script and
# overwrite the last file.
enable = 0

# Whether to hard reset the target, and if so where to set the PC entry point.
reset = 1
entry = 0x80000000

# Mask and value of PC to stop at
exit_mask = 0x00000000
exit_val  = 0x10000000

# Whether to hw single step
hwss = 1

# Output log filename. ".log", "-v.log", and "-vv.log" will be appended
output = "/home/jhogan/src/qemu/stepper-test"

maxcyc = 0x10000

# Output format
hexfmt = "0x%x"
#hexfmt = "0x%08x"
logeol = "\n"
flusheol = ""

# Register names to dump (and index of PC in that list). Each line of the dump
# contains the register name followed by a space and the value of the register
# in hex (and with L on the end if top bit is set).
pc_index = 1
regs = ["TXTACTCYC",
	"PC",   "PCX",
	"TXRPT","TXSTATUS",
	"D0Re0","D1Re0",
	"D0Ar6","D1Ar5",
	"D0Ar4","D1Ar3",
	"D0Ar2","D1Ar1",
	"D0FrT","D1RtP",
	"D0.5", "D1.5",
	"D0.6", "D1.6",
	"D0.7", "D1.7",
	"A0StP","A1GbP",
	"A0FrP","A1LbP",
	"A0.2", "A1.2",
	"A0.3", "A1.3",
	]
mems = []

###############################################################################

import codescape as cs
from codescape import da,wx
import sys

target = da.GetAllDAs()[0]     # Only do T0 right now

thread = da.GetCurrentThread()
thread.DTMStopAllRequests()
thread.DTMDeleteAllRequests()
thread.WriteDASetting("DCL High Precision Mode",1,4)

channel_no = 3
channel = thread.GetChannel(channel_no)
channel.Flush()

DTMPeriod               = 1       # In uS. Read as fast as we can
DTMAlwaysRequestID      = 0x9e7f1     # ID used to identify the DTMRequest
ChannelNo               = 3         # DA channel to use

timer = wx.Timer()

last_pc = 0

if enable:
	# Shortened log file.
	# This only includes a register dump after the end of basic blocks of code
	# (where QEMU would do it's trace dump).
	f = open(output + ".log", "w")

	# Verbose log file.
	# This includes a register dump after every instruction we execute
	fv = open(output + "-v.log", "w")

	# Extra verbose log file.
	# This includes a register dump after every cycle we execute
	fvv = open(output + "-vv.log", "w")

def log(msg, v):
	# Print to output?
	#print msg
	# Always print to extra verbose log
	fvv.write(msg+logeol)
	# Only print PC changes to verbose log
	if v < 2:
		fv.write(msg+logeol)
	# Only print to normal log if there's been a change in the control flow
	if v < 1:
		f.write(msg+logeol)

def log_flush(v):
	fvv.write(flusheol)
	fvv.flush()
	# No need to flush the verbose log unless something's been printed to
	# it.
	if v < 2:
		fv.write(flusheol)
		fv.flush()
	# No need to flush the normal log unless something's been printed to
	# it.
	if v < 1:
		f.write(flusheol)
		f.flush()

def Dump(data):
	# A change in control flow is indicated by a non-continuous PC. Note
	# that the PC may legitimately stay on the same instruction for
	# multiple hardware steps, for example with MGET and MSET.
	global last_pc
	pc = data[pc_index]
	v = 1
	if pc == last_pc:
		v = 2
	elif pc != last_pc + 4:
		v = 0
	last_pc = pc
	# Dump the register values.
	for i in range(len(regs)):
		reg = regs[i]
		log(reg+" "+(hexfmt % data[i]), v)
	# Dump the mem values
	for i in range(len(mems)):
		mem = mems[i];
		log((hexfmt % mem)+" "+(hexfmt % data[len(regs)+i]), v)
	log("", v)
	# Make sure we flush so that we can see how it's going and so we can
	# stop the script at any time.
	log_flush(v)
	# If at end point, stop recording
	if (pc & exit_mask) == exit_val:
		Finish()

def Finish():
	timer.Stop()
	thread.DTMStopAllRequests()
	thread.DTMDeleteAllRequests()
	# Disable hardware single stepping.
	thread.WriteRegister("TXPRIVEXT", thread.ReadRegister("TXPRIVEXT") & ~4)

	print "DONE"
	sys.exit()

# Main timer which does stuff every once in a while
def Timer(evt):
	global maxcyc

	bytes = channel.DataReady()
	chunks = bytes / ((len(regs) + len(mems)) * 4)
	for i in range(chunks):
		data = channel.ReadBlock(len(regs) + len(mems), 4)
		Dump(data);
		# Stop if we reach maxcyc
		maxcyc = maxcyc - 1
		if maxcyc == 0:
			Finish()

timer.Bind(wx.EVT_TIMER, Timer)

def setup_DTM():
	# Tear down any existing DTMs, and flush their old data
	thread.DTMStopAllRequests()
	thread.DTMDeleteAllRequests()
	channel.Flush()

	Repeat      = maxcyc # repeat 'n' times - 0 = always
	Flags       = 1    # 1 = raw data only - no ID or timestamps
	TimeStamp   = 0    # No timestamps
	repeating_dtm = thread.CreateDTMRequest(DTMAlwaysRequestID, DTMPeriod, Repeat, ChannelNo, Flags, TimeStamp)

	# Stop
	if not hwss:
		PortID      = 1    # 0 = Bulk Mem , 1 = CoreRegs, 2 = CoreMem
		PortParam   = 1    # 1 = expect a reg name - i.e. a string (when Core Reg is set)
		repeating_dtm.AddWriteRequest(PortID, PortParam, "TXENABLE", 0x0)

	# Read all the registers.
	PortID      = 1    # 0 = Bulk Mem , 1 = CoreRegs, 2 = CoreMem
	PortParam   = 1    # 1 = expect a reg name - i.e. a string (when Core Reg is set)
	for reg in regs:
		repeating_dtm.AddReadRequest(PortID, PortParam, reg)
	# Read memory locations
	PortID      = 0    # 0 = Bulk Mem , 1 = CoreRegs, 2 = CoreMem
	PortParam   = 0    # 1 = expect a reg name - i.e. a string (when Core Reg is set)
	for mem in mems:
		repeating_dtm.AddReadRequest(PortID, PortParam, mem)

	# Single step
	PortID      = 1    # 0 = Bulk Mem , 1 = CoreRegs, 2 = CoreMem
	PortParam   = 1    # 1 = expect a reg name - i.e. a string (when Core Reg is set)
	repeating_dtm.AddWriteRequest(PortID, PortParam, "TXENABLE", 0x1)

	success = thread.DTMStartAllRequests()
	print "DTM set up and started"

def Create():
	if reset:
		# Hard reset the target, clear the registers, and set the PC to
		# the entry point.
		target.cores[0].HardReset()
		for reg in regs:
			thread.WriteRegister(reg, 0)
		thread.WriteRegister("pc", entry)

	# Enable hardware single stepping.
	if hwss:
		thread.WriteRegister("TXPRIVEXT", thread.ReadRegister("TXPRIVEXT") | 4)
	setup_DTM()

	# Start the main timer and repeat as frequently as possible.
	timer.Start(1)

if enable:
	Create()
	cs.MainLoop()
	Finish()
