// SPDX-License-Identifier: MIT

// SPDX-FileCopyrightText: 2003 Andy Green
// SPDX-FileCopyrightText: 2004 Craig Edwards
// SPDX-FileCopyrightText: 2005-2006 Richard Osborne
// SPDX-FileCopyrightText: 2006 Guillaume Lamonoca
// SPDX-FileCopyrightText: 2017-2020 Jannik Vogel
// SPDX-FileCopyrightText: 2020-2021 Stefan Schmidt

#include <string.h>
#include <stdbool.h>
#include <hal/audio.h>
#include <xboxkrnl/xboxkrnl.h>

// The foundation for this file came from the Cromwell audio driver by 
// Andy (see his comments below).

// Andy@warmcat.com 2003-03-10:
//
// Xbox PC audio is an AC97 compatible audio controller in the MCPX chip
// http://www.amd.com/us-en/assets/content_type/white_papers_and_tech_docs/24467.pdf
// unlike standard AC97 all the regs appear as MMIO from 0xfec00000
// +0 - +7f = Mixer/Codec regs  <-- Wolfson Micro chip
// +100 - +17f = Busmaster regs

// The Wolfson Micro 9709 Codec fitted to the Xbox is a "dumb" codec that
//  has NO PC-controllable registers.  The only regs it has are the
//  manufacturer and device ID ones.

// S/PDIF comes out of the MCPX device and is controlled by an extra set
//  of busmaster DMA registers at +0x170.  These need their own descriptor
//  separate from the PCM Out, although that descriptor can share audio
//  buffers with PCM out successfully.

#define AUDIO_IRQ 6

static volatile unsigned int analogBufferCount;
static volatile unsigned int digitalBufferCount;
static bool analogDrained;
static bool digitalDrained;

static KINTERRUPT InterruptObject;
static KDPC DPCObject;
static bool interruptConnected = false;

// global reference to the ac97 device. Page-aligned: the DMA engines read the
// descriptor lists by physical address, so a list must not cross a page
// boundary (the next virtual page is usually not the next physical one).
AC97_DEVICE ac97Device __attribute__((aligned(4096)));


//DPCs avoid crashes inside non reentrant user callbacks called by nested ISRs.
//CAUTION : if you use fpu in DPC you have to save & restore yourself fpu state!!!
//(fpu=floating point unit, i.e the coprocessor executing floating point opcodes)
static void __stdcall DPC(PKDPC Dpc, 
					PVOID DeferredContext, 
					PVOID SystemArgument1, 
					PVOID SystemArgument2)
{
	AC97_DEVICE *pac97device;

	pac97device = &ac97Device;
	if (pac97device)
		if (pac97device->callback)
			(pac97device->callback)((void *)pac97device, pac97device->callbackData);

	return;
}
	
// Although we have to explicitly clear the S/PDIF interrupt sources, in fact
// the way we are set up PCM and S/PDIF are in lockstep and we only listen for
// PCM actions, since S/PDIF is always spooling through the same buffer.
static BOOLEAN __stdcall ISR(PKINTERRUPT Interrupt, PVOID ServiceContext)
{
	AC97_DEVICE *pac97device = &ac97Device;
	volatile unsigned char *pb = (unsigned char *)pac97device->mmio;

	unsigned char analogInterrupt = pb[0x116];
	unsigned char digitalInterrupt = pb[0x176];

	bool bufferCompleted = false;

	// The S/PDIF engine spools the same buffers as PCM out, so PCM out alone
	// paces the callback. Every completed buffer (BCIS, or LVBCI when the
	// chip has run dry) asks for one refill. Gating this on the two engines
	// being in step, or on a "drained" flag, lost the wakeup for good after
	// an underrun or a refill that landed between the two engines'
	// interrupts; the chip then replayed its last buffer forever.
	if (analogInterrupt) {
		if (analogInterrupt & (8 | 4)) {
			if (analogBufferCount > 0)
				analogBufferCount--;
			bufferCompleted = true;
		}
		analogDrained = analogInterrupt & 4;
		pb[0x116]=0xFF; // clear all int sources
	}

	if (digitalInterrupt) {
		if ((digitalInterrupt & (8 | 4)) && digitalBufferCount > 0)
			digitalBufferCount--;
		digitalDrained = digitalInterrupt & 4;
		pb[0x176]=0xFF; // clear all int sources
	}

	if (bufferCompleted) {
		//KeInsertQueueDpc queues Dpc and returns TRUE if Dpc not already queued.
		//CAUTION : if you use fpu in DPC you have to save & restore yourself fpu state!!!
		KeInsertQueueDpc(&DPCObject,NULL,NULL); //calls user callback soon
	}

	return TRUE;
}


void XDumpAudioStatus(void)
{
	volatile AC97_DEVICE *pac97device = &ac97Device;
	if (pac97device)
	{
		volatile unsigned char *pb = (unsigned char *)pac97device->mmio;
		//debugPrint("CIV=%02x LVI=%02x SR=%04x CR=%02x\n", pb[0x114], pb[0x115], pb[0x116], pb[0x11B]);
	}
}

// Writes a codec (mixer) register, once the codec access semaphore
// (bit 0 of +0x134, taken by reading it) is free; gives up after about 20 ms.
static void ac97WriteCodec(unsigned int reg, unsigned short value)
{
	AC97_DEVICE *pac97device = &ac97Device;
	volatile unsigned char *pb = (unsigned char *)pac97device->mmio;

	for (int i = 0; i < 1000; i++) {
		if (!(pb[0x134] & 1)) {
			*(volatile unsigned short *)&pb[reg] = value;
			return;
		}
		KeStallExecutionProcessor(20);
	}
}

// Reads a codec (mixer) register under the same semaphore. Returns false when
// the semaphore never frees or the codec does not answer (GLOB_STA bit 15,
// read completion status, set by the controller on a read timeout).
static bool ac97ReadCodec(unsigned int reg, unsigned short *value)
{
	AC97_DEVICE *pac97device = &ac97Device;
	volatile unsigned char *pb = (unsigned char *)pac97device->mmio;

	for (int i = 0; i < 1000; i++) {
		if (!(pb[0x134] & 1)) {
			pac97device->mmio[0x130>>2] = 0x8000; // clear a stale timeout
			unsigned short v = *(volatile unsigned short *)&pb[reg];
			if (pac97device->mmio[0x130>>2] & 0x8000) {
				pac97device->mmio[0x130>>2] = 0x8000;
				return false;
			}
			*value = v;
			return true;
		}
		KeStallExecutionProcessor(20);
	}
	return false;
}

// Initialises the audio subsystem.  This *must* be done before
// audio will work.  You can pass NULL as the callback, but if you
// do that, it is your responsibility to keep feeding the data to
// XAudioProvideSamples() manually.
//
// note that I currently ignore sampleSizeInBits and numChannels.  They
// are provided to cope with future enhancements. Currently supported samples
// are 16 bits, 2 channels (stereo)
void XAudioInit(int sampleSizeInBits, int numChannels, XAudioCallback callback, void *data)
{
	AC97_DEVICE * pac97device = &ac97Device;
	KIRQL irql;
	ULONG vector;

	// Re-init (SDL does this on close): drop the old ISR before re-initialising it.
	if (interruptConnected) {
		KeDisconnectInterrupt(&InterruptObject);
		KeRemoveQueueDpc(&DPCObject);
		interruptConnected = false;
	}

	// Hack to prevent an assertion in MmGetPhysicalAddress by locking the memory.
	// A future API redesign should use proper allocation
	// (MmAllocateContiguousMemory) instead.
	MmLockUnlockBufferPages((PVOID)pac97device, sizeof(AC97_DEVICE), FALSE);

	pac97device->mmio = (unsigned int *)0xfec00000;
	pac97device->nextDescriptor = 0;
	pac97device->callback = callback;
	pac97device->callbackData = data;
	pac97device->sampleSizeInBits = sampleSizeInBits;
	pac97device->numChannels = numChannels;

	volatile unsigned char *pb = (unsigned char *)pac97device->mmio;

	// initialise descriptors to all zero (no samples)
	for(unsigned int i = 0; i < 32; i++) {
		pac97device->pcmSpdifDescriptor[i].bufferStartAddress = 0;
		pac97device->pcmSpdifDescriptor[i].bufferLengthInSamples = 0;
		pac97device->pcmSpdifDescriptor[i].bufferControl = 0;
		pac97device->pcmOutDescriptor[i].bufferStartAddress = 0;
		pac97device->pcmOutDescriptor[i].bufferLengthInSamples = 0;
		pac97device->pcmOutDescriptor[i].bufferControl = 0;
	}

	// Bring the codec up. Only release the cold reset (GLOB_CNT bit 1) when it
	// is still held: cold-resetting a codec that is already running is not
	// seen at once. The primary codec ready bit (GLOB_STA bit 8) still reads
	// set right after such a reset, the codec drops out a few milliseconds
	// later, and the codec register writes made in between are lost (or the
	// codec stays not ready, and the next title's audio fails to start).
	unsigned int globalControl = pac97device->mmio[0x12C>>2];
	if (!(globalControl & 2)) {
		pac97device->mmio[0x12C>>2] = globalControl | 2;
	} else if (!(pac97device->mmio[0x130>>2] & 0x100)) {
		// Released but not ready (e.g. the AC-link was shut off): warm reset
		pac97device->mmio[0x12C>>2] = (globalControl | 4) & ~8;
		KeStallExecutionProcessor(10);
		pac97device->mmio[0x12C>>2] = globalControl & ~(4 | 8);
	}

	// wait until the codec is ready (at most 500 ms)
	for (int i = 0; i < 25000 && !(pac97device->mmio[0x130>>2] & 0x100); i++) {
		KeStallExecutionProcessor(20);
	}

	// power up all codec sections and set master and PCM out volume to 0 dB
	// (unmuted): after a cold reset, codecs that implement these registers
	// start muted. Only when the codec came up: a codec register access
	// without a running AC-link can hang the console.
	// Only change what the codec reports as off: a codec the kernel or the
	// last title already set up is left exactly as it is (blind writes here
	// gave static on some console revisions), and one that came out of reset
	// muted or powered down still gets switched on.
	if (pac97device->mmio[0x130>>2] & 0x100) {
		unsigned short v;
		if (ac97ReadCodec(0x26, &v) && (v & 0x7F00))   // PR0-PR6 power-down bits
			ac97WriteCodec(0x26, v & ~0x7F00);         // keep EAPD as it was
		if (ac97ReadCodec(0x02, &v) && (v & 0x8000))   // master muted
			ac97WriteCodec(0x02, 0x0000);
		if (ac97ReadCodec(0x18, &v) && (v & 0x8000))   // PCM out muted
			ac97WriteCodec(0x18, 0x0808);
	}

	// reset bus master registers for analog output
	pb[0x11B] = (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1);
	for (int i = 0; i < 1000 && (pb[0x11B] & (1 << 1)); i++) {
		KeStallExecutionProcessor(1);
	}

	// reset bus master registers for digital output
	pb[0x17B] = (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1);
	for (int i = 0; i < 1000 && (pb[0x17B] & (1 << 1)); i++) {
		KeStallExecutionProcessor(1);
	}

	// clear all interrupts
	pb[0x116] = 0xFF;
	pb[0x176] = 0xFF;

	// tell the audio chip where it should look for the descriptors
	unsigned int pcmAddress = (unsigned int)&pac97device->pcmOutDescriptor[0];
	unsigned int spdifAddress = (unsigned int)&pac97device->pcmSpdifDescriptor[0];
	pac97device->mmio[0x100>>2] = 0;  // no PCM input
	pac97device->mmio[0x110>>2] = MmGetPhysicalAddress((void *)pcmAddress);
	pac97device->mmio[0x170>>2] = MmGetPhysicalAddress((void *)spdifAddress);

	// default to being silent...
	XAudioPause();

	// reset buffer status
	analogBufferCount = 0;
	digitalBufferCount = 0;
	analogDrained = false;
	digitalDrained = false;
	
	// Register our ISR
	vector = HalGetInterruptVector(AUDIO_IRQ, &irql);

	KeInitializeDpc(&DPCObject,&DPC,NULL);

	KeInitializeInterrupt(&InterruptObject,
				&ISR,
				NULL,
				vector,
				irql,
				LevelSensitive,
				FALSE);
	
	interruptConnected = KeConnectInterrupt(&InterruptObject);
}

// Recover an engine that ran past its last valid descriptor: stop and reset
// both bus masters, clear their status, empty the descriptor lists and point
// the chip at them again. On an underrun the MCPX does not halt at LVI; it
// keeps lapping stale descriptors. Leaves the engines stopped; queue samples
// with XAudioProvideSamples, then call XAudioPlay.
void XAudioResync(void)
{
	AC97_DEVICE *pac97device = &ac97Device;
	volatile unsigned char *pb = (unsigned char *)pac97device->mmio;

	if (!pb) return;
	pb[0x11B] = 0;
	pb[0x17B] = 0;
	pb[0x11B] = (1 << 1);
	for (int i = 0; i < 1000 && (pb[0x11B] & (1 << 1)); i++)
		KeStallExecutionProcessor(1);
	pb[0x17B] = (1 << 1);
	for (int i = 0; i < 1000 && (pb[0x17B] & (1 << 1)); i++)
		KeStallExecutionProcessor(1);
	pb[0x116] = 0xFF;
	pb[0x176] = 0xFF;

	for (unsigned int i = 0; i < 32; i++) {
		pac97device->pcmOutDescriptor[i].bufferStartAddress = 0;
		pac97device->pcmOutDescriptor[i].bufferLengthInSamples = 0;
		pac97device->pcmOutDescriptor[i].bufferControl = 0;
		pac97device->pcmSpdifDescriptor[i].bufferStartAddress = 0;
		pac97device->pcmSpdifDescriptor[i].bufferLengthInSamples = 0;
		pac97device->pcmSpdifDescriptor[i].bufferControl = 0;
	}
	__asm__ __volatile__("" ::: "memory");
	pac97device->mmio[0x110>>2] = MmGetPhysicalAddress((void *)&pac97device->pcmOutDescriptor[0]);
	pac97device->mmio[0x170>>2] = MmGetPhysicalAddress((void *)&pac97device->pcmSpdifDescriptor[0]);
	pac97device->nextDescriptor = 0;
	analogBufferCount = 0;
	digitalBufferCount = 0;
	analogDrained = false;
	digitalDrained = false;
	pb[0x11B] = 0x1c; // stopped, interrupts enabled (as XAudioPause)
	pb[0x17B] = 0x1c;
}

// Stop audio before handing the console to another title. HalQuickReboot does
// not reset the MCPX: a completion IRQ afterwards would jump into this
// title's (now unmapped) ISR, and the descriptor bases would point into
// whatever loads over ac97Device.
void XAudioShutdown(void)
{
	AC97_DEVICE *pac97device = &ac97Device;
	volatile unsigned char *pb = (unsigned char *)pac97device->mmio;

	if (!pb) return;
	pac97device->callback = NULL;

	// Reset the bus masters (RR only: no interrupt enables while the ISR is
	// still connected), bounded so a codec that never acks cannot hang.
	pb[0x11B] = (1 << 1);
	for (int i = 0; i < 100000 && (pb[0x11B] & (1 << 1)); i++)
		KeStallExecutionProcessor(1);
	pb[0x17B] = (1 << 1);
	for (int i = 0; i < 100000 && (pb[0x17B] & (1 << 1)); i++)
		KeStallExecutionProcessor(1);

	// Stopped and status cleared before the descriptor bases go away.
	pb[0x116] = 0xFF;
	pb[0x176] = 0xFF;
	pb[0x11B] = 0;
	pb[0x17B] = 0;
	pac97device->mmio[0x110>>2] = 0;
	pac97device->mmio[0x170>>2] = 0;

	if (interruptConnected) {
		KeDisconnectInterrupt(&InterruptObject);
		KeRemoveQueueDpc(&DPCObject);
		interruptConnected = false;
	}
	analogBufferCount = 0;
	digitalBufferCount = 0;
	analogDrained = false;
	digitalDrained = false;
}

// tell the chip it is OK to play...
void XAudioPlay(void)
{
	AC97_DEVICE *pac97device = &ac97Device;
	volatile unsigned char *pb = (unsigned char *)pac97device->mmio;
	pb[0x11B] = 0x1d; // PCM out - run, allow interrupts
	pb[0x17B] = 0x1d; // PCM out - run, allow interrupts
}

// tell the chip it is paused.
void XAudioPause(void)
{
	AC97_DEVICE *pac97device = &ac97Device;
	volatile unsigned char *pb = (unsigned char *)pac97device->mmio;
	pb[0x11B] = 0x1c; // PCM out - PAUSE, allow interrupts
	pb[0x17B] = 0x1c; // PCM out - PAUSE, allow interrupts
}

// This is the function you should call when you want to give the
// audio chip some more data.  If you have registered a callback, it
// should call this method.  If you are providing the samples manually,
// you need to make sure you call this function often enough so the
// chip doesn't run out of data
void XAudioProvideSamples(unsigned char *buffer, unsigned short bufferLength, int isFinal)
{
	AC97_DEVICE *pac97device = &ac97Device;
	volatile unsigned char *pb = (unsigned char *)pac97device->mmio;

	unsigned short bufferControl = 0x8000;
	if (isFinal) 
		bufferControl |= 0x4000;

	unsigned int address = MmGetPhysicalAddress((PVOID)buffer);
	unsigned int wordCount = bufferLength / 2;

	pac97device->pcmOutDescriptor[pac97device->nextDescriptor].bufferStartAddress    = address;
	pac97device->pcmOutDescriptor[pac97device->nextDescriptor].bufferLengthInSamples = wordCount;
	pac97device->pcmOutDescriptor[pac97device->nextDescriptor].bufferControl         = bufferControl;
	__asm__ __volatile__("" ::: "memory"); // descriptor before LVI (-O2)
	pb[0x115] = pac97device->nextDescriptor; // set last active descriptor
	analogBufferCount++;

	pac97device->pcmSpdifDescriptor[pac97device->nextDescriptor].bufferStartAddress    = address;
	pac97device->pcmSpdifDescriptor[pac97device->nextDescriptor].bufferLengthInSamples = wordCount;
	pac97device->pcmSpdifDescriptor[pac97device->nextDescriptor].bufferControl         = bufferControl;
	__asm__ __volatile__("" ::: "memory"); // descriptor before LVI (-O2)
	pb[0x175] = pac97device->nextDescriptor; // set last active descriptor
	digitalBufferCount++;

	// increment to the next buffer descriptor (rolling around to 0 once you get to 31)
	pac97device->nextDescriptor = (pac97device->nextDescriptor + 1) % 32;
}
