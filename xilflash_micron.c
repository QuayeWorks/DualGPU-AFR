
/***************************** Include Files *********************************/

#ifndef XFL_MICRON_UART_PROGRESS
#define XFL_MICRON_UART_PROGRESS 1
#endif

#if XFL_MICRON_UART_PROGRESS
#include "include/utilities.h"
#endif

#include "include/xilflash.h"
#include "include/xstatus.h"
#ifdef SDT
#include "xilflash_config.h"
#endif

#ifdef XPAR_XFL_DEVICE_FAMILY_MICRON
#include "include/xilflash_micron.h"
#include "include/xilflash_cfi.h"


/************************** Constant Definitions *****************************/
#define XFL_MICRON_POLL_TIMEOUT    0x00800000U  /* Original / failed before */
//#define XFL_MICRON_POLL_TIMEOUT    0x02000000U  /* Small step up */
//#define XFL_MICRON_POLL_TIMEOUT    0x04000000U  /* Medium */
//#define XFL_MICRON_POLL_TIMEOUT    0x08000000U  /* Large */
//#define XFL_MICRON_POLL_TIMEOUT    0x10000000U  /* Very large */
//#define XFL_MICRON_POLL_TIMEOUT    0x20000000U  /* Known-good from your test */

#define XFL_MICRON_MT28EW128_DEVICE_ID_2	0x22U

/*
 * In XFL_LAYOUT_X16_X16_X2, two x16 devices are accessed in parallel
 * through one 32-bit CPU access. Status bits must be checked in both
 * halfwords, not only the lower x16 device.
 */
#define XFL_MICRON_SR_TOGGLE_MASK32 \
	((XFL_MICRON_SR_ERASE_COMPL_MASK << 16) | \
	 XFL_MICRON_SR_ERASE_COMPL_MASK)

#define XFL_MICRON_SR_ERROR_MASK32 \
	((XFL_MICRON_SR_ERASE_ERROR_MASK << 16) | \
	 XFL_MICRON_SR_ERASE_ERROR_MASK)

#define XFL_MICRON_GROUP_PROTECTED32 \
	((XFL_MICRON_GROUP_PROTECTED << 16) | \
	 XFL_MICRON_GROUP_PROTECTED)

#define XFL_MICRON_GROUP_UNPROTECTED32 \
	((XFL_MICRON_GROUP_UNPROTECTED << 16) | \
	 XFL_MICRON_GROUP_UNPROTECTED)

/**************************** Type Definitions *******************************/

/*
 * Define Micron specific data to be part of the instance. This structure will
 * overlay the XFlash_PartData structure attribute of the base class.
 */
typedef struct XFlashVendorData_MicronTag {
	/*
	 * The following functions are specific to the width of the data bus and
	 * will be assigned during initialization.
	 *
	 * SendCmd - Writes a single command to the devices.
	 * SendCmdSeq - Writes two commands in successive bus cycles to the
	 *		devices.
	 * WriteCmd - Writes command/control data at an arbitrary offset.
	 * WriteFlash - Writes raw program payload data at an arbitrary offset.
	 * WriteBuffer - Programming algorithm optimized to perform bulk writes
	 *		to devices.
	 * GetStatus - Retrieve and interpret the status registers of the
	 *		devices.
	 * PollSR - Poll the status register of the devices until the device
	 *		is ready.
	 */
	void (*SendCmd) (XFlash *InstancePtr, u32 BaseAddr, u32 Offset, u32 Cmd);
	void (*SendCmdSeq) (XFlash *InstancePtr, u32 BaseAddr, u32 Offset1,
			    u32 Offset2, u32 Cmd1, u32 Cmd2);
	void (*WriteCmd) (XFlash *InstancePtr, u32 BaseAddr, u32 Offset,
			  u32 Cmd);
	void (*WriteFlash) (u32 BaseAddr, u32 Offset, u32 Data);
	int (*WriteBuffer) (XFlash *InstancePtr, void *DestPtr,
			    void *SrcPtr, u32 Bytes);

	int (*GetStatus) (u32 BaseAddr, u32 BlockAddr);
	int (*PollSR) (u32 BaseAddr, u32 BlockAddr);
} XFlashVendorData_Micron;


/***************** Macros (Inline Functions) Definitions *********************/

/*****************************************************************************
* GET_PARTDATA - Type safe way to convert the base component VendorData type
*		to the family specific VendorData_Micron type.
*
* Macro signature:
*
*	XFlashVendorData_Micron *GET_PARTDATA(XFlash_PartData *BaseComponent)
*****************************************************************************/
#define GET_PARTDATA(BaseComponent) \
	((XFlashVendorData_Micron*) ((u32)(&(BaseComponent->VendorData))))


/************************** Function Prototypes ******************************/

static u32 XFlashMicron_ExpandCmdForBus(XFlash *InstancePtr, u32 Cmd);

static void SendCmd8(XFlash *InstancePtr, u32 BaseAddr, u32 Offset, u32 Cmd);
static void SendCmd16(XFlash *InstancePtr, u32 BaseAddr, u32 Offset, u32 Cmd);
static void SendCmd32(XFlash *InstancePtr, u32 BaseAddr, u32 Offset, u32 Cmd);

static void SendCmdSeq8(XFlash *InstancePtr, u32 BaseAddr, u32 Offset1,
			u32 Offset2, u32 Cmd1, u32 Cmd2);
static void SendCmdSeq16(XFlash *InstancePtr, u32 BaseAddr, u32 Offset1,
			 u32 Offset2, u32 Cmd1, u32 Cmd2);
static void SendCmdSeq32(XFlash *InstancePtr, u32 BaseAddr, u32 Offset1,
			 u32 Offset2, u32 Cmd1, u32 Cmd2);

static void WriteCmd8(XFlash *InstancePtr, u32 BaseAddr, u32 Offset, u32 Cmd);
static void WriteCmd16(XFlash *InstancePtr, u32 BaseAddr, u32 Offset, u32 Cmd);
static void WriteCmd32(XFlash *InstancePtr, u32 BaseAddr, u32 Offset, u32 Cmd);

static void WriteFlash8(u32 BaseAddr, u32 Offset, u32 Data);
static void WriteFlash16(u32 BaseAddr, u32 Offset, u32 Data);
static void WriteFlash32(u32 BaseAddr, u32 Offset, u32 Data);

static int WriteBuffer8(XFlash *InstancePtr, void *DestPtr,
			void *SrcPtr, u32 Bytes);
static int WriteBuffer16(XFlash *InstancePtr, void *DestPtr,
			 void *SrcPtr, u32 Bytes);
static int WriteBuffer32(XFlash *InstancePtr, void *DestPtr,
			 void *SrcPtr, u32 Bytes);

static int GetStatus8(u32 BaseAddr, u32 BlockAddr);
static int GetStatus16(u32 BaseAddr, u32 BlockAddr);
static int GetStatus32(u32 BaseAddr, u32 BlockAddr);

static int PollSR8(u32 BaseAddr, u32 BlockAddr);
static int PollSR16(u32 BaseAddr, u32 BlockAddr);
static int PollSR32(u32 BaseAddr, u32 BlockAddr);

static u32 XFlashMicron_ReadRawId(XFlash *InstancePtr, u32 Offset);
static u8 XFlashMicron_NormalizeIdByte(u32 RawId);
static void GetPartID(XFlash *InstancePtr);
static u16 EnqueueEraseBlocks(XFlash *InstancePtr, u16 *Region,
			      u16 *Block, u16 MaxBlocks);
static int EraseResume (XFlash *InstancePtr, u32 EraseAddrOff);
static int EraseSuspend (XFlash *InstancePtr, u32 EraseAddrOff);
static void EnterExtendedBlockMode(XFlash *InstancePtr);
static void ExitExtendedBlockMode(XFlash *InstancePtr);
static int CheckBlockProtection(XFlash *InstancePtr, u32 Offset);
static void  FlashPause(u32 MicroSeconds);
static int XFlashMicron_ResetBank(XFlash *InstancePtr, u32 Offset, u32 Bytes);
extern int XFlashGeometry_ToBlock(XFlashGeometry *InstancePtr,
				  u32 AbsoluteOffset,
				  u16 *Region, u16 *Block, u32 *BlockOffset);
extern int XFlashGeometry_ToAbsolute(XFlashGeometry *InstancePtr,
				     u16 Region,
				     u16 Block,
				     u32 BlockOffset, u32 *AbsoluteOffsetPtr);
static int WriteSingleBufferMicron(XFlash *InstancePtr, void *DestPtr,
				   void *SrcPtr, u32 Bytes);
static int WriteBufferMicronBypass(XFlash *InstancePtr, void *DestPtr,
				   void *SrcPtr, u32 Bytes);
static int WriteBufferSpansion(XFlash *InstancePtr, void *DestPtr,
			       void *SrcPtr, u32 Bytes);
				   
				  
volatile u32 XFlashMicronDbgPoll32BaseAddr = 0U;
volatile u32 XFlashMicronDbgPoll32BlockAddr = 0U;
volatile u32 XFlashMicronDbgPoll32LastStatus1 = 0U;
volatile u32 XFlashMicronDbgPoll32LastStatus2 = 0U;
volatile u32 XFlashMicronDbgPoll32PollCountRemaining = 0U;
volatile u32 XFlashMicronDbgPoll32ExitCode = 0U;

#if XFL_MICRON_UART_PROGRESS
static void XFlashMicron_PrintHex(char *Label, u32 Value);
static void XFlashMicron_PrintDec(char *Label, u32 Value);
static void XFlashMicron_PrintEraseProgress(u32 Done, u32 Total,
					    u32 Offset, u32 AbsoluteAddress);
#endif
/************************** Variable Definitions *****************************/


/*****************************************************************************/
/**
*
* Initializes the Micron flash driver portion of the XFlash instance. This
* routine selects the correct command, read, write, erase, and polling helper
* functions based on the flash memory layout reported by CFI.
*
* For the current board, the expected layout is a 32-bit flash bus made from
* two x16 Micron MT28EW devices. In that mode this routine assigns the x32
* helper functions so commands are written to both x16 devices in parallel.
*
* This routine also reads the Micron part ID, adjusts the geometry information
* used by the XFlash layer, recalculates erase-region offsets when required,
* and resets the flash back into read-array mode before returning.
*
* @param	InstancePtr is a pointer to the XFlash instance.
*
* @return
*		- XST_SUCCESS if initialization completed successfully.
*		- XST_FAILURE if InstancePtr is invalid or reset fails.
*		- XFLASH_PART_NOT_SUPPORTED if the flash layout is unsupported.
*
* @note		For the x32 MT28EW setup, the selected write path is
*		WriteBuffer32(), and the selected polling path is PollSR32().
*
******************************************************************************/

int XFlashMicron_Initialize(XFlash *InstancePtr)
{
	u8 Index;
	u8 BusWidth;
	u32 Layout;
	u32 PartMode;
	u32 ParamBlocks;
	u32 ParamBlockSize;
	u32 CurrentAbsoluteOffset;
	u32 CurrentAbsoluteBlock;
	XFlashVendorData_Micron *DevDataPtr;

	/*
	 * Verify inputs are valid.
	 */
	if (InstancePtr == NULL) {
		return (XST_FAILURE);
	}

	/*
	 * Grab layout and get Vendor specific part information.
	 */
	Layout = InstancePtr->Geometry.MemoryLayout;
	DevDataPtr = GET_PARTDATA(InstancePtr);

	/*
	 * Setup alignment of the write buffer.
	 */
	if (InstancePtr->Properties.ProgCap.WriteBufferSize != 0) {
		InstancePtr->Properties.ProgCap.WriteBufferAlignmentMask =
			InstancePtr->Properties.ProgCap.WriteBufferSize - 1;
	}

	/*
	 * Setup layout dependent attributes. These include:
	 *	- Part access primitive functions optimized for a specific bus
	 *	  width.
	 */
	switch (Layout) {
		case XFL_LAYOUT_X16_X8_X1:
			DevDataPtr->SendCmd = SendCmd8;
			DevDataPtr->SendCmdSeq = SendCmdSeq8;
			DevDataPtr->WriteCmd = WriteCmd8;
			DevDataPtr->WriteFlash = WriteFlash8;
			DevDataPtr->PollSR = PollSR8;
			DevDataPtr->WriteBuffer = WriteBuffer8;
			DevDataPtr->GetStatus = GetStatus8;
			break;

		case XFL_LAYOUT_X16_X16_X1:
			DevDataPtr->SendCmd = SendCmd16;
			DevDataPtr->SendCmdSeq = SendCmdSeq16;
			DevDataPtr->WriteCmd = WriteCmd16;
			DevDataPtr->WriteFlash = WriteFlash16;
			DevDataPtr->PollSR = PollSR16;
			DevDataPtr->WriteBuffer = WriteBuffer16;
			DevDataPtr->GetStatus = GetStatus16;
			break;

		case XFL_LAYOUT_X16_X16_X2:
			DevDataPtr->SendCmd = SendCmd32;
			DevDataPtr->SendCmdSeq = SendCmdSeq32;
			DevDataPtr->WriteCmd = WriteCmd32;
			DevDataPtr->WriteFlash = WriteFlash32;
			DevDataPtr->PollSR = PollSR32;
			DevDataPtr->WriteBuffer = WriteBuffer32;
			DevDataPtr->GetStatus = GetStatus32;
			break;

		default:
			return (XFLASH_PART_NOT_SUPPORTED);
	}

	/*
	 * Get part ID and Reset the part.
	 */
	GetPartID(InstancePtr);

	/*
	 * Setup bank information as per the boot block location.
	 */

	/*
	 * Get device operational mode.
	 */
	PartMode = (Layout & XFL_LAYOUT_PART_MODE_MASK);
	if (PartMode == XFL_LAYOUT_PART_MODE_8) {
		BusWidth = 1;
	} else if (PartMode == XFL_LAYOUT_PART_MODE_16) {
		BusWidth = 2;
	} else {
		return (XFLASH_PART_NOT_SUPPORTED);
	}
	/************Start replacing below this*****************/
	if (Layout != XFL_LAYOUT_X16_X16_X2) {
		InstancePtr->Geometry.DeviceSize =
			InstancePtr->Geometry.DeviceSize / BusWidth;

		for (Index = 0; Index < InstancePtr->Geometry.NumEraseRegions;
			 Index++) {
			InstancePtr->Geometry.EraseRegion[Index].Size =
				InstancePtr->Geometry.EraseRegion[Index].Size /
				BusWidth;
		}
	}
	/* If not seeing 0x00400000 then replace top with this version********/
	/*if (Layout != XFL_LAYOUT_X16_X16_X2) {
		InstancePtr->Geometry.DeviceSize =
			InstancePtr->Geometry.DeviceSize / BusWidth;

		for (Index = 0; Index < InstancePtr->Geometry.NumEraseRegions;
		     Index++) {
			InstancePtr->Geometry.EraseRegion[Index].Size =
				InstancePtr->Geometry.EraseRegion[Index].Size /
				BusWidth;
		}
	}*/
	
	/*
	 * If device is top boot then change erase region information.
	 */
	if (((InstancePtr->Geometry.BootMode == XFL_MICRON_TOP_BOOT) ||
	     (InstancePtr->Geometry.BootMode == XFL_MICRON_TOP_WP_UNIFORM)) &&
	    (InstancePtr->Geometry.NumEraseRegions > 1)) {
		/*
		 * Move boot block region to top.
		 */
		ParamBlocks = InstancePtr->Geometry.EraseRegion[0].Number;
		ParamBlockSize = InstancePtr->Geometry.EraseRegion[0].Size;

		InstancePtr->Geometry.EraseRegion[0].Number =
			InstancePtr->Geometry.EraseRegion[1].Number;
		InstancePtr->Geometry.EraseRegion[0].Size =
			InstancePtr->Geometry.EraseRegion[1].Size ;

		InstancePtr->Geometry.EraseRegion[1].Number = ParamBlocks ;
		InstancePtr->Geometry.EraseRegion[1].Size = ParamBlockSize;

		/*
		 * Calculate Erase region offset and block address.
		 */
		CurrentAbsoluteOffset = 0;
		CurrentAbsoluteBlock = 0;
		for (Index = 0;
		     Index < InstancePtr->Geometry.NumEraseRegions;
		     Index++) {
			/*
			 * Calculate part offset.
			 */
			InstancePtr->Geometry.EraseRegion[Index].AbsoluteOffset
				= CurrentAbsoluteOffset;
			InstancePtr->Geometry.EraseRegion[Index].AbsoluteBlock
				= CurrentAbsoluteBlock;

			/*
			 * Increment absolute counters.
			 */
			CurrentAbsoluteOffset +=
				(InstancePtr->Geometry.EraseRegion[Index].Size *
				 InstancePtr->Geometry.EraseRegion[Index].Number);
			CurrentAbsoluteBlock +=
				InstancePtr->Geometry.EraseRegion[Index].Number;
		}
	} else if (BusWidth != 1) {
		/*
		 * Calculate Erase region offset and block address.
		 */
		CurrentAbsoluteOffset = 0;
		CurrentAbsoluteBlock = 0;
		for (Index = 0;
		     Index < InstancePtr->Geometry.NumEraseRegions;
		     Index++) {
			/*
			 * Calculate part offset.
			 */
			InstancePtr->Geometry.EraseRegion[Index].AbsoluteOffset
				= CurrentAbsoluteOffset;
			InstancePtr->Geometry.EraseRegion[Index].AbsoluteBlock
				= CurrentAbsoluteBlock;

			/*
			 * Increment absolute counters.
			 */
			CurrentAbsoluteOffset +=
				(InstancePtr->Geometry.EraseRegion[Index].Size *
				 InstancePtr->Geometry.EraseRegion[Index].Number);
			CurrentAbsoluteBlock +=
				InstancePtr->Geometry.EraseRegion[Index].Number;
		}
	}


	(void) XFlashMicron_Reset(InstancePtr);

	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
*
* Reads data from the Micron flash into a user-provided RAM buffer. The caller
* provides an offset relative to the flash base address, not an absolute flash
* address.
*
* Before copying data, this routine resets the selected flash bank/range back
* into read-array mode. This prevents stale command modes such as auto-select,
* CFI query, unlock-bypass, erase, or program status mode from affecting the
* read.
*
* On the current x32 board, the final physical read address is:
*
*	FlashInstance.Geometry.BaseAddress + Offset
*
* Example:
*
*	BaseAddress = 0xFC000000
*	Offset      = 0x03FF6000
*	Read addr   = 0xFFFF6000
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	Offset is the flash offset relative to the flash base address.
* @param	Bytes is the number of bytes to read.
* @param	DestPtr is the RAM destination buffer.
*
* @return
*		- XST_SUCCESS if the read completed successfully.
*		- XST_FAILURE if an input parameter is invalid.
*		- XFLASH_ADDRESS_ERROR if the requested range is outside flash.
*		- XFLASH_PART_NOT_SUPPORTED if the layout is unsupported.
*
* @note		This routine does not erase or program flash. It only reads.
*
******************************************************************************/

int XFlashMicron_Read(XFlash *InstancePtr, u32 Offset, u32 Bytes, void *DestPtr)
{
	u8 *Dest8BitPtr;
	u8 *Src8BitPtr;
	u16 *Dest16BitPtr;
	u16 *Src16BitPtr;
	u32  PartMode;
	u32 Index;
	u32 Startoffset;
	u32 EndOffset;
	XFlashGeometry *GeomPtr;

	/* Verify inputs are valid. */
	if (InstancePtr == NULL) {
		return (XST_FAILURE);
	}

	if (DestPtr == NULL) {
		return (XST_FAILURE);
	}

	if (Bytes == 0) {
		return (XST_FAILURE);
	}

	GeomPtr = &InstancePtr->Geometry;

	if (InstancePtr->Geometry.MemoryLayout == XFL_LAYOUT_X16_X16_X1) {
		Startoffset = Offset >> 1;
		EndOffset = Offset + Bytes - 1;
		EndOffset = EndOffset >> 1;
	} else {
		Startoffset = Offset;
		EndOffset = Offset + Bytes - 1;
	}
	PartMode = (InstancePtr->Geometry.MemoryLayout &
		    XFL_LAYOUT_PART_MODE_MASK);

	/*
	 * Check to make sure start address is within the device.
	 */
	if (!XFL_GEOMETRY_IS_ABSOLUTE_VALID(&InstancePtr->Geometry,
					    Startoffset) ||
	    !XFL_GEOMETRY_IS_ABSOLUTE_VALID(&InstancePtr->Geometry,
					    EndOffset)) {
		return (XFLASH_ADDRESS_ERROR);
	}

	/*
	 * Reset the bank(s) so that it returns to the read mode.
	 */
	if (XFlashMicron_ResetBank(InstancePtr, Startoffset, Bytes)
	    != XST_SUCCESS) {
		return (XST_FAILURE);
	}
	
	if (InstancePtr->Geometry.MemoryLayout == XFL_LAYOUT_X16_X16_X2) {
		Src8BitPtr = (u8 *)(InstancePtr->Geometry.BaseAddress + Offset);
		Dest8BitPtr = (u8 *)DestPtr;

		for (Index = 0U; Index < Bytes; Index++) {
			Dest8BitPtr[Index] = Src8BitPtr[Index];
		}

		return XST_SUCCESS;
	}

	/*
	 * Perform copy to the user buffer from the flash buffer.
	 */
	if (PartMode == XFL_LAYOUT_PART_MODE_8) {
		/*
		 * Perform copy to the user buffer from the buffer.
		 */
		Src8BitPtr = (u8 *) (((volatile u8 *)InstancePtr->Geometry.
				      BaseAddress) + Offset);
		Dest8BitPtr = (u8 *) DestPtr;

		for (Index = 0; Index < Bytes; Index++) {
			Dest8BitPtr[Index] = Src8BitPtr[Index];
		}

	} else if (PartMode == XFL_LAYOUT_PART_MODE_16) {

		/* Perform copy to the user buffer from the buffer. */
		Src16BitPtr = (u16 *) (((volatile u16 *) InstancePtr->Geometry.
					BaseAddress) + Startoffset);
		Dest16BitPtr = (u16 *) DestPtr;
		for (Index = 0; Index < (Bytes / 2); Index++) {
			Dest16BitPtr[Index] = Src16BitPtr[Index];
		}
	} else {
		return (XFLASH_PART_NOT_SUPPORTED);
	}

	return (XST_SUCCESS);
}


/*****************************************************************************/
/**
*
* Programs data from a RAM buffer into the Micron flash. The caller provides
* an offset relative to the flash base address, not an absolute flash address.
*
* This routine does not erase flash before writing. The target flash range must
* already be erased to 0xFF before programming. NOR flash can program bits from
* 1 to 0, but it cannot program bits from 0 back to 1 without an erase.
*
* The routine validates the requested flash range, then dispatches to the
* width-specific WriteBuffer function selected during initialization. For the
* current x32 MT28EW board, this calls WriteBuffer32().
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	Offset is the flash offset relative to the flash base address.
* @param	Bytes is the number of bytes to program.
* @param	SrcPtr is the RAM source buffer containing data to write.
*
* @return
*		- XST_SUCCESS if the write completed successfully.
*		- XST_FAILURE if an input parameter is invalid.
*		- XFLASH_ADDRESS_ERROR if the requested range is outside flash.
*		- XFLASH_ALIGNMENT_ERROR if the write is not bus-width aligned.
*		- XFLASH_ERROR or XFLASH_BUSY if flash polling fails.
*
* @note		The caller should verify the programmed data by reading flash
*		back and comparing it against the RAM source buffer.
*
******************************************************************************/

int XFlashMicron_Write(XFlash *InstancePtr, u32 Offset, u32 Bytes, void *SrcPtr)
{
	int Status = (int)XST_FAILURE;
	u32 StartOffset;
	u32 EndOffset;
	XFlashVendorData_Micron *DevDataPtr;

	/* Verify inputs are valid. */
	if (InstancePtr == NULL) {
		return (XST_FAILURE);
	}

	if (SrcPtr == NULL) {
		return (XST_FAILURE);
	}

	/* Nothing specified to be programmed. */
	if (Bytes == 0) {
		return (XST_SUCCESS);
	}

	if (InstancePtr->Geometry.MemoryLayout == XFL_LAYOUT_X16_X16_X1) {
		StartOffset = Offset >> 1;
		EndOffset = Offset + Bytes - 1;
		EndOffset = EndOffset >> 1;
	} else {
		StartOffset = Offset ;
		EndOffset = Offset + Bytes - 1;
	}

	/* Verify the address range is within the part. */
	if (!XFL_GEOMETRY_IS_ABSOLUTE_VALID(&InstancePtr->Geometry, StartOffset)
	    || !XFL_GEOMETRY_IS_ABSOLUTE_VALID(&InstancePtr->Geometry,
					       EndOffset)) {
		return (XFLASH_ADDRESS_ERROR);
	}

	/* Call the proper write buffer function. */
	DevDataPtr = GET_PARTDATA(InstancePtr);
	Status = DevDataPtr->WriteBuffer(InstancePtr, (void *)StartOffset, SrcPtr,
					 (Bytes));

	/* Reset the bank(s) so that it returns to the read mode. */
	(void) XFlashMicron_ResetBank(InstancePtr, StartOffset, Bytes);

	return (Status);
}


#if XFL_MICRON_UART_PROGRESS

static void XFlashMicron_PrintHex(char *Label, u32 Value)
{
	char Text[20];

	print_chars(Label);
	bin2hex(Text, Value);
	print_line(Text);
}

static void XFlashMicron_PrintDec(char *Label, u32 Value)
{
	char Text[20];

	print_chars(Label);
	bin2ascii(Text, Value);
	print_line(Text);
}

static void XFlashMicron_PrintEraseProgress(u32 Done, u32 Total,
					    u32 Offset, u32 AbsoluteAddress)
{
	char Text[20];

	print_chars("Erasing sector ");
	bin2ascii(Text, Done);
	print_chars(Text);

	print_chars(" / ");
	bin2ascii(Text, Total);
	print_chars(Text);

	print_chars("  offset ");
	bin2hex(Text, Offset);
	print_chars(Text);

	print_chars("  abs ");
	bin2hex(Text, AbsoluteAddress);
	print_line(Text);
}

#endif


/*****************************************************************************/
/**
*
* Erases the specified address range in the AMD Flash device.
*
* @param	InstancePtr is the pointer to the XFlash instance.
* @param	Offset is the offset into the device(s) address space from which
*		to begin erasure.
* @param	Bytes is the number of bytes to erase.
*
* @return
*		- XST_SUCCESS if successful.
*		- XFLASH_ADDRESS_ERROR if the destination address range is
*		  not completely within the addressable areas of the device(s).
*		- XST_FAILURE if failed.
*
* @note		Application has to check block protection status of all the
*		which needs to be erased before calling this API. If any block
* 		is protected then this API will return error.
*
******************************************************************************/
int XFlashMicron_Erase(XFlash *InstancePtr, u32 Offset, u32 Bytes)
{
	u16 StartRegion;
	u16 EndRegion;
	u16 StartBlock;
	u16 EndBlock;
	u16 BlocksLeft;
	u16 BlocksQueued;
	u32 Dummy;
	u32 StartOffset;
	u32 EndOffset;
	u32 BlockOffset;
	u32 AbsoluteAddress;
	u32 TotalBlocks;
	u32 DoneBlocks;
	int Status = (int)XST_FAILURE;
	XFlashGeometry *GeomPtr;
	XFlashVendorData_Micron *DevDataPtr;

	/* Verify inputs are valid. */
	if (InstancePtr == NULL) {
		return (XST_FAILURE);
	}

	GeomPtr = &InstancePtr->Geometry;

	/* Handle case when zero bytes is provided. */
	if (Bytes == 0) {
		return (XST_SUCCESS);
	}

	if (InstancePtr->Geometry.MemoryLayout == XFL_LAYOUT_X16_X16_X1) {
		StartOffset = Offset >> 1;
		EndOffset = Offset + Bytes - 1;
		EndOffset = EndOffset >> 1;
	} else {
		StartOffset = Offset;
		EndOffset = Offset + Bytes - 1;
	}

	/*
	 * Convert the starting address to block coordinates. This also verifies
	 * the starting address is within the instance's address space.
	 */
	Status = XFlashGeometry_ToBlock(GeomPtr, StartOffset, &StartRegion,
					&StartBlock, &Dummy);
	if (Status != XST_SUCCESS) {
		return (XFLASH_ADDRESS_ERROR);
	}

	/*
	 * Convert the ending address to block coordinates. This also verifies
	 * the ending address is within the instance's address space.
	 */
	Status = XFlashGeometry_ToBlock(GeomPtr, EndOffset,
					&EndRegion, &EndBlock, &Dummy);
	if (Status != XST_SUCCESS) {
		return (XFLASH_ADDRESS_ERROR);
	}

	DevDataPtr = GET_PARTDATA(InstancePtr);

	BlocksLeft = XFL_GEOMETRY_BLOCK_DIFF(GeomPtr, StartRegion, StartBlock,
					     EndRegion, EndBlock);
	TotalBlocks = BlocksLeft;
	DoneBlocks = 0U;

#if XFL_MICRON_UART_PROGRESS
	print_line("");
	print_line("XFlashMicron_Erase begin");
	XFlashMicron_PrintHex("Erase offset = ", Offset);
	XFlashMicron_PrintHex("Erase bytes  = ", Bytes);
	XFlashMicron_PrintDec("Erase sectors = ", TotalBlocks);
#endif

	while (BlocksLeft > 0) {
		/*
		 * Capture current block address before EnqueueEraseBlocks()
		 * increments StartRegion/StartBlock.
		 */
		BlockOffset = 0U;
		AbsoluteAddress = 0U;

		(void)XFlashGeometry_ToAbsolute(GeomPtr, StartRegion,
						StartBlock, 0,
						&BlockOffset);

		AbsoluteAddress = GeomPtr->BaseAddress + BlockOffset;

#if XFL_MICRON_UART_PROGRESS
		XFlashMicron_PrintEraseProgress(DoneBlocks + 1U,
						TotalBlocks,
						BlockOffset,
						AbsoluteAddress);
#endif

		BlocksQueued = EnqueueEraseBlocks(InstancePtr, &StartRegion,
						  &StartBlock, BlocksLeft);
		BlocksLeft -= BlocksQueued;

		Status = DevDataPtr->PollSR(GeomPtr->BaseAddress, BlockOffset);

		if (Status != XFLASH_READY) {
#if XFL_MICRON_UART_PROGRESS
			print_line("XFlashMicron_Erase poll failed");
			XFlashMicron_PrintHex("Poll status = ", (u32)Status);
			XFlashMicron_PrintHex("Block offset = ", BlockOffset);
			XFlashMicron_PrintHex("Block abs    = ", AbsoluteAddress);
#endif
			(void) XFlashMicron_ResetBank(InstancePtr, StartOffset,
						   Bytes);
			return (Status);
		}

		DoneBlocks += BlocksQueued;

#if XFL_MICRON_UART_PROGRESS
		print_chars("Erased sector ");
		{
			char Text[20];

			bin2ascii(Text, DoneBlocks);
			print_chars(Text);
			print_chars(" / ");
			bin2ascii(Text, TotalBlocks);
			print_line(Text);
		}
#endif
	}

	/* Reset the bank(s) so that it returns to the read mode. */
	(void) XFlashMicron_ResetBank(InstancePtr, StartOffset, Bytes);

#if XFL_MICRON_UART_PROGRESS
	print_line("XFlashMicron_Erase complete");
#endif

	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
*
* Locks the blocks in the specified range of the Micron flash device.
*
* @param	InstancePtr is the pointer to the XFlash instance.
* @param	Offset is the offset into the device(s) address space from which
*		to begin block locking.
* @param	Bytes indicates the number of bytes to Lock in the Block
*		starting from Offset.
*
* @return
* 		- XST_SUCCESS if successful.
*		- XFLASH_ADDRESS_ERROR if the destination address range is
*		  not completely within the addressable areas of the device(s).
*		- XST_FAILURE if failed.
*
* @note		This API should be called after applying VID (voltage) to the RP
*		pin of flash device. This API is not tested.
*
******************************************************************************/
int XFlashMicron_Lock(XFlash *InstancePtr, u32 Offset, u32 Bytes)
{
	u8 Mode;
	u8 AddrMul;
	volatile u16 StatusReg;
	u16 Region;
	u16 Block;
	u16 NumAttempt = 0;
	u32 Dummy;
	u32 BaseAddress;
	u32 BlockAddress;
	u32 GroupAddress;
	int Status = (int)XST_FAILURE;
	XFlashGeometry *GeomPtr;
	XFlashVendorData_Micron *DevDataPtr;

	/*
	 * Verify inputs are valid.
	 */
	if (InstancePtr == NULL) {
		return (XST_FAILURE);
	}

	DevDataPtr = GET_PARTDATA(InstancePtr);

	GeomPtr = &InstancePtr->Geometry;
	BaseAddress = GeomPtr->BaseAddress;

	Mode = (InstancePtr->Geometry.MemoryLayout &
		XFL_LAYOUT_PART_MODE_MASK) >> 8;

	if (Mode == 2) {
		AddrMul = 1;
	} else if (Mode == 1) {
		AddrMul = 2;
	} else {
		return (XST_FAILURE);
	}

	/*
	 * Convert the starting address to block coordinates. This also verifies
	 * the starting address is within the instance's address space.
	 */
	Status = XFlashGeometry_ToBlock(GeomPtr, Offset, &Region, &Block,
					&Dummy);
	if (Status != XST_SUCCESS) {
		return (XFLASH_ADDRESS_ERROR);
	}

	/*
	 * Get the physical address to write the command to and send command.
	 */
	(void) XFlashGeometry_ToAbsolute(GeomPtr, Region, Block, 0,
					 &BlockAddress);

	GroupAddress = (BlockAddress | (XFL_MICRON_PROT_STATUS_OFFSET * AddrMul));

	/*
	 * Set-up Phase.
	 */
	DevDataPtr->WriteCmd(InstancePtr, BaseAddress,
			     GroupAddress, XFL_MICRON_CMD_GROUP_PROTECT1);

	Status = XST_FAILURE;
	do {
		/*
		 * Protect Phase.
		 */
		DevDataPtr->WriteCmd(InstancePtr, BaseAddress,
				     GroupAddress, XFL_MICRON_CMD_GROUP_PROTECT1);

		FlashPause(100);

		/*
		 * Verify Phase.
		 */
		DevDataPtr->WriteCmd(InstancePtr, BaseAddress,
				     GroupAddress, XFL_MICRON_CMD_GROUP_PROTECT2);

		FlashPause(4);

		StatusReg = DevDataPtr->GetStatus(InstancePtr->Geometry.
						  BaseAddress, GroupAddress);
		/*
		 * Check Protection completed.
		 */
		if (StatusReg == XFL_MICRON_GROUP_PROTECTED) {
			Status = XST_SUCCESS;
			break;
		}
	} while (++NumAttempt < 25);

	/*
	 * Reset the bank(s) so that it returns to the read mode.
	 */
	(void) XFlashMicron_ResetBank(InstancePtr, Offset, Bytes);

	return (Status);
}

/*****************************************************************************/
/**
*
* Unlocks the all blocks in the Micron flash device.
*
* @param	InstancePtr is the pointer to the XFlash instance.
* @param	Offset is the offset into the device(s) address space from which
*		to begin block UnLocking. The first three bytes of every block
*		is reserved for special purpose. The offset should be at least
*		three bytes from start of the block.
* @param	Bytes indicates the number of bytes to UnLock in the Block
*		starting from Offset.
*
* @return
*		- XST_SUCCESS if successful.
*		- XST_FAILURE if failed.
*
* @note		This API should be called after applying VID (voltage) to the RP
*		pin of flash device. This API is not tested.
*
******************************************************************************/
int XFlashMicron_Unlock(XFlash *InstancePtr, u32 Offset, u32 Bytes)
{
	u8 Mode;
	u8 AddrMul;
	u8 TopBoot = 0;
	u8 MakeUnprotectStep = 0;
	u8 NoOfBlockInGroup;
	volatile u16 StatusReg;
	u16 Region = 0;
	u16 NumAttempt = 0;
	u16 Block = 0;
	u32 BaseAddress;
	u32 Index;
	u32 GroupAddress;
	int Status = (int)XST_FAILURE;
	XFlashGeometry *GeomPtr;
	XFlashVendorData_Micron *DevDataPtr;

	/*
	 * Verify inputs are valid.
	 */
	if (InstancePtr == NULL) {
		return XST_FAILURE;
	}

	DevDataPtr = GET_PARTDATA(InstancePtr);
	GeomPtr = &InstancePtr->Geometry;
	BaseAddress = GeomPtr->BaseAddress;

	if (InstancePtr->Geometry.BootMode == XFL_MICRON_TOP_BOOT) {
		TopBoot = 1;
	}

	Mode = (InstancePtr->Geometry.MemoryLayout &
		XFL_LAYOUT_PART_MODE_MASK) >> 8;

	if (Mode == 2) {
		AddrMul = 1;
	} else if (Mode == 1) {
		AddrMul = 2;
	} else {
		return (XST_FAILURE);
	}


	/*
	 * Protect all groups of block in the device.
	 */
	for (Block = 0; Block < GeomPtr->NumBlocks; Block++) {
		/*
		 * Get the physical address.
		 */
		(void) XFlashGeometry_ToAbsolute(GeomPtr, Region, Block, 0,
						 &GroupAddress);

		if (XFlashMicron_Lock(InstancePtr, GroupAddress, 0) !=
		    XST_SUCCESS ) {
			return XST_FAILURE;
		}

		if (TopBoot) {
			if ((Block == 0) || (Block >= 63)) {
				NoOfBlockInGroup = 1;
			} else if ((Block == 1) || (Block == 60)) {
				NoOfBlockInGroup = 3;
			} else {
				NoOfBlockInGroup = 4;
			}
		} else {
			if ((Block <= 7) || (Block == 70)) {
				NoOfBlockInGroup = 1;
			} else if ((Block == 8) || (Block == 67)) {
				NoOfBlockInGroup = 3;
			} else {
				NoOfBlockInGroup = 4;
			}
		}

		for (Index = 0; Index < NoOfBlockInGroup; Index++) {
			/*
			 * Increment Region/Block.
			 */
			XFL_GEOMETRY_INCREMENT(GeomPtr, Region, Block);
			Block++;
		}
	}

	/*
	 * Setup Phase.
	 */
	GroupAddress = (0x0000 | (AddrMul * XFL_MICRON_CHIP_UNPROTECT_ADDR));

	DevDataPtr->WriteCmd(InstancePtr, BaseAddress,
			     GroupAddress, XFL_MICRON_CMD_GROUP_PROTECT1);

	Block = 0;
	Region = 0;
	MakeUnprotectStep = 0;
	while (NumAttempt < 1000) {
		(void) XFlashGeometry_ToAbsolute(GeomPtr, Region, Block, 0,
						 &GroupAddress);

		GroupAddress = (GroupAddress |
				(AddrMul * XFL_MICRON_CHIP_UNPROTECT_ADDR));

		if (MakeUnprotectStep == 0) {
			/*
			 * Unprotect phase.
			 */
			DevDataPtr->WriteCmd(InstancePtr,
					     InstancePtr->Geometry.BaseAddress,
					     GroupAddress,
					     XFL_MICRON_CMD_GROUP_PROTECT1);

			FlashPause(10000);
		}

		/*
		 * Verify Phase.
		 */
		DevDataPtr->WriteCmd(InstancePtr,
				     InstancePtr->Geometry.BaseAddress,
				     GroupAddress,
				     XFL_MICRON_CMD_GROUP_PROTECT2);

		FlashPause(4);

		StatusReg = DevDataPtr->GetStatus(InstancePtr->Geometry.
						  BaseAddress, GroupAddress);

		if (StatusReg == XFL_MICRON_GROUP_UNPROTECTED) {
			if (Block == ((GeomPtr->NumBlocks) - 1)) {
				Status = XST_SUCCESS;
				break;
			} else {
				MakeUnprotectStep = 1;
				if (TopBoot) {
					if ((Block == 0) || (Block >= 63)) {
						NoOfBlockInGroup = 1;
					} else if ((Block == 1) || (Block == 60)) {
						NoOfBlockInGroup = 3;
					} else {
						NoOfBlockInGroup = 4;
					}
				} else {
					if ((Block <= 7) || (Block == 70)) {
						NoOfBlockInGroup = 1;
					} else if ((Block == 8) || (Block == 67)) {
						NoOfBlockInGroup = 3;
					} else {
						NoOfBlockInGroup = 4;
					}
				}

				for (Index = 0; Index < NoOfBlockInGroup;
				     Index++) {
					/*
					 * Increment Region/Block.
					 */
					XFL_GEOMETRY_INCREMENT(GeomPtr, Region,
							       Block);
					Block++;
				}
			}
		} else {
			MakeUnprotectStep = 0;
			NumAttempt++;
		}
	}

	/*
	 * Reset the bank(s) so that it returns to the read mode.
	 */
	(void) XFlashMicron_ResetBank(InstancePtr, Offset, Bytes);

	return (Status);
}

/*****************************************************************************/
/**
*
* Erases one or more Micron flash erase blocks. The caller provides an offset
* relative to the flash base address, not an absolute flash address.
*
* The requested range is converted into flash geometry region/block coordinates.
* Each block is then erased using the Micron block erase command sequence. The
* driver polls the flash until the erase operation completes.
*
* On the current board, erase requests should be aligned to the Micron erase
* block size of 0x00040000 bytes.
*
* Example:
*
*	Image starts at absolute address 0xFFFF6000
*	Flash base is                 0xFC000000
*	Image offset is               0x03FF6000
*	Erase must start at offset    0x03FC0000
*	Erase size is                 0x00040000
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	Offset is the flash offset relative to the flash base address.
* @param	Bytes is the number of bytes covering the erase range.
*
* @return
*		- XST_SUCCESS if the erase completed successfully.
*		- XST_FAILURE if an input parameter is invalid.
*		- XFLASH_ADDRESS_ERROR if the requested range is outside flash.
*		- XFLASH_ERROR or XFLASH_BUSY if erase polling fails.
*
* @note		This routine erases whole flash blocks. The caller should align
*		Offset and Bytes to erase-block boundaries before calling.
*
******************************************************************************/

int XFlashMicron_EraseChip(XFlash *InstancePtr)
{
	int Status = (int)XST_FAILURE;
	XFlashGeometry *GeomPtr;
	XFlashVendorData_Micron *DevDataPtr;

	/*
	 * Verify inputs are valid.
	 */
	if (InstancePtr == NULL) {
		return (XST_FAILURE);
	}

	DevDataPtr = GET_PARTDATA(InstancePtr);

	GeomPtr = &InstancePtr->Geometry;

	/*
	 * Send Chip Erase command.
	 */
	DevDataPtr->SendCmdSeq(InstancePtr, GeomPtr->BaseAddress,
			       XFL_MICRON_CMD1_ADDR, XFL_MICRON_CMD2_ADDR,
			       XFL_MICRON_CMD1_DATA, XFL_MICRON_CMD2_DATA);
	DevDataPtr->SendCmd(InstancePtr, GeomPtr->BaseAddress,
			    XFL_MICRON_CMD1_ADDR, XFL_MICRON_CMD_ERASE1);
	DevDataPtr->SendCmdSeq(InstancePtr, GeomPtr->BaseAddress,
			       XFL_MICRON_CMD1_ADDR, XFL_MICRON_CMD2_ADDR,
			       XFL_MICRON_CMD1_DATA, XFL_MICRON_CMD2_DATA);

	DevDataPtr->SendCmd(InstancePtr, GeomPtr->BaseAddress,
			    XFL_MICRON_CMD1_ADDR, XFL_MICRON_CMD_ERASE_CHIP);


	/*
	 * Wait until Program/Erase Controller starts.
	 */
	Status = DevDataPtr->PollSR(GeomPtr->BaseAddress, 0);
	(void) XFlashMicron_Reset(InstancePtr);
	if (Status != XFLASH_READY) {
		return (Status);
	}

	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
*
* Resets one or more Micron flash banks/regions back into read-array mode.
* This routine is used after reads, writes, erases, ID reads, CFI queries, and
* error cases to recover the flash from command/status modes.
*
* The routine first converts the provided offset into flash geometry coordinates.
* It then walks each affected erase region and writes the reset sequence using
* the selected bus-width command helper.
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	Offset is the flash offset relative to the flash base address.
* @param	Bytes is the number of bytes in the affected range.
*
* @return
*		- XST_SUCCESS if the reset completed successfully.
*		- XST_FAILURE if InstancePtr is invalid.
*		- XFLASH_ADDRESS_ERROR if the range is outside flash.
*
* @note		The reset sequence also includes unlock-bypass reset commands so
*		the flash can recover even if it was left in unlock-bypass mode.
*
******************************************************************************/

int XFlashMicron_Reset(XFlash *InstancePtr)
{
	int Status = (int)XST_FAILURE;

	/*
	 * Verify inputs are valid.
	 */
	if (InstancePtr == NULL) {
		return XST_FAILURE;
	}

	/*
	 * Send reset for all region/bank(s) in the device.
	 */
	Status = XFlashMicron_ResetBank(InstancePtr, 0,
				     InstancePtr->Geometry.DeviceSize);
	return Status;
}

/*****************************************************************************/
/**
*
* Resets the entire Micron flash device back into read-array mode. This allows
* the flash to be read as normal memory after command operations such as erase,
* program, CFI query, auto-select, or unlock-bypass mode.
*
* This routine calls XFlashMicron_ResetBank() over the whole flash device size.
*
* @param	InstancePtr is a pointer to the XFlash instance.
*
* @return
*		- XST_SUCCESS if the reset completed successfully.
*		- XST_FAILURE if InstancePtr is invalid.
*		- XFLASH_ADDRESS_ERROR if the geometry range is invalid.
*
* @note		Most higher-level operations call reset before or after command
*		sequences to force the flash into a known state.
*
******************************************************************************/

static int XFlashMicron_ResetBank(XFlash *InstancePtr, u32 Offset, u32 Bytes)
{
	XFlashVendorData_Micron *DevDataPtr;
	u16 Region;
	u16 Block;
	u32 Dummy;
	u32 Status;
	u32 RegionOffset;
	XFlashGeometry *GeomPtr;

	if (InstancePtr == NULL) {
		return XST_FAILURE;
	}

	if (Bytes == 0U) {
		return XST_SUCCESS;
	}

	DevDataPtr = GET_PARTDATA(InstancePtr);
	GeomPtr = &InstancePtr->Geometry;

	Status = XFlashGeometry_ToBlock(GeomPtr, Offset, &Region,
					&Block, &Dummy);
	if (Status != XST_SUCCESS) {
		return XFLASH_ADDRESS_ERROR;
	}

	while ((Region < GeomPtr->NumEraseRegions) &&
	       (GeomPtr->EraseRegion[Region].AbsoluteOffset <=
		(Offset + Bytes - 1U))) {

		RegionOffset = GeomPtr->EraseRegion[Region].AbsoluteOffset;

		/*
		 * If the device is in unlock-bypass mode, READ/RESET does not
		 * exit that mode. Send unlock-bypass reset first.
		 */
		DevDataPtr->WriteCmd(InstancePtr, GeomPtr->BaseAddress,
				     RegionOffset,
				     XFL_MICRON_CMD_UNLOCK_BYPASS_RESET1);

		DevDataPtr->WriteCmd(InstancePtr, GeomPtr->BaseAddress,
				     RegionOffset,
				     XFL_MICRON_CMD_UNLOCK_BYPASS_RESET2);

		/*
		 * Normal read-array reset sequence for MT28EW:
		 * AA / 55 / F0.
		 */
		DevDataPtr->SendCmdSeq(InstancePtr, GeomPtr->BaseAddress,
				       XFL_MICRON_CMD1_ADDR,
				       XFL_MICRON_CMD2_ADDR,
				       XFL_MICRON_CMD1_DATA,
				       XFL_MICRON_CMD2_DATA);

		DevDataPtr->WriteCmd(InstancePtr, GeomPtr->BaseAddress,
				     RegionOffset,
				     XFL_MICRON_CMD_RESET);

		Region++;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* Performs Micron-specific device control operations that are not simple read,
* write, erase, or reset operations.
*
* Supported operations include geometry query, property query, erase suspend,
* erase resume, extended block mode entry/exit, protection status query, and
* chip erase.
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	Command is the device-control command to execute.
* @param	Parameters points to the command-specific parameter structure.
*
* @return
*		- XST_SUCCESS if the command completed successfully.
*		- XST_FAILURE if InstancePtr is invalid or the operation fails.
*		- XFLASH_NOT_SUPPORTED if the command is not supported.
*
* @note		Chip erase is destructive. Normal image programming should use
*		sector erase instead.
*
******************************************************************************/

int XFlashMicron_DeviceControl(XFlash *InstancePtr, u32 Command,
			    DeviceCtrlParam *Parameters)
{
	u32 BlockAddr;
	u32 Offset;
	int Status = (int)XST_FAILURE;
	XFlashVendorData_Micron *DevDataPtr;

	/*
	 * Verify inputs are valid.
	 */
	if (InstancePtr == NULL) {
		return (XST_FAILURE);
	}

	DevDataPtr = GET_PARTDATA(InstancePtr);

	switch (Command) {
		case XFL_DEVCTL_GET_GEOMETRY:
			Parameters->GeometryParam.GeometryPtr =
				&InstancePtr->Geometry;
			Status = (XST_SUCCESS);
			break;

		case XFL_DEVCTL_GET_PROPERTIES:
			Parameters->PropertiesParam.PropertiesPtr =
				&InstancePtr->Properties;
			Status = (XST_SUCCESS);
			break;

		case XFL_DEVCTL_ERASE_RESUME:
			BlockAddr = (*((u32 *)Parameters));
			Status = EraseResume (InstancePtr, BlockAddr);
			break;

		case XFL_DEVCTL_ERASE_SUSPEND:
			BlockAddr = (*((u32 *)Parameters));
			Status = EraseSuspend (InstancePtr, BlockAddr);
			break;

		case XFL_DEVCTL_ENTER_EXT_MODE:
			EnterExtendedBlockMode(InstancePtr);
			Status = (XST_SUCCESS);
			break;

		case XFL_DEVCTL_EXIT_EXT_MODE:
			ExitExtendedBlockMode(InstancePtr);
			Status = (XST_SUCCESS);
			break;

		case XFL_DEVCTL_PROTECTION_STATUS:
			Offset = (*((u32 *)Parameters));
			Status = CheckBlockProtection(InstancePtr, Offset);
			break;

		case XFL_DEVCTL_CHIP_ERASE:
			Status = XFlashMicron_EraseChip(InstancePtr);
			break;

		default:
			Status = (XFLASH_NOT_SUPPORTED);
	}

	return (Status);
}

/*****************************************************************************/
/**
*
* Expands a Micron command value for the active flash bus layout.
*
* On a single x16 flash, a command such as 0x00AA is written as 0x00AA.
* On the current x32 board, two x16 flash devices are connected in parallel.
* Therefore the command must be duplicated into both halves of the 32-bit bus.
*
* Example x32 command expansion:
*
*	0x00AA -> 0x00AA00AA
*	0x0055 -> 0x00550055
*	0x00A0 -> 0x00A000A0
*	0x00F0 -> 0x00F000F0
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	Cmd is the Micron command value to expand.
*
* @return	The command value expanded for the active bus layout.
*
* @note		This helper is only for command/control writes. Normal program
*		payload data must not be expanded.
*
******************************************************************************/

static u32 XFlashMicron_ExpandCmdForBus(XFlash *InstancePtr, u32 Cmd)
{
	u32 Cmd16;
	u32 Layout;

	Cmd16 = Cmd & 0xFFFFU;
	Layout = InstancePtr->Geometry.MemoryLayout;

#if XFL_MICRON_USE_LEGACY_FULL_BYTE_COMMANDS
	if (Cmd16 == 0x00AAU) {
		Cmd16 = 0xAAAAU;
	} else if (Cmd16 == 0x0055U) {
		Cmd16 = 0x5555U;
	} else if (Cmd16 == 0x0025U) {
		Cmd16 = 0x2525U;
	} else if (Cmd16 == 0x0029U) {
		Cmd16 = 0x2929U;
	}
#endif

	if (Layout == XFL_LAYOUT_X16_X16_X2) {
		return (Cmd16 << 16) | Cmd16;
	}

	return Cmd16;
}

/*****************************************************************************/
/**
*
* Converts a Micron command-address offset to a CPU byte offset for the
* 32-bit parallel x16/x16 layout.
*
* In XFL_LAYOUT_X16_X16_X2, command offsets such as 0x555 and 0x2AA are
* bus-word offsets. The CPU performs 32-bit accesses, so the physical byte
* offset is command_offset * 4.
*
******************************************************************************/
static u32 XFlashMicron_Cmd32ByteOffset(u32 Offset)
{
	return Offset << 2;
}

/*****************************************************************************/
/**
*
* Writes one Micron command cycle using an 8-bit flash bus access.
*
* This routine expands the command as required, adjusts the command address for
* the x16/x8 layout, and writes the command to the memory-mapped flash bus.
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	BaseAddr is the flash base address.
* @param	Offset is the command address offset.
* @param	Cmd is the Micron command value to write.
*
* @return	None.
*
* @note		This helper is selected only for 8-bit layout operation.
*
******************************************************************************/

static void SendCmd8(XFlash *InstancePtr, u32 BaseAddr, u32 Offset, u32 Cmd)
{
	u32 ExpandedCmd = XFlashMicron_ExpandCmdForBus(InstancePtr, Cmd);

	Offset = ((2 * Offset) + (!(Offset & 0x1)));
	WRITE_FLASH_8(((volatile u8 *)BaseAddr) + Offset, ExpandedCmd);
}

/*****************************************************************************/
/**
*
* Writes one Micron command cycle using a 16-bit flash bus access.
*
* This routine expands the command as required and writes it to the selected
* command address using a 16-bit memory-mapped flash write.
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	BaseAddr is the flash base address.
* @param	Offset is the command address offset.
* @param	Cmd is the Micron command value to write.
*
* @return	None.
*
* @note		This helper is selected only for 16-bit layout operation.
*
******************************************************************************/

static void SendCmd16(XFlash *InstancePtr, u32 BaseAddr, u32 Offset, u32 Cmd)
{
	u32 ExpandedCmd = XFlashMicron_ExpandCmdForBus(InstancePtr, Cmd);

	WRITE_FLASH_16(((volatile u16 *)BaseAddr) + Offset, ExpandedCmd);
}

/*****************************************************************************/
/**
*
* Writes one Micron command cycle using a 32-bit flash bus access.
*
* On the current board, this routine writes a duplicated command value to both
* x16 Micron flash devices at the same time.
*
* Example:
*
*	Cmd 0x00AA becomes 0x00AA00AA on the x32 bus.
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	BaseAddr is the flash base address.
* @param	Offset is the command address offset.
* @param	Cmd is the Micron command value to write.
*
* @return	None.
*
* @note		This helper writes command/control values only. Program payload
*		data is written by WriteFlash32().
*
******************************************************************************/

static void SendCmd32(XFlash *InstancePtr, u32 BaseAddr, u32 Offset, u32 Cmd)
{
	u32 ExpandedCmd;
	u32 ByteOffset;

	ExpandedCmd = XFlashMicron_ExpandCmdForBus(InstancePtr, Cmd);
	ByteOffset = XFlashMicron_Cmd32ByteOffset(Offset);

	WRITE_FLASH_32((volatile u32 *)(BaseAddr + ByteOffset), ExpandedCmd);
}


/*****************************************************************************/
/**
*
* Writes two Micron command cycles back-to-back using an 8-bit flash bus access.
*
* This is mainly used for standard unlock sequences such as:
*
*	0x00AA to command address 0x0555
*	0x0055 to command address 0x02AA
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	BaseAddr is the flash base address.
* @param	Offset1 is the first command address offset.
* @param	Offset2 is the second command address offset.
* @param	Cmd1 is the first Micron command value.
* @param	Cmd2 is the second Micron command value.
*
* @return	None.
*
* @note		This helper is selected only for 8-bit layout operation.
*
******************************************************************************/

static void SendCmdSeq8(XFlash *InstancePtr, u32 BaseAddr, u32 Offset1,
			u32 Offset2, u32 Cmd1, u32 Cmd2)
{
	u32 ExpandedCmd1 = XFlashMicron_ExpandCmdForBus(InstancePtr, Cmd1);
	u32 ExpandedCmd2 = XFlashMicron_ExpandCmdForBus(InstancePtr, Cmd2);

	Offset1 = ((2 * Offset1) + (!(Offset1 & 0x1)));
	Offset2 = ((2 * Offset2) + (!(Offset2 & 0x1)));
	WRITE_FLASH_8(((volatile u8 *)BaseAddr) + Offset1, ExpandedCmd1);
	WRITE_FLASH_8(((volatile u8 *)BaseAddr) + Offset2, ExpandedCmd2);
}

/*****************************************************************************/
/**
*
* Writes two Micron command cycles back-to-back using a 16-bit flash bus access.
*
* This is mainly used for standard unlock sequences such as:
*
*	0x00AA to command address 0x0555
*	0x0055 to command address 0x02AA
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	BaseAddr is the flash base address.
* @param	Offset1 is the first command address offset.
* @param	Offset2 is the second command address offset.
* @param	Cmd1 is the first Micron command value.
* @param	Cmd2 is the second Micron command value.
*
* @return	None.
*
* @note		This helper is selected only for 16-bit layout operation.
*
******************************************************************************/

static void SendCmdSeq16(XFlash *InstancePtr, u32 BaseAddr, u32 Offset1,
			 u32 Offset2, u32 Cmd1, u32 Cmd2)
{
	u32 ExpandedCmd1 = XFlashMicron_ExpandCmdForBus(InstancePtr, Cmd1);
	u32 ExpandedCmd2 = XFlashMicron_ExpandCmdForBus(InstancePtr, Cmd2);

	WRITE_FLASH_16(((volatile u16 *)BaseAddr) + Offset1, ExpandedCmd1);
	WRITE_FLASH_16(((volatile u16 *)BaseAddr) + Offset2, ExpandedCmd2);
}

/*****************************************************************************/
/**
*
* Writes two Micron command cycles back-to-back using a 32-bit flash bus access.
*
* On the current board, this is used to issue the standard Micron unlock pair
* to both x16 flash devices in parallel:
*
*	0x00AA00AA to command address 0x0555
*	0x00550055 to command address 0x02AA
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	BaseAddr is the flash base address.
* @param	Offset1 is the first command address offset.
* @param	Offset2 is the second command address offset.
* @param	Cmd1 is the first Micron command value.
* @param	Cmd2 is the second Micron command value.
*
* @return	None.
*
* @note		This helper writes command/control values only.
*
******************************************************************************/

static void SendCmdSeq32(XFlash *InstancePtr, u32 BaseAddr, u32 Offset1,
			 u32 Offset2, u32 Cmd1, u32 Cmd2)
{
	u32 ExpandedCmd1;
	u32 ExpandedCmd2;
	u32 ByteOffset1;
	u32 ByteOffset2;

	ExpandedCmd1 = XFlashMicron_ExpandCmdForBus(InstancePtr, Cmd1);
	ExpandedCmd2 = XFlashMicron_ExpandCmdForBus(InstancePtr, Cmd2);

	ByteOffset1 = XFlashMicron_Cmd32ByteOffset(Offset1);
	ByteOffset2 = XFlashMicron_Cmd32ByteOffset(Offset2);

	WRITE_FLASH_32((volatile u32 *)(BaseAddr + ByteOffset1), ExpandedCmd1);
	WRITE_FLASH_32((volatile u32 *)(BaseAddr + ByteOffset2), ExpandedCmd2);
}

/*****************************************************************************/
/**
*
* Writes a command/control value to an arbitrary flash offset using an 8-bit bus
* access.
*
* This is used for command cycles that are not necessarily sent to the standard
* unlock address, such as erase confirm, unlock-bypass reset, or protection
* command operations.
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	BaseAddr is the flash base address.
* @param	Offset is the flash offset where the command is written.
* @param	Cmd is the Micron command/control value.
*
* @return	None.
*
* @note		This helper is selected only for 8-bit layout operation.
*
******************************************************************************/

static void WriteCmd8(XFlash *InstancePtr, u32 BaseAddr, u32 Offset, u32 Cmd)
{
	u32 ExpandedCmd = XFlashMicron_ExpandCmdForBus(InstancePtr, Cmd);

	WRITE_FLASH_8(((volatile u8 *)BaseAddr) + Offset, ExpandedCmd);
}

/*****************************************************************************/
/**
*
* Writes a command/control value to an arbitrary flash offset using a 16-bit bus
* access.
*
* This is used for command cycles that are not necessarily sent to the standard
* unlock address, such as erase confirm, unlock-bypass reset, or protection
* command operations.
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	BaseAddr is the flash base address.
* @param	Offset is the flash offset where the command is written.
* @param	Cmd is the Micron command/control value.
*
* @return	None.
*
* @note		This helper is selected only for 16-bit layout operation.
*
******************************************************************************/

static void WriteCmd16(XFlash *InstancePtr, u32 BaseAddr, u32 Offset, u32 Cmd)
{
	u32 ExpandedCmd = XFlashMicron_ExpandCmdForBus(InstancePtr, Cmd);

	WRITE_FLASH_16(((volatile u16 *)BaseAddr) + Offset, ExpandedCmd);
}

/*****************************************************************************/
/**
*
* Writes a command/control value to an arbitrary flash offset using a 32-bit bus
* access.
*
* On the current board, the command is duplicated across both x16 devices before
* being written to the memory-mapped flash address.
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	BaseAddr is the flash base address.
* @param	Offset is the byte offset where the command is written.
* @param	Cmd is the Micron command/control value.
*
* @return	None.
*
* @note		This helper is for command/control writes. Program payload data
*		must use WriteFlash32() so the actual data is not duplicated.
*
******************************************************************************/

static void WriteCmd32(XFlash *InstancePtr, u32 BaseAddr, u32 Offset, u32 Cmd)
{
	u32 ExpandedCmd;

	ExpandedCmd = XFlashMicron_ExpandCmdForBus(InstancePtr, Cmd);

	WRITE_FLASH_32((volatile u32 *)(BaseAddr + Offset), ExpandedCmd);
}


/*****************************************************************************/
/**
*
* Writes one byte of program payload data to flash using an 8-bit bus access.
*
* Unlike command writes, payload data is not expanded or modified. The value
* provided by the caller is written directly to the flash bus.
*
* @param	BaseAddr is the flash base address.
* @param	Offset is the flash offset where data is written.
* @param	Data is the payload data byte to program.
*
* @return	None.
*
* @note		This routine writes payload data, not command values.
*
******************************************************************************/

static void WriteFlash8(u32 BaseAddr, u32 Offset, u32 Data)
{
	WRITE_FLASH_8(((volatile u8 *)BaseAddr) + Offset, Data);
}

/*****************************************************************************/
/**
*
* Writes one 16-bit word of program payload data to flash using a 16-bit bus
* access.
*
* Unlike command writes, payload data is not expanded or modified. The value
* provided by the caller is written directly to the flash bus.
*
* @param	BaseAddr is the flash base address.
* @param	Offset is the flash offset where data is written.
* @param	Data is the payload data word to program.
*
* @return	None.
*
* @note		This routine writes payload data, not command values.
*
******************************************************************************/

static void WriteFlash16(u32 BaseAddr, u32 Offset, u32 Data)
{
	WRITE_FLASH_16(((volatile u16 *)BaseAddr) + Offset, Data);
}

/*****************************************************************************/
/**
*
* Writes one 32-bit word of program payload data to flash using a 32-bit bus
* access.
*
* On the current board, this writes the actual 32-bit program data to the two
* x16 Micron devices. The data is not command-expanded. For example, payload
* 0x9401FFFC is written as 0x9401FFFC.
*
* @param	BaseAddr is the flash base address.
* @param	Offset is the flash byte offset where data is written.
* @param	Data is the 32-bit payload data word to program.
*
* @return	None.
*
* @note		This routine writes payload data, not command values.
*
******************************************************************************/

static void WriteFlash32(u32 BaseAddr, u32 Offset, u32 Data)
{
	WRITE_FLASH_32((volatile u32 *)(BaseAddr + Offset), Data);
}

/*****************************************************************************/
/**
*
* Programs data to an 8-bit Micron flash layout using unlock-bypass program
* mode.
*
* The routine first enters unlock-bypass mode, then repeatedly issues the
* program command and writes one byte of data. After each byte write, the flash
* is polled until the operation completes.
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	DestPtr is the flash destination offset.
* @param	SrcPtr is the RAM source buffer.
* @param	Bytes is the number of bytes to program.
*
* @return
*		- XST_SUCCESS if programming completed successfully.
*		- XFLASH_ERROR or XFLASH_BUSY if flash polling fails.
*
* @note		The target flash range must already be erased.
*
******************************************************************************/

static int WriteBuffer8(XFlash *InstancePtr, void *DestPtr,
			void *SrcPtr, u32 Bytes)
{
	u8 *SourcePtr = (u8 *)SrcPtr;
	u8 *DestinationPtr = (u8 *)DestPtr;
	u32 BaseAddress;
	u32 Index = 0;
	int Status = (int)XST_FAILURE;
	XFlashVendorData_Micron *DevDataPtr;

	DevDataPtr = GET_PARTDATA(InstancePtr);
	BaseAddress = InstancePtr->Geometry.BaseAddress;

	/*
	 * Send the Unlock Bypass command.
	 */
	DevDataPtr->SendCmdSeq(InstancePtr, BaseAddress,
			       XFL_MICRON_CMD1_ADDR, XFL_MICRON_CMD2_ADDR,
			       XFL_MICRON_CMD1_DATA, XFL_MICRON_CMD2_DATA);

	DevDataPtr->SendCmd(InstancePtr, BaseAddress,
			    XFL_MICRON_CMD1_ADDR,
			    XFL_MICRON_CMD_UNLOCK_BYPASS);

	Index = 0;
	while (Bytes != 0) {
		DevDataPtr->WriteCmd(InstancePtr, BaseAddress,
				     0, XFL_MICRON_CMD_PROGRAM);

		DevDataPtr->WriteFlash(BaseAddress,
				       (u32)DestinationPtr, SourcePtr[Index]);

		Status = DevDataPtr->PollSR(BaseAddress,
					    (u32)DestinationPtr);
		if (Status != XFLASH_READY) {
			(void) XFlashMicron_ResetBank(InstancePtr, (u32)DestPtr,
						   Bytes);
			return (Status);
		}
		DestinationPtr++;
		Index++;
		Bytes -= 1;
	}

	/*
	 * Unlock Bypass Reset.
	 */
	DevDataPtr->WriteCmd(InstancePtr, BaseAddress, 0,
			     XFL_MICRON_CMD_UNLOCK_BYPASS_RESET1);
	DevDataPtr->WriteCmd(InstancePtr, BaseAddress, 0,
			     XFL_MICRON_CMD_UNLOCK_BYPASS_RESET2);

	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
*
* Programs data to a 16-bit flash layout. This routine validates 16-bit
* alignment, then selects the proper vendor programming path.
*
* For Micron devices, the unlock-bypass programming path is used. For Spansion
* devices, the Spansion/buffer programming path is used.
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	DestPtr is the flash destination offset.
* @param	SrcPtr is the RAM source buffer.
* @param	Bytes is the number of bytes to program.
*
* @return
*		- XST_SUCCESS if programming completed successfully.
*		- XFLASH_ALIGNMENT_ERROR if source or destination is unaligned.
*		- XFLASH_ERROR or XFLASH_BUSY if flash polling fails.
*
* @note		This routine is not the active path for the x32 MT28EW board.
*
******************************************************************************/

static int WriteBuffer16(XFlash *InstancePtr, void *DestPtr,
			 void *SrcPtr, u32 Bytes)
{
	u32 Status = (u32)XST_FAILURE;
	u16 *SrcWordPtr = (u16 *)SrcPtr;
	u16 *DestWordPtr = (u16 *)DestPtr;

	/*
	 * Make sure DestPtr and SrcPtr are aligned to a 16-bit word.
	 */
	if (((int) SrcWordPtr & 1) || ((int) DestWordPtr & 1)) {
		return (XFLASH_ALIGNMENT_ERROR);
	}

	if (InstancePtr->Properties.PartID.ManufacturerID == 0x01) {
		Status = WriteBufferSpansion(InstancePtr, DestPtr, SrcPtr, Bytes);
	} else {
		Status = WriteBufferMicronBypass(InstancePtr, DestPtr, SrcPtr,
						 Bytes);
	}

	return Status;
}


/*****************************************************************************/
/**
*
* This function is used to program Spansion devices. It does not erase
* the flash first and will fail if the block(s) are not erased first. The
* device(s) are programmed in parallel.
*
* @param	InstancePtr is the instance to work on.
* @param	DestPtr is the physical destination address in flash memory
*		space.
* @param	SrcPtr is the source data.
* @param	Bytes is the number of bytes to program.
*
* @return
*		- XST_SUCCESS if successful.
*		- XFLASH_ERROR if a write error occurred. This error is
*		  usually device specific.
*
* @note		None.
*
******************************************************************************/
static int WriteBufferSpansion(XFlash *InstancePtr, void *DestPtr,
			       void *SrcPtr, u32 Bytes)
{
	u32 DestinationPtr = (u32)DestPtr;
	u32 BaseAddress = InstancePtr->Geometry.BaseAddress;
	u16 *Tempsrcptr = (u16 *)SrcPtr;
	u8 *DestBytePtr = (u8 *)DestPtr;
	u32 Index = 0;
	int Status = (int)XST_FAILURE;
	XFlashVendorData_Micron *DevDataPtr = GET_PARTDATA(InstancePtr);
	u32 BufferSize = InstancePtr->Properties.ProgCap.WriteBufferSize;

	while (Bytes != 0) {
		/* Bytes to write should not exceed the buffer size. */
		if (Bytes > BufferSize) {
			Status = WriteSingleBufferMicron(InstancePtr, DestPtr,
							 Tempsrcptr,
							 BufferSize);
			Bytes = Bytes - BufferSize;
			DestBytePtr = DestBytePtr + BufferSize;
			DestPtr = (void *)DestBytePtr;
			Tempsrcptr = Tempsrcptr + BufferSize / 2;
		} else {
			Status = WriteSingleBufferMicron(InstancePtr, DestPtr,
							 Tempsrcptr,
							 Bytes);
			Bytes = 0;
		}
		if (Status != XST_SUCCESS) {
			return Status;
		}
	}

	Status = (int)XST_SUCCESS;

	return Status;
}


/*****************************************************************************/
/**
*
* This function is used to program Amd devices. It does not erase
* the flash first and will fail if the block(s) are not erased first. The
* device(s) are programmed in parallel.
*
* @param	InstancePtr is the instance to work on.
* @param	DestPtr is the physical destination address in flash memory
*		space.
* @param	SrcPtr is the source data.
* @param	Bytes is the number of bytes to program.
*
* @return
*		- XST_SUCCESS if successful.
*		- XFLASH_ERROR if a write error occurred. This error is
*		  usually device specific.
*
* @note		None.
*
******************************************************************************/
static int WriteBufferMicronBypass(XFlash *InstancePtr, void *DestPtr,
				   void *SrcPtr, u32 Bytes)
{
	u16 *SourcePtr = (u16 *)SrcPtr;
	u32 DestinationPtr = (u32)DestPtr;
	u32 BaseAddress = InstancePtr->Geometry.BaseAddress;
	u32 Index = 0;
	int Status = (int)XST_FAILURE;
	XFlashVendorData_Micron *DevDataPtr = GET_PARTDATA(InstancePtr);

	/* Send the Unlock Bypass command. */
	DevDataPtr->SendCmdSeq(InstancePtr, BaseAddress,
			       XFL_MICRON_CMD1_ADDR, XFL_MICRON_CMD2_ADDR,
			       XFL_MICRON_CMD1_DATA, XFL_MICRON_CMD2_DATA);

	DevDataPtr->SendCmd(InstancePtr, BaseAddress,
			    XFL_MICRON_CMD1_ADDR,
			    XFL_MICRON_CMD_UNLOCK_BYPASS);

	Index = 0;
	while (Bytes != 0) {
		DevDataPtr->WriteCmd(InstancePtr, BaseAddress,
				     0, XFL_MICRON_CMD_PROGRAM);

		DevDataPtr->WriteFlash(BaseAddress,
				       (u32)DestinationPtr, SourcePtr[Index]);

		Status = DevDataPtr->PollSR(BaseAddress,
					    (u32)DestinationPtr);
		if (Status != XFLASH_READY) {
			(void) XFlashMicron_ResetBank(InstancePtr, (u32)DestPtr,
						   Bytes);
			return (Status);
		}
		DestinationPtr++;
		Index++;
		Bytes -= 2;
	}

	/* Unlock Bypass Reset. */
	DevDataPtr->WriteCmd(InstancePtr, BaseAddress, 0,
			     XFL_MICRON_CMD_UNLOCK_BYPASS_RESET1);
	DevDataPtr->WriteCmd(InstancePtr, BaseAddress, 0,
			     XFL_MICRON_CMD_UNLOCK_BYPASS_RESET2);

	return (XST_SUCCESS);
}
/*****************************************************************************/
/**
*
* This function is used to program Buffer sized data. It does not erase
* the flash first and will fail if the block(s) are not erased first. The
* device(s) are programmed in parallel.
*
* @param	InstancePtr is the instance to work on.
* @param	DestPtr is the physical destination address in flash memory
*		space.
* @param	SrcPtr is the source data.
* @param	Bytes is the number of bytes to program.
*
* @return
*		- XST_SUCCESS if successful.
*		- XFLASH_ERROR if a write error occurred. This error is
*		  usually device specific.
*
* @note		None.
*
******************************************************************************/
static int WriteSingleBufferMicron(XFlash *InstancePtr, void *DestPtr,
				   void *SrcPtr, u32 Bytes)
{
	u16 *SourcePtr = (u16 *)SrcPtr;
	u32 DestinationPtr = (u32)DestPtr;
	u32 BaseAddress;
	u32 Index = 0;
	int Status = (int)XST_FAILURE;
	XFlashVendorData_Micron *DevDataPtr;
	u32 SectorAddress;
	u32 WordCount = Bytes / 2;
	u16 StartRegion;
	u16 EndRegion;
	u16 StartBlock;
	u16 EndBlock;
	u32 Dummy;
	u32 BlockAddress;
	volatile u16 FlashStatus;
	XFlashGeometry *GeomPtr;

	GeomPtr = &InstancePtr->Geometry;
	Status = XFlashGeometry_ToBlock(GeomPtr, (u32)DestPtr / 2, &StartRegion,
					&StartBlock, &Dummy);
	(void) XFlashGeometry_ToAbsolute(GeomPtr, StartRegion, StartBlock, 0,
					 &BlockAddress);
	SectorAddress = BlockAddress * 2;
	DevDataPtr = GET_PARTDATA(InstancePtr);
	BaseAddress = InstancePtr->Geometry.BaseAddress;
	DestinationPtr = DestinationPtr / 2;

	/* Send two Unlock cycles Commands. */
	DevDataPtr->SendCmdSeq(InstancePtr, BaseAddress,
			       XFL_MICRON_CMD1_ADDR, XFL_MICRON_CMD2_ADDR,
			       XFL_MICRON_CMD1_DATA, XFL_MICRON_CMD2_DATA);
	/* Send Write to Buffer Command. */
	DevDataPtr->SendCmd(InstancePtr, BaseAddress, (SectorAddress / 2),
			    XFL_MICRON_CMD_WRITE_BUFFER);
	/* Issue Number of words to Write at sector address. */
	DevDataPtr->SendCmd(InstancePtr, BaseAddress, (SectorAddress / 2),
			    (WordCount -  1));
	Index = 0;

	/* Write Data to Buffer. */
	while (WordCount != 0) {
		DevDataPtr->WriteFlash(BaseAddress,
				       (u32)DestinationPtr, SourcePtr[Index]);
		DestinationPtr++;
		Index++;
		WordCount -= 1;
	}

	/* Send Write buffer program confirm command. */
	DevDataPtr->WriteCmd(InstancePtr, BaseAddress, (SectorAddress / 2),
			     XFL_MICRON_CMD_PROGRAM_BUFFER);
	Status = DevDataPtr->PollSR(BaseAddress, (u32)(DestinationPtr * 2));
	if (Status != XFLASH_READY) {
		(void) XFlashMicron_ResetBank(InstancePtr, (u32)DestPtr,
					   Bytes);
		return Status;
	}
	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* Programs data to a 32-bit Micron flash bus one 32-bit word at a time.
*
* For each 32-bit word, this routine sends the standard Micron program command
* sequence:
*
*	0x00AA to command address 0x0555
*	0x0055 to command address 0x02AA
*	0x00A0 to command address 0x0555
*	DATA   to the destination flash offset
*
* After writing each word, the flash is polled at the destination address until
* the program operation completes.
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	DestPtr is the flash destination offset.
* @param	SrcPtr is the RAM source buffer.
* @param	Bytes is the number of bytes to program.
*
* @return
*		- XST_SUCCESS if programming completed successfully.
*		- XST_FAILURE if an input parameter is invalid.
*		- XFLASH_ALIGNMENT_ERROR if source, destination, or length is
*		  not 32-bit aligned.
*		- XFLASH_ERROR or XFLASH_BUSY if flash polling fails.
*
* @note		This is the main write path used by the current x32 MT28EW board.
*		The target flash range must already be erased.
*
******************************************************************************/

static int WriteBuffer32(XFlash *InstancePtr, void *DestPtr, void *SrcPtr, u32 Bytes)
{
	u32 *SourcePtr;
	u32 DestinationOffset;
	u32 BaseAddress;
	u32 Index;
	int Status;
	XFlashVendorData_Micron *DevDataPtr;

	if ((InstancePtr == NULL) || (DestPtr == NULL) || (SrcPtr == NULL)) {
		return XST_FAILURE;
	}

	SourcePtr = (u32 *)SrcPtr;
	DestinationOffset = (u32)DestPtr;
	BaseAddress = InstancePtr->Geometry.BaseAddress;
	Index = 0U;
	Status = XST_FAILURE;
	DevDataPtr = GET_PARTDATA(InstancePtr);

	if ((((u32)SourcePtr) & 3U) != 0U) {
		return XFLASH_ALIGNMENT_ERROR;
	}

	if ((DestinationOffset & 3U) != 0U) {
		return XFLASH_ALIGNMENT_ERROR;
	}

	if ((Bytes & 3U) != 0U) {
		return XFLASH_ALIGNMENT_ERROR;
	}

	while (Bytes != 0U) {
		/*
		 * Standard PROGRAM command sequence:
		 *   AA -> 555
		 *   55 -> 2AA
		 *   A0 -> 555
		 *   DATA -> destination
		 */
		DevDataPtr->SendCmdSeq(InstancePtr, BaseAddress,
				       XFL_MICRON_CMD1_ADDR,
				       XFL_MICRON_CMD2_ADDR,
				       XFL_MICRON_CMD1_DATA,
				       XFL_MICRON_CMD2_DATA);

		DevDataPtr->SendCmd(InstancePtr, BaseAddress,
				    XFL_MICRON_CMD1_ADDR,
				    XFL_MICRON_CMD_PROGRAM);

		DevDataPtr->WriteFlash(BaseAddress,
				       DestinationOffset,
				       SourcePtr[Index]);

		Status = DevDataPtr->PollSR(BaseAddress, DestinationOffset);
		if (Status != XFLASH_READY) {
			(void)XFlashMicron_ResetBank(InstancePtr,
						     (u32)DestPtr,
						     Bytes);
			return Status;
		}

		DestinationOffset += 4U;
		Index++;
		Bytes -= 4U;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* Reads status/data from flash using an 8-bit bus access.
*
* @param	BaseAddr is the flash base address.
* @param	Offset is the flash offset to read.
*
* @return	The 8-bit value read from flash.
*
* @note		This helper is selected only for 8-bit layout operation.
*
******************************************************************************/

static int GetStatus8(u32 BaseAddr, u32 Offset)
{
	return (READ_FLASH_8(((volatile u8 *)BaseAddr) + Offset));
}


/*****************************************************************************/
/**
*
* Reads status/data from flash using a 16-bit bus access.
*
* @param	BaseAddr is the flash base address.
* @param	Offset is the flash offset to read.
*
* @return	The 16-bit value read from flash.
*
* @note		This helper is selected only for 16-bit layout operation.
*
******************************************************************************/

static int GetStatus16(u32 BaseAddr, u32 Offset)
{
	return (READ_FLASH_16(((volatile u16 *)BaseAddr) + Offset));
}

/*****************************************************************************/
/**
*
* Reads status/data from flash using a 32-bit bus access.
*
* @param	BaseAddr is the flash base address.
* @param	Offset is the flash byte offset to read.
*
* @return	The 32-bit value read from flash.
*
* @note		This is the status/data read helper used by the current x32
*		MT28EW board.
*
******************************************************************************/

static int GetStatus32(u32 BaseAddr, u32 Offset)
{
	return READ_FLASH_32((volatile u32 *)(BaseAddr + Offset));
}

/*****************************************************************************/
/**
*
* Polls an 8-bit Micron flash operation until program or erase completion.
*
* This routine uses Micron/AMD-style toggle-bit polling. While the flash is
* busy, DQ6 toggles between reads. When DQ6 stops toggling, the operation has
* completed. If DQ5 is set while DQ6 continues toggling, the operation is
* treated as a failure.
*
* @param	BaseAddr is the flash base address.
* @param	BlockAddr is the flash address/offset being polled.
*
* @return
*		- XFLASH_READY if the operation completed.
*		- XFLASH_ERROR if flash reported a program/erase error.
*		- XFLASH_BUSY if software polling timed out.
*
* @note		This helper is selected only for 8-bit layout operation.
*
******************************************************************************/

static int PollSR8(u32 BaseAddr, u32 BlockAddr)
{
	u8 StatusReg1;
	u8 StatusReg2;
	u32 PollCount = XFL_MICRON_POLL_TIMEOUT;

	while (PollCount != 0U) {
		PollCount--;

		/*
		 * Read DQ5 and DQ6 (into word).
		 */
		StatusReg1 = READ_FLASH_8(((volatile u8 *) BaseAddr) +
					  BlockAddr);

		/*
		 * Read DQ6 (into another word).
		 */
		StatusReg2 = READ_FLASH_8(((volatile u8 *) BaseAddr) +
					  BlockAddr);

		/*
		 * If DQ6 did not toggle between two reads then return
		 * Flash_Success.
		 */
		if ((StatusReg1 & XFL_MICRON_SR_ERASE_COMPL_MASK) ==
		    (StatusReg2 & XFL_MICRON_SR_ERASE_COMPL_MASK)) {
			/*
			 * DQ6 == NO Toggle.
			 */
			return (XFLASH_READY);
		}

		/*
		 * If DQ5 is zero then operation is not yet complete.
		 */
		if ((StatusReg2 & XFL_MICRON_SR_ERASE_ERROR_MASK) !=
		    XFL_MICRON_SR_ERASE_ERROR_MASK) {
			continue;
		}

		/*
		 * Else (DQ5 == 1), read DQ6 twice.
		 */
		StatusReg1 = READ_FLASH_8(((volatile u8 *) BaseAddr) +
					  BlockAddr);
		StatusReg2 = READ_FLASH_8(((volatile u8 *) BaseAddr) +
					  BlockAddr);

		/*
		 * If DQ6 did not toggle between two reads then return
		 * Flash_Success.
		 */
		if ((StatusReg1 & XFL_MICRON_SR_ERASE_COMPL_MASK) ==
		    (StatusReg2 & XFL_MICRON_SR_ERASE_COMPL_MASK)) {
			/*
			 * DQ6 == NO Toggle.
			 */
			return (XFLASH_READY);
		} else {
			/*
			 * Else return Flash_ToggleFail; DQ6 == Toggle here
			 * means fail.
			 */
			return (XFLASH_ERROR);
		}
	}

	return (XFLASH_BUSY);
}

/*****************************************************************************/
/**
*
* Polls a 16-bit Micron flash operation until program or erase completion.
*
* This routine uses Micron/AMD-style toggle-bit polling. While the flash is
* busy, DQ6 toggles between reads. When DQ6 stops toggling, the operation has
* completed. If DQ5 is set while DQ6 continues toggling, the operation is
* treated as a failure.
*
* @param	BaseAddr is the flash base address.
* @param	BlockAddr is the flash address/offset being polled.
*
* @return
*		- XFLASH_READY if the operation completed.
*		- XFLASH_ERROR if flash reported a program/erase error.
*		- XFLASH_BUSY if software polling timed out.
*
* @note		This helper is selected only for 16-bit layout operation.
*
******************************************************************************/

static int PollSR16(u32 BaseAddr, u32 BlockAddr)
{
	volatile u16 StatusReg1;
	volatile u16 StatusReg2;
	u32 PollCount = XFL_MICRON_POLL_TIMEOUT;

	while (PollCount != 0U) {
		PollCount--;

		/* Read DQ5 and DQ6 (into word). */
		StatusReg1 = READ_FLASH_16(((volatile u16 *)BaseAddr) +
					   BlockAddr);

		/* Read DQ6 (into another word). */
		StatusReg2 = READ_FLASH_16(((volatile u16 *)BaseAddr) +
					   BlockAddr);

		/*
		 * If DQ6 did not toggle between two reads then return
		 * Flash_Success.
		 */
		if ((StatusReg1 & XFL_MICRON_SR_ERASE_COMPL_MASK) ==
		    (StatusReg2 & XFL_MICRON_SR_ERASE_COMPL_MASK)) {
			/* DQ6 == NO Toggle. */
			return (XFLASH_READY);
		}

		/* If DQ5 is zero then operation is not yet complete. */
		if ((StatusReg2 & XFL_MICRON_SR_ERASE_ERROR_MASK) !=
		    XFL_MICRON_SR_ERASE_ERROR_MASK) {
			continue;
		}

		/* Else (DQ5 == 1), read DQ6 twice. */
		StatusReg1 = READ_FLASH_16(((volatile u16 *)BaseAddr) +
					   BlockAddr);
		StatusReg2 = READ_FLASH_16(((volatile u16 *)BaseAddr) +
					   BlockAddr);

		/*
		 * If DQ6 did not toggle between two reads then return
		 * Flash_Success.
		 */
		if ((StatusReg1 & XFL_MICRON_SR_ERASE_COMPL_MASK) ==
		    (StatusReg2 & XFL_MICRON_SR_ERASE_COMPL_MASK)) {
			/* DQ6 == NO Toggle. */
			return (XFLASH_READY);
		} else {
			/*
			 * Else return Flash_ToggleFail; DQ6 == Toggle here
			 * means fail.
			 */
			return (XFLASH_ERROR);
		}
	}

	return (XFLASH_BUSY);
}

/*****************************************************************************/
/**
*
* Polls a 32-bit Micron flash operation until program or erase completion.
*
* This routine uses Micron/AMD-style toggle-bit polling. While the flash is
* busy, DQ6 toggles between reads. When DQ6 stops toggling, the operation has
* completed. If DQ5 is set while DQ6 continues toggling, the operation is
* treated as a failure.
*
* On the current board, this polling read observes both x16 Micron devices
* through the 32-bit flash bus.
*
* @param	BaseAddr is the flash base address.
* @param	BlockAddr is the flash byte offset being polled.
*
* @return
*		- XFLASH_READY if the operation completed.
*		- XFLASH_ERROR if flash reported a program/erase error.
*		- XFLASH_BUSY if software polling timed out.
*
* @note		This is the main polling routine used by erase and write on the
*		current x32 MT28EW board.
*
******************************************************************************/

static int PollSR32(u32 BaseAddr, u32 BlockAddr)
{
	u32 StatusReg1;
	u32 StatusReg2;
	u32 PollCount;

	/*
	 * x32 bus = two x16 MT28EW devices in parallel.
	 * DQ6 appears in both halfwords:
	 *   low  x16 DQ6 = 0x00000040
	 *   high x16 DQ6 = 0x00400000
	 */
	const u32 Dq6Mask32 = 0x00400040U;

	PollCount = XFL_MICRON_POLL_TIMEOUT;

	while (PollCount != 0U) {
		PollCount--;

		StatusReg1 = READ_FLASH_32((volatile u32 *)(BaseAddr + BlockAddr));
		StatusReg2 = READ_FLASH_32((volatile u32 *)(BaseAddr + BlockAddr));

		/*
		 * If DQ6 is no longer toggling on either x16 lane, the
		 * operation is complete.
		 */
		if (((StatusReg1 ^ StatusReg2) & Dq6Mask32) == 0U) {
			return XFLASH_READY;
		}
	}

	return XFLASH_BUSY;
}

/*****************************************************************************/
/**
*
* Reads a raw Micron electronic signature value from the flash using the active
* bus layout.
*
* The flash must already be in auto-select mode before this routine is called.
* For the x32 layout, the returned value contains the response from both x16
* devices in the upper and lower halves of the 32-bit word.
*
* Example x32 raw manufacturer value:
*
*	0x00890089
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	Offset is the electronic signature offset to read.
*
* @return	The raw bus-width ID value read from flash.
*
* @note		The caller is responsible for entering and exiting auto-select
*		mode.
*
******************************************************************************/

static u32 XFlashMicron_ReadRawId(XFlash *InstancePtr, u32 Offset)
{
	u32 BaseAddress = InstancePtr->Geometry.BaseAddress;
	u32 ReadOffset;

	switch (InstancePtr->Geometry.MemoryLayout) {
		case XFL_LAYOUT_X16_X8_X1:
			ReadOffset = ((2U * Offset) + (!(Offset & 0x1U)));
			return (u32)READ_FLASH_8(((volatile u8 *)BaseAddress) +
						 ReadOffset);

		case XFL_LAYOUT_X16_X16_X1:
			return (u32)READ_FLASH_16(((volatile u16 *)BaseAddress) +
						  Offset);

		case XFL_LAYOUT_X16_X16_X2:
			return READ_FLASH_32(((volatile u32 *)BaseAddress) +
					     Offset);

		default:
			return 0U;
	}
}

/*****************************************************************************/
/**
*
* Normalizes a raw Micron electronic signature value to an 8-bit ID byte.
*
* On the current x32 bus, both x16 devices return the same ID value. This helper
* extracts the low byte so the result can be compared against the expected
* Micron manufacturer/device ID values.
*
* Example:
*
*	Raw value:   0x00890089
*	Normalized: 0x89
*
* @param	RawId is the raw bus-width ID value read from flash.
*
* @return	The normalized 8-bit ID value.
*
* @note		This helper does not access flash directly.
*
******************************************************************************/

static u8 XFlashMicron_NormalizeIdByte(u32 RawId)
{
	return (u8)(RawId & 0xFFU);
}

/*****************************************************************************/
/**
*
* Reads the Micron MT28EW electronic signature from the flash.
*
* This routine resets the flash, enters auto-select mode, reads the raw
* manufacturer and device ID values, normalizes them to 8-bit fields, and then
* resets the flash back into read-array mode.
*
* The Micron auto-select command sequence is:
*
*	0x00AA to command address 0x0555
*	0x0055 to command address 0x02AA
*	0x0090 to command address 0x0555
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	IdPtr points to the structure that receives the normalized ID
*		values.
*
* @return
*		- XST_SUCCESS if the ID was read successfully.
*		- XST_FAILURE if an input parameter is invalid.
*
* @note		This routine is non-destructive. It does not erase or program
*		flash.
*
******************************************************************************/

int XFlashMicron_ReadMt28ewId(XFlash *InstancePtr, XFlashMicronId *IdPtr)
{
	u32 RawManufacturer;
	u32 RawDevice1;
	u32 RawDevice2;
	u32 RawDevice3;
	XFlashVendorData_Micron *DevDataPtr;

	if ((InstancePtr == NULL) || (IdPtr == NULL)) {
		return XST_FAILURE;
	}

	DevDataPtr = GET_PARTDATA(InstancePtr);

	(void) XFlashMicron_Reset(InstancePtr);

	DevDataPtr->SendCmdSeq(InstancePtr, InstancePtr->Geometry.BaseAddress,
			       XFL_MICRON_CMD1_ADDR, XFL_MICRON_CMD2_ADDR,
			       XFL_MICRON_CMD1_DATA, XFL_MICRON_CMD2_DATA);
	DevDataPtr->SendCmd(InstancePtr, InstancePtr->Geometry.BaseAddress,
			    XFL_MICRON_CMD1_ADDR,
			    XFL_MICRON_CMD_AUTO_SELECT);

	RawManufacturer = XFlashMicron_ReadRawId(InstancePtr,
				XFL_MICRON_MANUFACTURER_ID_OFFSET);
	RawDevice1 = XFlashMicron_ReadRawId(InstancePtr,
				XFL_MICRON_DEVICE_ID_1_OFFSET);
	RawDevice2 = XFlashMicron_ReadRawId(InstancePtr,
				XFL_MICRON_DEVICE_ID_2_OFFSET);
	RawDevice3 = XFlashMicron_ReadRawId(InstancePtr,
				XFL_MICRON_DEVICE_ID_3_OFFSET);

	IdPtr->ManufacturerID = XFlashMicron_NormalizeIdByte(RawManufacturer);
	IdPtr->DeviceID1 = XFlashMicron_NormalizeIdByte(RawDevice1);
	IdPtr->DeviceID2 = XFlashMicron_NormalizeIdByte(RawDevice2);
	IdPtr->DeviceID3 = XFlashMicron_NormalizeIdByte(RawDevice3);

	(void) XFlashMicron_Reset(InstancePtr);

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* Checks whether the connected Micron flash matches the expected MT28EW128
* electronic signature.
*
* This routine reads the normalized Micron ID fields and compares them against
* the expected manufacturer and device ID constants.
*
* @param	InstancePtr is a pointer to the XFlash instance.
*
* @return
*		- TRUE if the connected flash matches the expected MT28EW128 ID.
*		- FALSE if the ID read fails or does not match.
*
* @note		This routine is useful as a board sanity check before running
*		destructive erase or program tests.
*
******************************************************************************/

int XFlashMicron_IsMt28ew128(XFlash *InstancePtr)
{
	XFlashMicronId Id;

	if (XFlashMicron_ReadMt28ewId(InstancePtr, &Id) != XST_SUCCESS) {
		return FALSE;
	}

	if ((Id.ManufacturerID == XFL_MICRON_MT28EW128_MANUFACTURER_ID) &&
	    (Id.DeviceID1 == XFL_MICRON_MT28EW128_DEVICE_ID_1) &&
	    (Id.DeviceID2 == XFL_MICRON_MT28EW128_DEVICE_ID_2) &&
	    (Id.DeviceID3 == XFL_MICRON_MT28EW128_DEVICE_ID_3)) {
		return TRUE;
	}

	return FALSE;
}

/*****************************************************************************/
/**
*
* Reads the Micron part ID and stores it into the generic XFlash part
* properties.
*
* This routine is called during Micron driver initialization so higher-level
* code can see the detected manufacturer and device information.
*
* @param	InstancePtr is a pointer to the XFlash instance.
*
* @return	None.
*
* @note		This routine is non-destructive. It uses auto-select ID reads
*		and returns the flash to read-array mode.
*
******************************************************************************/

static void GetPartID(XFlash *InstancePtr)
{
	XFlashMicronId Id;

	if (XFlashMicron_ReadMt28ewId(InstancePtr, &Id) == XST_SUCCESS) {
		InstancePtr->Properties.PartID.ManufacturerID =
			Id.ManufacturerID;
		InstancePtr->Properties.PartID.DeviceID = Id.DeviceID1;
	}
}

/*****************************************************************************/
/**
*
* Issues the Micron block erase command sequence for one erase block.
*
* The current region/block coordinates are converted into an absolute flash
* offset. The Micron block erase command sequence is then written:
*
*	0x00AA to command address 0x0555
*	0x0055 to command address 0x02AA
*	0x0080 to command address 0x0555
*	0x00AA to command address 0x0555
*	0x0055 to command address 0x02AA
*	0x0030 to the target block address
*
* After the command is issued, the region/block input values are advanced to
* the next block.
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	Region points to the current erase region number.
* @param	Block points to the current erase block number.
* @param	MaxBlocks is the maximum number of blocks remaining to erase.
*
* @return	The number of erase blocks queued. This implementation queues
*		one block at a time.
*
* @note		The caller is responsible for polling the flash after this command
*		is issued.
*
******************************************************************************/
static u16 EnqueueEraseBlocks(XFlash *InstancePtr, u16 *Region,
			      u16 *Block, u16 MaxBlocks)
{
	u32 BlockAddress;
	XFlashGeometry *GeomPtr;
	XFlashVendorData_Micron *DevDataPtr;
	volatile u16 FlashStatus;

	/*
	 * If for some reason the maximum number of blocks to enqueue is
	 * zero, then don't do anything.
	 */
	if (MaxBlocks == 0) {
		return (0);
	}

	GeomPtr = &InstancePtr->Geometry;
	DevDataPtr = GET_PARTDATA(InstancePtr);

	/*
	 * Get the physical address to write the command to and send command.
	 */
	(void) XFlashGeometry_ToAbsolute(GeomPtr, *Region, *Block, 0,
					 &BlockAddress);

	/* Write Block Erase command. */
	DevDataPtr->SendCmdSeq(InstancePtr, GeomPtr->BaseAddress,
			       XFL_MICRON_CMD1_ADDR, XFL_MICRON_CMD2_ADDR,
			       XFL_MICRON_CMD1_DATA, XFL_MICRON_CMD2_DATA);
	DevDataPtr->SendCmd(InstancePtr, GeomPtr->BaseAddress,
			    XFL_MICRON_CMD1_ADDR, XFL_MICRON_CMD_ERASE1);
	DevDataPtr->SendCmdSeq(InstancePtr, GeomPtr->BaseAddress,
			       XFL_MICRON_CMD1_ADDR, XFL_MICRON_CMD2_ADDR,
			       XFL_MICRON_CMD1_DATA, XFL_MICRON_CMD2_DATA);

	DevDataPtr->WriteCmd(InstancePtr, GeomPtr->BaseAddress, BlockAddress,
			     XFL_MICRON_CMD_ERASE_BLOCK);

	/* Increment Region/Block. */
	XFL_GEOMETRY_INCREMENT(GeomPtr, *Region, *Block);

	/* Return the number of blocks enqueued. */
	return (1);
}

/*****************************************************************************/
/**
*
* Sends the Micron erase resume command to a flash bank that previously had an
* erase operation suspended.
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	EraseAddrOff is the flash offset inside the erase bank.
*
* @return
*		- XST_SUCCESS if the command was issued.
*		- XST_FAILURE if InstancePtr is invalid.
*
* @note		The normal erase API is blocking, so this helper is not used in
*		the standard programming flow.
*
******************************************************************************/
static int EraseResume(XFlash *InstancePtr, u32 EraseAddrOff)
{
	XFlashVendorData_Micron *DevDataPtr;

	/*
	 * Verify inputs are valid.
	 */
	if (InstancePtr == NULL) {
		return (XST_FAILURE);
	}

	DevDataPtr = GET_PARTDATA(InstancePtr);

	/*
	 * Send the erase resume command.
	 */
	DevDataPtr->WriteCmd(InstancePtr, InstancePtr->Geometry.BaseAddress,
			     EraseAddrOff, XFL_MICRON_CMD_ERASE_RESUME);
	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
*
* Sends the Micron erase suspend command to a flash bank and waits until flash
* toggling stops.
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	EraseAddrOff is the flash offset inside the erase bank.
*
* @return
*		- XST_SUCCESS if the command was issued.
*		- XST_FAILURE if InstancePtr is invalid.
*
* @note		The normal erase API is blocking, so this helper is not used in
*		the standard programming flow.
*
******************************************************************************/
static int EraseSuspend(XFlash *InstancePtr, u32 EraseAddrOff)
{
	XFlashVendorData_Micron *DevDataPtr;

	/*
	 * Verify inputs are valid.
	 */
	if (InstancePtr == NULL) {
		return (XST_FAILURE);
	}

	DevDataPtr = GET_PARTDATA(InstancePtr);

	/*
	 * Send the erase suspend command.
	 */
	DevDataPtr->WriteCmd(InstancePtr, InstancePtr->Geometry.BaseAddress,
			     EraseAddrOff, XFL_MICRON_CMD_ERASE_SUSPEND);

	/*
	 * Wait until Toggle Stops.
	 */
	(void) DevDataPtr->PollSR(InstancePtr->Geometry.BaseAddress,
				  EraseAddrOff);

	/*
	 * Return to Read mode.
	 */
	(void) XFlashMicron_Reset(InstancePtr);

	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
*
* Places the Micron flash into extended block mode. Extended block mode allows
* access to the device extended block area using the required Micron command
* sequence.
*
* @param	InstancePtr is a pointer to the XFlash instance.
*
* @return	None.
*
* @note		This routine is not part of the normal application image
*		programming path.
*
******************************************************************************/
static void EnterExtendedBlockMode(XFlash *InstancePtr)
{
	XFlashVendorData_Micron *DevDataPtr;

	DevDataPtr = GET_PARTDATA(InstancePtr);

	/*
	 * Issue Enter Extended Block Command.
	 */
	DevDataPtr->SendCmdSeq(InstancePtr, InstancePtr->Geometry.BaseAddress,
			       XFL_MICRON_CMD1_ADDR, XFL_MICRON_CMD2_ADDR,
			       XFL_MICRON_CMD1_DATA, XFL_MICRON_CMD2_DATA);

	DevDataPtr->SendCmd(InstancePtr, InstancePtr->Geometry.BaseAddress,
			    XFL_MICRON_CMD1_ADDR,
			    XFL_MICRON_CMD_ENTER_EXT_MODE);
}

/*****************************************************************************/
/**
*
* Exits Micron extended block mode and returns the flash toward normal read
* operation.
*
* @param	InstancePtr is a pointer to the XFlash instance.
*
* @return	None.
*
* @note		This routine is not part of the normal application image
*		programming path.
*
******************************************************************************/

static void ExitExtendedBlockMode(XFlash *InstancePtr)
{
	XFlashVendorData_Micron *DevDataPtr;

	DevDataPtr = GET_PARTDATA(InstancePtr);

	/*
	 * Issue Exit Extended Block Command.
	 */
	DevDataPtr->SendCmdSeq(InstancePtr, InstancePtr->Geometry.BaseAddress,
			       XFL_MICRON_CMD1_ADDR, XFL_MICRON_CMD2_ADDR,
			       XFL_MICRON_CMD1_DATA, XFL_MICRON_CMD2_DATA);

	DevDataPtr->SendCmd(InstancePtr, InstancePtr->Geometry.BaseAddress,
			    XFL_MICRON_CMD1_ADDR, XFL_MICRON_CMD_EXIT_EXT_MODE);

	DevDataPtr->WriteCmd(InstancePtr, InstancePtr->Geometry.BaseAddress,
			     0, 0);
}

/*****************************************************************************/
/**
*
* Reads and interprets the protection status for the flash block containing the
* provided offset.
*
* This routine enters auto-select mode, reads the protection status field for
* the selected block, interprets the result, and then resets the flash back to
* read-array mode.
*
* @param	InstancePtr is a pointer to the XFlash instance.
* @param	Offset is the flash offset to check.
*
* @return
*		- XST_SUCCESS if the block is unprotected.
*		- XFLASH_BLOCK_PROTECTED if the block is protected.
*		- XFLASH_ADDRESS_ERROR if the offset is outside flash.
*		- XST_FAILURE if the protection state is unclear.
*
* @note		This routine checks protection status only. It does not erase or
*		program flash.
*
******************************************************************************/

static int CheckBlockProtection(XFlash *InstancePtr, u32 Offset)
{
	u16 Region;
	u16 Block;
	u32 Dummy;
	u32 BlockAddress;
	u32 ProtStatusAddress;
	u32 ProtStatus;
	int Status;
	XFlashGeometry *GeomPtr;
	XFlashVendorData_Micron *DevDataPtr;

	if (InstancePtr == NULL) {
		return XST_FAILURE;
	}

	GeomPtr = &InstancePtr->Geometry;
	DevDataPtr = GET_PARTDATA(InstancePtr);

	Status = XFlashGeometry_ToBlock(GeomPtr, Offset, &Region, &Block,
					&Dummy);
	if (Status != XST_SUCCESS) {
		return XFLASH_ADDRESS_ERROR;
	}

	(void)XFlashGeometry_ToAbsolute(GeomPtr, Region, Block, 0,
					&BlockAddress);

	/*
	 * Enter Auto Select mode:
	 * AA -> 555
	 * 55 -> 2AA
	 * 90 -> 555
	 */
	DevDataPtr->SendCmdSeq(InstancePtr, GeomPtr->BaseAddress,
			       XFL_MICRON_CMD1_ADDR,
			       XFL_MICRON_CMD2_ADDR,
			       XFL_MICRON_CMD1_DATA,
			       XFL_MICRON_CMD2_DATA);

	DevDataPtr->SendCmd(InstancePtr, GeomPtr->BaseAddress,
			    XFL_MICRON_CMD1_ADDR,
			    XFL_MICRON_CMD_AUTO_SELECT);

	if (GeomPtr->MemoryLayout == XFL_LAYOUT_X16_X16_X2) {
		/*
		 * In x32 parallel mode, protection status word offset 0x02
		 * is reached by byte offset 0x02 * 4 = 0x08.
		 */
		ProtStatusAddress = BlockAddress +
				    (XFL_MICRON_PROT_STATUS_OFFSET << 2);

		ProtStatus = DevDataPtr->GetStatus(GeomPtr->BaseAddress,
						   ProtStatusAddress);

		if (ProtStatus == XFL_MICRON_GROUP_UNPROTECTED32) {
			Status = XST_SUCCESS;
		} else if (ProtStatus == XFL_MICRON_GROUP_PROTECTED32) {
			Status = XFLASH_BLOCK_PROTECTED;
		} else {
			Status = XST_FAILURE;
		}
	} else {
		/*
		 * Preserve original behavior for non-x32 layouts.
		 */
		ProtStatusAddress = BlockAddress + XFL_MICRON_PROT_STATUS_OFFSET;

		ProtStatus = DevDataPtr->GetStatus(GeomPtr->BaseAddress,
						   ProtStatusAddress);

		if (ProtStatus == XFL_MICRON_GROUP_UNPROTECTED) {
			Status = XST_SUCCESS;
		} else if (ProtStatus == XFL_MICRON_GROUP_PROTECTED) {
			Status = XFLASH_BLOCK_PROTECTED;
		} else {
			Status = XST_FAILURE;
		}
	}

	(void)XFlashMicron_Reset(InstancePtr);

	return Status;
}
/*****************************************************************************/
/**
*
* Returns after MicroSeconds have elapsed. This routine is used in several
* parts of the code to generate small delays required for flash command timing.
*
* The delay is implemented as a simple busy-wait loop using a board-specific
* count value.
*
* @param	MicroSeconds is the approximate delay length in microseconds.
*
* @return	None.
*
* @note		This is not a hardware-timer-based delay. It is only a simple
*		software delay loop.
*
******************************************************************************/
static void  FlashPause(u32 MicroSeconds)
{
	static u32 Counter;

	/*
	 * Compute the count size.
	 */
	Counter = MicroSeconds * XFL_MICRON_COUNT_FOR_A_MICROSECOND;

	/*
	 * Count to the required size.
	 */
	while (Counter > 0) {
		/*
		 * Count down.
		 */
		Counter--;
	}
}

#endif /* XPAR_XFL_DEVICE_FAMILY_MICRON */
