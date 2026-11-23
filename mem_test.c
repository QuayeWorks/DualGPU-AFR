
#include "include/typedefs.h"
#include "include/xilflash.h"
#include "include/utilities.h"
#include "include/calculate_crc32.h"

#define WORD                      4

#ifndef FLASH_BASE_ADDRESS
#define FLASH_BASE_ADDRESS              XPAR_FLASH_MEM0_BASEADDR
#endif

#define FLASH_MEM_WIDTH                 4U
#define FLASH_END_ADDRESS               0xFFFFFFFFU

#define MICRON_EXPECTED_COMMAND_SET     0x0002U
#define MICRON_EXPECTED_BOOT_MODE       0x04U

/*
#ifndef FLASH_BASE_ADDRESS
#define FLASH_BASE_ADDRESS              0xFC000000U
#endif*/

#define OFS_IMAGE_LENGTH            0x00380000U
#define OFS_FLASH_SLOT_LENGTH       0x00400000U
#define GNU_DIAG_LENGTH             0x00380000U
#define OFS_GNU_COMBINED_LENGTH     (OFS_IMAGE_LENGTH + GNU_DIAG_LENGTH)
#define GNU_RAM_OFFSET              OFS_IMAGE_LENGTH



#define RAM_START_ADDRESS           0x00100000
#define BOOT_RAM_START              0x00020000
#define BOOT_LENGTH                 0x0000A000
#define MFG_DIAG_LENGTH             0x00070000
#define RAM_LOAD_ADDRESS            0x00000000
#define BOOT_LOAD_ADDRESS           0xFFFF6000
#define DEFAULT_IMAGE_LENGTH        0x00380000  
#define PC_START_ADDRESS            0x0000A000
#define MFG_PC_START                0x00001000
#define PZT_RAM_START_ADDRESS       0x00800000
#define PZT_IMAGE_LENGTH            0x00002000
#define BOOTSTRAP_START_ADDRESS     0xFFFF6000
#define PROGRAM1_START_ADDRESS      0xFE000000
#define PROGRAM2_START_ADDRESS      (PROGRAM1_START_ADDRESS + DEFAULT_IMAGE_LENGTH)   //old 0xF400000
#define MFG_DIAG_START_ADDRESS      0xFE000000
#define PZT_TABLE_START_ADDR        0xFFA00000
#define DOWNLOAD_TABLE_ADDR         0xFFC00000

#define MEMORY_UPLOAD_LINK_ADR      0x1F000
// #define MEMORY_UPLOAD_LINK_ADR      0x8ffcf000

typedef enum
{
    UP_BOOTSTAP = 0x55,
    UP_OFS = 0x56,
    UP_GNU = 0x57,
    UP_PZT_TABLE = 0x58,
    UP_MFG_DIAG = 0x59,
    UP_SPCL_BLD = 0x5A,
    UP_BLIND_LOAD = 0x5B,
    UP_OFSGNU_COMBINED = 0x5C,
    UP_MCDIAGGNU_COMB = 0x5D,
    UP_REPROGRAM = 0x5E,
    UP_SERIAL_NUMBER = 0x5F
} uptype_signature;

typedef struct
{
    UInt32 Up_Start_Address;    // starting location in memory for read from
    UInt32 Up_Length;           // length of read
    uptype_signature Up_Type;   // application type
    UInt32 Up_LoadAddress;      // RAM load point on load
    UInt32 Up_PC_Address;       // PC address (entry point)
} upload_format_t;

typedef struct
{
    UInt32 Source;
    UInt32 Destination;
    UInt32 PC_Start;
    UInt32 Image_Length;
    UInt32 Image_Crc;
    UInt32 Image_Type;
} Download_Entry_Type;


typedef enum
{
    DOWNLOAD_TBL_FLASH,
    PROGRAM1_FLASH,
    PROGRAM2_FLASH,
    SPARE_WITH_CRC,
    MAX_IMAGES_IN_TABLE
} memory_utilization_type;

typedef struct
{
    Download_Entry_Type Images[MAX_IMAGES_IN_TABLE];
} Download_Table_Type;

#pragma ghs section text = ".bootstrap_exec"

// Global variables

int    Status;                    // Xilinx status (unused)

// Init global load/crc flag indicating that no failures have occurred.
Bool   loadFailure = False;

Download_Table_Type *Download_Table  = (Download_Table_Type *)DOWNLOAD_TABLE_ADDR;
Download_Table_Type Download_Table_Copy;

void flash_protect(UInt32 length, UInt32 offset);
void flash_unprotect(UInt32 length, UInt32 offset);
Bool program_image(UInt32 ram_start_address, UInt32 ram_length, UInt32 rom_start_address, UInt32 ignore);

static void mem_print_hex(char *label, UInt32 value);
static void mem_print_dec(char *label, UInt32 value);
static void mem_print_progress(char *label, UInt32 done, UInt32 total);
static void mem_print_range(char *label, UInt32 offset, UInt32 length);
static Bool mem_verify_erased_range(UInt32 offset, UInt32 length);
static void mem_print_flash_sample(UInt32 offset, UInt32 length);
static Bool mem_compare_ram_to_flash(UInt32 ram_start, UInt32 flash_offset, UInt32 length);
static void mem_debug_crc_window(char *label, UInt32 start, UInt32 length);
static void mem_find_word_in_ram(char *label, UInt32 ram_start,
                                 UInt32 length, UInt32 target_word);
								 
XFlash FlashInstance; /* XFlash Instance. */

typedef enum
{
	FLASH_UNKNOWN,
	FLASH_INTEL,
	FLASH_MICRON
} flash_type_t;

static UInt32 FlashEraseBlockSize;
static flash_type_t FlashType = FLASH_UNKNOWN;

int mymemcmp(void *s1, void *s2, int len)
{
    int result;
    int word_count;
    UInt32 *u32s1ptr;
    UInt32 *u32s2ptr;

    result = 0;
    word_count = len / 4;
    u32s1ptr = (UInt32 *)s1;
    u32s2ptr = (UInt32 *)s2;

    while ((word_count > 0) && (result == 0))
    {
        if (*u32s1ptr > *u32s2ptr) {
            result = 1;
        } else if (*u32s1ptr < *u32s2ptr) {
            result = -1;
        } else {
            --word_count;
            ++u32s1ptr;
            ++u32s2ptr;
        }
    }

    return result;
}

void flash_unprotect(UInt32 length, UInt32 offset)
{
	if(FlashType == FLASH_INTEL){

		Status = XFlash_Unlock(&FlashInstance, offset, length);
	}
    else
	{
		(void)length;
		(void)offset;
		Status = XST_SUCCESS;
	} 
}

void flash_protect(UInt32 length, UInt32 offset)
{
	if(FlashType == FLASH_INTEL){
		Status = XFlash_Lock(&FlashInstance, offset, length);
	}
    else
	{
		(void)length;
		(void)offset;
		Status = XST_SUCCESS;
	} 
}


static UInt32 flash_align_down(UInt32 value, UInt32 align)
{
    return value & ~(align - 1U);
}

static UInt32 flash_align_up(UInt32 value, UInt32 align)
{
    return (value + align - 1U) & ~(align - 1U);
}

static Bool flash_absolute_to_offset(UInt32 absolute_address, UInt32 *offset_out)
{
    if (offset_out == 0) {
        return False;
    }

    if (absolute_address < FLASH_BASE_ADDRESS) {
        return False;
    }

    *offset_out = absolute_address - FLASH_BASE_ADDRESS;
    return True;
}

static Bool flash_range_is_valid(UInt32 absolute_address, UInt32 length)
{
    UInt32 last_address;

    if (length == 0U) {
        return False;
    }

    last_address = absolute_address + length - 1U;

    if (absolute_address < FLASH_BASE_ADDRESS) {
        return False;
    }

    if (last_address < absolute_address) {
        return False;
    }

    if (last_address > FLASH_END_ADDRESS) {
        return False;
    }

    return True;
}

static void mem_print_hex(char *label, UInt32 value)
{
    char msg[20];

    print_chars(label);
    bin2hex(msg, value);
    print_line(msg);
}

static void mem_print_dec(char *label, UInt32 value)
{
    char msg[20];

    print_chars(label);
    bin2ascii(msg, value);
    print_line(msg);
}

static void mem_print_progress(char *label, UInt32 done, UInt32 total)
{
    char msg[20];

    print_chars(label);
    print_chars(" ");

    bin2hex(msg, done);
    print_chars(msg);

    print_chars(" / ");

    bin2hex(msg, total);
    print_line(msg);
}

static void mem_print_range(char *label, UInt32 offset, UInt32 length)
{
    UInt32 absolute_start;
    UInt32 absolute_end;

    absolute_start = FLASH_BASE_ADDRESS + offset;
    absolute_end = absolute_start + length - 1U;

    print_line("");
    print_line(label);
    mem_print_hex("Offset start = ", offset);
    mem_print_hex("Length       = ", length);
    mem_print_hex("Absolute start = ", absolute_start);
    mem_print_hex("Absolute end   = ", absolute_end);
}

static Bool mem_verify_erased_range(UInt32 offset, UInt32 length)
{
    UInt32 current_offset;
    UInt32 bytes_left;
    UInt32 this_read;
    UInt32 index;
    UInt32 words_read;
    UInt32 read_buffer[64];
    UInt32 completed;
    int local_status;

    current_offset = offset;
    bytes_left = length;
    completed = 0U;

    print_line("");
    print_line("Verify erased flash range begin");
    mem_print_hex("Verify offset = ", offset);
    mem_print_hex("Verify length = ", length);

    while (bytes_left != 0U)
    {
        this_read = sizeof(read_buffer);

        if (this_read > bytes_left)
        {
            this_read = bytes_left;
        }

        local_status = XFlash_Reset(&FlashInstance);
        if (local_status != XST_SUCCESS)
        {
            print_line("**** XFlash_Reset failed during erase verify ****");
            mem_print_hex("Status = ", (UInt32)local_status);
            mem_print_hex("Offset = ", current_offset);
            return False;
        }

        local_status = XFlash_Read(&FlashInstance,
                                   current_offset,
                                   this_read,
                                   (void *)read_buffer);
        if (local_status != XST_SUCCESS)
        {
            print_line("**** XFlash_Read failed during erase verify ****");
            mem_print_hex("Status = ", (UInt32)local_status);
            mem_print_hex("Offset = ", current_offset);
            return False;
        }

        words_read = this_read / sizeof(UInt32);

        for (index = 0U; index < words_read; ++index)
        {
            if (read_buffer[index] != 0xFFFFFFFFU)
            {
                print_line("**** Erase verify failed ****");
                mem_print_hex("Fail offset = ",
                              current_offset + (index * sizeof(UInt32)));
                mem_print_hex("Fail abs    = ",
                              FLASH_BASE_ADDRESS + current_offset +
                              (index * sizeof(UInt32)));
                mem_print_hex("Expected    = ", 0xFFFFFFFFU);
                mem_print_hex("Actual      = ", read_buffer[index]);
                return False;
            }
        }

        current_offset += this_read;
        bytes_left -= this_read;
        completed += this_read;

        if ((completed == length) || ((completed & 0x00003FFFU) == 0U))
        {
            mem_print_progress("Verify erased bytes", completed, length);
        }
    }

    print_line("Verify erased flash range complete");
    return True;
}

static void mem_print_flash_sample(UInt32 offset, UInt32 length)
{
    UInt32 first_word;
    UInt32 last_word;
    UInt32 last_word_offset;
    int local_status;

    if (length < sizeof(UInt32))
    {
        return;
    }

    first_word = 0U;
    last_word = 0U;
    last_word_offset = offset + length - sizeof(UInt32);

    local_status = XFlash_Reset(&FlashInstance);
    if (local_status != XST_SUCCESS)
    {
        print_line("**** XFlash_Reset failed before sample read ****");
        mem_print_hex("Status = ", (UInt32)local_status);
        return;
    }

    local_status = XFlash_Read(&FlashInstance, offset,
                               sizeof(UInt32), (void *)&first_word);
    if (local_status != XST_SUCCESS)
    {
        print_line("**** First word sample read failed ****");
        mem_print_hex("Status = ", (UInt32)local_status);
        return;
    }

    local_status = XFlash_Read(&FlashInstance, last_word_offset,
                               sizeof(UInt32), (void *)&last_word);
    if (local_status != XST_SUCCESS)
    {
        print_line("**** Last word sample read failed ****");
        mem_print_hex("Status = ", (UInt32)local_status);
        return;
    }

    print_line("");
    print_line("Flash sample");
    mem_print_hex("First word offset = ", offset);
    mem_print_hex("First word abs    = ", FLASH_BASE_ADDRESS + offset);
    mem_print_hex("First word value  = ", first_word);
    mem_print_hex("Last word offset  = ", last_word_offset);
    mem_print_hex("Last word abs     = ", FLASH_BASE_ADDRESS + last_word_offset);
    mem_print_hex("Last word value   = ", last_word);
}

static void mem_debug_crc_window(char *label, UInt32 start, UInt32 length)
{
	UInt32 crc;
	UInt32 stored_crc;

	print_line("");
	print_line(label);
	mem_print_hex("Start      = ", start);
	mem_print_hex("Length     = ", length);
	mem_print_hex("End excl   = ", start + length);
	mem_print_hex("CRC word @ = ", start + length - sizeof(UInt32));

	stored_crc = *(UInt32 *)(start + length - sizeof(UInt32));
	crc = calculate_crc32(start, start + length);

	mem_print_hex("Stored CRC = ", stored_crc);
	mem_print_hex("Calc CRC   = ", crc);
}

static void mem_find_word_in_ram(char *label, UInt32 ram_start,
                                 UInt32 length, UInt32 target_word)
{
    UInt32 offset;
    UInt32 word;

    print_line("");
    print_line(label);
    mem_print_hex("RAM start  = ", ram_start);
    mem_print_hex("Length     = ", length);
    mem_print_hex("Target     = ", target_word);

    for (offset = 0U; offset < length; offset += sizeof(UInt32))
    {
        word = *(UInt32 *)(ram_start + offset);

        if (word == target_word)
        {
            print_line("Found target word");
            mem_print_hex("RAM addr   = ", ram_start + offset);
            mem_print_hex("Offset     = ", offset);
            return;
        }
    }

    print_line("Target word not found");
}

static Bool mem_compare_ram_to_flash(UInt32 ram_start, UInt32 flash_offset, UInt32 length)
{
    UInt32 index;
    UInt32 ram_word;
    UInt32 flash_word;
    UInt32 flash_abs;
	
	int local_status;

	local_status = XFlash_Reset(&FlashInstance);
	if (local_status != XST_SUCCESS)
	{
		print_line("**** XFlash_Reset failed before RAM/FLASH compare ****");
		mem_print_hex("Status = ", (UInt32)local_status);
		return False;
	}

    for (index = 0U; index < length; index += sizeof(UInt32))
    {
        ram_word = *(UInt32 *)(ram_start + index);
        flash_abs = FLASH_BASE_ADDRESS + flash_offset + index;
        flash_word = *(UInt32 *)flash_abs;

        if (ram_word != flash_word)
        {
            print_line("**** RAM/FLASH mismatch ****");
            mem_print_hex("RAM addr    = ", ram_start + index);
            mem_print_hex("Flash off   = ", flash_offset + index);
            mem_print_hex("Flash abs   = ", flash_abs);
            mem_print_hex("RAM word    = ", ram_word);
            mem_print_hex("Flash word  = ", flash_word);
            return False;
        }

        if ((index != 0U) && ((index & 0x00003FFFU) == 0U))
        {
            mem_print_progress("RAM/FLASH compare bytes", index, length);
        }
    }

    mem_print_progress("RAM/FLASH compare bytes", length, length);
    print_line("RAM/FLASH compare passed");
    return True;
}

Bool program_image(UInt32 ram_start_address, UInt32 ram_length, UInt32 rom_start_address, UInt32 ignore)
{
    UInt32 start_address=0;             // Flash starting memory address
    UInt32 ram_start;                 // RAM starting memory address
    UInt32 rom_offset;                    // Flash offset from base address
    UInt32 length;                    // Image size in bytes
    UInt32 RetryCounter;
	UInt32 erase_sector_count;
    char msg[20];

    // Initialize local flag indicating RAM image verification.  Set to False
    //   if either of the two images fail their CRC check.
    Bool passed_RAM_CRC = True;

    // Initialize local flag indicating that no failures have occurred.
    Bool failedLoadCRC = False;

    unsigned int ram_crc;
    unsigned int rom_crc;
    unsigned int passed_crc_in_rom;
    unsigned int passed_crc_in_ram;
    unsigned int remaining_length;
    unsigned int original_rom_start;
    unsigned int original_ram_start;
    unsigned int original_ram_length;
    unsigned int original_rom_offset;
	
	/* Added for 256 KiB Micron erase-block alignment */
	UInt32 image_start_offset;
	UInt32 image_end_offset;
	UInt32 erase_start;
	UInt32 erase_end;
	UInt32 erase_offset;

	start_address = rom_start_address;
	ram_start = ram_start_address;

	if (flash_range_is_valid(rom_start_address, ram_length) == False)
	{
		print_line("**** Flash destination range is invalid ****");
		return True;
	}

    if ( start_address )
    {
        // The offset is the absolute address minus the base address
        // of the FLASH.
        if (flash_absolute_to_offset(rom_start_address, &rom_offset) == False) {
		print_line("**** Invalid flash destination address ****");
		return True;
		}

        original_ram_length = ram_length;
        original_ram_start = ram_start;
        original_rom_start = start_address;
        original_rom_offset = rom_offset;

		if ((original_ram_length & (FLASH_MEM_WIDTH - 1U)) != 0U)
		{
			print_line("**** Image length is not flash-bus aligned ****");
			return True;
		}

        // Verify the images moved from the file on the PC to RAM on the MC CCA

        print_line("Verifying RAM image(s)   ");

		if (failedLoadCRC == False)
		{
			// Check for a single image or multiple images (i.e. OFS + GNU Diag) being loaded
			// Check for OFS + GNU combined first because OFS is 0x003C0000,
			// but GNU starts at the 0x00400000 flash slot boundary.
			if ((original_rom_start == PROGRAM1_START_ADDRESS) &&
				(original_ram_length == OFS_GNU_COMBINED_LENGTH))
			{
				ram_crc = calculate_crc32(original_ram_start,
										  original_ram_start + OFS_IMAGE_LENGTH);

				if (ram_crc == 0)
				{
					print_line(" - Verified OFS image CRC in RAM");
				}
				else
				{
					print_line("**** RAM-CRC verification failure (OFS image) ****");
					passed_RAM_CRC = False;
					failedLoadCRC = True;
				}

				ram_crc = calculate_crc32(original_ram_start + OFS_FLASH_SLOT_LENGTH,
										  original_ram_start + OFS_FLASH_SLOT_LENGTH +
										  GNU_DIAG_LENGTH);

				if (ram_crc == 0)
				{
					print_line(" - Verified GNU Diagnostics image CRC in RAM");
				}
				else
				{
					print_line("**** RAM-CRC verification failure (GNU Diagnostics image) ****");
					passed_RAM_CRC = False;
					failedLoadCRC = True;
				}
			}
			else if (original_ram_length > DEFAULT_IMAGE_LENGTH)
			{
				// For multiple images

				// Calculate the CRC over the first image in RAM
				ram_crc = calculate_crc32(original_ram_start, original_ram_start + DEFAULT_IMAGE_LENGTH - ignore);

				if (ram_crc == 0)
				{
					print_line(" - Verified 1st image CRC in RAM");
				}
				else
				{
					print_line("**** RAM-CRC verification failure (1st image) ****");
					passed_RAM_CRC = False;
					failedLoadCRC = True;
				}

				// Calculate the CRC over the second image in RAM
				ram_crc = calculate_crc32(original_ram_start + DEFAULT_IMAGE_LENGTH, original_ram_start + original_ram_length - ignore);

				if (ram_crc == 0)
				{
					print_line(" - Verified 2nd image CRC in RAM");
				}
				else
				{
					print_line("**** RAM-CRC verification failure (2nd image) ****");
					passed_RAM_CRC = False;
					failedLoadCRC = True;
				}
			}
			else
			{
				// For a single image

				//calculate the CRC over the image in RAM
				ram_crc = calculate_crc32(original_ram_start, original_ram_start + original_ram_length - ignore);

				// No need to compute ROM CRC if RAM CRC is not the expected value
				if (ram_crc == 0)
				{
					print_line(" - Verified image CRC in RAM");
				}
				else
				{
					print_line("**** RAM-CRC verification failure (single image) ****");
					passed_RAM_CRC = False;
					failedLoadCRC = True;
				}
			}
		}


        // RAM image verified.  Now erase flash and write image from RAM to flash
        if (passed_RAM_CRC)
        {
            print_line("Erasing Flash");

			image_start_offset = original_rom_offset;
			image_end_offset = original_rom_offset + original_ram_length;

			erase_start = flash_align_down(image_start_offset, FlashEraseBlockSize);
			erase_end = flash_align_up(image_end_offset, FlashEraseBlockSize);
			erase_offset = erase_start;

			erase_sector_count = ((erase_end - erase_start) /
									  FlashEraseBlockSize);

			mem_print_hex("RAM start        = ", original_ram_start);
			mem_print_hex("RAM length       = ", original_ram_length);
			mem_print_hex("Flash abs start  = ", original_rom_start);
			mem_print_hex("Flash offset     = ", original_rom_offset);
			mem_print_hex("Erase start off  = ", erase_start);
			mem_print_hex("Erase end off    = ", erase_end);
			mem_print_hex("Erase start abs  = ", FLASH_BASE_ADDRESS + erase_start);
			mem_print_hex("Erase end abs    = ", FLASH_BASE_ADDRESS + erase_end - 1U);
			mem_print_dec("Erase sectors    = ", erase_sector_count);

			while (erase_offset < erase_end)
			{
				UInt32 sector_number;
				UInt32 sector_total;

				sector_total = (erase_end - erase_start) / FlashEraseBlockSize;
				sector_number = ((erase_offset - erase_start) / FlashEraseBlockSize) + 1U;

				print_line("");
				print_chars("Erasing sector ");
				bin2ascii(msg, sector_number);
				print_chars(msg);
				print_chars(" / ");
				bin2ascii(msg, sector_total);
				print_line(msg);

				mem_print_hex("Erase offset = ", erase_offset);
				mem_print_hex("Erase abs    = ", FLASH_BASE_ADDRESS + erase_offset);

				RetryCounter = 3U;

				do
				{
					Status = XFlash_Reset(&FlashInstance);
					if (Status != XST_SUCCESS)
					{
						print_line("Flash Reset 1 failed ");
						break;
					}

					flash_unprotect(FlashEraseBlockSize, erase_offset);
					if (Status != XST_SUCCESS)
					{
						print_line("Flash Unprotect failed ");
						break;
					}

					Status = XFlash_Erase(&FlashInstance,
										  erase_offset,
										  FlashEraseBlockSize);

					if (Status != XST_SUCCESS)
					{
                        print_chars("Flash Erase failed ");
						break;
					}
                    
                    print_line("Call flash_protect");
					
					flash_protect(FlashEraseBlockSize, erase_offset);

					if (XFlash_Reset(&FlashInstance) != XST_SUCCESS)
					{
						Status = XST_FAILURE;
						print_line("Flash Reset 2 failed ");
						break;
					}
					--RetryCounter;

				} while ((RetryCounter != 0U) && (Status != XST_SUCCESS));

				if (Status != XST_SUCCESS)
				{
					print_line("**** Sector erase failed ****");
					mem_print_hex("Status = ", (UInt32)Status);
					mem_print_hex("Offset = ", erase_offset);
					mem_print_hex("Abs    = ", FLASH_BASE_ADDRESS + erase_offset);
					break;
				}

				print_line("Sector erase complete");

				erase_offset += FlashEraseBlockSize;
			}

			if (Status == XST_SUCCESS)
			{
				if (mem_verify_erased_range(erase_start, erase_end - erase_start) == False)
				{
					Status = XST_FAILURE;
				}
			}


            if ( Status != XST_SUCCESS )
            {
                print_line("Erase Failure");

                // Set return boolean to indicate failure status
                failedLoadCRC = True;

            }
            else
            {
                print_line("Complete");
                print_line("Writing: ");

                erase_sector_count = original_ram_length / 0x2000;

                remaining_length = original_ram_length;
				ram_start = original_ram_start;
				start_address = original_rom_start;
				rom_offset = original_rom_offset;

				while (remaining_length > 0U)
				{
					UInt32 bytes_written;

					bytes_written = original_ram_length - remaining_length;

					if ((bytes_written == 0U) ||
						((bytes_written & 0x0000FFFFU) == 0U))
					{
						mem_print_progress("Writing bytes", bytes_written, original_ram_length);
						mem_print_hex("Write offset = ", rom_offset);
						mem_print_hex("Write abs    = ", FLASH_BASE_ADDRESS + rom_offset);
						mem_print_hex("RAM source   = ", ram_start);
					}

					length = 0x2000U;
					if (length > remaining_length)
					{
						length = remaining_length;
					}
					Status = XFlash_Reset(&FlashInstance);
					if (Status != XST_SUCCESS)
					{
						print_line("**** Flash reset failure before write ****");
						mem_print_hex("Status = ", (UInt32)Status);
						failedLoadCRC = True;
						break;
					}

					flash_unprotect(length, rom_offset);

					mem_print_hex("Unlock Status = ", Status);

					if (Status != XST_SUCCESS)
					{
						print_line("**** Flash unprotect failure before write ****");
						mem_print_hex("Status = ", (UInt32)Status);
						failedLoadCRC = True;
						break;
					}

					mem_print_hex("Write Offset = ", rom_offset);
					mem_print_hex("Write Length = ", length);
					mem_print_hex("Ram Start = ", ram_start);
					Status = XFlash_Write(&FlashInstance,
										  rom_offset,
										  length,
										  (void *)ram_start);

					mem_print_hex("Write Status = ", Status);

					if (Status != XST_SUCCESS)
					{
						print_line("**** Flash write failure ****");
						mem_print_hex("Status = ", (UInt32)Status);
						failedLoadCRC = True;
					}

					flash_protect(length, rom_offset);

					mem_print_hex("Protect Status = ", Status);

					if (Status != XST_SUCCESS)
					{
						print_line("**** Flash protect failure after write ****");
						mem_print_hex("Status = ", (UInt32)Status);
						failedLoadCRC = True;
						break;
					}

					Status = XFlash_Reset(&FlashInstance);

					if (Status != XST_SUCCESS)
					{
						print_line("**** Flash reset failure after write ****");
						mem_print_hex("Status = ", (UInt32)Status);
						failedLoadCRC = True;
						break;
					}

					rom_offset += length;
					ram_start += length;
					remaining_length -= length;
				}

				if (failedLoadCRC == False)
				{
					mem_print_progress("Writing bytes", original_ram_length, original_ram_length);
					print_line("Complete");
					mem_print_flash_sample(original_rom_offset, original_ram_length);
				}
				else
				{
					print_line("Writing Failed");
				}

				if (failedLoadCRC == False)
				{
					print_line("");
					print_line("Comparing RAM image to FLASH image");

					if (mem_compare_ram_to_flash(original_ram_start,
                             original_rom_offset,
                             original_ram_length) == False)
					{
						failedLoadCRC = True;
					}
				}
				
				if (failedLoadCRC == False) 
				{
					// Check for a single image or multiple images (i.e. OFS + GNU Diag) being loaded
					if ((original_rom_start == PROGRAM1_START_ADDRESS) &&
						(original_ram_length == OFS_GNU_COMBINED_LENGTH))
					{
						ram_crc = calculate_crc32(original_ram_start,
												  original_ram_start + OFS_IMAGE_LENGTH);

						passed_crc_in_ram =
							*(unsigned int *)(original_ram_start + OFS_IMAGE_LENGTH -
											  sizeof(UInt32));

						passed_crc_in_rom =
							*(unsigned int *)(original_rom_start + OFS_IMAGE_LENGTH -
											  sizeof(UInt32));

						if ((ram_crc == 0) && (passed_crc_in_ram == passed_crc_in_rom))
						{
							print_line("Verified OFS RAM/ROM CRC word");
							print_chars("Verifying OFS FLASH image: ");

							rom_crc = calculate_crc32(original_rom_start,
													  original_rom_start + OFS_IMAGE_LENGTH);

							if (rom_crc)
							{
								print_line("**** OFS Flash image CRC Failure ****");
								failedLoadCRC = True;
							}
							else
							{
								print_line("Verified");
								print_chars("OFS CRC:               ");
								bin2hex(msg, passed_crc_in_rom);
								print_line(msg);
							}
						}
						else
						{
							if (ram_crc)
							{
								print_line("**** OFS RAM-CRC verification failure ****");
							}

							if (passed_crc_in_ram != passed_crc_in_rom)
							{
								print_line("**** OFS RAM-ROM image CRC mismatch ****");
							}

							failedLoadCRC = True;
						}

						if (failedLoadCRC == False)
						{
							ram_crc = calculate_crc32(original_ram_start + OFS_FLASH_SLOT_LENGTH,
													  original_ram_start + OFS_FLASH_SLOT_LENGTH +
													  GNU_DIAG_LENGTH);

							passed_crc_in_ram =
								*(unsigned int *)(original_ram_start + OFS_FLASH_SLOT_LENGTH +
												  GNU_DIAG_LENGTH - sizeof(UInt32));

							passed_crc_in_rom =
								*(unsigned int *)(original_rom_start + OFS_FLASH_SLOT_LENGTH +
												  GNU_DIAG_LENGTH - sizeof(UInt32));

							if ((ram_crc == 0) && (passed_crc_in_ram == passed_crc_in_rom))
							{
								print_line("Verified GNU Diagnostics RAM/ROM CRC word");
								print_chars("Verifying GNU Diagnostics FLASH image: ");

								rom_crc = calculate_crc32(original_rom_start + OFS_FLASH_SLOT_LENGTH,
														  original_rom_start + OFS_FLASH_SLOT_LENGTH +
														  GNU_DIAG_LENGTH);

								if (rom_crc)
								{
									print_line("**** GNU Diagnostics Flash image CRC Failure ****");
									failedLoadCRC = True;
								}
								else
								{
									print_line("Verified");
									print_chars("GNU Diagnostics CRC:   ");
									bin2hex(msg, passed_crc_in_rom);
									print_line(msg);
								}
							}
							else
							{
								if (ram_crc)
								{
									print_line("**** GNU Diagnostics RAM-CRC verification failure ****");
								}

								if (passed_crc_in_ram != passed_crc_in_rom)
								{
									print_line("**** GNU Diagnostics RAM-ROM image CRC mismatch ****");
								}

								failedLoadCRC = True;
							}
						}
					}
					else if (original_ram_length > DEFAULT_IMAGE_LENGTH)
					{
						// For multiple images

						// Calculate the CRC over the first image in RAM
						ram_crc = calculate_crc32(original_ram_start, original_ram_start + DEFAULT_IMAGE_LENGTH - ignore);

						// CRC of data block is located at last word of image in RAM & ROM
						passed_crc_in_ram = *(unsigned int *)(original_ram_start + DEFAULT_IMAGE_LENGTH - ignore - sizeof(UInt32));
						passed_crc_in_rom = *(unsigned int *)(original_rom_start + DEFAULT_IMAGE_LENGTH - ignore - sizeof(UInt32));

						// No need to compute ROM CRC if RAM CRC is not the expected value
						if ( (ram_crc == 0) && (passed_crc_in_ram == passed_crc_in_rom) )
						{
							print_line("Verified");
							//calculate the CRC over the image in ROM
							print_chars("Verifying FLASH image: ");

							rom_crc = calculate_crc32(original_rom_start, original_rom_start + DEFAULT_IMAGE_LENGTH - ignore);
							if ( rom_crc )
							{
								print_line("**** Flash image CRC Failure ****");
								failedLoadCRC = True;
							}
							else
							{
								print_line("Verified");

								print_chars("CRC:                   ");
								bin2hex(msg, passed_crc_in_rom);
								print_line(msg);
							}
						}
						else
						{
							if ( ram_crc )
							{
								print_line("**** RAM-CRC verification failure ****");
							}
							if ( passed_crc_in_ram != passed_crc_in_rom )
							{
								print_line("**** RAM-ROM image CRC mismatch ****");
							}

							failedLoadCRC = True;
						}

						// Calculate the CRC over the second image in RAM
						ram_crc = calculate_crc32(original_ram_start + DEFAULT_IMAGE_LENGTH, original_ram_start + original_ram_length - ignore);

						// CRC of data block is located at last word of image in RAM & ROM
						passed_crc_in_ram = *(unsigned int *)(original_ram_start + original_ram_length - ignore - sizeof(UInt32));
						passed_crc_in_rom = *(unsigned int *)(original_rom_start + original_ram_length - ignore - sizeof(UInt32));

						// No need to compute ROM CRC if RAM CRC is not the expected value
						if ( (ram_crc == 0) && (passed_crc_in_ram == passed_crc_in_rom) )
						{
							print_line("Verified");
							//calculate the CRC over the image in ROM
							print_chars("Verifying FLASH image: ");

							rom_crc = calculate_crc32(original_rom_start + DEFAULT_IMAGE_LENGTH, original_rom_start + original_ram_length - ignore);
							if ( rom_crc )
							{
								print_line("**** Flash image CRC Failure ****");
								failedLoadCRC = True;
							}
							else
							{
								print_line("Verified");

								print_chars("CRC:                   ");
								bin2hex(msg, passed_crc_in_rom);
								print_line(msg);
							}
						}
						else
						{
							if ( ram_crc )
							{
								print_line("**** RAM-CRC verification failure ****");
							}
							if ( passed_crc_in_ram != passed_crc_in_rom )
							{
								print_line("**** RAM-ROM image CRC mismatch ****");
							}

							failedLoadCRC = True;
						}
					}
					else
					{
						// For a single image

						//calculate the CRC over the image in RAM
						ram_crc = calculate_crc32(original_ram_start, original_ram_start + original_ram_length - ignore);

						// CRC of data block is located at last word of image in RAM & ROM
						passed_crc_in_ram = *(unsigned int *)(original_ram_start + original_ram_length - ignore - sizeof(UInt32));
						passed_crc_in_rom = *(unsigned int *)(original_rom_start + original_ram_length - ignore - sizeof(UInt32));

						// No need to compute ROM CRC if RAM CRC is not the expected value
						if ( (ram_crc == 0) && (passed_crc_in_ram == passed_crc_in_rom) )
						{
							print_line("Verified");
							//calculate the CRC over the image in ROM
							print_chars("Verifying FLASH image: ");

							rom_crc = calculate_crc32(original_rom_start, original_rom_start + original_ram_length - ignore);
							if ( rom_crc )
							{
								print_line("**** Flash image CRC Failure ****");
								failedLoadCRC = True;
							}
							else
							{
								print_line("Verified");

								print_chars("CRC:                   ");
								bin2hex(msg, passed_crc_in_rom);
								print_line(msg);
							}
						}
						else
						{
							if ( ram_crc )
							{
								print_line("**** RAM-CRC verification failure ****");
							}
							if ( passed_crc_in_ram != passed_crc_in_rom )
							{
								print_line("**** RAM-ROM image CRC mismatch ****");
							}

							failedLoadCRC = True;
						}
					}
				}
            }	
        } // end IF RAM CRC Passed
		
    }   // end IF we have a valid start address

    return failedLoadCRC;
}

// The Flash Uploader program and the .mbs file used by the AdaMULTI debugger must be
// in agreement as to the locations where the files for Bootstrap, OFS, MC Diagnostics,
// GNU Diagnostics, and PZT are placed in RAM prior to be reprogrammed to flash.
//
// Using the .mbs file, the AdaMULTI debugger will place files at the following
// locations in RAM for the Flash Uploader to load to flash:
//      0x00_0000    Flash Uploader Executable (fu.out) (size ~= 0x00_1200)
//      0x01_F000    "Information" on programs being loaded
//      0x02_0000    Bootstrap (size = 00_A000)
//      0x10_0000    OFS + GNU Diag (size = 0x60_0000) or MC Diag + GNU Diag (size = 0x37_0000)
//      0x80_0000    PZT (size = 0x00_2000)
//
void program_memory(void)
{
    UInt32 start_address=0;             // Flash starting memory address
    UInt32 ram_start;                 // RAM starting memory address
    UInt32 image_length;                    // Image size in bytes
    upload_format_t *upload_request = (upload_format_t *)MEMORY_UPLOAD_LINK_ADR;
    Int32 table_index;
    UInt32 passed_crc;
    UInt32 ignore;

    Setup_Serial_115_8_N_1();
	
	loadFailure = False;


    // initialize the FLASH information
    Status = XFlash_Initialize(&FlashInstance, FLASH_BASE_ADDRESS,FLASH_MEM_WIDTH, 0);
	
	switch(FlashInstance.CommandSet)
	{
		case 0x0001:
			FlashType = FLASH_INTEL;
			print_line("Intel Flash Detected");
			break;
		
		case 0x0002:
			FlashType = FLASH_MICRON;
			print_line("Micron Flash Detected");
			break;
			
		default:
			FlashType = FLASH_UNKNOWN;
			print_line("Unsupported Flash Device");
			while(1){
				
			}
	}
	
	FlashEraseBlockSize = FlashInstance.Geometry.EraseRegion[0].Size;
	
	print_line("Erase block size detected");
	
	if (Status != XST_SUCCESS) {
		print_line("**** XFlash_Initialize failed ****");
		while (1) {
		}
	}

	Status = XFlash_Reset(&FlashInstance);
	if (Status != XST_SUCCESS) {
		print_line("**** XFlash_Reset failed ****");
		while (1) {
		}
	}
	
	print_line("");
	print_line("Flash init complete");
	mem_print_hex("Flash base    = ", FlashInstance.Geometry.BaseAddress);
	mem_print_hex("Command set   = ", FlashInstance.CommandSet);
	mem_print_hex("Memory layout = ", FlashInstance.Geometry.MemoryLayout);
	mem_print_hex("Device size   = ", FlashInstance.Geometry.DeviceSize);
	mem_print_hex("Erase regions = ", FlashInstance.Geometry.NumEraseRegions);
	mem_print_hex("Num blocks    = ", FlashInstance.Geometry.NumBlocks);
	mem_print_hex("Boot mode     = ", FlashInstance.Geometry.BootMode);

	if (FlashInstance.Geometry.NumEraseRegions > 0U)
	{
		mem_print_hex("R0 offset     = ",
			FlashInstance.Geometry.EraseRegion[0].AbsoluteOffset);
		mem_print_hex("R0 size       = ",
			FlashInstance.Geometry.EraseRegion[0].Size);
		mem_print_hex("R0 blocks     = ",
			FlashInstance.Geometry.EraseRegion[0].Number);
	}
	
	if (FlashType == FLASH_UNKNOWN)
    {
        print_line("Unsupported Flash Type");
        while (1) {
        }
    }

    if (FlashType == FLASH_MICRON)
    {
		if (FlashInstance.Geometry.BootMode != MICRON_EXPECTED_BOOT_MODE)
		{
			print_line("**** Unexpected Micron boot mode ****");
			while (1) {
			}
		}

    }

    // Print program header
    print_line("");
    print_line("");
    print_line("FLASH UPLOADER SOFTWARE (X200.0.1a) ");

    // get a copy of the existing download table
    Download_Table_Copy = *Download_Table;

    while ( upload_request->Up_Type )
    {
        ram_start = RAM_START_ADDRESS;
        image_length = DEFAULT_IMAGE_LENGTH;
        switch ( upload_request->Up_Type )
        {
            case UP_BOOTSTAP:  // bootstrap
                image_length = BOOT_LENGTH;
                start_address = 0xFFFFFFFF - image_length + 1;
                ram_start = BOOT_RAM_START;
                ignore        = WORD;  // Skip 1 word on CRC calculation to show meaningful value
                table_index = -1;   // no table entry
                print_line("");
                print_line("=========================================================");
                print_line("Reprogramming Bootstrap");
                print_line("-----------------------");
                break;
            case UP_OFS:  // OFS
                start_address = PROGRAM1_START_ADDRESS;
                table_index = PROGRAM1_FLASH;
                ignore        = 0;
                print_line("");
                print_line("=========================================================");
                print_line("Reprogramming OFS");
                print_line("-----------------");
                break;
            case UP_GNU:  // GNU
                start_address = PROGRAM1_START_ADDRESS;
                table_index = PROGRAM1_FLASH;
                ignore        = 0;
                print_line("");
                print_line("=========================================================");
                print_line("Reprogramming GNU Diagnostics");
                print_line("-----------------------------");
                break;
            case UP_PZT_TABLE:   // PZT
                start_address = PZT_TABLE_START_ADDR;
                ignore        = 0;
                table_index = -1;   // no table entry
                ram_start = PZT_RAM_START_ADDRESS;    // PZT is loaded into RAM at this address by .mbs file
                image_length = PZT_IMAGE_LENGTH;   // size of PZT
                print_line("");
                print_line("=========================================================");
                print_line("Reprogramming Pseudorandom Zeroization Table (PZT) ");
                print_line("------------------------------------------------- ");
                break;
            case UP_MFG_DIAG:   // Mfg Diag
                image_length = MFG_DIAG_LENGTH;
                start_address = PROGRAM1_START_ADDRESS;
                table_index = PROGRAM1_FLASH;
                ignore        = 0;
                print_line("");
                print_line("=========================================================");
                print_line("Reprogramming MC Diagnostics");
                print_line("----------------------------");
                break;
            case UP_SPCL_BLD:   // special build
                start_address = PROGRAM1_START_ADDRESS;
                table_index = PROGRAM1_FLASH;
                ignore        = 0;
                print_line("");
                print_line("=========================================================");
                print_line("Reprogramming Special Build");
                print_line("---------------------------");
                break;
            case UP_BLIND_LOAD:   // blind load
                start_address = PROGRAM1_START_ADDRESS;
                table_index = PROGRAM1_FLASH;
                ignore        = 0;
                print_line("");
                print_line("=========================================================");
                print_line("Blind Load");
                print_line("----------");
                break;
            case UP_REPROGRAM:   // reprogrammer
                start_address = PROGRAM1_START_ADDRESS;
                table_index = PROGRAM1_FLASH;
                ignore        = 0;
                print_line("");
                print_line("=========================================================");
                print_line("Reprogramming Reprogrammer");
                print_line("--------------------------");
                break;
            case UP_OFSGNU_COMBINED:  // OFS slot + GNU diagnostics
				start_address = PROGRAM1_START_ADDRESS;
				table_index = PROGRAM1_FLASH;
				image_length = OFS_GNU_COMBINED_LENGTH;
                ignore        = 0;
                print_line("");
                print_line("=========================================================");
                print_line("Reprogramming OFS & GNU Diagnostics Combined");
                print_line("--------------------------------------------");
				
				
                break;
            case UP_MCDIAGGNU_COMB:  // two programs
                start_address = PROGRAM1_START_ADDRESS;
                table_index = PROGRAM1_FLASH;
                image_length = DEFAULT_IMAGE_LENGTH + MFG_DIAG_LENGTH;
                ignore        = 0;
                print_line("");
                print_line("=========================================================");
                print_line("Reprogramming MC Diagnostics & GNU Diagnostics Combined");
                print_line("-------------------------------------------------------");
                break;
            case UP_SERIAL_NUMBER:  // serial number
                table_index = -1;   // signal no table index so no further action
                ram_start = 0;
                Download_Table_Copy.Images[SPARE_WITH_CRC].Source = upload_request->Up_Start_Address;
                Download_Table_Copy.Images[SPARE_WITH_CRC].Destination = upload_request->Up_Length;
                print_line("=========================================================");
                print_line("Updating Projectile Serial Number");
                print_line("---------------------------------");
                print_line("Complete");
                break;
            default:
                // not a recognized upload, zero the ram start for exit
                ram_start = 0;
                ignore        = 0;
                break;
        }

        // if the start address is valid, then program
        if ( ram_start )
        {
            // if this is not the boot loader, then update the table
            if ( table_index >= 0 )
            {
                if (upload_request->Up_Type == UP_OFSGNU_COMBINED)
				{
					print_line("Download Table update: OFS & GNU Diagnostics images");

					/* OFS table entry */
					table_index = PROGRAM1_FLASH;
					Download_Table_Copy.Images[table_index].Source = PROGRAM1_START_ADDRESS;
					Download_Table_Copy.Images[table_index].Destination = RAM_LOAD_ADDRESS;
					Download_Table_Copy.Images[table_index].Image_Length = OFS_IMAGE_LENGTH;
					Download_Table_Copy.Images[table_index].PC_Start = PC_START_ADDRESS;
					Download_Table_Copy.Images[table_index].Image_Type = table_index;
					passed_crc = *(unsigned int *)(ram_start + OFS_IMAGE_LENGTH - sizeof(UInt32));
					Download_Table_Copy.Images[table_index].Image_Crc = passed_crc;

					/* GNU Diagnostics table entry */
					table_index = PROGRAM2_FLASH;
					Download_Table_Copy.Images[table_index].Source = PROGRAM2_START_ADDRESS;
					Download_Table_Copy.Images[table_index].Destination = RAM_LOAD_ADDRESS;
					Download_Table_Copy.Images[table_index].Image_Length = GNU_DIAG_LENGTH;
					Download_Table_Copy.Images[table_index].PC_Start = PC_START_ADDRESS;
					Download_Table_Copy.Images[table_index].Image_Type = table_index;
					passed_crc = *(unsigned int *)(ram_start + GNU_RAM_OFFSET +
												   GNU_DIAG_LENGTH - sizeof(UInt32));
					Download_Table_Copy.Images[table_index].Image_Crc = passed_crc;

					/*
					 * Temporary CRC-window debug.
					 * This runs before program_image(), so use ram_start/image_length/start_address.
					 */
					if ((start_address == PROGRAM1_START_ADDRESS) &&
						(image_length == OFS_GNU_COMBINED_LENGTH))
					{
						mem_debug_crc_window("CRC debug: OFS 0x003C0000",
							ram_start,
							OFS_IMAGE_LENGTH);

						mem_debug_crc_window("CRC debug: OFS full slot 0x00400000",
							ram_start,
							OFS_FLASH_SLOT_LENGTH);

						mem_debug_crc_window("CRC debug: GNU at 0x00400000 length 0x00300000",
							ram_start + OFS_FLASH_SLOT_LENGTH,
							GNU_DIAG_LENGTH);

						mem_debug_crc_window("CRC debug: GNU immediately after OFS length 0x00340000",
							ram_start + OFS_IMAGE_LENGTH,
							OFS_GNU_COMBINED_LENGTH - OFS_IMAGE_LENGTH);
							
						mem_find_word_in_ram("Find OFS CRC marker C795AA2A",
											 ram_start,
											 OFS_GNU_COMBINED_LENGTH,
											 0xC795AA2AU);

						mem_find_word_in_ram("Find GNU CRC marker 83F33C32",
											 ram_start,
											 OFS_GNU_COMBINED_LENGTH,
											 0x83F33C32U);
					}
					
				}
				
                if (upload_request->Up_Type == UP_MCDIAGGNU_COMB)
                {
                    print_line("Download Table update: MC Diagnostics & GNU Diagnostics images");
                    // GNU table entry
                    Download_Table_Copy.Images[table_index].Source = PROGRAM1_START_ADDRESS;
                    Download_Table_Copy.Images[table_index].Destination = RAM_LOAD_ADDRESS;
                    Download_Table_Copy.Images[table_index].Image_Length = DEFAULT_IMAGE_LENGTH;
                    Download_Table_Copy.Images[table_index].PC_Start = PC_START_ADDRESS;
                    Download_Table_Copy.Images[table_index].Image_Type = table_index;
                    passed_crc = *(unsigned int *)(ram_start + Download_Table_Copy.Images[table_index].Image_Length - sizeof(UInt32));
                    Download_Table_Copy.Images[table_index].Image_Crc = passed_crc;

                    // mc diag table entry
                    table_index = PROGRAM2_FLASH;
                    Download_Table_Copy.Images[table_index].Source = PROGRAM2_START_ADDRESS;
                    Download_Table_Copy.Images[table_index].Destination = RAM_LOAD_ADDRESS;
                    Download_Table_Copy.Images[table_index].Image_Length = MFG_DIAG_LENGTH ;
                    Download_Table_Copy.Images[table_index].PC_Start = MFG_PC_START ;
                    Download_Table_Copy.Images[table_index].Image_Type = table_index;
                    passed_crc = *(unsigned int *)(ram_start + (MFG_DIAG_LENGTH + DEFAULT_IMAGE_LENGTH) - sizeof(UInt32));
                    Download_Table_Copy.Images[table_index].Image_Crc = passed_crc;
                }
                else if ((upload_request->Up_Type == UP_OFS) || (upload_request->Up_Type == UP_GNU) || (upload_request->Up_Type == UP_BLIND_LOAD)
                        || (upload_request->Up_Type == UP_SPCL_BLD) || (upload_request->Up_Type == UP_REPROGRAM) )
                {
                    print_line("Download Table update: Single image");
                    // the primary image is the flashed image
                    Download_Table_Copy.Images[table_index].Source = start_address;
                    Download_Table_Copy.Images[table_index].Destination = RAM_LOAD_ADDRESS;
                    Download_Table_Copy.Images[table_index].Image_Length = image_length;
                    Download_Table_Copy.Images[table_index].PC_Start = PC_START_ADDRESS;
                    Download_Table_Copy.Images[table_index].Image_Type = table_index;
                    passed_crc = *(unsigned int *)(ram_start + Download_Table_Copy.Images[table_index].Image_Length - sizeof(UInt32));
                    Download_Table_Copy.Images[table_index].Image_Crc = passed_crc;
                }
                else if (upload_request->Up_Type == UP_MFG_DIAG)
                {
                    print_line("Download Table update: Single image MC Diagnostics");
                    // the primary image is the flashed image
                    Download_Table_Copy.Images[table_index].Source = start_address;
                    Download_Table_Copy.Images[table_index].Destination = RAM_LOAD_ADDRESS;
                    Download_Table_Copy.Images[table_index].Image_Length = image_length;
                    Download_Table_Copy.Images[table_index].PC_Start = MFG_PC_START;
                    Download_Table_Copy.Images[table_index].Image_Type = table_index;
                    passed_crc = *(unsigned int *)(ram_start + Download_Table_Copy.Images[table_index].Image_Length - sizeof(UInt32));
                    Download_Table_Copy.Images[table_index].Image_Crc = passed_crc;
                }
            }
            // program the image. OR  the result of this load attempt overall status.
            if (upload_request->Up_Type == UP_OFSGNU_COMBINED)
			{
				print_line("");
				print_line("Programming OFS image into PROGRAM1 flash slot");
				loadFailure |= program_image(ram_start,
											 OFS_IMAGE_LENGTH,
											 PROGRAM1_START_ADDRESS,
											 0);

				print_line("");
				print_line("Programming GNU Diagnostics image into PROGRAM2 flash slot");
				loadFailure |= program_image(ram_start + GNU_RAM_OFFSET,
											 GNU_DIAG_LENGTH,
											 PROGRAM2_START_ADDRESS,
											 0);
			}
			else
			{
				loadFailure |= program_image(ram_start, image_length, start_address, ignore);
			}
        }
        ++upload_request;
    }

    print_line("");
	print_line("=================================================");

	if (loadFailure)
	{
		print_line("Skipping Download Table update");
		print_line("Reason: one or more image loads failed");
	}
	else
	{
		print_line("Updating Download Table");
		print_line("-----------------------");

		Download_Table_Copy.Images[SPARE_WITH_CRC].Image_Type =
			calculate_crc32((UInt32)&Download_Table_Copy,
				((UInt32)&Download_Table_Copy) +
				sizeof(Download_Table_Copy) - sizeof(UInt32));

		loadFailure |= program_image((UInt32)&Download_Table_Copy,
									 sizeof(Download_Table_Copy),
									 (UInt32)Download_Table,
									 0);
	}

    print_line("");
    print_line("=========================");


    // Signal completion.  Clears RAM load address to prevent additional loads
    //   unless power is removed and Flash Uploader is restarted.
    upload_request = (upload_format_t *)MEMORY_UPLOAD_LINK_ADR;
    upload_request->Up_Start_Address = 0;
	upload_request->Up_Length = 0;
	upload_request->Up_Type = 0;
	upload_request->Up_LoadAddress = 0;
	upload_request->Up_PC_Address = 0;


    // Determine output status line based on overall load boolean.
    if (loadFailure)
    {
        print_line("**** FAILED ****");
        print_line("=========================");

        // Loop to allow operator to stop Green Hills and see that the load has
        //   FAILED
        while ( 1 )
        {
        }

    }
    else // no errors detected in load or crc calculation
    {
        print_line("All Done");
        print_line("=========================");

        // Loop to allow operator to stop Green Hills and see that the load was
        //   SUCCESSFUL
        while ( 1 )
        {
        }
    }

}

