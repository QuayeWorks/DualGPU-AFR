
#include <typedefs.h>
#include "xilinx_drivers\xilflash.h"

void Setup_Serial_115_8_N_1(void);
void print_line(char *textOut);
void print_chars(char *textOut);

#define WORD                      4

#define BOOTSTRAP_START_ADDRESS     0xFFFF6000
#define PROGRAM1_START_ADDRESS      0xFE000000
#define PROGRAM2_START_ADDRESS      0xFE380000
#define MFG_DIAG_START_ADDRESS      0xFE000000
#define PZT_TABLE_START_ADDR        0xFFA00000
#define DOWNLOAD_TABLE_ADDR         0xFFC00000

#define RAM_START_ADDRESS           0x00100000
#define BOOT_RAM_START              0x00020000
#define BOOT_LENGTH                 0x0000A000
#define MFG_DIAG_LENGTH             0x00070000
#define RAM_LOAD_ADDRESS            0x00000000
#define BOOT_LOAD_ADDRESS           0xFFFF6000
#define DEFAULT_IMAGE_LENGTH        0x00380000
#define PC_START_ADDRESS            0x0000A000
#define MFG_DIAG_LENGTH             0x00070000
#define MFG_PC_START                0x00001000
#define PZT_RAM_START_ADDRESS       0x00800000
#define PZT_IMAGE_LENGTH            0x00002000

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

XFlash FlashInstance; /* XFlash Instance. */

int mymemcmp(void *s1, void *s2, int len)
{
    int result;
    int word_count = len / 4;
    UInt32 *u32s1ptr = (UInt32 *)s1;
    UInt32 *u32s2ptr = (UInt32 *)s2;

    while ( (word_count > 0) && ((result = (*u32s1ptr - *u32s2ptr) ) == 0) )
    {
        --word_count;
        ++u32s1ptr;
        ++u32s2ptr;
    }
    return result;
}

void flash_unprotect(UInt32 length, UInt32 offset)
{
    // Note: bytes parameter in XFlash_Unlock is not used
    Status = XFlash_Unlock(&FlashInstance, offset, length);
}

void flash_protect(UInt32 length, UInt32 offset)
{
    // Note: bytes parameter in XFlash_Lock is not used
    Status = XFlash_Lock(&FlashInstance, offset, length);
}


Bool program_image(UInt32 ram_start_address, UInt32 ram_length, UInt32 rom_start_address, UInt32 ignore)
{
    UInt32 start_address=0;             // Flash starting memory address
    UInt32 ram_start;                 // RAM starting memory address
    UInt32 rom_offset;                    // Flash offset from base address
    UInt32 length;                    // Image size in bytes
    Int32 NumBlockToWrite;
    UInt32 tmp;
    UInt32 RetryCounter;
    UInt32 *VerifyErasePtr;
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

    start_address = rom_start_address;
    ram_start     = ram_start_address;

    if ( start_address )
    {
        // The offset is the absolute address minus the base address
        // of the FLASH.
        rom_offset    = rom_start_address - XPAR_FLASH_MEM0_BASEADDR;

        original_ram_length = ram_length;
        original_ram_start = ram_start;
        original_rom_start = start_address;
        original_rom_offset = rom_offset;



        // Verify the images moved from the file on the PC to RAM on the MC CCA

        print_line("Verifying RAM image(s)   ");

        // Check for a single image or multiple images (i.e. OFS + GNU Diag) being loaded
        if (original_ram_length > DEFAULT_IMAGE_LENGTH)
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


        // RAM image verified.  Now erase flash and write image from RAM to flash
        if (passed_RAM_CRC)
        {
            print_line("Erasing Flash: ");
            length = 128*1024;
            NumBlockToWrite = ram_length / length;

            while ( ram_length > 0 )
            {
                if ( ram_length < length )
                {
                    length = ram_length;
                }
                RetryCounter = 3;
                do
                {
                    Status = XFlash_Reset(&FlashInstance);
                    flash_unprotect(length, rom_offset);
                    Status = XFlash_Erase(&FlashInstance, rom_offset, length);
                    flash_protect(length, rom_offset);
                    Status = XFlash_Reset(&FlashInstance);
                    --RetryCounter;
                } while ( RetryCounter && (Status != XST_SUCCESS) );

                ram_length -= length;
                rom_offset += length;
                --NumBlockToWrite;

                if ( NumBlockToWrite > 0 )
                {
                    print_chars("    ");
                    msg[0] = 0x0d;
                    msg[1] = 0;
                    print_chars(msg);
                    bin2ascii(msg, NumBlockToWrite);
                    tmp = strlen(msg);
                    msg[tmp] = 0x0d;
                    msg[tmp+1] = 0;
                    print_chars(msg);
                }
            }

            VerifyErasePtr = (UInt32 *)original_rom_start;
            for ( RetryCounter=0;
                    (Status == XST_SUCCESS) && (RetryCounter<(original_ram_length/sizeof(UInt32)));
                    ++RetryCounter )
            {
                if ( *VerifyErasePtr++ != 0xFFFFFFFF )
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

                NumBlockToWrite = original_ram_length / 0x2000;

                remaining_length = original_ram_length;
                ram_start = original_ram_start;
                start_address = original_rom_start;
                rom_offset = original_rom_offset;
                while ( remaining_length > 0 )
                {
                    if ( (NumBlockToWrite % 10) == 0 )
                    {
                        print_chars("    ");
                        msg[0] = 0x0d;
                        msg[1] = 0;
                        print_chars(msg);
                        bin2ascii(msg, NumBlockToWrite);
                        tmp = strlen(msg);
                        msg[tmp] = 0x0d;
                        msg[tmp+1] = 0;
                        print_chars(msg);
                    }

                    length = 0x2000;
                    if ( length > remaining_length )
                    {
                        length = remaining_length;
                    }
                    Status = XFlash_Reset(&FlashInstance);
                    flash_unprotect(length, rom_offset);
                    XFlash_Write(&FlashInstance, rom_offset, length, (void *)ram_start);
                    flash_protect(length, rom_offset);
                    Status = XFlash_Reset(&FlashInstance);
                    rom_offset += length;
                    ram_start += length;
                    remaining_length -= length;
                    --NumBlockToWrite;
                }

                print_line("Complete");

                print_chars("Verifying RAM image:   ");

                // Check for a single image or multiple images (i.e. OFS + GNU Diag) being loaded
                if (original_ram_length > DEFAULT_IMAGE_LENGTH)
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

    // initialize the FLASH information
    Status = XFlash_Initialize(&FlashInstance);

    // Print program header
    print_line("");
    print_line("");
    print_line("FLASH UPLOADER SOFTWARE (X100.0.12d) ");

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
            case UP_OFSGNU_COMBINED:  // two programs
                start_address = PROGRAM1_START_ADDRESS;
                table_index = PROGRAM1_FLASH;
                image_length = 2 * DEFAULT_IMAGE_LENGTH;
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
                    // OFS table entry
                    Download_Table_Copy.Images[table_index].Source = PROGRAM1_START_ADDRESS;
                    Download_Table_Copy.Images[table_index].Destination = RAM_LOAD_ADDRESS;
                    Download_Table_Copy.Images[table_index].Image_Length = DEFAULT_IMAGE_LENGTH;
                    Download_Table_Copy.Images[table_index].PC_Start = PC_START_ADDRESS;
                    Download_Table_Copy.Images[table_index].Image_Type = table_index;
                    passed_crc = *(unsigned int *)(ram_start + Download_Table_Copy.Images[table_index].Image_Length - sizeof(UInt32));
                    Download_Table_Copy.Images[table_index].Image_Crc = passed_crc;

                    // GNU table entry
                    table_index = PROGRAM2_FLASH;
                    Download_Table_Copy.Images[table_index].Source = PROGRAM2_START_ADDRESS;
                    Download_Table_Copy.Images[table_index].Destination = RAM_LOAD_ADDRESS;
                    Download_Table_Copy.Images[table_index].Image_Length = DEFAULT_IMAGE_LENGTH;
                    Download_Table_Copy.Images[table_index].PC_Start = PC_START_ADDRESS;
                    Download_Table_Copy.Images[table_index].Image_Type = table_index;
                    passed_crc = *(unsigned int *)(ram_start + (2 * Download_Table_Copy.Images[table_index].Image_Length) - sizeof(UInt32));
                    Download_Table_Copy.Images[table_index].Image_Crc = passed_crc;
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
            loadFailure |= program_image(ram_start, image_length, start_address, ignore);
        }
        ++upload_request;
    }

    print_line("");
    print_line("=================================================");
    print_line("Updating Download Table");
    print_line("-----------------------");

    // Calculate the CRC of the table
    Download_Table_Copy.Images[SPARE_WITH_CRC].Image_Type =
        calculate_crc32((UInt32) &Download_Table_Copy,
                ((UInt32)&Download_Table_Copy) + sizeof(Download_Table_Copy) - sizeof(UInt32));

    // Write the Download Table to FLASH. OR  the result of this load attempt overall status.
    loadFailure |= program_image((UInt32)&Download_Table_Copy, sizeof(Download_Table_Copy), (UInt32) Download_Table, 0);

    print_line("");
    print_line("=========================");


    // Signal completion.  Clears RAM load address to prevent additional loads
    //   unless power is removed and Flash Uploader is restarted.
    upload_request = (upload_format_t *)MEMORY_UPLOAD_LINK_ADR;
    upload_request->Up_Start_Address = 0;


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

