/*******************************************************************************
  Microchip File System API defintion

  Summary: File System Abstraction Layer

  Description: Provides a consistant interface to the services offered by
               multiple file systems
 ******************************************************************************/

/*******************************************************************************
FileName:  sys_fs.h
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that isintegratedinto your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
#ifndef SYS_FS_H
#define SYS_FS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
//#include <sys/types.h>

#ifndef off_t
#define off_t   int32_t
#endif
#ifndef ssize_t
#define ssize_t long
#endif


#include <system/errno.h>

#include "mpfs2_config.h"
#include "sys_fs_config.h"
#include "system/fs/mdd_fs.h"
#include "system/drivers/drv_media.h"
#include "system/fs/system_fs_private.h"

#ifndef RETURN_SUCCESS
#define RETURN_SUCCESS 0
#endif

#ifndef RETURN_FAILED
#define RETURN_FAILED -1
#endif

#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2

typedef int32_t FILE_HANDLE;

typedef enum
{
    SYS_FS_DIR,        // Directory
    SYS_FS_SLINK,      // Soft link
    SYS_FS_HLINK,      // hardlink
    SYS_FS_DATAFILE,   // data file
    SYS_FS_MOUNT_LABEL,// A mount name
    SYS_FS_CDEVICE,    // Character device
    SYS_FS_BDEVICE,    // Block device
    SYS_FS_NDEVICE     // Network device
}SYS_FS_FileTypes;

typedef enum
{
    SYS_FS_MEDIA_UNKNOWN,          // invalid media type
    SYS_FS_MEDIA_DEFAULT,          // default media;
                                   // mainly for systems where only one media supported
    SYS_FS_MEDIA_EEPROM,           // EEPROM
    SYS_FS_MEDIA_SPI_FLASH,        // SPI flash:  SST 25VF016B, etc
    SYS_FS_MEDIA_INTERNAL_FLASH,   // on chip Flash
    SYS_FS_MEDIA_MDD_FS,       // SD card through SPI interface
}SYS_FS_MEDIA_TYPE;

typedef enum
{
    /* Up and running, ready to start new operations */
    SYS_FS_CLIENT_STATUS_READY = 0,

    /* Operation in progress, unable to start a new one */
    SYS_FS_CLIENT_STATUS_BUSY,

    /* client Invalid */
    SYS_FS_STATUS_INVALID = -1,

    /* bad parameter */
    SYS_FS_STATUS_PARAM_ERROR = -2,

    /* not supported operation */
    SYS_FS_STATUS_OPERATION_ERROR = -3,

    /* storage full */
    SYS_FS_STATUS_EOS = -4,

    /* range error */
    SYS_FS_STATUS_RANGE_ERR = -5,


    /* read/write error */
    SYS_FS_STATUS_RW_ERR = -6,

} SYS_FS_CLIENT_STATUS;

typedef enum
{
    /* Read */
    SYS_FS_IO_INTENT_READ = 1 << 0 ,

    /* Write */
    SYS_FS_IO_INTENT_WRITE = 1 << 1,

    /* Read and Write*/
    SYS_FS_IO_INTENT_READWRITE = SYS_FS_IO_INTENT_READ|SYS_FS_IO_INTENT_WRITE,

    /* The driver will block and will return when the operation is complete */
    SYS_FS_IO_INTENT_BLOCKING = 0 << 2,

    /* The driver will return immediately; Not supported */
    SYS_FS_IO_INTENT_NONBLOCKING = 1 << 2,

    /* The driver will support only one client at a time */
    SYS_FS_IO_INTENT_EXCLUSIVE  = 1 << 3 ,

    /* The driver will support multiple clients at a time; Not supported */
    SYS_FS_IO_INTENT_SHARED = 0 << 3

}SYS_FS_IO_INTENT;

// The following types should be included in an appropriate H file
typedef int16_t blksize_t;
typedef int16_t blkcnt_t;
typedef int32_t time_t;

#define SYS_FS_ATTR_CPRS_MASK 0x0F
#define SYS_FS_ATTR_SYS_MASK  0xF0
typedef enum
{
    // Compressed status
    SYS_FS_ATTR_UNCOMPRESSED  = 0x0,
    SYS_FS_ATTR_HUFFMAN_COMPRESSED = 0x1,
    SYS_FS_ATTR_ZIP_COMPRESSED = 0x2,
    SYS_FS_ATTR_RAR_COMPRESSED = 0x3,
    SYS_FS_ATTR_7Z_COMPRESSED = 0x4,

    // System status
    SYS_FS_ATTR_READ_ONLY = 0x10,
    SYS_FS_ATTR_WRITE_ONLY = 0x20,
    SYS_FS_ATTR_READ_WRITE = 0x30,
    SYS_FS_ATTR_HIDDEN = 0x40,

}SYS_FS_ATTR;

typedef struct {
    SYS_FS_ATTR st_attr;  /* File status, such as compressed, etc... */
#if 0
    ssize_t   st_size;    /* total size, in bytes */
    blksize_t st_blksize; /* blocksize for file system I/O */
    blkcnt_t  st_blocks;  /* number of 512B blocks allocated */
    time_t    st_atime;   /* time of last access */
    time_t    st_mtime;   /* time of last modification */
    time_t    st_ctime;   /* time of last status change */
#endif
} FILE_STAT;

/****************************** SYS_FS_createMBRDEF ****************************
 * Description:
 *         This function can be used to create a master boot record for a device.
 *         Note that this function should not be used on a device that is already
 *         formatted with a master boot record (i.e. most SD cards, CF cards, USB
 *         keys). This function will fill the global data buffer with appropriate
 *         partition information for a FAT partition with a type determined by
 *         the number of sectors available to the partition. It will then write
 *         the MBR information to the first sector on the device. This function
 *         should be followed by a call to SYS_FS_format, which will create a boot
 *         sector, root dir, and FAT appropriate the the information contained in
 *         the new master boot record. Note that SYS_FS_format only supports FAT12
 *         and FAT16 formatting at this time, and so cannot be used to format a
 *         device with more than 0x3FFD5F sectors
 *
 * Params: first_sector - The first sector of the partition on the device
 *                        (cannot be 0; that's the MBR)
 *         num_sectors -  The number of sectors available in memory (including the MBR)
 *         fsType - file system type string, such as "mdd" or "mpfs2"
 *
 * Return: 0 - RETURN_SUCCESS, -1 - RETURN_FAILED
 *
 * Note:   It supports underlying MDD FS ONLY now
 ******************************************************************************/
int32_t SYS_FS_createMBR(uint32_t first_sector, uint32_t num_sectors, char *fsType);

/****************************** SYS_FS_formatDEF ****************************
 * Description:
 *         The SYS_FS_format function can be used to create a new boot sector on
 *         a device, based on the information in the master boot record. This
 *         function will first initialize the I/O pins and the device, and then
 *         attempts to read the master boot record. If the MBR cannot be loaded
 *         successfully, the function will fail. Next, if the 'mode' argument is
 *         specified as '0' the existing boot sector information will be loaded.
 *         If the 'mode' argument is '1' an entirely new boot sector will be
 *         constructed using the disk values from the master boot record. Once
 *         the boot sector has been successfully loaded/created, the locations of
 *         the FAT and root will be loaded from it, and they will be completely
 *         erased. If the user has specified a volumeID parameter, a VOLUME
 *         attribute entry will be created in the root directory to name the device.
 *         FAT12, FAT16 and FAT32 formatting are supported.
 *         Based on the number of sectors, the format function automatically compute
 *         the smallest possible value for the cluster size in order to accommodate
 *         the physical size of the media. In this case, if a media with a big
 *         capacity is formatted, the format function may take a very long time to
 *         write all the FAT tables.
 *         Therefore, the FORMAT_SECTORS_PER_CLUSTER macro may be used to specify
 *         the exact cluster size (in multiples of sector size). This macro can be
 *         defined in sys_fs_config.h
 *
 * Params: mode - 0: Just erase the FAT and root; 1: Create a new boot sector
 *         serial_num - Serial number to write to the card
 *         volume_id - Name of the card
 *         fsType - file system type string, such as "mdd" or "mpfs2"
 *
 * Return: 0 - RETURN_SUCCESS, -1 - RETURN_FAILED
 *
 * Note:   It supports underlying MDD FS ONLY now
 ******************************************************************************/
int32_t SYS_FS_format(char mode, uint32_t serial_num, char *volume_id, char *fsType);

/////////////////////////// ANSI/POSIX like API /////////////////////
// Non-buffered I/O
/****************************** SYS_FS_openDEF *********************************
 * Function:
 *         int32_t SYS_FS_open(const char *pathname, int32_t oflag, ...)
 *
 * Summary:
 * Provides an FS aware wrapper around the open() command, thus providing the
 * ability to perform file access on name regardless of what file system and
 * media are used to store name. It examines the string pattern of name
 * for clues that can be used to determine what device and file system are
 * used name. Although the file system type and device are not part of the name
 * the string pattern of name can be used to reference FS state info that can
 * ultimatley determine that type of info
 *
 * Description:
 * Directories are files.
 * A complete file name such as MyFile1/MyFile2/Somefile.txt? has multiple filename
 * entries that complete the filename path.
 * A contiguous set of string character beginning at the first byte up to and
 * excluding any present file separator shall be known throughout this document
 * as the first file name.
 * If first file name cannot be found in the current directory then it will be
 * assumed that the first file name is a mount point residing off of the top-most
 * directory (a.k.a root dir). If filename does not match the string pattern of
 * any mount name stored in the system mount table then it will be assumed
 * that filename does not exist and follow the behavior appropriate for mode (an
 * FS owned private mount table called SYS_FS_pMountRec contains a list of  mount
 * points and their associated device).

 * Example: If name="email/index.htm", this function will determine if "email" is
 * a name of a folder or a mount name. If it is not the name of a mount name, then
 * is will assume that it is the name of a folder. It will then proceed to looking
 * at "index.htm" and assume that is the name of the file to open.
 *
 * Parameters:
 *             name - A null terminated string sequence representing a full file
 *             path or relative file path of a file to open.
 *
 * Returns: A file handle generated by the underlying fs on success
 *         Returns RETURN_FAILED on error and sets the global var errno.
 *         errno set to ENOENT iff the file could not be found.
 *         errno set to EINVAL if the filepath contains an illegal character
 *
 ******************************************************************************/
int32_t SYS_FS_open(const char *filename, int32_t oflag, ...);

/****************************** SYS_FS_closeDEF ********************************
 * Description: Close a file that has been opend by SYS_FS_open()
 *
 * Params: filedes  - a file descriptor returned by SYS_FS_open()
 *
 * Return:
 *         Returns RETURN_SUCCESS if no errors were detected
 *         Upon error returns -1 and sets the global variable errno to a value
 *         the indicates the cause of the error.
 *         The var errno is set to EBADF if the descriptor is not valid.
 *         The var errno is set to SYS_FS_MEDIA_UNKNOWN if the media type is not
 *         recognized.
 ******************************************************************************/
int32_t SYS_FS_close (int32_t filedes);

/******************************** SYS_FS_fsize ******************************
 * Description:   The SYS_FS_fsize will return the size of specified file
 *
 * Params: filedes - A file descriptor returned by a call to SYS_FS_open
 * Return: file size of -1 if any error
 ******************************************************************************/
ssize_t SYS_FS_fsize(int32_t filedes);

/******************************** SYS_FS_ftell ******************************
 * Description:   The SYS_FS_ftell function will return the current position in
 *        the file handle 'filedes', which is used to keep track of the absolute
 *        location of the current position in the file.
 *
 * Params: filedes - A file descriptor returned by a call to SYS_FS_open
 * Return: Current location in the file or -1 if any error encountered
 ******************************************************************************/
ssize_t SYS_FS_ftell(int32_t filedes);

/******************************** SYS_FS_fstat ******************************
 * Description:   The SYS_FS_fstat function will return the current position in
 *        the file handle 'filedes', which is used to keep track of the absolute
 *        location of the current position in the file.
 *
 * Params: filedes - A file descriptor returned by a call to SYS_FS_open
 *         buf  - Pointer to attribute struct
 * Return: -1 on failure/error
 ******************************************************************************/
int32_t SYS_FS_fstat(int32_t filedes, FILE_STAT *buf);

/******************************** SYS_FS_fgetname ******************************
 * Description:
 *         The SYS_FS_fgetname function will get the file name into the buffer.
 *
 * Params: filedes - A file descriptor returned by a call to SYS_FS_open
 *         buf  - Pointer to memory which stores the got file name
 * Return: -1 on failure/error
 ******************************************************************************/
int32_t SYS_FS_fgetname(int32_t filedes, uint8_t *buf, uint16_t len);

/******************************** SYS_FS_readDEF ******************************
 * Description:   Reads a file that was previsously opened with SYS_FS_open()
 *
 * Params: filedes - A file descriptor returned by a call to SYS_FS_open
 *         buffer - A pointer to an allocated area of memory. This memory space
 *                  will be populated with the file contents read.
 *         size   - The maximum number of bytes to write to buffer.
 * Return: The number of bytes read or -1 on error.
 *         On error the global variable errno is set to EIO if there was some
 *         unrecognised error I/O error generated by the underlying file system.
 *         Errno is set to SYS_FS_MEDIA_UNKNOWN if filedes is not a recognized
 *         file system
 ******************************************************************************/
ssize_t SYS_FS_read(int32_t filedes, void *buffer, size_t size);

/******************************** SYS_FS_writeDEF ******************************
 * Description:   Write file that was previsously opened with SYS_FS_open()
 *
 * Params: filedes - A file descriptor returned by a call to SYS_FS_open
 *         buffer - A pointer to an allocated area of memory. This memory space
 *                  will be populated with the file contents read.
 *         size   - The maximum number of bytes to write to buffer.
 * Return: The number of bytes written or -1 on error.
 *         On error the global variable errno is set to EIO if there was some
 *         unrecognised error I/O error generated by the underlying file system.
 *         Errno is set to SYS_FS_MEDIA_UNKNOWN if filedes is not a recognized
 *         file system
 ******************************************************************************/
ssize_t SYS_FS_write(int32_t filedes, void *buffer, size_t size);

//ssize_t SYS_FS_write(int32_t filedes, const void *buffer, size_t size);
//int32_t SYS_FS_remove( const char *fname );

// Buffered I/O
//FILE *SYS_FS_fopen(const char *filename,const char *mode);
//int32_t SYS_FS_fclose(FILE *fp);
//int32_t SYS_FS_fflush(FILE *fp);
//int32_t SYS_FS_fread(void *buffer,size_t size,size_t count,FILE *stream);
//size_t SYS_FS_fwrite(const void *buffer,size_t size,size_t count,FILE *stream );
//int32_t SYS_FS_GetFileType(const char *path);

//Working Directory
//char *SYS_FS_getcwd(const char *pathname,char *buf, size_t size);

/**************************** SYS_FS_getcwdDEF *********************************
 * Description:
 *        Get/Copy the current working directory out
 *
 * Params:
 *        pcwd - Pointer to where the CWD string copy
 *
 * Return: 0 - RETURN_SUCCESS, -1 - RETURN_FAILED
 *
 * Note:   MPFS2 has no directory support.
 *         If "FirstFilename" of the path is same as one of mount point, it will
 *         be always recognized as an absolute directory. In other word, it is
 *         better NOT to use relative path which begins with Mount-Point's name.
 ******************************************************************************/
int32_t SYS_FS_getcwd(char *pcwd);

/**************************** SYS_FS_chdirDEF ******************************
 * Description:
 *         Sets the current working directory
 *
 * Params:
 *        path - Null terminated string to a valid directory path to be used
 *               as the new current directory
 *
 * Return: RETURN_SUCCESS
 ******************************************************************************/
int32_t SYS_FS_chdir(const char *path);

/**************************** SYS_FS_mkdirDEF ******************************
 * Description:
 *         Create a directory as per input ascii string, and the new direcotry
 *         is relative to current working directory
 *
 * Params:
 *        path - Null terminated string to a valid directory path to be created
 *
 * Return: RETURN_SUCCESS
 *
 * Note:   MPFS2 has no directory concept.
 *         This function doesn't move the current working directory setting
 ******************************************************************************/
int32_t SYS_FS_mkdir(char *path);

/***************************** SYS_FS_rmdirDEF *********************************
 * Description:
 *         Remove/delete a directory from current working directory
 *         as per input ascii string
 *
 * Params:
 *        path - Null terminated string to a valid directory path to be created
 *               This is a relative directory.
 *
 * Return: 0 - RETURN_SUCCESS, other value - failed
 *
 * Note:   MPFS2 has no directory support.
 *         This function wont delete the current working directory.
 ******************************************************************************/
int32_t SYS_FS_rmdir(char *path, bool rmsubdirs);

////////////////////////////////////////////////////////////////////////////////
/////////////////////////// SYS_FS non ANSI/POSIX mappings /////////////////////
////////////////////////////////////////////////////////////////////////////////

/**************************** SYS_FS_initializeDEF *****************************
 * Description:
 *           Prepares the virtual file system (FS) by allocating memory and
 *           initializing state descriptor tables
 *
 * Params:
 *        intData - null pointer to initializatio data values. Unused at this time
 *
 * Return: true for SUCCESS; false for FAILURE
 ******************************************************************************/
bool SYS_FS_initialize(const void* const initData);

/**************************** SYS_FS_deinitializeDEF *****************************
 * Description:
 *           Prepares the virtual file system (FS) for shutting down by
 *           deallocating memory and and freeing descriptors within the descriptor
 *           tables
 *
 * Params:
 *        intData - null pointer to initializatio data values. Unused at this time
 *
 * Return: RETURN_SUCCESS
 ******************************************************************************/
int32_t SYS_FS_deinitialize(const void* const initData);

/**************************** SYS_FS_reinitializeDEF *****************************
 * Description:
 *           Reinializes the FS
 *
 * Params:
 *        intData - null pointer to initializatio data values. Unused at this time
 *
 * Return: RETURN_SUCCESS
 ******************************************************************************/
int32_t SYS_FS_reinitialize(const void* const initData);



/////////////////////////// Filepath parsing ////////////////////////////////

/*************************** SYS_FS_IsLegalDirStringDEF ***********************
 * Description:
 * Determines if buffer contains any characters which are not supported by
 * FS directory names.
 * PARAMS: buffer - A null terminated string of characters representing a dir name
 *         len - the size in bytes of the memory pointed to by buffer
 *
 * Return: RETURN_SUCCESS if no error.
 *         On error returns the ASCII value of the offending character.
*******************************************************************************/
int32_t SYS_FS_IsLegalDirString(const char * buffer,int32_t len);

/*************************** SYS_FS_IsLegalFileStringDEF ***********************
 * Description:
 * Determines if buffer contains any characters which are not supported by
 * FS file names.
 * PARAMS: buffer - A null terminated string of characters representing a dir name
 *         len - the size in bytes of the memory pointed to by buffer
 *
 * Return: RETURN_SUCCESS if no error.
 *         On error returns the ASCII value of the offending character.
*******************************************************************************/
int32_t SYS_FS_IsLegalFileString(const char * buffer,int32_t len);

/************************** SYS_FS_IsLegalFileCharDEF **************************
 * Description:
 * The characters SYS_FS_CHAR is illegal as are all characters that are not
 * valid characters within a directory name
 * Returns RETURN_SUCCESS if the character within buf is legal.
 * Returns RETURN_FAILED if buf is an illegal character
 *******************************************************************************/
int32_t SYS_FS_IsLegalFileChar(const char buf);

/************************** SYS_FS_IsLegalDirCharDEF ***************************
 * Description:
 * All characters between and including ASCII values 0x20 and 0x5f are legal.
 * Returns RETURN_SUCCESS if the character within buf is legal.
 * Returns RETURN_FAILED if buf is an illegal character
 *  ***************************************************************************/
int32_t SYS_FS_IsLegalDirChar(const char buf);

/************************** SYS_FS_ExtractPathnameDEF **************************
 * Description:
 * Given that filepath is a null terminated string representing a file name
 * appended to at least one directory name, buffer will be populate with
 * the file path portion of filepath minus the file name.
 * Params:
 *        filepath - A full path name of a file
 *        buffer - A pointer to an allocated area of memory large enough to
 *                 hold the pathname portion of filepath. Upon completion
 *                 buffer will contain a null terminated string of characters
 *                 representing filepath with the filename removed.
 *        len      The total number of bytes allocated to buffer.
 * Return:
 *         None
 *  ***************************************************************************/
void SYS_FS_ExtractPathname(const char *filepath, char * buffer,int32_t len);

/************************** SYS_FS_ExtractFilenameDEF **************************
 * Description:
 * Given that filepath is a null terminated string representing a file name
 * appended to at least one directory name, buffer will be populate with
 * the filename portion of filepath.
 * Params
 *        filepath - A full pathname of a file
 *        buffer - A pointer to an allocated area of memory large enough to
 *                 hold the filename portion of filepath. Upon completion
 *                 buffer will contain a null terminated string of characters
 *                 representing a filename
 *        len      The total number of bytes allocated to buffer.
 *  Return:
 *         None
 *  ***************************************************************************/
void SYS_FS_ExtractFilename(const char *filepath, char * buffer,int32_t len);

/**************************** SYS_FS_mountDEF ******************************
 * Description:
 *         Assignes an alias name (i.e. target) to an assocaiation established
 * between source and target. The association details are added to the mount
 * table. The provided name to (target) will serve as future reference when
 * performing file operations.
 *
 * Params:
 *         source - Normally "source" should represent a device file, however,
 *                  since the concept of device files are not yet known by the
 *                  system, we must instead choose a device directly. The source
 *                  must contain  a previously created file system which can be
 *                  created using the SYS_FS_format() command.
 *
 *         target - A pathname to use as an alias for the mount point.
 *         A directory will be created using this name provided. If no directoy
 *         path is provided within pathname, the current working directory will
 *         be used as the root and pathname will be created within the working
 *         directory
 *
 * Return: - if the mount was successfull ERR_NOWORKINGDIRECTORY - If no current
 *           working directory is established. This can occure if a call to
 *           SYS_FS_setdefdir() or SYS_FS_chgdir() has not been successfully
 *           called.
 *           SYS_FS_STATUS_PARAM_ERROR if filesystemtype is not a recognized
 *           supported type
 ******************************************************************************/
int32_t SYS_FS_mount(SYS_FS_MEDIA_TYPE source
               , const char *target
               , const char *filesystemtype
               , SYS_FS_IO_INTENT mountflags
               , const void *data);

//int32_t SYS_FS_unmount(const char *target);

/************************** SYS_FS_FSFromMountNameDEF **************************
 * Description:
 *             Search for mountName in the mount table. Populates FSType with the
 *             file system type (i.e "mpfs2") associated with mountName
 *
 * Params:
 *        mountName - A null terminted string representing a mount name
 *                    previously created by a call to SYS_FS_mount()
 *        FSType - A null terminated string containing the string ID of the
 *        file system of interest.
 * Return: RETURN_SUCCESS
 *         ENOENT if mountName is not found within the mount table
 ******************************************************************************/
int32_t SYS_FS_FSFromMountName(const char *mountName, char *FSType);

/******************************* FS_MountRecGetDEF *****************************
 * Description:
 *            Returns the mount record associated with a mount name
 *
 * Params:
 *        name - A null terminated string representing the name if a mount point
 *               that has been succfully mounted by calling SYS_FS_mount()
 *
 * Return: SYS_FS_DATAFILE if path is a data file or directory
 *         SYS_FS_MOUNT_LABEL if path is a mount name
 ******************************************************************************/
int32_t FS_MountRecGet(const char *name,SYS_FS_MOUNT_REC_t *rec);
//int32_t SYS_FS_format(char mode, long serialNumber, char * mountName);

//Working Directory
/**************************** SYS_FS_setdefdirDEF ******************************
 * Description:
 *          Set the default cur working directory on boot
 * Since this sets the global current working directory for the system, this
 * function must be called at least once after boot and prior to mounting any device.
 * This should be a root priviledged call.
 *
 * Params:
 *        path - Null terminated string to a valid directory path to be used
 *               as the default directory whenever the system boots
 *
 * Return: RETURN_SUCCESS
 *         RETURN_FAILED
 ******************************************************************************/
int32_t SYS_FS_setdefdir(const char *path);

/****************************** SYS_FS_FSFromCWDDEF ****************************
 * Description:   Get the file system type of the current working directory
 *
 * Params:
 *        FSType - A null terminated string containing the string ID of the
 *        file system of interest.
 * Return: RETURN_SUCCESS
 *         RETURN_FAILED - If FSType could not be associate with a mount name
 ******************************************************************************/
int32_t SYS_FS_FSFromCWD(char *FSType);


/****************************** SYS_FS_lseekDEF ****************************
 * Description: sition read/write file offset  
 *
 * Params:
 *         fd - File descriptor returned by the call to SYS_FS_open()
 *         offset - the offset of the open file 
 *         whence - is as follows:
           SEEK_SET: The offset is set to offset bytes. 
 *         SEEK_CUR: The offset is set to its current location plus offset bytes. 
 *         SEEK_END: The offset is set to the size of the file plus offset bytes
 * Return: Upon successful completion, lseek() returns the resulting offset 
 *         location as measured in bytes from the beginning of the file. 
 *         On error, the value (off_t) -1 is returned and errno is set to 
 *         indicate the error. 
 ******************************************************************************/
off_t SYS_FS_lseek(int32_t filedes, off_t offset, int32_t whence);

/************************** SYS_FS_IsBadStringDEF **************************
 * Description:
 * Checks the filepath pointer to make sure it is not null.
 * Checks the string filepath to make sure that it is not null
 * Checks for a null terminator
 * Params:
 *        str - A pointer to memory to validate the string properties.
 *        len - Len of memory pointed to by str
 * Return:
 *         1 = Pointer is NULL
 *         2 = String is empty
 *         3 = No terminator found within len bytes from start of str
 *  ***************************************************************************/
int SYS_FS_IsBadString(const char *str,int len);

#endif
