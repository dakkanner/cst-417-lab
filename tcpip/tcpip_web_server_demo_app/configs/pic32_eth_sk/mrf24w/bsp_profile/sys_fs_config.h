/*******************************************************************************

  Summary:

  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  sys_fs_config.h
Copyright ?2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED ?AS IS?WITHOUT WARRANTY OF ANY KIND,
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

#ifndef __SYS_FS_CONFIG_H_
#define __SYS_FS_CONFIG_H_


/*****************************************************************************
 * The system MPFS file system module
 * Use to enable the system MPFS file system functionality.
 *****************************************************************************/
#define SYS_FS_FILESYSTEM_MPFS

/*****************************************************************************
 * The system MDD file system module
 * Use to enable the system MDD file system functionality.
 *****************************************************************************/
//#define SYS_FS_FILESYSTEM_MDD


/*****************************************************************************
 * The file seperator used in directory paths.
 *****************************************************************************/
#define SYS_FS_CHAR '/' // File seperator
#define SYS_FS_STRING "/"

#ifndef SYS_FS_MAX_DIR
#define SYS_FS_MAX_DIR   235 //Maximum length of directory component
#endif
#ifndef SYS_FS_MAX_FNAME
#define SYS_FS_MAX_FNAME 8   //Maximum length of filename component
#endif
#ifndef SYS_FS_MAX_DRIVE
#define SYS_FS_MAX_DRIVE SYS_FS_MAX_FNAME   //Maximum length of a drive, volume name or mount point
#endif
#ifndef SYS_FS_MAX_EXT
#define SYS_FS_MAX_EXT   4   //Maximum length of extension component (including '.')
#endif
#ifndef SYS_FS_MAX_PATH
#define SYS_FS_MAX_PATH 256  //Maximum length of full path (components above not to exceed)
#endif
#ifndef SYS_FS_MAX_DESCRIPTORS
#define SYS_FS_MAX_DESCRIPTORS 1000 // Per file system
#endif

#define SYS_FS_MOUNT_POINT "MyDrive1"
#define SYS_FS_DEFAULT_FS "mpfs2"
#define SYS_FS_DEFAULT_MEDIA DRV_MEDIA_INTERNAL_FLASH

#endif	// __SYS_FS_CONFIG_H_

