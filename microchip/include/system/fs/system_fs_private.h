#ifndef SYSTEM_FS_PRIVATE
#define SYSTEM_FS_PRIVATE

#include <system/fs/mpfs2.h>


typedef enum
{
    _SYS_FS_MASK_MPFS   = 0x01,
    _SYS_FS_MASK_MDD    = 0x02,
    // add some other supported types here

}_SYS_FS_TYPE_MASK; // associated mask for each of the supported file systems



bool        SYS_FS_mpfs2fs_Init(void);
void        SYS_FS_mpfs2fs_Deinit(void);
MPFS_HANDLE SYS_FS_mpfs2fs_Open(const char* cFile);
MPFS_HANDLE SYS_FS_mpfs2fs_OpenID(uint16_t hFatID);
void        SYS_FS_mpfs2fs_Close(MPFS_HANDLE hMPFS);
bool        SYS_FS_mpfs2fs_Get(MPFS_HANDLE hMPFS, uint8_t* c);
uint32_t    SYS_FS_mpfs2fs_GetArray(MPFS_HANDLE hMPFS, uint8_t* cData, uint32_t wLen);
bool        SYS_FS_mpfs2fs_GetLong(MPFS_HANDLE hMPFS, uint32_t* ul);
bool        SYS_FS_mpfs2fs_Seek(MPFS_HANDLE hMPFS, uint32_t dwOffset, MPFS_SEEK_MODE tMode);
MPFS_HANDLE SYS_FS_mpfs2fs_Format(void);
uint32_t    SYS_FS_mpfs2fs_PutArray(MPFS_HANDLE hMPFS, uint8_t* cData, uint32_t wLen);
bool        SYS_FS_mpfs2fs_PutEnd(MPFS_HANDLE hMPFS, bool final);
uint32_t    SYS_FS_mpfs2fs_GetTimestamp(MPFS_HANDLE hMPFS);
uint32_t    SYS_FS_mpfs2fs_GetMicrotime(MPFS_HANDLE hMPFS);
uint16_t    SYS_FS_mpfs2fs_GetFlags(MPFS_HANDLE hMPFS);
uint32_t    SYS_FS_mpfs2fs_GetSize(MPFS_HANDLE hMPFS);
uint32_t    SYS_FS_mpfs2fs_GetBytesRem(MPFS_HANDLE hMPFS);
MPFS_PTR    SYS_FS_mpfs2fs_GetStartAddr(MPFS_HANDLE hMPFS);
MPFS_PTR    SYS_FS_mpfs2fs_GetEndAddr(MPFS_HANDLE hMPFS);
bool        SYS_FS_mpfs2fs_GetFilename(MPFS_HANDLE hMPFS, uint8_t* cName, uint16_t wLen);
uint32_t    SYS_FS_mpfs2fs_GetPosition(MPFS_HANDLE hMPFS);
uint16_t    SYS_FS_mpfs2fs_GetID(MPFS_HANDLE hMPFS);



typedef struct
{
  char mount_name[SYS_FS_MAX_DRIVE+1]; // The name of the mount point
  char mount_path[SYS_FS_MAX_PATH+1];  // The full path of the mount point
  DRV_MEDIA_TYPE dev;
  char cpFileSystemType[9]; // 8 chars plus null term
}SYS_FS_MOUNT_REC_t;

typedef struct
{
  int32_t *fd;  // A pointer to a block of memory representing descriptors assigned by the underlying file system
  int fsIndex;  // Next available fd record
  char fsName[8];
} SYS_FS_FILE_DESC_t;

typedef struct
{
   int nFileSystems;
   SYS_FS_FILE_DESC_t **ppfs;
}SYS_FS_t;


#endif
