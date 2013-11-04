/*
 * ImgTec Debug Adapter (DA) FS op driver
 *
 * Copyright (C) 2013 Imagination Technologies Ltd.
 *
 * Authors:
 *  Paul Burton <paul.burton@imgtec.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

#include "cpu.h"
#include "meta_damisc.h"
#include "meta_switch.h"
#include "sysemu.h"
#include <glob.h>

#define DEBUG_LEVEL 0

#if DEBUG_LEVEL >= 2
#  define DAFSLOG(...) fprintf(stderr, ## __VA_ARGS__)
#elif DEBUG_LEVEL >= 1
#  define DAFSLOG(...) qemu_log(__VA_ARGS__)
#else
#  define DAFSLOG(...)
#endif

typedef struct MetaDAFSState {
    MetaSwitchDevice dev;

    char *root;
    char *cwd;
    uint8_t sandbox;
    uint8_t noexec;
} MetaDAFSState;

typedef struct DAFSFile {
    int fd;
    uint8_t affinity;
} DAFSFile;

typedef struct DAFSDir {
    glob_t globs;
    uint32_t idx;
} DAFSDir;

typedef struct DAFSRet {
    int32_t ret;
    int32_t err;
} DAFSRet;

static struct {
    enum {
        DAFS_AVAIL = 0,
        DAFS_FILE  = 1,
        DAFS_DIR   = 2,
    } type;

    union {
        DAFSFile *file;
        DAFSDir *dir;
        void *ptr;
    };
} dafs_handles[512];

static DAFSFile dafs_stdio[3];

/* SWITCH subgroup 0 operations */
enum {
    OP_OPEN      = 0,
    OP_CREAT     = 1,
    OP_READ      = 2,
    OP_WRITE     = 3,
    OP_CLOSE     = 4,
    OP_LINK      = 5,
    OP_LSEEK     = 6,
    OP_UNLINK    = 7,
    OP_ISATTY    = 8,
    OP_FCNTL     = 9,
    OP_STAT      = 10,
    OP_FSTAT     = 11,
    OP_GETCWD    = 12,
    OP_CHDIR     = 13,
    OP_MKDIR     = 14,
    OP_RMDIR     = 15,
    OP_FINDFIRST = 16,
    OP_FINDNEXT  = 17,
    OP_FINDCLOSE = 18,
    OP_CHMOD     = 19,
    OP_PREAD     = 20,
    OP_PWRITE    = 21,
};

/* open flags */
enum {
    DAFS_O_RDONLY  = 0x0000,
    DAFS_O_WRONLY  = 0x0001,
    DAFS_O_RDWR    = 0x0002,
    DAFS_O_APPEND  = 0x0008,
    DAFS_O_CREAT   = 0x0200,
    DAFS_O_TRUNC   = 0x0400,
    DAFS_O_EXCL    = 0x0800,

#define DAFS_O_AFFINITY_SHIFT 16
    DAFS_O_AFFINITY_T0 = 0x10000,
    DAFS_O_AFFINITY_T1 = 0x20000,
    DAFS_O_AFFINITY_T2 = 0x40000,
    DAFS_O_AFFINITY_T3 = 0x80000,
};

/* findnext attributes */
enum {
    DAFS_A_SUBDIR  = 0x10,
};

static int open_flags_dafs_to_host(MetaDAFSState *dafs, uint32_t f)
{
    int ret = 0;
    ret |= (f & DAFS_O_WRONLY) ? O_WRONLY : 0;
    ret |= (f & DAFS_O_RDWR)   ? O_RDWR   : 0;
    ret |= (f & DAFS_O_APPEND) ? O_APPEND : 0;
    ret |= (f & DAFS_O_CREAT)  ? O_CREAT  : 0;
    ret |= (f & DAFS_O_TRUNC)  ? O_TRUNC  : 0;
    ret |= (f & DAFS_O_EXCL)   ? O_EXCL   : 0;
    return ret;
}

static mode_t file_mode_dafs_to_host(MetaDAFSState *dafs, uint32_t m)
{
    mode_t mode = m;

    if (dafs->noexec) {
        m &= ~(S_IXUSR | S_IXGRP | S_IXOTH);
    }

    return mode;
}

static DAFSRet stat_to_target(CPUArchState *env,
                              target_ulong pstat, struct stat *st)
{
    DAFSRet ret = { -1, EFAULT };
    struct {
        int16_t _st_dev;
        uint16_t _st_ino;
        uint32_t _st_mode;
        uint16_t _st_nlink;
        uint16_t _st_uid;
        uint16_t _st_gid;
        int16_t _st_rdev;
        int32_t _st_size;
        int32_t _st_atime;
        int32_t _st_spare1;
        int32_t _st_mtime;
        int32_t _st_spare2;
        int32_t _st_ctime;
        int32_t _st_spare3;
        int32_t _st_blksize;
        int32_t _st_blocks;
        int32_t _st_spare4[2];
    } st_dafs;

    memset(&st_dafs, 0, sizeof(st_dafs));
    st_dafs._st_dev = cpu_to_le16(st->st_dev);
    st_dafs._st_ino = cpu_to_le16(st->st_ino);
    st_dafs._st_mode = cpu_to_le32(st->st_mode);
    st_dafs._st_nlink = cpu_to_le16(st->st_nlink);
    st_dafs._st_uid = cpu_to_le16(st->st_uid);
    st_dafs._st_gid = cpu_to_le16(st->st_gid);
    st_dafs._st_rdev = cpu_to_le16(st->st_rdev);
    st_dafs._st_size = cpu_to_le32(st->st_size);
    st_dafs._st_atime = cpu_to_le32(st->st_atime);
    st_dafs._st_mtime = cpu_to_le32(st->st_mtime);
    st_dafs._st_ctime = cpu_to_le32(st->st_ctime);
    st_dafs._st_blksize = cpu_to_le32(st->st_blksize);
    st_dafs._st_blocks = cpu_to_le32(st->st_blocks);

    if (cpu_memory_rw_debug(env, pstat, (uint8_t *)&st_dafs,
                            sizeof(st_dafs), 1) < 0) {
        ret.ret = -1;
        ret.err = EFAULT;
    } else {
        ret.ret = ret.err = 0;
    }

    return ret;
}

static DAFSRet finddata_to_target(CPUArchState *env, target_ulong pdata,
                                  uint32_t sz, uint32_t attrib,
                                  const char *name)
{
    DAFSRet ret = { -1, EFAULT };
    struct {
        uint32_t size;
        uint32_t attrib;
        char name[260];
    } data;

    data.size = sz;
    data.attrib = attrib;
    strncpy(data.name, name, sizeof(data.name));

    /* write it to memory */
    if (cpu_memory_rw_debug(env, pdata, (uint8_t *)&data,
                            sizeof(data), 1) < 0) {
        goto out;
    }
    ret.ret = ret.err = 0;
out:
    return ret;
}

enum {
    PATH_NOCWD = (1 << 0),
};

static char *sanitise_path(MetaDAFSState *dafs, const char *path, int flags)
{
    const char *paths[3] = { dafs->root, dafs->cwd, path };
    gchar **path_cmps[3] = { NULL };
    gchar *components[256];
    gchar *root = NULL, *ret = NULL;
    int p, idx = 0, reldepth = 0;

    if (flags & PATH_NOCWD) {
        paths[1] = NULL;
    }

    for (p = 0; p < ARRAY_SIZE(paths); p++) {
        const gchar *post_root;
        gchar **curr;

        if (!paths[p]) {
            continue;
        }

        /* look for root if this is dafs->root, or we're not sandboxing */
        if ((!p || !dafs->sandbox) && !root) {
            post_root = g_path_skip_root(paths[p]);
        } else {
            post_root = NULL;
        }

        if (post_root) {
            /* path is absolute, set first component to root */
            root = g_strndup(paths[p], post_root - paths[p]);
            components[idx++] = root;
        } else {
            /* path is not absolute */
            post_root = paths[p];
        }

        /* append each component */
        path_cmps[p] = g_strsplit(post_root, G_DIR_SEPARATOR_S, 256);
        for (curr = path_cmps[p];
             *curr && (idx < ARRAY_SIZE(components));
             curr++) {
            /* handle relative paths if not dafs->root */
            if (p) {
                if (!strcmp(*curr, ".") && idx) {
                    /* current directory, just skip it */
                    continue;
                }
                if (!strcmp(*curr, "..")) {
                    /* parent directory */
                    if (idx <= 0) {
                        /* outside of root */
                        components[idx++] = *curr;
                    } else {
                        /* remove the last component */
                        idx--;
                    }
                    if ((--reldepth < 0) && dafs->sandbox) {
                        /* outside of the sandbox */
                        goto out;
                    }
                    continue;
                }
            }
            components[idx++] = *curr;
            reldepth += !!p;
        }
    }

    if (idx >= ARRAY_SIZE(components)) {
        fprintf(stderr, "too many components in path '%s' '%s'\n",
                dafs->root, path);
        goto out;
    }

    /* combine components */
    components[idx] = NULL;
    ret = g_build_pathv(G_DIR_SEPARATOR_S, components);

out:
    if (root) {
        g_free(root);
    }
    for (p = 0; p < ARRAY_SIZE(paths); p++) {
        if (paths[p]) {
            g_strfreev(path_cmps[p]);
        }
    }
#if DEBUG_LEVEL >= 3
    DAFSLOG("dafs path \"%s\" -> \"%s\"\n", path, ret);
#endif
    return ret;
}

static int available_handle(void)
{
    int hnd;
    for (hnd = 0; hnd < ARRAY_SIZE(dafs_handles); hnd++) {
        if (dafs_handles[hnd].type != DAFS_AVAIL) {
            continue;
        }
        return hnd;
    }
    return -1;
}

static void free_handle(MetaDAFSState *dafs, int hnd)
{
    if (hnd >= ARRAY_SIZE(dafs_handles)) {
        return;
    }
    assert(hnd > 2);
    g_free(dafs_handles[hnd].ptr);
    dafs_handles[hnd].type = DAFS_AVAIL;
}

static DAFSFile *lookup_file(MetaDAFSState *dafs, CPUArchState *env, int hnd)
{
    DAFSFile *file = NULL;

    /* lookup file by its descriptor */
    if ((hnd < ARRAY_SIZE(dafs_handles)) &&
        (dafs_handles[hnd].type == DAFS_FILE)) {
        file = dafs_handles[hnd].file;
    }

    /* check affinity includes this thread */
    if (file &&
        !(file->affinity & (1 << env->thread_num))) {
        file = NULL;
    }

    return file;
}

static DAFSDir *lookup_dir(MetaDAFSState *dafs, CPUArchState *env, int hnd)
{
    if ((hnd < ARRAY_SIZE(dafs_handles)) &&
        (dafs_handles[hnd].type == DAFS_DIR)) {
        return dafs_handles[hnd].dir;
    }
    return NULL;
}

static DAFSFile *new_file(MetaDAFSState *dafs, int *phnd)
{
    DAFSFile *file;
    int hnd = available_handle();

    if (hnd == -1) {
        return NULL;
    }
    file = g_malloc0(sizeof(*file));
    dafs_handles[hnd].type = DAFS_FILE;
    dafs_handles[hnd].file = file;
    if (phnd) {
        *phnd = hnd;
    }
    return file;
}

static DAFSDir *new_dir(MetaDAFSState *dafs, int *phnd)
{
    DAFSDir *dir;
    int hnd = available_handle();

    if (hnd == -1) {
        return NULL;
    }
    dir = g_malloc0(sizeof(*dir));
    dafs_handles[hnd].type = DAFS_DIR;
    dafs_handles[hnd].dir = dir;
    if (phnd) {
        *phnd = hnd;
    }
    return dir;
}

static DAFSRet meta_dafs_op_open(MetaDAFSState *dafs, CPUArchState *env,
                                 const char *path_unsafe,
                                 uint32_t flags_dafs, uint32_t mode_dafs)
{
    char *path = sanitise_path(dafs, path_unsafe, 0);
    DAFSFile *file;
    int flags = open_flags_dafs_to_host(dafs, flags_dafs);
    mode_t mode = file_mode_dafs_to_host(dafs, mode_dafs);
    int fd_dafs = -1;
    DAFSRet ret = { -1, ENOENT };

    /* check the path is valid */
    if (!path) {
        goto out;
    }

    /* find an unused file struct */
    file = new_file(dafs, &fd_dafs);
    if (!file) {
        fprintf(stderr, "DAFS reached maximum file count\n");
        goto out;
    }

    /* set file thread affinity */
    file->affinity = (flags >> DAFS_O_AFFINITY_SHIFT) & 0xf;
    if (!file->affinity) {
        file->affinity = 1 << env->thread_num;
    }

    /* open the file */
    file->fd = open(path, flags, mode);

    /* setup return */
    if (file->fd == -1) {
        ret.ret = -1;
        ret.err = errno;
    } else {
        ret.ret = fd_dafs;
    }
out:
    g_free(path);
    DAFSLOG("dafs open(\"%s\",0x%x,0x%x) = { %d, %d }\n",
            path_unsafe, flags_dafs, mode_dafs, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_creat(MetaDAFSState *dafs, CPUArchState *env,
                                  const char *path_unsafe,
                                  uint32_t mode_dafs)
{
    char *path = sanitise_path(dafs, path_unsafe, 0);
    mode_t mode = file_mode_dafs_to_host(dafs, mode_dafs);
    DAFSRet ret = { -1, ENOENT };
    int fd_dafs = -1;
    DAFSFile *file;

    /* check the path is valid */
    if (!path) {
        goto out;
    }

    /* find an unused file struct */
    file = new_file(dafs, &fd_dafs);
    if (!file) {
        fprintf(stderr, "DAFS reached maximum file count\n");
        goto out;
    }

    /* set file thread affinity */
    file->affinity = 1 << env->thread_num;

    /* open the file */
    file->fd = creat(path, mode);

    /* setup return */
    if (file->fd == -1) {
        ret.ret = -1;
        ret.err = errno;
    } else {
        ret.ret = fd_dafs;
    }
out:
    g_free(path);
    DAFSLOG("dafs creat(\"%s\",0x%x) = { %d, %d }\n",
            path_unsafe, mode_dafs, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_read(MetaDAFSState *dafs, CPUArchState *env,
                                 int32_t fd, target_ulong pbuf, uint32_t sz)
{
    DAFSFile *file = lookup_file(dafs, env, fd);
    DAFSRet ret = { -1, EBADF };
    MetaSwitchDevice *damisc = NULL;
    target_ulong pcurr = pbuf;
    uint8_t buf[1024];
    size_t curr_sz, rem = sz;
    ssize_t nread;

    /* stdin */
    if (fd == 0) {
        damisc = meta_switch_sibling(&dafs->dev, env, META_SWITCH_MISC);
    }

    /* ensure a valid file */
    if (!file) {
        goto out;
    }

    ret.ret = ret.err = 0;
    while (rem) {
        curr_sz = MIN(rem, sizeof(buf));

        if (damisc) {
            /* read from channel 1 for stdin */
            nread = meta_damisc_chan_read(damisc, env, 1, buf, curr_sz);
        } else {
            /* read from the file */
            nread = read(file->fd, buf, curr_sz);
        }

        if (!nread) {
            break;
        } else if (damisc && (nread < 0)) {
            ret.ret = -1;
            ret.err = meta_damisc_chan_errno(-nread);
            break;
        } else if (nread == -1) {
            ret.ret = -1;
            ret.err = errno;
            break;
        }

        /* write data to memory */
        if (cpu_memory_rw_debug(env, pcurr, buf, nread, 1) < 0) {
            ret.ret = -1;
            ret.err = EFAULT;
            break;
        }

        /* advance */
        rem -= nread;
        pcurr += nread;
        ret.ret += nread;
    }
out:
    DAFSLOG("dafs read(%d,0x%08x,0x%x) = { %d, %d }\n",
            fd, pbuf, sz, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_write(MetaDAFSState *dafs, CPUArchState *env,
                                  int32_t fd,
                                  target_ulong pbuf, uint32_t sz)
{
    DAFSFile *file = lookup_file(dafs, env, fd);
    DAFSRet ret = { -1, EBADF };
    MetaSwitchDevice *damisc = NULL;
    target_ulong pcurr = pbuf;
    uint8_t buf[1024], *curr_buf;
    size_t curr_sz, curr_rem, rem = sz;
    ssize_t nwrote;

    /* stdout, stderr */
    if (fd == 1 || fd == 2) {
        damisc = meta_switch_sibling(&dafs->dev, env, META_SWITCH_MISC);
    }

    /* ensure a valid file */
    if (!file) {
        goto out;
    }

    ret.ret = ret.err = 0;
    while (rem) {
        curr_sz = MIN(rem, sizeof(buf));

        /* read data from memory */
        if (cpu_memory_rw_debug(env, pcurr, buf, curr_sz, 0) < 0) {
            ret.ret = -1;
            ret.err = EFAULT;
            break;
        }

        /* write to the file */
        curr_rem = curr_sz;
        curr_buf = buf;
        while (curr_rem) {
            if (damisc) {
                nwrote = meta_damisc_chan_write(damisc, env, 1, curr_buf, curr_sz);
            } else {
                nwrote = write(file->fd, curr_buf, curr_sz);
            }

            if (nwrote < 0) {
                ret.ret = -1;
                if (damisc) {
                    ret.err = meta_damisc_chan_errno(-nwrote);
                } else {
                    ret.err = errno;
                }
                break;
            }

            curr_rem -= nwrote;
            curr_buf += nwrote;
        }

        /* advance */
        rem -= curr_sz;
        pcurr += curr_sz;
        ret.ret += curr_sz;
    }

out:
    DAFSLOG("dafs write(%d,0x%08x,0x%x) = { %d, %d }\n",
            fd, pbuf, sz, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_close(MetaDAFSState *dafs, CPUArchState *env,
                                  int32_t fd)
{
    DAFSFile *file = lookup_file(dafs, env, fd);
    DAFSRet ret = { -1, EBADF };

    /* ensure a valid file, not stdio */
    if (!file || fd < 3) {
        goto out;
    }

    ret.ret = close(file->fd);
    if (ret.ret) {
        ret.err = errno;
    } else {
        free_handle(dafs, fd);
        ret.err = 0;
    }
out:
    DAFSLOG("dafs close(%d) = { %d, %d }\n",
            fd, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_link(MetaDAFSState *dafs, CPUArchState *env,
                                 const char *path1_unsafe,
                                 const char *path2_unsafe)
{
    char *path1 = sanitise_path(dafs, path1_unsafe, 0);
    char *path2 = sanitise_path(dafs, path2_unsafe, 0);
    DAFSRet ret = { -1, ENOENT };

    /* check the paths are valid */
    if (!path1 || !path2) {
        goto out;
    }

    ret.ret = link(path1, path2);
    ret.err = ret.ret ? errno : 0;
out:
    g_free(path1);
    g_free(path2);
    DAFSLOG("dafs link(\"%s\",\"%s\") = { %d, %d }\n",
            path1_unsafe, path2_unsafe, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_lseek(MetaDAFSState *dafs, CPUArchState *env,
                                  int32_t fd, uint32_t off, uint32_t whence)
{
    DAFSFile *file = lookup_file(dafs, env, fd);
    DAFSRet ret = { -1, EBADF };

    /* ensure a valid file */
    if (!file) {
        goto out;
    }

    ret.ret = lseek(fd, off, whence);
    ret.err = (ret.ret == -1) ? errno : 0;
out:
    DAFSLOG("dafs lseek(%d,0x%x,0x%x) = { %d, %d }\n",
            fd, off, whence, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_unlink(MetaDAFSState *dafs, CPUArchState *env,
                                   const char *path_unsafe)
{
    char *path = sanitise_path(dafs, path_unsafe, 0);
    DAFSRet ret = { -1, ENOENT };

    /* check the path is valid */
    if (!path) {
        goto out;
    }

    ret.ret = unlink(path);
    ret.err = ret.ret ? errno : 0;
out:
    g_free(path);
    DAFSLOG("dafs unlink(\"%s\") = { %d, %d }\n",
            path_unsafe, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_isatty(MetaDAFSState *dafs, CPUArchState *env,
                                   int32_t fd)
{
    DAFSFile *file = lookup_file(dafs, env, fd);
    DAFSRet ret = { -1, EBADF };

    /* ensure a valid file */
    if (!file) {
        goto out;
    }

    ret.ret = isatty(file->fd);
    ret.err = ret.ret ? 0 : errno;
out:
    DAFSLOG("dafs isatty(%d) = { %d, %d }\n",
            fd, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_stat(MetaDAFSState *dafs, CPUArchState *env,
                                 const char *path_unsafe, target_ulong pstat)
{
    char *path = sanitise_path(dafs, path_unsafe, 0);
    DAFSRet ret = { -1, ENOENT };
    struct stat st;

    /* check the path is valid */
    if (!path) {
        goto out;
    }

    /* perform the stat call */
    ret.ret = stat(path, &st);
    if (ret.ret) {
        ret.err = errno;
        goto out;
    }

    /* translate & write to target */
    ret = stat_to_target(env, pstat, &st);
out:
    g_free(path);
    DAFSLOG("dafs stat(\"%s\",0x%08x) = { %d, %d }\n",
            path_unsafe, pstat, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_fstat(MetaDAFSState *dafs, CPUArchState *env,
                                  int32_t fd, target_ulong pstat)
{
    DAFSFile *file = lookup_file(dafs, env, fd);
    DAFSRet ret = { -1, EBADF };
    struct stat st;

    /* ensure a valid file */
    if (!file) {
        goto out;
    }

    /* perform the stat call */
    ret.ret = fstat(file->fd, &st);
    if (ret.ret) {
        ret.err = errno;
        goto out;
    }

    /* translate & write to target */
    ret = stat_to_target(env, pstat, &st);
out:
    DAFSLOG("dafs fstat(%d,0x%08x) = { %d, %d }\n",
            fd, pstat, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_getcwd(MetaDAFSState *dafs, CPUArchState *env,
                                   target_ulong pbuf, uint32_t sz)
{
    DAFSRet ret = { -1, ENOMEM };
    uint8_t *buf = NULL;

    /* allocate buf */
    buf = g_malloc(sz);
    if (!buf) {
        goto out;
    }

    /* get the current directory */
    if (!dafs->cwd) {
        strncpy((char *)buf, ".", sz);
    } else {
        strncpy((char *)buf, dafs->cwd, sz);
    }

    /* write the buffer to memory */
    if (cpu_memory_rw_debug(env, pbuf, buf, sz, 1) < 0) {
        ret.ret = -1;
        ret.err = EFAULT;
        goto out;
    }

    ret.ret = pbuf;
    ret.err = 0;
out:
    g_free(buf);
    DAFSLOG("dafs getcwd(0x%08x,0x%x) = { %d, %d }\n",
            pbuf, sz, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_chdir(MetaDAFSState *dafs, CPUArchState *env,
                                  const char *path_unsafe)
{
    char *path = sanitise_path(dafs, path_unsafe, PATH_NOCWD);
    DAFSRet ret = { -1, ENOENT };

    /* check the path is valid */
    if (!path) {
        goto out;
    }

    /* set cwd */
    if (dafs->cwd) {
        g_free(dafs->cwd);
    }
    dafs->cwd = path;
    ret.ret = ret.err = 0;
out:
    g_free(path);
    DAFSLOG("dafs chdir(\"%s\") = { %d, %d }\n",
            path_unsafe, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_mkdir(MetaDAFSState *dafs, CPUArchState *env,
                                  const char *path_unsafe, uint32_t mode_dafs)
{
    char *path = sanitise_path(dafs, path_unsafe, 0);
    mode_t mode = file_mode_dafs_to_host(dafs, mode_dafs);
    DAFSRet ret = { -1, ENOENT };

    /* check the path is valid */
    if (!path) {
        goto out;
    }

    ret.ret = mkdir(path, mode);
    ret.err = ret.ret ? errno : 0;
out:
    g_free(path);
    DAFSLOG("dafs mkdir(\"%s\",0x%x) = { %d, %d }\n",
            path_unsafe, mode_dafs, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_rmdir(MetaDAFSState *dafs, CPUArchState *env,
                                  const char *path_unsafe)
{
    char *path = sanitise_path(dafs, path_unsafe, 0);
    DAFSRet ret = { -1, ENOENT };

    /* check the path is valid */
    if (!path) {
        goto out;
    }

    ret.ret = rmdir(path);
    ret.err = ret.ret ? errno : 0;
out:
    g_free(path);
    DAFSLOG("dafs rmdir(\"%s\") = { %d, %d }\n",
            path_unsafe, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_findfirst(MetaDAFSState *dafs, CPUArchState *env,
                                      const char *path_unsafe,
                                      target_ulong pdata)
{
    char *path = sanitise_path(dafs, path_unsafe, 0);
    DAFSRet ret = { -1, ENOENT };
    int hnd = -1;
    DAFSDir *dir;

    /* check the path is valid */
    if (!path) {
        goto out;
    }

    /* open the directory */
    dir = new_dir(dafs, &hnd);
    if (!dir) {
        goto out;
    }

    /* find matching files */
    ret.ret = glob(path, GLOB_ERR, NULL, &dir->globs);

    /* write finddata */
    if (!ret.ret) {
        ret = finddata_to_target(env, pdata, 0, 0, "");
    }

    /* cleanup on error */
    if (ret.ret) {
        globfree(&dir->globs);
        free_handle(dafs, hnd);
        goto out;
    }

    ret.ret = hnd;
    ret.err = 0;
out:
    g_free(path);
    DAFSLOG("dafs findfirst(\"%s\",0x%08x) = { %d, %d }\n",
            path_unsafe, pdata, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_findnext(MetaDAFSState *dafs, CPUArchState *env,
                                     int32_t hnd, target_ulong pdata)
{
    DAFSDir *dir = lookup_dir(dafs, env, hnd);
    DAFSRet ret = { -1, EBADF };
    struct stat st;
    const char *name;
    char *fname = NULL;
    uint32_t sz, attrib = 0;

    /* check handle is valid */
    if (!dir) {
        goto out;
    }

    /* check there is a next entry */
    if ((dir->idx >= dir->globs.gl_pathc) ||
        !dir->globs.gl_pathv[dir->idx]) {
        ret.err = ENOENT;
        goto out;
    }

    /* next entry */
    name = dir->globs.gl_pathv[dir->idx++];
    if (!stat(name, &st)) {
        sz = st.st_size;
        if (st.st_mode & S_IFDIR) {
            attrib |= DAFS_A_SUBDIR;
        }
    } else {
        sz = 0;
    }

    /* only filename */
    fname = g_path_get_basename(name);

    /* write finddata */
    ret = finddata_to_target(env, pdata, sz, attrib, fname);
out:
    g_free(fname);
    DAFSLOG("dafs findnext(%d,0x%08x) = { %d, %d }\n",
            hnd, pdata, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_findclose(MetaDAFSState *dafs, CPUArchState *env,
                                      int32_t hnd)
{
    DAFSDir *dir = lookup_dir(dafs, env, hnd);
    DAFSRet ret = { -1, EBADF };

    /* check handle is valid, not stdio */
    if (!dir || hnd < 3) {
        goto out;
    }

    /* close */
    globfree(&dir->globs);
    free_handle(dafs, hnd);
    ret.ret = ret.err = 0;
out:
    DAFSLOG("dafs findclose(%d) = { %d, %d }\n",
            hnd, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_chmod(MetaDAFSState *dafs, CPUArchState *env,
                                  const char *path_unsafe, uint32_t mode_dafs)
{
    char *path = sanitise_path(dafs, path_unsafe, 0);
    mode_t mode = file_mode_dafs_to_host(dafs, mode_dafs);
    DAFSRet ret = { -1, ENOENT };

    /* check the path is valid */
    if (!path) {
        goto out;
    }

    ret.ret = chmod(path, mode);
    ret.err = ret.ret ? errno : 0;
out:
    g_free(path);
    DAFSLOG("dafs chmod(\"%s\",0x%x) = { %d, %d }\n",
            path_unsafe, mode_dafs, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_pread(MetaDAFSState *dafs, CPUArchState *env,
                                  int32_t fd, target_ulong pbuf,
                                  uint32_t sz, uint32_t off)
{
    DAFSFile *file = lookup_file(dafs, env, fd);
    DAFSRet ret = { -1, EBADF };
    MetaSwitchDevice *damisc = NULL;
    target_ulong pcurr = pbuf;
    uint8_t buf[1024];
    size_t curr_sz, rem = sz;
    off_t curr_off = off;
    ssize_t nread;

    /* stdin */
    if (fd == 0) {
        damisc = meta_switch_sibling(&dafs->dev, env, META_SWITCH_MISC);
    }

    /* ensure a valid file */
    if (!file) {
        goto out;
    }

    ret.ret = ret.err = 0;
    while (rem) {
        curr_sz = MIN(rem, sizeof(buf));

        if (damisc) {
            /* read from channel 1 for stdin */
            nread = meta_damisc_chan_read(damisc, env, 1, buf, curr_sz);
        } else {
            /* read from the file */
            nread = pread(file->fd, buf, curr_sz, curr_off);
        }

        if (!nread) {
            break;
        } else if (damisc && (nread < 0)) {
            ret.ret = -1;
            ret.err = meta_damisc_chan_errno(-nread);
            break;
        } else if (nread == -1) {
            ret.ret = -1;
            ret.err = errno;
            break;
        }

        /* write data to memory */
        if (cpu_memory_rw_debug(env, pcurr, buf, nread, 1) < 0) {
            ret.ret = -1;
            ret.err = EFAULT;
            break;
        }

        /* advance */
        rem -= nread;
        pcurr += nread;
        curr_off += nread;
        ret.ret += nread;
    }
out:
    DAFSLOG("dafs pread(%d,0x%08x,0x%x,0x%x) = { %d, %d }\n",
            fd, pbuf, sz, off, ret.ret, ret.err);
    return ret;
}

static DAFSRet meta_dafs_op_pwrite(MetaDAFSState *dafs, CPUArchState *env,
                                   int32_t fd, target_ulong pbuf,
                                   uint32_t sz, uint32_t off)
{
    DAFSFile *file = lookup_file(dafs, env, fd);
    DAFSRet ret = { -1, EBADF };
    MetaSwitchDevice *damisc = NULL;
    target_ulong pcurr = pbuf;
    uint8_t buf[1024], *curr_buf;
    size_t curr_sz, curr_rem, rem = sz;
    off_t curr_off = off;
    ssize_t nwrote;

    /* stdout, stderr */
    if (fd == 1 || fd == 2) {
        damisc = meta_switch_sibling(&dafs->dev, env, META_SWITCH_MISC);
    }

    /* ensure a valid file */
    if (!file) {
        goto out;
    }

    ret.ret = ret.err = 0;
    while (rem) {
        curr_sz = MIN(rem, sizeof(buf));

        /* read data from memory */
        if (cpu_memory_rw_debug(env, pcurr, buf, curr_sz, 0) < 0) {
            ret.ret = -1;
            ret.err = EFAULT;
            break;
        }

        /* write to the file */
        curr_rem = curr_sz;
        curr_buf = buf;
        while (curr_rem) {
            if (damisc) {
                nwrote = meta_damisc_chan_write(damisc, env, 1, curr_buf, curr_sz);
            } else {
                nwrote = pwrite(file->fd, curr_buf, curr_sz, curr_off);
            }

            if (nwrote < 0) {
                ret.ret = -1;
                if (damisc) {
                    ret.err = meta_damisc_chan_errno(-nwrote);
                } else {
                    ret.err = errno;
                }
                break;
            }

            curr_rem -= nwrote;
            curr_buf += nwrote;
            curr_off += nwrote;
        }

        /* advance */
        rem -= curr_sz;
        pcurr += curr_sz;
        ret.ret += curr_sz;
    }

out:
    DAFSLOG("dafs pwrite(%d,0x%08x,0x%x,0x%x) = { %d, %d }\n",
            fd, pbuf, sz, off, ret.ret, ret.err);
    return ret;
}

static int meta_dafs_handler(MetaSwitchDevice *dev, CPUArchState *env,
                             const uint32_t *args, int nargs,
                             uint32_t *ret, int nret)
{
    MetaDAFSState *dafs = DO_UPCAST(MetaDAFSState, dev, dev);
    DAFSRet op_ret = { -1, EINVAL };

    if (nargs != 6 || nret != 2) {
        return -1;
    }

    switch (args[5]) {
    case OP_OPEN: {
            char *path = meta_switch_arg_string(env, args[0], PATH_MAX);
            uint32_t flags = args[1];
            uint32_t mode = args[2];
            if (path) {
                op_ret = meta_dafs_op_open(dafs, env, path, flags, mode);
                g_free(path);
            }
            break;
        }

    case OP_CREAT: {
            char *path = meta_switch_arg_string(env, args[0], PATH_MAX);
            uint32_t mode = args[1];
            if (path) {
                op_ret = meta_dafs_op_creat(dafs, env, path, mode);
                g_free(path);
            }
            break;
        }

    case OP_READ:
        op_ret = meta_dafs_op_read(dafs, env, args[0], args[1], args[2]);
        break;

    case OP_WRITE:
        op_ret = meta_dafs_op_write(dafs, env, args[0], args[1], args[2]);
        break;

    case OP_CLOSE:
        op_ret = meta_dafs_op_close(dafs, env, args[0]);
        break;

    case OP_LINK: {
            char *path1 = meta_switch_arg_string(env, args[0], PATH_MAX);
            char *path2 = meta_switch_arg_string(env, args[1], PATH_MAX);
            if (path1 && path2) {
                op_ret = meta_dafs_op_link(dafs, env, path1, path2);
            }
            g_free(path1);
            g_free(path2);
            break;
        }

    case OP_LSEEK:
        op_ret = meta_dafs_op_lseek(dafs, env, args[0], args[1], args[2]);
        break;

    case OP_UNLINK: {
            char *path = meta_switch_arg_string(env, args[0], PATH_MAX);
            if (path) {
                op_ret = meta_dafs_op_unlink(dafs, env, path);
                g_free(path);
            }
            break;
        }

    case OP_ISATTY:
        op_ret = meta_dafs_op_isatty(dafs, env, args[0]);
        break;

    case OP_STAT: {
            char *path = meta_switch_arg_string(env, args[0], PATH_MAX);
            if (path) {
                op_ret = meta_dafs_op_stat(dafs, env, path, args[1]);
                g_free(path);
            }
            break;
        }

    case OP_FSTAT:
        op_ret = meta_dafs_op_fstat(dafs, env, args[0], args[1]);
        break;

    case OP_GETCWD:
        op_ret = meta_dafs_op_getcwd(dafs, env, args[0], args[1]);
        break;

    case OP_CHDIR: {
            char *path = meta_switch_arg_string(env, args[0], PATH_MAX);
            if (path) {
                op_ret = meta_dafs_op_chdir(dafs, env, path);
                g_free(path);
            }
            break;
        }

    case OP_MKDIR: {
            char *path = meta_switch_arg_string(env, args[0], PATH_MAX);
            if (path) {
                op_ret = meta_dafs_op_mkdir(dafs, env, path, args[1]);
                g_free(path);
            }
            break;
        }

    case OP_RMDIR: {
            char *path = meta_switch_arg_string(env, args[0], PATH_MAX);
            if (path) {
                op_ret = meta_dafs_op_rmdir(dafs, env, path);
                g_free(path);
            }
            break;
        }

    case OP_FINDFIRST: {
            char *path = meta_switch_arg_string(env, args[0], PATH_MAX);
            if (path) {
                op_ret = meta_dafs_op_findfirst(dafs, env, path, args[1]);
                g_free(path);
            }
            break;
        }

    case OP_FINDNEXT:
        op_ret = meta_dafs_op_findnext(dafs, env, args[0], args[1]);
        break;

    case OP_FINDCLOSE:
        op_ret = meta_dafs_op_findclose(dafs, env, args[0]);
        break;

    case OP_CHMOD: {
            char *path = meta_switch_arg_string(env, args[0], PATH_MAX);
            if (path) {
                op_ret = meta_dafs_op_chmod(dafs, env, path, args[1]);
                g_free(path);
            }
            break;
        }

    case OP_PREAD:
        op_ret = meta_dafs_op_pread(dafs, env, args[0], args[1],
                                    args[2], args[3]);
        break;

    case OP_PWRITE:
        op_ret = meta_dafs_op_pwrite(dafs, env, args[0], args[1],
                                     args[2], args[3]);
        break;

    default:
        fprintf(stderr, "DAFS unhandled op %d\n", args[5]);
    }

    /* set result */
    ret[0] = op_ret.err;
    ret[1] = op_ret.ret;
    return 0;
}

static int meta_dafs_initfn(MetaSwitchDevice *dev)
{
    dev->handler = meta_dafs_handler;
    return 0;
}

static Property meta_dafs_properties[] = {
    DEFINE_PROP_STRING("root", MetaDAFSState, root),
    DEFINE_PROP_UINT8("sandbox", MetaDAFSState, sandbox, 1),
    DEFINE_PROP_UINT8("noexec", MetaDAFSState, noexec, 0),
    DEFINE_PROP_UINT8("thread_mask", MetaDAFSState, dev.thread_mask, 0xf),
    DEFINE_PROP_END_OF_LIST(),
};

static void meta_dafs_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    MetaSwitchDeviceClass *sc = META_SWITCH_DEVICE_CLASS(klass);
    sc->init = meta_dafs_initfn;
    sc->subgroup = META_SWITCH_FILE;
    sc->per_thread = 1;
    dc->props = meta_dafs_properties;
}

static TypeInfo meta_dafs_info = {
    .name          = "dafs",
    .parent        = TYPE_META_SWITCH_DEVICE,
    .instance_size = sizeof(MetaDAFSState),
    .class_init    = meta_dafs_class_initfn,
};

static void meta_dafs_register(void)
{
    int i;

    memset(dafs_handles, 0, sizeof(dafs_handles));

    /* init stdio passthrough */
    for (i = 0; i < 3; i++) {
        dafs_stdio[i].fd = i;
        dafs_stdio[i].affinity = (1 << META_MAX_THREADS) - 1;
        dafs_handles[i].type = DAFS_FILE;
        dafs_handles[i].file = &dafs_stdio[i];
    }

    type_register_static(&meta_dafs_info);
}

type_init(meta_dafs_register)
