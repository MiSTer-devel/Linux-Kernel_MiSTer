/* SPDX-License-Identifier: GPL-2.0 */
/*
 * In-kernel interface to the loop driver.
 *
 * This header exists only so that init/do_mounts.c can bind a loop device
 * before there is any userspace to run losetup(8) -- see the `loop=` boot
 * parameter.  Everything else in the tree talks to the loop driver through
 * the UAPI ioctls in <uapi/linux/loop.h>, which is re-exported below so that
 * this header is a strict superset of the one it shadows on the include path
 * (include/ is searched before include/uapi/, so a bare <linux/loop.h> that
 * did NOT include the UAPI header would silently break any existing
 * LOOP_SET_FD user that reaches the UAPI definitions that way).
 */
#ifndef _LINUX_LOOP_H
#define _LINUX_LOOP_H

#include <linux/types.h>
#include <uapi/linux/loop.h>

struct file;

/*
 * The number of partitions the loop driver reserves per device, as fixed at
 * loop_init() time from the max_part module parameter.  A caller that has to
 * mknod a loop device node before /dev exists needs this to work out the minor
 * number: loopN's first minor is N << part_shift, i.e. N * (max_part + 1).
 */
int loop_max_part(void);

/*
 * Bind @backing_fd to the loop device that @lo_file has open, exactly as the
 * LOOP_SET_FD ioctl would.  Split out of lo_ioctl() so that in-kernel callers
 * (init/do_mounts.c) can do this without a syscall: init code can allocate a
 * descriptor for itself with get_unused_fd_flags() + fd_install(), but there is
 * no init_ioctl(), and vfs_ioctl() is static to fs/ioctl.c.
 */
int loop_set_backing_fd(struct file *lo_file, int backing_fd);

#endif /* _LINUX_LOOP_H */
