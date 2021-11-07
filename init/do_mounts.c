// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/ctype.h>
#include <linux/fd.h>
#include <linux/tty.h>
#include <linux/suspend.h>
#include <linux/root_dev.h>
#include <linux/security.h>
#include <linux/delay.h>
#include <linux/mount.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/initrd.h>
#include <linux/async.h>
#include <linux/fs_struct.h>
#include <linux/slab.h>
#include <linux/ramfs.h>
#include <linux/shmem_fs.h>
#include <linux/ktime.h>

#include <linux/nfs_fs.h>
#include <linux/nfs_fs_sb.h>
#include <linux/nfs_mount.h>
#include <linux/raid/detect.h>
#include <uapi/linux/mount.h>

#include <linux/fdtable.h>
#include <linux/file.h>
#include <linux/loop.h>

#include "do_mounts.h"

/*
 * MS_NOATIME|MS_NODIRATIME are in the default set because MiSTer's root lives
 * on an SD card: an atime update is a read-modify-write of an erase block, and
 * nothing on the system ever reads the result back.
 */
int root_mountflags = MS_RDONLY | MS_SILENT | MS_NOATIME | MS_NODIRATIME;
static char __initdata saved_root_name[64];
static int root_wait;

dev_t ROOT_DEV;

static int __init load_ramdisk(char *str)
{
	pr_warn("ignoring the deprecated load_ramdisk= option\n");
	return 1;
}
__setup("load_ramdisk=", load_ramdisk);

/*
 * loop=<path>
 *
 * Path, relative to the root of the exFAT filesystem on the root= device, of a
 * disk image to loop-mount as the real root.  Set, mount_block_root() mounts
 * root= at /root2 as exFAT, attaches <path> to /dev/loop8, mounts that as
 * /root, and bind-mounts /root2 back onto /root/media/fat.  Unset, the root=
 * device is mounted directly and none of that code runs.
 *
 * This is what lets a MiSTer ship its whole system as one file
 * (linux/linux.img) on a card that is otherwise a plain exFAT data partition
 * the user manages from a PC.
 */
static char * __initdata loop_name;
static int __init set_loop_name(char *str)
{
	loop_name = str;
	return 1;
}
__setup("loop=", set_loop_name);

static int __init readonly(char *str)
{
	if (*str)
		return 0;
	root_mountflags |= MS_RDONLY;
	return 1;
}

static int __init readwrite(char *str)
{
	if (*str)
		return 0;
	root_mountflags &= ~MS_RDONLY;
	return 1;
}

__setup("ro", readonly);
__setup("rw", readwrite);

static int __init root_dev_setup(char *line)
{
	strscpy(saved_root_name, line, sizeof(saved_root_name));
	return 1;
}

__setup("root=", root_dev_setup);

static int __init rootwait_setup(char *str)
{
	if (*str)
		return 0;
	root_wait = -1;
	return 1;
}

__setup("rootwait", rootwait_setup);

static int __init rootwait_timeout_setup(char *str)
{
	int sec;

	if (kstrtoint(str, 0, &sec) || sec < 0) {
		pr_warn("ignoring invalid rootwait value\n");
		goto ignore;
	}

	if (check_mul_overflow(sec, MSEC_PER_SEC, &root_wait)) {
		pr_warn("ignoring excessive rootwait value\n");
		goto ignore;
	}

	return 1;

ignore:
	/* Fallback to indefinite wait */
	root_wait = -1;

	return 1;
}

__setup("rootwait=", rootwait_timeout_setup);

static char * __initdata root_mount_data;
static int __init root_data_setup(char *str)
{
	root_mount_data = str;
	return 1;
}

static char * __initdata root_fs_names;
static int __init fs_names_setup(char *str)
{
	root_fs_names = str;
	return 1;
}

static unsigned int __initdata root_delay;
static int __init root_delay_setup(char *str)
{
	root_delay = simple_strtoul(str, NULL, 0);
	return 1;
}

__setup("rootflags=", root_data_setup);
__setup("rootfstype=", fs_names_setup);
__setup("rootdelay=", root_delay_setup);

/* This can return zero length strings. Caller should check */
static int __init split_fs_names(char *page, size_t size)
{
	int count = 1;
	char *p = page;

	strscpy(p, root_fs_names, size);
	while (*p++) {
		if (p[-1] == ',') {
			p[-1] = '\0';
			count++;
		}
	}

	return count;
}

static int __init do_mount_root(const char *name, const char *fs,
				 const int flags, const void *data)
{
	struct super_block *s;
	struct page *p = NULL;
	char *data_page = NULL;
	int ret;

	if (data) {
		/* init_mount() requires a full page as fifth argument */
		p = alloc_page(GFP_KERNEL);
		if (!p)
			return -ENOMEM;
		data_page = page_address(p);
		strscpy_pad(data_page, data, PAGE_SIZE);
	}

	ret = init_mount(name, "/root", fs, flags, data_page);
	if (ret)
		goto out;

	init_chdir("/root");
	s = current->fs->pwd.dentry->d_sb;
	ROOT_DEV = s->s_dev;
	printk(KERN_INFO
	       "VFS: Mounted root (%s filesystem)%s on device %u:%u.\n",
	       s->s_type->name,
	       sb_rdonly(s) ? " readonly" : "",
	       MAJOR(ROOT_DEV), MINOR(ROOT_DEV));

out:
	if (p)
		put_page(p);
	return ret;
}

void __init mount_root_generic(char *name, char *pretty_name, int flags)
{
	struct page *page = alloc_page(GFP_KERNEL);
	char *fs_names = page_address(page);
	char *p;
	char b[BDEVNAME_SIZE];
	int num_fs, i;

	scnprintf(b, BDEVNAME_SIZE, "unknown-block(%u,%u)",
		  MAJOR(ROOT_DEV), MINOR(ROOT_DEV));
	if (root_fs_names)
		num_fs = split_fs_names(fs_names, PAGE_SIZE);
	else
		num_fs = list_bdev_fs_names(fs_names, PAGE_SIZE);
retry:
	for (i = 0, p = fs_names; i < num_fs; i++, p += strlen(p)+1) {
		int err;

		if (!*p)
			continue;
		err = do_mount_root(name, p, flags, root_mount_data);
		switch (err) {
			case 0:
				goto out;
			case -EACCES:
			case -EINVAL:
#ifdef CONFIG_BLOCK
				init_flush_fput();
#endif
				continue;
		}
	        /*
		 * Allow the user to distinguish between failed sys_open
		 * and bad superblock on root device.
		 * and give them a list of the available devices
		 */
		printk("VFS: Cannot open root device \"%s\" or %s: error %d\n",
				pretty_name, b, err);
		printk("Please append a correct \"root=\" boot option; here are the available partitions:\n");
		printk_all_partitions();

		if (root_fs_names)
			num_fs = list_bdev_fs_names(fs_names, PAGE_SIZE);
		if (!num_fs)
			pr_err("Can't find any bdev filesystem to be used for mount!\n");
		else {
			pr_err("List of all bdev filesystems:\n");
			for (i = 0, p = fs_names; i < num_fs; i++, p += strlen(p)+1)
				pr_err(" %s", p);
			pr_err("\n");
		}

		panic("VFS: Unable to mount root fs on %s", b);
	}
	if (!(flags & SB_RDONLY)) {
		flags |= SB_RDONLY;
		goto retry;
	}

	printk("List of all partitions:\n");
	printk_all_partitions();
	printk("No filesystem could mount root, tried: ");
	for (i = 0, p = fs_names; i < num_fs; i++, p += strlen(p)+1)
		printk(" %s", p);
	printk("\n");
	panic("VFS: Unable to mount root fs on \"%s\" or %s", pretty_name, b);
out:
	put_page(page);
}
 
#ifdef CONFIG_ROOT_NFS

#define NFSROOT_TIMEOUT_MIN	5
#define NFSROOT_TIMEOUT_MAX	30
#define NFSROOT_RETRY_MAX	5

static void __init mount_nfs_root(void)
{
	char *root_dev, *root_data;
	unsigned int timeout;
	int try;

	if (nfs_root_data(&root_dev, &root_data))
		goto fail;

	/*
	 * The server or network may not be ready, so try several
	 * times.  Stop after a few tries in case the client wants
	 * to fall back to other boot methods.
	 */
	timeout = NFSROOT_TIMEOUT_MIN;
	for (try = 1; ; try++) {
		if (!do_mount_root(root_dev, "nfs", root_mountflags, root_data))
			return;
		if (try > NFSROOT_RETRY_MAX)
			break;

		/* Wait, in case the server refused us immediately */
		ssleep(timeout);
		timeout <<= 1;
		if (timeout > NFSROOT_TIMEOUT_MAX)
			timeout = NFSROOT_TIMEOUT_MAX;
	}
fail:
	pr_err("VFS: Unable to mount root fs via NFS.\n");
}
#else
static inline void mount_nfs_root(void)
{
}
#endif /* CONFIG_ROOT_NFS */

#ifdef CONFIG_CIFS_ROOT

#define CIFSROOT_TIMEOUT_MIN	5
#define CIFSROOT_TIMEOUT_MAX	30
#define CIFSROOT_RETRY_MAX	5

static void __init mount_cifs_root(void)
{
	char *root_dev, *root_data;
	unsigned int timeout;
	int try;

	if (cifs_root_data(&root_dev, &root_data))
		goto fail;

	timeout = CIFSROOT_TIMEOUT_MIN;
	for (try = 1; ; try++) {
		if (!do_mount_root(root_dev, "cifs", root_mountflags,
				   root_data))
			return;
		if (try > CIFSROOT_RETRY_MAX)
			break;

		ssleep(timeout);
		timeout <<= 1;
		if (timeout > CIFSROOT_TIMEOUT_MAX)
			timeout = CIFSROOT_TIMEOUT_MAX;
	}
fail:
	pr_err("VFS: Unable to mount root fs via SMB.\n");
}
#else
static inline void mount_cifs_root(void)
{
}
#endif /* CONFIG_CIFS_ROOT */

static bool __init fs_is_nodev(char *fstype)
{
	struct file_system_type *fs = get_fs_type(fstype);
	bool ret = false;

	if (fs) {
		ret = !(fs->fs_flags & FS_REQUIRES_DEV);
		put_filesystem(fs);
	}

	return ret;
}

static int __init mount_nodev_root(char *root_device_name)
{
	char *fs_names, *fstype;
	int err = -EINVAL;
	int num_fs, i;

	fs_names = (void *)__get_free_page(GFP_KERNEL);
	if (!fs_names)
		return -EINVAL;
	num_fs = split_fs_names(fs_names, PAGE_SIZE);

	for (i = 0, fstype = fs_names; i < num_fs;
	     i++, fstype += strlen(fstype) + 1) {
		if (!*fstype)
			continue;
		if (!fs_is_nodev(fstype))
			continue;
		err = do_mount_root(root_device_name, fstype, root_mountflags,
				    root_mount_data);
		if (!err)
			break;
	}

	free_page((unsigned long)fs_names);
	return err;
}

#ifdef CONFIG_BLOCK
/*
 * The loop= path needs loop_set_backing_fd()/loop_max_part() resolved at
 * vmlinux link time.  IS_BUILTIN(), not IS_ENABLED(): with
 * CONFIG_BLK_DEV_LOOP=m those symbols live in a module that cannot possibly
 * be loaded before the root filesystem is mounted, and referencing them from
 * here would break the vmlinux link of every =m configuration in the world,
 * allmodconfig included.
 */
#if IS_BUILTIN(CONFIG_BLK_DEV_LOOP)
/*
 * Attach @file to the loop device node @device -- the in-kernel equivalent of
 *
 *	losetup /dev/loop8 /root2/linux/linux.img
 *
 * The ioctl half is a driver export because there is no init_ioctl(); see
 * loop_set_backing_fd() in drivers/block/loop.c.  The descriptor half has to
 * happen here, because loop_configure() fget()s config.fd.
 *
 * Note that fs/init.c's init_dup() cannot be used for this: despite the name it
 * does not return a descriptor.  It allocates one, fd_install()s a reference of
 * its own, and then returns 0 -- it exists for console_on_rootfs(), whose
 * callers only care whether it succeeded.  Using it here would silently bind
 * fd 0, which by this point in the boot is /dev/console (console_on_rootfs()
 * runs in kernel_init_freeable() before prepare_namespace()), so the loop
 * device would be handed a character device and refuse it.  So the descriptor
 * is allocated directly.
 *
 * fd_install() consumes the filp_open() reference: from that point the
 * descriptor owns it, and close_fd() below is what drops it.  On success
 * loop_configure() has taken a reference of its own via fget(), so the loop
 * device keeps the backing file alive after we close our descriptor.
 */
static int __init loop_setup(const char *file, const char *device)
{
	struct file *backing, *lo_file;
	int backing_fd, err;

	backing = filp_open(file, O_RDWR | O_LARGEFILE, 0);
	if (IS_ERR(backing)) {
		pr_emerg("Failed to open backing file (%s): %ld\n",
			 file, PTR_ERR(backing));
		return PTR_ERR(backing);
	}

	backing_fd = get_unused_fd_flags(0);
	if (backing_fd < 0) {
		pr_emerg("Failed to get a descriptor for (%s): %d\n",
			 file, backing_fd);
		fput(backing);
		return backing_fd;
	}
	fd_install(backing_fd, backing);

	/*
	 * Opening the node is not merely how we name the device: it is what
	 * brings the device into existence.  loop8 is one past
	 * CONFIG_BLK_DEV_LOOP_MIN_COUNT's default of 8 devices, so the driver
	 * has not instantiated it; blkdev_get_no_open() finds no inode and
	 * falls through to blk_request_module() -> blk_probe_dev() ->
	 * loop_probe(), which loop_add()s it.
	 *
	 * That fallthrough is gated on CONFIG_BLOCK_LEGACY_AUTOLOAD, which is
	 * `default y` but is documented as deprecated and prints a
	 * pr_warn_ratelimited() saying it "will be removed".  When it goes,
	 * this open starts returning -ENXIO and this boot method needs a
	 * different way to instantiate the device -- hence the explicit second
	 * message below, so that day produces an actionable panic and not a
	 * puzzle.  (The block Kconfig help text calls out exactly this case:
	 * "scripts that manually create device nodes and then call losetup".)
	 */
	lo_file = filp_open(device, O_RDWR | O_LARGEFILE, 0);
	if (IS_ERR(lo_file)) {
		pr_emerg("Failed to open device (%s): %ld\n",
			 device, PTR_ERR(lo_file));
		pr_emerg("loop= instantiates %s by opening it, which needs CONFIG_BLOCK_LEGACY_AUTOLOAD=y\n",
			 device);
		close_fd(backing_fd);
		return PTR_ERR(lo_file);
	}

	err = loop_set_backing_fd(lo_file, backing_fd);
	if (err)
		pr_emerg("Failed to set fd: %d\n", err);

	fput(lo_file);
	close_fd(backing_fd);
	return err;
}

static void __init mount_loop_root(char *root_device_name)
{
	char *lname;
	int err;

	err = init_mkdir("/root2", 0777);
	if (err)
		pr_emerg("Failed mkdir /root2: %d\n", err);

	err = init_mount("/dev/root", "/root2", "exfat",
			 MS_DIRSYNC | MS_SYNCHRONOUS | MS_NOATIME |
			 MS_NODIRATIME, "");
	if (err)
		pr_emerg("Failed to mount /dev/root as exFAT: %d\n", err);

	/*
	 * /dev is still the rootfs one -- devtmpfs is not mounted until
	 * prepare_namespace() is done with us -- so the node has to be made by
	 * hand, with the minor computed the way loop_add() does it.
	 */
	err = create_dev("/dev/loop8",
			 MKDEV(LOOP_MAJOR, (loop_max_part() + 1) * 8));
	if (err < 0)
		pr_emerg("Failed to create /dev/loop8: %d\n", err);

	/*
	 * kasprintf() rather than sprintf() into a fixed buffer: loop_name
	 * points straight into the kernel command line and is bounded only by
	 * COMMAND_LINE_SIZE, so any fixed-size buffer here is a stack overflow
	 * waiting for a long enough loop= argument.
	 */
	lname = kasprintf(GFP_KERNEL, "/root2/%s", loop_name);
	if (!lname)
		panic("VFS: out of memory building the loop= backing path");

	err = loop_setup(lname, "/dev/loop8");
	if (err)
		pr_emerg("Failed to loop_setup: %d\n", err);
	kfree(lname);

	mount_root_generic("/dev/loop8", "/dev/loop8", root_mountflags);

	err = init_mount("/root2", "/root/media/fat", "", MS_BIND, "");
	if (err)
		pr_emerg("Failed to bind-mount %s to /root/media/fat : %d\n",
			 root_device_name, err);
}
#else
static void __init mount_loop_root(char *root_device_name)
{
	/*
	 * Fail loudly.  Falling through to mounting root= directly would try to
	 * boot the exFAT data partition as the root filesystem: no /sbin/init,
	 * no recognisable rootfs, and a panic several confusing steps further
	 * on -- or worse, a successful mount of something that is not the
	 * system the user asked for.
	 */
	panic("VFS: loop=%s needs CONFIG_BLK_DEV_LOOP=y (it is not built in)",
	      loop_name);
}
#endif /* IS_BUILTIN(CONFIG_BLK_DEV_LOOP) */

static void __init mount_block_root(char *root_device_name)
{
	int err = create_dev("/dev/root", ROOT_DEV);

	if (err < 0)
		pr_emerg("Failed to create /dev/root: %d\n", err);
	if (loop_name)
		mount_loop_root(root_device_name);
	else
		mount_root_generic("/dev/root", root_device_name,
				   root_mountflags);
}
#else
static inline void mount_block_root(char *root_device_name)
{
}
#endif /* CONFIG_BLOCK */

void __init mount_root(char *root_device_name)
{
	switch (ROOT_DEV) {
	case Root_NFS:
		mount_nfs_root();
		break;
	case Root_CIFS:
		mount_cifs_root();
		break;
	case Root_Generic:
		mount_root_generic(root_device_name, root_device_name,
				   root_mountflags);
		break;
	case 0:
		if (root_device_name && root_fs_names &&
		    mount_nodev_root(root_device_name) == 0)
			break;
		fallthrough;
	default:
		mount_block_root(root_device_name);
		break;
	}
}

/* wait for any asynchronous scanning to complete */
static void __init wait_for_root(char *root_device_name)
{
	ktime_t end;

	if (ROOT_DEV != 0)
		return;

	pr_info("Waiting for root device %s...\n", root_device_name);

	end = ktime_add_ms(ktime_get_raw(), root_wait);

	while (!driver_probe_done() ||
	       early_lookup_bdev(root_device_name, &ROOT_DEV) < 0) {
		msleep(5);
		if (root_wait > 0 && ktime_after(ktime_get_raw(), end))
			break;
	}

	async_synchronize_full();

}

static dev_t __init parse_root_device(char *root_device_name)
{
	int error;
	dev_t dev;

	if (!strncmp(root_device_name, "mtd", 3) ||
	    !strncmp(root_device_name, "ubi", 3))
		return Root_Generic;
	if (strcmp(root_device_name, "/dev/nfs") == 0)
		return Root_NFS;
	if (strcmp(root_device_name, "/dev/cifs") == 0)
		return Root_CIFS;
	if (strcmp(root_device_name, "/dev/ram") == 0)
		return Root_RAM0;

	error = early_lookup_bdev(root_device_name, &dev);
	if (error) {
		if (error == -EINVAL && root_wait) {
			pr_err("Disabling rootwait; root= is invalid.\n");
			root_wait = 0;
		}
		return 0;
	}
	return dev;
}

/*
 * Prepare the namespace - decide what/where to mount, load ramdisks, etc.
 */
void __init prepare_namespace(void)
{
	if (root_delay) {
		printk(KERN_INFO "Waiting %d sec before mounting root device...\n",
		       root_delay);
		ssleep(root_delay);
	}

	/*
	 * wait for the known devices to complete their probing
	 *
	 * Note: this is a potential source of long boot delays.
	 * For example, it is not atypical to wait 5 seconds here
	 * for the touchpad of a laptop to initialize.
	 */
	wait_for_device_probe();

	md_run_setup();

	if (saved_root_name[0])
		ROOT_DEV = parse_root_device(saved_root_name);

	if (initrd_load(saved_root_name))
		goto out;

	if (root_wait)
		wait_for_root(saved_root_name);
	mount_root(saved_root_name);
out:
	devtmpfs_mount();
	init_mount(".", "/", NULL, MS_MOVE, NULL);
	init_chroot(".");
}

static bool is_tmpfs;
static int rootfs_init_fs_context(struct fs_context *fc)
{
	if (IS_ENABLED(CONFIG_TMPFS) && is_tmpfs)
		return shmem_init_fs_context(fc);

	return ramfs_init_fs_context(fc);
}

struct file_system_type rootfs_fs_type = {
	.name		= "rootfs",
	.init_fs_context = rootfs_init_fs_context,
	.kill_sb	= kill_litter_super,
};

void __init init_rootfs(void)
{
	if (IS_ENABLED(CONFIG_TMPFS)) {
		if (!saved_root_name[0] && !root_fs_names)
			is_tmpfs = true;
		else if (root_fs_names && !!strstr(root_fs_names, "tmpfs"))
			is_tmpfs = true;
	}
}
