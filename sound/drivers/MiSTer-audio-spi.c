#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/fs.h> 
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>

#define DRIVER_DESC	"MiSTer Audio (SPI)"
#define DRIVER_VERSION	"1.0"
#define DRIVER_NAME     "MrAudio"

#define BUFFER_LEN       512*1024 // Holds about 2.6 seconds of audio.

static int major = -1;
static struct cdev mycdev;
static struct class *myclass = NULL;

static char msg[1024];          // The msg the device will give when asked */
static char *msg_Ptr;
static int  MrBufferWriteCount  = 0;
static int  MrBufferMaxWriteLen = 0;

static char *MrBuffer = 0;

typedef struct Info
{
    unsigned int addr;
    unsigned int len;
    unsigned int ptr;
    unsigned int reserved; //IRQ rate may be.
} Info_t;

static Info_t MrBufferInfo;

static struct spi_device *g_spi;

////////////////////////////////////////////////////////////////////////////////////////////////

static int show(struct seq_file *m, void *v)
{
	seq_printf(m, "TEST-->");
	return 0;
}

static int device_open(struct inode *inode, struct file *file)
{
	int len;
	int comp = 0;
	int rptr = -1;

	if(spi_read(g_spi, &rptr, 4)) rptr = -1;
	rptr >>= 8;

	if(rptr >= 0)
	{
		comp = rptr & 7;
		rptr &= ~7;
	}

	len = (MrBufferInfo.ptr < (unsigned int)rptr) ? (MrBufferInfo.ptr + MrBufferInfo.len - rptr) : (MrBufferInfo.ptr - rptr);

	sprintf(msg, "rptr: %6d, wptr: %6d, len: %6d, comp: %1d\n", rptr, MrBufferInfo.ptr, len, comp);

	msg_Ptr = msg;
	return single_open(file, show, NULL);
}

static ssize_t device_read(struct file *filp,   
                           char *buffer,        
                           size_t length,       
                           loff_t * offset)
{
	int bytes_read = 0; // <-- Number of bytes actually written to the buffer

	if (*msg_Ptr == 0)  // If we're at the end of the message, 
		return 0;       // return 0 signifying end of file

	while (length && *msg_Ptr) // Actually put the data into the buffer
	{
		// The buffer is in the user data segment, not the kernel
		// segment so "*" assignment won't work.  We have to use
		// put_user which copies data from the kernel data segment to
		// the user data segment.
		put_user(*(msg_Ptr++), buffer++);
		length--;
		bytes_read++;
	}

	return bytes_read;  // Most read functions return the number of bytes put into the buffer 
}

static ssize_t device_write(struct file *filp,
                            const char *userBuf,
                            size_t userBufLen,
                            loff_t * off)
{
	size_t count,i;
	unsigned int buf[64];

	userBufLen = userBufLen & ~3; // assume the length aligned to 32bits (2x16 samples) for faster access

	MrBufferWriteCount++;
	if(userBufLen > MrBufferMaxWriteLen) MrBufferMaxWriteLen = userBufLen;

	if(!userBufLen || userBufLen > BUFFER_LEN) return -EFAULT;

	count = userBufLen;
	while (count)
	{
		size_t c = count;
		if (c > sizeof(buf)) c = sizeof(buf);
		if (copy_from_user(buf, userBuf, c)) return -EFAULT;
		userBuf += c;
		count -= c;
		c >>= 2;
		for(i=0; i < c; i++)
		{
			// align to 64bit access
			writel(buf[i], &MrBuffer[MrBufferInfo.ptr]);
			MrBufferInfo.ptr +=4;
			if(MrBufferInfo.ptr>=BUFFER_LEN) MrBufferInfo.ptr = 0;
		}
	}

	spi_write(g_spi, &MrBufferInfo, sizeof(MrBufferInfo));
	return userBufLen;
}

static const struct file_operations fops =
{
	.open = device_open,
	.owner = THIS_MODULE,
	.read = device_read,
	.write = device_write,
	.release = single_release,
};


////////////////////////////////////////////////////////////////////////////////////////////////

static void cleanup(int device_created)
{
	if (device_created)
	{
		device_destroy(myclass, major);
		cdev_del(&mycdev);
	}

	if (myclass) class_destroy(myclass);
	if (major != -1) unregister_chrdev_region(major, 1);

	if(MrBuffer) dma_free_coherent(&g_spi->dev, BUFFER_LEN, MrBuffer, MrBufferInfo.addr);
	MrBuffer = 0;
}

static int device_init(void)
{
	int device_created = 0;
	int i;

	if (dma_set_coherent_mask(&g_spi->dev, DMA_BIT_MASK(32)))
	{
		printk(KERN_INFO "Bad mask.\n");
		goto error;
	}

	memset(&MrBufferInfo, 0, sizeof(MrBufferInfo));
	MrBufferInfo.len = BUFFER_LEN;
	MrBuffer = dma_alloc_coherent(&g_spi->dev, BUFFER_LEN, &MrBufferInfo.addr, GFP_KERNEL);
	if(!MrBuffer)
	{
		printk(KERN_INFO "MrAudio ERROR:--> dma_alloc_coherent(0x%X)\n", BUFFER_LEN);
		goto error;
	}

	printk(KERN_INFO "MrAudio: CMA addr = 0x%X\n", MrBufferInfo.addr);

	for(i=0; i<BUFFER_LEN; i+=4) writel(0, &MrBuffer[i]);

	// cat /proc/devices 
	if (alloc_chrdev_region(&major, 0, 1, DRIVER_NAME "_proc") < 0)
	{
		printk(KERN_INFO "MrAudio ERROR:--> alloc_chrdev_region(%d, '%s')\n", major, DRIVER_NAME "_proc");
		goto error;
	}
	// ls /sys/class 
	if ((myclass = class_create(THIS_MODULE, DRIVER_NAME "_sys")) == NULL)
	{
		printk(KERN_INFO "MrAudio ERROR:--> class_create('%s')\n", DRIVER_NAME "_SYS");
		goto error;
	}
	// ls /dev/ 
	if (device_create(myclass, NULL, major, NULL, DRIVER_NAME) == NULL)
	{
		printk(KERN_INFO "MrAudio ERROR:--> device_create('%s', %d, '%s')\n", 
		       myclass->name, 
		       major, 
		       DRIVER_NAME);
		goto error;
	}
	device_created = 1;
	cdev_init(&mycdev, &fops);
	if (cdev_add(&mycdev, major, 1) == -1)
	{
		printk(KERN_INFO "MrAudio ERROR:--> cdev_add(%d)\n", major);
		goto error;
	}

	printk(KERN_INFO "MrAudio Audio buffer '/dev/%s' created.\n"
                     "Class --> '%s'\n"
                     "Major --> %d\n", 
                     DRIVER_NAME,
                     myclass->name,  
                     major);
	return 0;

error:
	cleanup(device_created);
	return -1;
}

////////////////////////////////////////////////////////////////////////////////////////////////

static int device_probe(struct spi_device *spi)
{
	g_spi = spi;

	if (spi_setup(spi) < 0)
	{
		dev_err(&spi->dev, "Unable to setup SPI bus");
		return -EFAULT;
	}

	if (device_init())
	{
		dev_err(&spi->dev, "device_init() failed\n");
		return -EFAULT;
	}
	return 0;
}

static int device_remove(struct spi_device *spi)
{
	cleanup(1);
	return 0;
}

static const struct of_device_id device_of_match_table[] = {
	{ .compatible = "MiSTer,spi-audio", },
	{},
};
MODULE_DEVICE_TABLE(of, device_of_match_table);

static struct spi_driver MiSTer_audio_spi_driver = {
	.probe		= device_probe,
	.remove		= device_remove,
	.driver		=
	{
		.name		= "MiSTer-audio-spi",
		.of_match_table = of_match_ptr(device_of_match_table),
	},
};

module_spi_driver(MiSTer_audio_spi_driver);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Sorgelig");
MODULE_LICENSE("GPL");

