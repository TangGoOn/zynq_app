/*********************************************************
File Name:    my_axi.c
Author:       laozhentang
Version:      v0.1 2017-05-09
Description:  Driver of my_axi peripheral.

*********************************************************/
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/string.h> 
#include <asm/io.h>
#include <asm/page.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/uaccess.h>

#define DEVICE_NAME        "AXI4_command_dev"

#define MY_axi_PHY_ADDR    0x43c00000 //Modify the address to your peripheral
#define MY_axi_REG_NUM     0x1000000

/* 定义幻数 */
//#define AXIDEV_IOC_MAGIC  'k'
/* 定义命令 */
//#define AXIDEV_IOCREAD   _IOR(AXIDEV_IOC_MAGIC, 1, int)
//#define AXIDEV_IOCWRITE  _IOW(AXIDEV_IOC_MAGIC, 2, int)
//#define AXIDEV_IOC_MAXNR 2
//int mmapdrv_mmap(struct file *file, struct vm_area_struct *vma);

static struct resource  *my_mem; 
static void __iomem *axi_Regs;

static int my_axi_open(struct inode * inode , struct file * filp)
{
	return 0;
}

static int my_axi_release(struct inode * inode, struct file *filp)
{
	return 0;
}

static int my_axi_read(struct file *filp, char *buffer, size_t length, loff_t * offset)
{
	unsigned long axi_offset = *offset;
	int bytes_read = 0; 
	int i=0;

	if(axi_offset > MY_axi_REG_NUM)
	{
		return 0;
	}

	if (filp->f_flags & O_NONBLOCK)
	return -EAGAIN;

	if (length>0 && length<=(MY_axi_REG_NUM))
	{
		for(i=0;i<length;i++)
		{
			*(buffer+i)=(char)ioread8(axi_Regs+axi_offset+i);
		}
		bytes_read=length;
	}
	return bytes_read;
}

static int my_axi_write(struct file *filp, char *buffer, size_t length, loff_t * offset)
{
	unsigned long axi_offset = *offset;
	char val = 0;
	int  bytes_write = 0; 
	int  i = 0;

	if(axi_offset > MY_axi_REG_NUM)
	{
		return 0;
	}

	if (length>0 && length<=(MY_axi_REG_NUM))
	{
		for(i=0;i<length;i++)
		{
			val = *(buffer+i);
			iowrite32(val,axi_Regs+axi_offset+i);
		}
		bytes_write=i;
	}

	return bytes_write;
}

static loff_t my_axi_llseek(struct file *filp, loff_t offset,int orig)
{
	loff_t ret = 0;

	switch(orig)
	{
		case 0:
			if(offset < 0)
			{
				ret = -EINVAL;
				break;
			}
			if((unsigned int)offset > MY_axi_REG_NUM)
			{
				ret = -EINVAL;
				break;	
			}
			filp->f_pos = (unsigned int)offset;
			ret = filp->f_pos;
			break;

		case 1:
			if((filp->f_pos + offset) < 0)
			{
				ret = -EINVAL;
				break;
			}
			if((filp->f_pos + offset) > MY_axi_REG_NUM)
			{
				ret = -EINVAL;
				break;	
			}
			filp->f_pos += offset;
			ret = filp->f_pos;
			break;

		default:
			ret = -EINVAL;
			break;
	}

	return ret;
}

static const struct file_operations my_axi_fops =
{
	.owner   =    THIS_MODULE,
	.open    =    my_axi_open,
	.release =    my_axi_release,
	.read    =    my_axi_read,
	.write   =    my_axi_write,
	.llseek  =    my_axi_llseek, 
};

static struct miscdevice my_axi_dev =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &my_axi_fops,
};

int __init my_axi_init(void)
{
	int ret;

	//Request I/O memory
	my_mem = request_mem_region(MY_axi_PHY_ADDR, MY_axi_REG_NUM, "My_FPGA_AXI_SRAM");
	if(my_mem == NULL)
	{
		printk("my_axi:[ERROR] Failed to request I/O memory\n");
		return -EBUSY;
	}
	printk("request_mem_region success! my_mem is:0x%x\n",(unsigned int)my_mem);

	//Verify it's non-null!IO内存资源的物理地址映射到内核虚拟地址空间 
	axi_Regs = ioremap(MY_axi_PHY_ADDR, MY_axi_REG_NUM);
	if(axi_Regs == NULL)
	{
		printk("my_axi:[ERROR] Access address is NULL!\n");
		return -EIO;
	}  
	printk("my_axi: Access address to device is:0x%x\n", (unsigned int)axi_Regs);

	ret = misc_register(&my_axi_dev);
	if (ret)
	{
		printk("my_axi:[ERROR] Misc device register failed\n");
		return ret;
	} 

	printk("my_axi: Module init complete!\n");
	return 0; /* Success */
}

void __exit my_axi_exit(void)
{
	//release_mem_region(my_mem, MY_axi_REG_NUM);
	release_resource(my_mem);  
      kfree(my_mem);
	printk("Release AXI_SRAM mem region success!\n"); 

	iounmap(axi_Regs);
	misc_deregister(&my_axi_dev);

	printk("my_axi: Module exit\n");
}

module_init(my_axi_init);
module_exit(my_axi_exit);

MODULE_AUTHOR("LZT");
MODULE_ALIAS("my_axi");
MODULE_DESCRIPTION("bv my_axi module");
MODULE_LICENSE("GPL");

