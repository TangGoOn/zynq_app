#include <stdio.h> 
#include <stdlib.h> 
#include <fcntl.h> 
#include <linux/poll.h> 
#include <sys/ioctl.h> 
#include <linux/usb/g_printer.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>

#define Device			"/dev/g_printer" 
#define BUF_SIZE			512
#define SUCCESS               0
int fd;
char pbuf[BUF_SIZE];

int open_usb()
{
	fd = open(Device,O_RDWR);
	if(fd == NULL)
	{
		printf("open %s fail",Device);
		close(fd);
		return -1;
	} 
	return SUCCESS;
}

#if 0
int read_usb()
{
	
	unsigned int bytes_read;

	bytes_read = fread(pbuf, 1, 1, fd);
	if (bytes_read == 0) 
	{ 
		printf("Reading file error!\n");
	}
	
	return bytes_read;
}
#endif

//处理函数，没什么好讲的，用户自己定义
void read_handler()
{
	int len;    
	len = read(fd,pbuf,1);
	if(len == 0)
	{
		printf("Read none data!\n");
	}
	printf("Read data :%c\n", pbuf[0]);
}

void main()
{
	open_usb();
	memset(pbuf,0,BUF_SIZE);

	int oflags;

	//启动信号驱动机制,将SIGIO信号同input_handler函数关联起来,一旦产生SIGIO信号,就会执行	input_handler
	signal(SIGIO, read_handler);    

	//STDIN_FILENO是打开的设备文件描述符,F_SETOWN用来决定操作是干什么的,getpid()是个系统调用，
	//功能是返回当前进程的进程号,整个函数的功能是STDIN_FILENO设置这个设备文件的拥有者为当前进程。
	fcntl(fd, F_SETOWN, getpid());    

	//得到打开文件描述符的状态
	oflags = fcntl(fd, F_GETFL);

	//设置文件描述符的状态为oflags | FASYNC属性,一旦文件描述符被设置成具有FASYNC属性的状态，
	//也就是将设备文件切换到异步操作模式。这时系统就会自动调用驱动程序的fasync方法。
	fcntl(fd, F_SETFL, oflags | FASYNC);  

	//最后进入一个死循环，程序什么都不干了，只有信号能激发input_handler的运行
	//如果程序中没有这个死循环，会立即执行完毕

	while(1);
}
