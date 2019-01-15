#include<stdio.h>  
#include<unistd.h>  
#include<sys/mman.h>  
#include<sys/types.h>  
#include<sys/stat.h>  
#include<fcntl.h>  
#include<stdlib.h>
#include<sys/time.h>
#include<string.h>

#define buf_size    4*1024*1024
#define AXI_DEVICE  "/dev/AXI4_command_dev"

int main()  
{  
	  
	int fd,i;
	unsigned int bytes_read,bytes_write;  
	
	char *rbuf = (char *)calloc(sizeof(char),buf_size);

	unsigned int *wbuf = (unsigned int *)calloc(sizeof(unsigned int),buf_size);

	fd = open(AXI_DEVICE, O_RDWR);  
	if (fd == -1)  
	{  
		printf("open failed!\n");	  
		return (-1);  
	} 

	lseek(fd, 0xc0, SEEK_SET);//set the offset of my file
	bytes_read = read(fd,rbuf,32);
	if(bytes_read==-1)
	{
		printf("read failed!\n");
	}
	for(i=0;i<32;i++)
	{
		printf("%c",rbuf[i]);			
	}
	printf("\n");
	

	memset(wbuf,0xf,512);

	for(i=0;i<512;i++)  //write CIS1 512*4 BIT
	{
		lseek(fd, 0x40000+i*4, SEEK_SET);//set the offset of my file
		bytes_write = write(fd,wbuf+i,1);
		if(bytes_write==-1)
		{
			printf("write failed!\n");
		}
	}
	printf("write CIS1 512*4 bit done!\n");

	for(i=0;i<512;i++)
	{
		lseek(fd, 0x60000+i*4, SEEK_SET);//set the offset of my file
		bytes_write = write(fd,wbuf+i,1);
		if(bytes_write==-1)
		{
			printf("write failed!\n");
		}
	}
	printf("write CIS2 512*4 bit done!\n");

	close(fd);  
	return 0;  
}  
