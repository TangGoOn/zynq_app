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

#define PRINTER_FILE			"/dev/g_printer" 
#define myfile                      "/tmp/version.txt"
#define BUF_SIZE				65536

#define use_fwrite_write

int main() 
{ 
	struct pollfd	fd[1];
	FILE *fd_read = NULL;
	unsigned long file_len,remainder;
	char *pbuf;
	char *pMap = NULL;
	int i=0,count,retval;
	FILE *fd_printer = NULL;


	/* Open device file for printer gadget. */
/*	fd[0].fd = open (PRINTER_FILE, O_RDWR|O_SYNC);
	if (fd[0].fd < 0) 
	{ 
		printf("Error %d opening %s\n", fd[0].fd, PRINTER_FILE);
		close(fd[0].fd);
		return(-1); 
	}  //write  */
	fd_printer = fopen(PRINTER_FILE,"w");
	if(fd_printer == NULL)
	{
		printf("open %s fail",PRINTER_FILE);
		fclose(fd_printer);
		return -1;
	} //fwrite

	fd_read = fopen(myfile,"rb+");
	if(fd_read == NULL)
	{
		printf("open %s fail",myfile);
		fclose(fd_read);
		return -1;
	}

	fseek(fd_read, 0, SEEK_END);
	file_len = ftell(fd_read);
	rewind(fd_read);
  
	/* 分配内存存储整个文件 */   
	pbuf = (char*) malloc (sizeof(char)*file_len);  
	if (pbuf == NULL)  
	{  
	  printf("Memory error\n");   
	  return -1;
	} 

	int bytes_read = fread(pbuf, 1, file_len, fd_read);
	if (bytes_read != file_len) 
	{ 
		printf("Reading file error!\n");
		return -1;
	}
	printf("read file done,file len : %ld\n",file_len);

#ifdef use_mmap
	
	if((file_len % BUF_SIZE > 0)&&(file_len > BUF_SIZE))
	{
		count = file_len / BUF_SIZE;
		remainder = file_len % BUF_SIZE;
		pMap = (char *)calloc(1, BUF_SIZE);

		for(i=0; i<count; i++)
		{
			
			pMap = (char *)mmap(0,BUF_SIZE, PROT_WRITE, MAP_SHARED, fd[0].fd, i*BUF_SIZE); // 每次打开512bytes
			memcpy(pMap, pbuf, BUF_SIZE);
			munmap(pMap, BUF_SIZE);
		}

		pMap = (char *)calloc(1, remainder);
		pMap = (char *)mmap(0,remainder, PROT_WRITE, MAP_SHARED, fd[0].fd, remainder); // mmap remainder
		memcpy(pMap, pbuf+BUF_SIZE*count, remainder);
		munmap(pMap, BUF_SIZE);

	}

	printf("write done!\n");

#endif 

#ifdef use_fwrite_write
	if((file_len % BUF_SIZE > 0)&&(file_len > BUF_SIZE))
	{
		count = file_len / BUF_SIZE;
		remainder = file_len % BUF_SIZE;

		while (count) 
		{
			//retval = write(fd[0].fd, pbuf+i*BUF_SIZE, BUF_SIZE);
			retval = fwrite(pbuf+i*BUF_SIZE, BUF_SIZE,1,fd_printer);
			fflush(fd_printer);
			if (retval < 0) 
			{ 
				printf("Error %d writing to %s\n", fd[0].fd,PRINTER_FILE); 
				close(fd[0].fd);
				return(-1); 
			}
	
			i++;
			count--;

			if(count == 0)
			{
				//retval = write(fd[0].fd, (pbuf+count*BUF_SIZE), remainder);
				retval = fwrite(pbuf+count*BUF_SIZE, remainder,1,fd_printer);
				fflush(fd_printer);
				if (retval < 0) 
				{ 
					printf("Error %d writing to %s\n", fd[0].fd,PRINTER_FILE); 
					close(fd[0].fd);
					return(-1); 
				}
			 	else 
				{ 
					break;
				}
			}
		
		}
	}

	
	else if(file_len % BUF_SIZE == 0)
	{
		count = file_len / BUF_SIZE;
		remainder = 0;
		while (count) 
		{
			//retval = write(fd[0].fd, pbuf+i*BUF_SIZE, BUF_SIZE);
			retval = fwrite(pbuf+i*BUF_SIZE, BUF_SIZE,1,fd_printer);			
			fflush(fd_printer);
			if (retval < 0) 
			{ 
				printf("Error %d writing to %s\n", fd[0].fd,PRINTER_FILE); 
				close(fd[0].fd);
				return(-1); 
			}
			i++;	
			count--;	
		}


	}
	else if(file_len < BUF_SIZE)
	{
		count = 0;
		remainder = file_len;

		while(remainder)
		{
			//retval = write(fd[0].fd, pbuf, remainder);
			retval = fwrite(pbuf, remainder,1,fd_printer);
			fflush(fd_printer);
			if (retval < 0) 
			{ 
				printf("Error %d writing to %s\n", fd[0].fd,PRINTER_FILE); 
				close(fd[0].fd);
				return(-1); 
			}
			remainder -= retval;
		}
	}
	/* Wait until the data has been sent. */
	fsync(fd[0].fd);
#endif
	/* Close the device file. */
	close(fd[0].fd);
	fclose(fd_read);
	return 0; 
}















