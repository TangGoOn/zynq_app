#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <string.h>
#include<sys/time.h>
#define AXI_BASE 0x43c00000

int main()
{
    int fd,i;
    unsigned char *start;
    struct timeval tpstart,tpend; 
    float timeuse; 
    char *buf;
    unsigned int addr_start, addr_offset;  
    unsigned int PageSize, PageMask; 
    
    /*打开文件*/
    //fd = open("/dev/memdev0",O_RDWR);
    fd = open("/dev/mem",O_RDWR|O_NDELAY);
    if(-1 == fd)
    {
	printf("open failed!\n");
	return 0;
    }
	printf("open success\n");
  
    PageSize = sysconf(_SC_PAGESIZE);   //页大小  
    PageMask = (~(PageSize-1));         //页掩码
    printf("PageSize:%d,PageMask:%.8X\r\n",PageSize,PageMask);   
    addr_start = AXI_BASE & PageMask;  
    addr_offset = AXI_BASE & ~PageMask;
    printf("addr_start:%.8X,addr_offset:%.8X\r\n",addr_start,addr_offset);

    buf = (char*)calloc(16*1024*1024,sizeof(char));
  
    start=(unsigned char *)mmap(NULL,0xFF,PROT_READ,MAP_SHARED,fd,AXI_BASE);
    if(start == MAP_FAILED)
    {
        printf("buffer map error\n");
        return -1;
    }

    printf("%x%x%x%x \n", *(volatile unsigned int *)(start+0x0c),*(volatile unsigned int *)(start+0x0d),*(volatile unsigned int *)(start+0x0e),*(volatile unsigned int *)(start+0x0f)); //打印该寄存器地址的value

//	gettimeofday(&tpstart,NULL);

//	memcpy(buf,start,1024);
	
//	printf("start: 0x%x,buf:%s\n",start,buf); 

//	memcpy(start,buf,16*1024*1024);
//	gettimeofday(&tpend,NULL);
//	timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec; 
//	timeuse/=1000000; 
//	printf("Used Time:%f\n",timeuse); 
	
//	memcpy(start,buf,1024*1024);	

    /* 读出数据 */
//    strcpy(buf,start);
//    sleep (1);
	
//    printf("buf 1 = %s\n",buf);
#if 0
    for(i=0; i<10;i++)
    {
    	printf("start %d =0x%x\n",i,(unsigned int)(start++));    
    }
    /* 写入数据 */
    strcpy(start,"Buf Is Not Null!");
    
    memset(buf, 0, 100);
    strcpy(buf,start);
    sleep (1);
    printf("buf 2 = %s\n",buf);
#endif
       
    munmap(start,0xFF); /*解除映射*/
    free(buf);
    close(fd);  
    return 0;    
}
