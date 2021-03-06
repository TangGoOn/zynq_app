#include<stdio.h>  
#include<unistd.h>  
#include<sys/mman.h>  
#include<sys/types.h>  
#include<sys/stat.h>  
#include<fcntl.h>  
#include<stdlib.h>
#include<sys/time.h>
#include<string.h>

#define mmap_addr   0x2000000
#define mmap_size   32*1024*1024
#define buf_size    16*1024*1024

int main()  
{  
	unsigned int *map_base;  
	FILE *f;  
	int n, fd;  
	struct timeval tpstart,tpend; 
	float timeuse; 

	fd = open("/dev/mem", O_RDWR/*|O_SYNC*/);  
	if (fd == -1)  
	{  
	    return (-1);  
	}  

	map_base = mmap(NULL, mmap_size , PROT_READ|PROT_WRITE, MAP_SHARED, fd, mmap_addr);  

	if (map_base == 0)  
	{  
	    printf("NULL pointer!\n");  
	}  
	else  
	{  
	    printf("mmap Successful!\n");  
	}  

	unsigned long addr;  
	unsigned char content;  

	unsigned int *p = (unsigned int *)malloc(buf_size*sizeof(unsigned int));
	unsigned int *q = (unsigned int *)malloc(buf_size*sizeof(unsigned int));
//	unsigned int *p = (unsigned int *)calloc(buf_size,sizeof(unsigned int));
//	unsigned int *q = (unsigned int *)calloc(buf_size,sizeof(unsigned int));

	int i;  
	for(i=0;i<buf_size;i++)  
	{ 
		p[i] = i;
	}

	gettimeofday(&tpstart,NULL);

	memcpy(map_base,p,buf_size);
	memcpy(p,map_base,buf_size);

	gettimeofday(&tpend,NULL);
	
	timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec; 
	timeuse/=1000000; 
	printf("Used Time:%f\n",timeuse); 


#if 0
	for(i = 0; i < 16; ++i)
	{
		printf("address: 0x%lx  ddr content 0x%x\t", (unsigned long)(map_base+i), (unsigned int)map_base[i]); 
 		printf("address: 0x%lx  p content 0x%x\n",(unsigned long)p+i, p[i]); 
	}


  	for (i = 0;i < 16; ++i)  
    	{  
		addr = (unsigned long)(map_base + i);  
		map_base[i] = (unsigned char)i;  
		content = map_base[i];  
		printf("updated address: 0x%lx   ddr content 0x%x\t", addr, (unsigned int)content);  
		printf("address: 0x%lx  p content 0x%x\n",(unsigned long)p+i, p[i]); 
    	}  
#endif

	close(fd);  
	munmap(map_base, mmap_size);  
	return 0;  
}  
