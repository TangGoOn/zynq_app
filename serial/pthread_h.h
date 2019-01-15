//串口相关的头文件
#include<stdio.h>      /*标准输入输出定义*/
#include<stdlib.h>     /*标准函数库定义*/
#include<unistd.h>     /*Unix 标准函数定义*/
#include<sys/types.h> 
#include<sys/stat.h>   
#include<fcntl.h>      /*文件控制定义*/
#include<termios.h>    /*PPSIX 终端控制定义*/
#include<errno.h>      /*错误号定义*/
#include<string.h>
#include<sys/signal.h> 


 //宏定义
#define  port  "/dev/ttyPS0" 
#define FALSE  -1
#define TRUE   0
#define noflag 0  
#define serial_size	1024
#define flag  1 

extern int UART0_Open(int fd);
extern void UART0_Close(int fd);
extern int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);
extern int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity);
extern int UART0_Recv();
extern int UART0_Send();
extern unsigned short CRC_X16plusX15plusX2plus1(unsigned char *pbyData, unsigned long dwDataLen);






