/************************Copyright(c)*******************************
**                       GZ Smartteller
**                       
**                                     
**                       
**------------------------------------------FileInfo-------------------------------------------------------
** File name:                 main.c
** Last modified Date:      2017-04-26
** Last Version:              1.0
** Descriptions:            
**------------------------------------------------------------------------------------------------------
** Created by:               lzt
** Created date:            2017-04-25
** Version:                  1.0
** Descriptions:            The original version
**------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Descriptions:
*******************************************************************/
 
#include "pthread_h.h"

int fd; 
char recv_buf[serial_size];
char send_buf[serial_size]

volatile int wait_flag = noflag; 
volatile int STOP = 0;  

volatile int send_len = 0;
volatile int recv_len = 0;

pthread_t thread[3];
pthread_mutex_t mut;

void signal_handler_IO(int status)  
{  
	printf ("received SIGIO signale.\n"); 
	wait_flag = noflag;  
}  

/*******************************************************************
* 名称：                  UART0_Open
* 功能：                打开串口并返回串口设备文件描述
* 入口参数：        fd    :文件描述符     port :串口号(ttyPS0)
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int UART0_Open(int fd)
{ 
	fd = open(port, O_RDWR|O_NOCTTY|O_NDELAY);
	if (FALSE == fd)
	{
		perror("Can't Open Serial Port");
		return(FALSE);
	}
	//恢复串口为阻塞状态                               
	if(fcntl(fd, F_SETFL, 0) < 0)
	{
		printf("fcntl failed!\n");
		return(FALSE);
	}     
	else
	{
		printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
	}
	//测试是否为终端设备    
	if(0 == isatty(STDIN_FILENO))
	{
		printf("standard input is not a terminal device\n");
		return(FALSE);
	}
	else
	{
		printf("isatty success!\n");
	}              
		printf("fd->open=%d\n",fd);
		return fd;
}
/*******************************************************************
* 名称：                UART0_Close
* 功能：                关闭串口并返回串口设备文件描述
* 入口参数：        fd    :文件描述符     port :串口号(ttyS0,ttyS1,ttyS2)
* 出口参数：        void
*******************************************************************/
 
void UART0_Close(int fd)
{
    close(fd);
}
 
/*******************************************************************
* 名称：                UART0_Set
* 功能：                设置串口数据位，停止位和效验位
* 入口参数：        fd        串口文件描述符
*                              speed     串口速度
*                              flow_ctrl   数据流控制
*                           databits   数据位   取值为 7 或者8
*                           stopbits   停止位   取值为 1 或者2
*                           parity     效验类型 取值为N,E,O,,S
*出口参数：          正确返回为1，错误返回为0
*******************************************************************/
int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
   
	int   i;
	int   status;
	int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};
	int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};
		   
    struct termios options;
   
    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
    */
    if  ( tcgetattr( fd,&options)  !=  0)
       {
          perror("SetupSerial 1");    
          return(FALSE); 
       }
  
    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
         if  (speed == name_arr[i])
         {             
               cfsetispeed(&options, speed_arr[i]); 
               cfsetospeed(&options, speed_arr[i]);  
         }
    }     
   
    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;
  
    //设置数据流控制
    switch(flow_ctrl)
    {
      
       case 0 ://不使用流控制
              options.c_cflag &= ~CRTSCTS;
              break;   
      
       case 1 ://使用硬件流控制
              options.c_cflag |= CRTSCTS;
              break;
       case 2 ://使用软件流控制
              options.c_cflag |= IXON | IXOFF | IXANY;
              break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {  
       case 5    :
                 options.c_cflag |= CS5;
                 break;
       case 6    :
                 options.c_cflag |= CS6;
                 break;
       case 7    :    
                 options.c_cflag |= CS7;
                 break;
       case 8:    
                 options.c_cflag |= CS8;
                 break;  
       default:   
                 fprintf(stderr,"Unsupported data size\n");
                 return (FALSE); 
    }
    //设置校验位
    switch (parity)
    {  
       case 'n':
       case 'N': //无奇偶校验位。
                 options.c_cflag &= ~PARENB; 
                 options.c_iflag &= ~INPCK;    
                 break; 
       case 'o':  
       case 'O'://设置为奇校验    
                 options.c_cflag |= (PARODD | PARENB); 
                 options.c_iflag |= INPCK;             
                 break; 
       case 'e': 
       case 'E'://设置为偶校验  
                 options.c_cflag |= PARENB;       
                 options.c_cflag &= ~PARODD;       
                 options.c_iflag |= INPCK;      
                 break;
       case 's':
       case 'S': //设置为空格 
                 options.c_cflag &= ~PARENB;
                 options.c_cflag &= ~CSTOPB;
                 break; 
        default:  
                 fprintf(stderr,"Unsupported parity\n");    
                 return (FALSE); 
    } 
    // 设置停止位 
    switch (stopbits)
    {  
       case 1:   
           options.c_cflag &= ~CSTOPB; break; 
       case 2:   
           options.c_cflag |= CSTOPB; break;
       default:   
     		fprintf(stderr,"Unsupported stop bits\n"); 
     		return (FALSE);
    }
   
    options.c_lflag &= ~( ECHO | ECHOE | ISIG);
    options.c_lflag |=ICANON; //关闭ICANON标志就使终端处于非规范模式 现在处于打开 处于规范模式下
    options.c_iflag &= ~(BRKINT | ICRNL | IXON | INPCK | ISTRIP); //屏蔽作用控制符，不屏蔽会过滤0x0d 0x11 0x13
    
    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */  
    options.c_cc[VMIN] = 0; /* 读取字符的最少个数为1 */
   
    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);
   
    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)  
    {
        perror("com set error!\n");  
        return (FALSE); 
    }
    return (TRUE); 
}
/*******************************************************************
* 名称： UART0_Init()
* 功能： 串口初始化
* 入口参数： fd       :  文件描述符   
*           speed  :  串口速度
*           flow_ctrl  数据流控制
*           databits   数据位   取值为 7 或者8
*           stopbits   停止位   取值为 1 或者2
*           parity     效验类型 取值为N,E,O,,S
*                      
* 出口参数：     正确返回为1，错误返回为0
*******************************************************************/
int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
	int err;
	//设置串口数据帧格式
	if (UART0_Set(fd,speed,flow_ctrl,databits,stopbits,parity) == FALSE)
	{                                                         
		return FALSE;
	}
	else
	{
		return  TRUE;
	}
}
 
/*******************************************************************
* 名称：                  UART0_Recv
* 功能：                接收串口数据
* 入口参数：        fd                  :文件描述符    
*                              rcv_buf     :接收串口中数据存入rcv_buf缓冲区中
*                              data_len    :一帧数据的长度
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int UART0_Recv()
{
	int i,nread = 0;
	char ask[2] = {0x10,0x06};
	memset (recv_buf, 0, sizeof(recv_buf));

	while (STOP == 0)  
	{  	
		usleep (100000);  
		/* after receving SIGIO ,wait_flag = FALSE,input is availabe and can be read */  
		if (wait_flag == 0)  
		{  
			
			nread = read(fd, recv_buf, serial_size);
			if(nread > 0)
			{
				recv_len += nread;
			}
			else if(nread == -1)
			{
				printf("read failed!\n");
				return -1;
			}

			if(recv_buf[5]+11 == recv_len)
			{
				STOP = 1;
				recv_len = 0; 
			}
			else if(recv_buf[0] == 0x10 && recv_buf[1]==0x05)
			{
				recv_len = 0;
				//usleep(100000);
				memset(ask,0xFF,2);
				write(fd,ask,2);
			}			 			
		}  
	}	

	//close(fd);
	//return 1;
}
/********************************************************************
* 名称：                  UART0_Send
* 功能：                发送数据
* 入口参数：        fd                  :文件描述符    
*                              send_buf    :存放串口发送数据
*                              data_len    :一帧数据的个数
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int UART0_Send()
{
	int len = 0;
					
	if(STOP)
	{
		len = write(fd,send_buf,send_len);
		if (len == data_len )
		{
			return len;
		}     
		else   
		{
			tcflush(fd,TCOFLUSH);
			return FALSE;
		}
   	}
}


void *pthread_serial_recv()
{
	printf ("thread_recv : I'm thread 1\n");
	int bytes_read = 0;

	pthread_mutex_lock(&mut);

	UART0_Recv();
	
	//printf("I'm serial task!\n");

	pthread_mutex_unlock(&mut);
	sleep(2);	

	pthread_exit(NULL);

}

void *pthread_serial_send()
{
	printf ("thread_send : I'm thread 2\n");
	int bytes_write = 0;

	pthread_mutex_lock(&mut);

	UART0_Send();
	
	//printf("I'm serial task!\n");

	pthread_mutex_unlock(&mut);
	sleep(1);	

	pthread_exit(NULL);

}


void *pthread_usb()
{
	printf ("thread_usb : I'm thread 3\n");
	pthread_mutex_lock(&mut);

	printf("I'm usb task!\n");
			
	pthread_mutex_unlock(&mut);
	sleep(3);	

	pthread_exit(NULL);
}

void thread_create(void)
{
	int temp;
	memset(&thread, 0, sizeof(thread));          
	/*创建线程*/
	if((temp = pthread_create(&thread[0], NULL, pthread_serial_recv, NULL)) != 0)     
		printf("pthread 1 create failed!\n");
	else
		printf("pthread 1 was created\n");

	if((temp = pthread_create(&thread[1], NULL, pthread_serial_send, NULL)) != 0)  
	   	printf("pthread 2 create failed!\n");
	else
		printf("pthread 2 was created\n");

     //   if((temp = pthread_create(&thread[2], NULL, pthread_usb, NULL)) != 0)  
     //           printf("pthread 3 create failed!\n");
     //   else
     //           printf("pthread 3 was created\n");
}

void thread_wait(void)
{
	/*等待线程结束*/
	if(thread[0] !=0)
	{               
		pthread_join(thread[0],NULL);
		printf("pthread 1 has been over\n");
	}
	if(thread[1] !=0) 
	{  
		pthread_join(thread[1],NULL);
		printf("pthread 2 has been over\n");
	}
}

void delay(unsigned long num)
{
	unsigned long i;
	for(i=0; i<num; i++)
	{
		
	}
}

unsigned short CRC_X16plusX15plusX2plus1(unsigned char *pbyData, unsigned long dwDataLen)
{
	#if 0
unsigned short wDataCRC = 0xFFFF;
	unsigned short wDataTemp1,wDataTemp2;

	while(dwDataLen--)
	{
		wDataTemp1 = (unsigned char)*(pbyData++);
		for(int i=0;i<8;i++)
		{
			wDataTemp2 = wDataCRC^wDataTemp1;
			wDataCRC >>= 1;
			if(wDataTemp2 & 0x01)
			{
				wDataCRC ^= 0xA001;
			}
			wDataTemp1 >>= 1;
		}
	}

	return wDataCRC;
#endif
}

int main()
{
	int err;                           //返回调用函数的状态
	int i,len;                             
	char *send_buf = "I'm ready!";
	struct sigaction saio; 

      pthread_mutex_init(&mut,NULL);/*用默认属性初始化互斥锁*/

	fd = UART0_Open(fd); //打开串口，返回文件描述符
    
	do{
            err = UART0_Init(fd,57600,0,8,1,'N');
            printf("Set Port Exactly!\n");
       }while(FALSE == err || FALSE == fd);
   

//	len = UART0_Send(fd,send_buf,(strlen(send_buf)-1));
//	if(len > 0)
//	   printf("send data successful\n");
//	else
//	   printf("send data failed!\n");

	/*通过软中断方式，使用信号signal机制读取串口，这里需要注意的是硬件中断是设备驱动层级的，
	而读写串口是用户级行为，只能通过信号机制模拟中断，信号机制的发生和处理其实于硬件中断无异*/
	saio.sa_handler = signal_handler_IO;  //通过signal机制读取数据
	sigemptyset (&saio.sa_mask);  
	saio.sa_flags = 0;  
	saio.sa_restorer = NULL;  
	sigaction (SIGIO, &saio, NULL);

	fcntl (fd, F_SETOWN, getpid());  //allow the process to receive SIGIO  
	fcntl (fd, F_SETFL, FASYNC);      //make the file descriptor asynchronous  

	thread_create();
	
	while(1)
	{
		delay(0xA000000);

		printf("I'm main task!!\n");
				
		if(wait_flag)
		{
			printf("recv len: %d \n",recv_len);
			for(i=0;i<recv_len;i++)
			{
				printf("0x%X ",recv_buf[i]);
				if(i%7 == 0)
				{
					printf("\n");
				}
			}
		}		
	}
	printf("wait!\n");
      thread_wait();       	
}
  
/*********************************************************************                            End Of File                          **
*******************************************************************/

