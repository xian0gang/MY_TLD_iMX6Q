#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <unistd.h>     /*Unix标准函数定义*/
#include <sys/types.h>  /**/
#include <sys/stat.h>   /**/
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <sys/time.h>
#include <string.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include "Init_dev.h"



//***********************************************************************
// TCP_Init()：  TCP服务端初始化
//
//***********************************************************************
int TCP_SERVER_Init(int* TCP_socketCon, int TCP_SERVER_PORT)
{
	/* 创建TCP连接的Socket套接字 */
	*TCP_socketCon = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if(*TCP_socketCon < 0)
	{  
		printf("创建TCP连接套接字失败\n");  
		return -1;	
	}

	struct sockaddr_in TCP_Server_addr;   //定义sockaddr_in 网络套接字
	bzero(&TCP_Server_addr,sizeof(struct sockaddr_in)); 
	TCP_Server_addr.sin_family = AF_INET;
	TCP_Server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	//TCP_Server_addr.sin_addr.s_addr = inet_addr(TCP_SERVER_IP);
	TCP_Server_addr.sin_port = htons(TCP_SERVER_PORT);

	///bind，成功返回0，出错返回-1
	if(bind(*TCP_socketCon, (struct sockaddr *)&TCP_Server_addr, sizeof(TCP_Server_addr))==-1)
	{
		perror("TCP bind error !!!\n");
		return -1;
	}

	printf("listen %d port !!!\n", TCP_SERVER_PORT);
	///listen，成功返回0，出错返回-1
	if(listen(*TCP_socketCon, 5) == -1)
	{
		perror("TCP listen error !!!\n");
		return -1;
	}

	return 0;
}

//***********************************************************************
// TCP_CLIENT_Init()：  TCP客户端初始化
//
//***********************************************************************
int TCP_CLIENT_Init(int* TCP_socketCon, char* TCP_SERVER_IP, int TCP_SERVER_PORT)
{
	/* 创建TCP连接的Socket套接字 */  
	*TCP_socketCon = socket(AF_INET, SOCK_STREAM, 0);  
	if(*TCP_socketCon < 0)
	{  
		printf("创建TCP连接套接字失败\n");  
		return -1;  
	}  
	/* 填充客户端端口地址信息，以便下面使用此地址和端口监听 */  
	struct sockaddr_in server_addr;  
	bzero(&server_addr,sizeof(struct sockaddr_in));  
	server_addr.sin_family = AF_INET;  
	server_addr.sin_addr.s_addr = inet_addr(TCP_SERVER_IP);//服务端IP  
	server_addr.sin_port = htons(TCP_SERVER_PORT);         //服务端端口号
	//printf("连接之前的socketCon:%d",TCP_socketCon);  
	
	/* 连接服务器 */  
	int res_con = connect(*TCP_socketCon,(struct sockaddr *)(&server_addr),sizeof(struct sockaddr));  
	if(res_con != 0)
	{  
		// 关闭套接字	
		close(*TCP_socketCon); 			
		return -1;  
	}  
	printf("连接TCP服务端成功****\n");  
	return 0;
}


/*@brief  设置串口通信速率
*@param  fd     类型 int  打开串口的文件句柄
*@param  speed  类型 int  串口速度
*@return void
*/
#define TRUE 1
#define FALSE -1

int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300 };
int name_arr[]  = { 115200,  38400,  19200,  9600,  4800,  2400,  1200,  300 };

void set_speed(int fd, int speed)
{
	int   i;
	int   status;
	struct termios   Opt;
	tcgetattr(fd, &Opt);
	for(i=0; i<sizeof(speed_arr)/sizeof(int); i++)
	{
		if(speed == name_arr[i])
		{
		    tcflush(fd, TCIOFLUSH);
			/*  设置串口的波特率 */
			cfsetispeed(&Opt, speed_arr[i]);
			cfsetospeed(&Opt, speed_arr[i]);
			status = tcsetattr(fd, TCSANOW, &Opt);
			if(status != 0)
		        perror("tcsetattr fd1");
		 	return;
	 	}
		tcflush(fd,TCIOFLUSH);
	}
}

/**
*@brief   设置串口数据位，停止位和效验位
*@param  fd       类型  int  打开的串口文件句柄*
*@param  databits 类型  int  数据位   取值为 7 或者8*
*@param  stopbits 类型  int  停止位   取值为 1 或者2*
*@param  parity   类型  char 效验类型 取值为 N,E,O,S
*/
int set_Parity(int fd, int databits, int stopbits, char parity)
{
	struct termios options;
	if(tcgetattr(fd,&options) != 0)
	{
		perror("SetupSerial 1");
		return(FALSE);
	}
	options.c_cflag &= ~CSIZE;
	switch (databits) /*设置数据位数*/
	{
	  	case 7:
	  		options.c_cflag |= CS7;
	  		break;
	  	case 8:
			options.c_cflag |= CS8;
			break;
		default:
			fprintf(stderr,"Unsupported data size\n");
			return (FALSE);
	}
	
	switch (parity)
  	{
	  	case 'n':
		case 'N':
			options.c_cflag &= ~PARENB;   /* Clear parity enable */
			options.c_iflag &= ~INPCK;     /* Enable parity checking */
			options.c_iflag &= ~(ICRNL|IGNCR);
			options.c_lflag &= ~(ICANON | ECHO);       // 设置输入模式为非标准输入  ,取消ICANON,   取消ECHO 回显
			// options.c_lflag &= ~(ECHO);             // 设置输入模式为非标准输入  ,取消ICANON   ,取消ECHO 回显
			// options.c_lflag |= ICANON;              // 设置输入模式为标准输入模式
		    options.c_lflag = 0;
			break;
		case 'o':
		case 'O':
			options.c_cflag |= (PARODD | PARENB);  /* 设置为奇效验*/ 
			options.c_iflag |= INPCK;             /* Disnable parity checking */
			break;
		case 'e':
		case 'E':
			options.c_cflag |= PARENB;    /* Enable parity */
			options.c_cflag &= ~PARODD;   /* 转换为偶效验*/  
			options.c_iflag |= INPCK;     /* Disnable parity checking */
			break;
		case 'S':
		case 's':  /*as no parity*/
			options.c_cflag &= ~PARENB;
			options.c_cflag &= ~CSTOPB;
			break;
		default:
			fprintf(stderr,"Unsupported parity\n");
			return (FALSE);
	}

	/* 设置停止位*/   
	switch (stopbits)
	{
		case 1:
			options.c_cflag &= ~CSTOPB;
			break;
		case 2:
			options.c_cflag |= CSTOPB;
			break;
		default:
			fprintf(stderr,"Unsupported stop bits\n");
			return (FALSE);
	}

	/* Set input parity option */
	if (parity != 'n')
		options.c_iflag |= INPCK;
	//设置等待时间和最小接收字符
	options.c_cc[VTIME] = 0;   // 1/10 second    /* 读取一个字符等待num*(1/10)s */  
	options.c_cc[VMIN]  = 0;                     /* 读取字符的最少个数为1 */
/*
	输入模式为非标准输入***************************************
	1.VTIME=0,VMIN=0 :接收到字符(至少一个)立即返回；
	2.VTIME=5,VMIN=0 :接收到字符(至少一个)立即返回，字符间隔5/10s；
	3.VTIME=5,VMIN=3 :接收到(至少3个)字符返回或者字符间隔超过5/10s；
	4.VTIME=0,VMIN=3 :接收到(至少3个)字符才返回；
	且注意串口打开方式：含有O_NDELAY 则返回结果如同现象1.VTIME=0,VMIN=0；
*/
	//如果发生数据溢出，接收数据，但是不再读取
	tcflush(fd,TCIFLUSH); /* Update the options and do it NOW */
	//激活配置 (将修改后的termios数据设置到串口中）
	if (tcsetattr(fd,TCSANOW,&options) != 0)
	{
		perror("SetupSerial 3");
		return(FALSE);
	}
	return (TRUE);
}

void set_serial_Non_Canonical(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    set_speed(fd, nSpeed);     //设置波特率
	set_Parity(fd, nBits, nStop, nEvent);  //设置传递参数 
}


int set_serial(int fd,int nSpeed,int nBits,char nEvent,int nStop)
{
    struct termios newttys1,oldttys1;

	/*保存原有串口配置*/  
	if(tcgetattr(fd,&oldttys1)!=0)
	{  
		perror("Setupserial 1");
		return -1;  
	}  
	memset(&newttys1, 0, sizeof(newttys1));/* 先将新串口配置清0 */  
	newttys1.c_cflag |= (CLOCAL|CREAD ); /* CREAD 开启串行数据接收，CLOCAL并打开本地连接模式 */  

	newttys1.c_cflag &=~CSIZE;/* 设置数据位 */
	/* 数据位选择 */  
	switch(nBits)  
	{  
		case 7:  
			newttys1.c_cflag |=CS7;  
			break;  
		case 8:  
			newttys1.c_cflag |=CS8;  
			break;  
	}  
	/* 设置奇偶校验位 */  
	switch( nEvent )  
	{  
		case '0': /* 奇校验 */  
			newttys1.c_cflag |= PARENB;/* 开启奇偶校验 */  
			newttys1.c_iflag |= (INPCK | ISTRIP);/*INPCK打开输入奇偶校验；ISTRIP去除字符的第八个比特  */  
			newttys1.c_cflag |= PARODD;/*启用奇校验(默认为偶校验)*/  
			break;  
		case 'E': /*偶校验*/  
			newttys1.c_cflag |= PARENB; /*开启奇偶校验  */  
			newttys1.c_iflag |= (INPCK | ISTRIP);/*打开输入奇偶校验并去除字符第八个比特*/  
			newttys1.c_cflag &= ~PARODD;/*启用偶校验*/  
			break;  
		case 'N': /*无奇偶校验*/  
			newttys1.c_cflag &= ~PARENB;  
			break;  
	}  
	/* 设置波特率 */  
    switch( nSpeed )  
    {  
        case 2400:  
            cfsetispeed(&newttys1, B2400);  
            cfsetospeed(&newttys1, B2400);  
            break;  
        case 4800:  
            cfsetispeed(&newttys1, B4800);  
            cfsetospeed(&newttys1, B4800);  
            break;  
        case 9600:  
            cfsetispeed(&newttys1, B9600);  
            cfsetospeed(&newttys1, B9600);  
            break;  
		case 19200:  
            cfsetispeed(&newttys1, B19200);  
            cfsetospeed(&newttys1, B19200);  
            break; 
		case 38400:  
            cfsetispeed(&newttys1, B38400);  
            cfsetospeed(&newttys1, B38400);  
            break; 
        case 115200:  
            cfsetispeed(&newttys1, B115200);  
            cfsetospeed(&newttys1, B115200);  
            break;  
        default:  
            cfsetispeed(&newttys1, B115200);  
            cfsetospeed(&newttys1, B115200);  
            break;  
    }  
    /*设置停止位*/  
    if( nStop == 1)/* 设置停止位；若停止位为1，则清除CSTOPB，若停止位为2，则激活CSTOPB */  
    {  
        newttys1.c_cflag &= ~CSTOPB;/*默认为一位停止位； */  
    }  
    else if( nStop == 2)  
    {  
        newttys1.c_cflag |= CSTOPB;/* CSTOPB表示送两位停止位 */  
    }  
  
    /* 设置最少字符和等待时间，对于接收字符和等待时间没有特别的要求时*/  
    newttys1.c_cc[VTIME] = 0; /* 非规范模式读取时的超时时间；*/  
    newttys1.c_cc[VMIN]  = 0; /* 非规范模式读取时的最小字符数*/  
    tcflush(fd ,TCIFLUSH);    /* tcflush清空终端未完成的输入/输出请求及数据；TCIFLUSH表示清空正收到的数据，且不读取出来 */  
  
    /*激活配置使其生效*/  
    if((tcsetattr(fd, TCSANOW,&newttys1))!=0)  
    {  
        perror("com set error");  
        exit(1);  
    }  
  
    return 0;  
}

//*********************
//#define Non_Canonical 1
//是否使用非标准模式
//*********************

int Uart_Init(int* UART_fd, const char *file_name, unsigned int BaudRate) //串口初始化
{
#if Non_Canonical
	if((*UART_fd = open(file_name,O_RDWR|O_NOCTTY))<0) 
	{	
		printf("Can't Open the %s Serial Port,Set BaudRate = %d!!!! \n", file_name, BaudRate);	
		return -1;
	} 
	set_serial_Non_Canonical(*UART_fd, BaudRate, 8, 'N', 1);
#else
	if((*UART_fd = open(file_name,O_RDWR|O_NOCTTY|O_NDELAY))<0) 
	{   
		printf("Can't Open the %s Serial Port,Set BaudRate = %d!!!! \n", file_name, BaudRate);  
		return -1;
	} 
	set_serial(*UART_fd, BaudRate, 8, 'N', 1);
#endif
	return 0;
}

void Uart_Ack(int fd, char *pBuf, int pBufLen)
{
	int ret;
	ret = write(fd, pBuf, pBufLen);
	if(ret<0)
		printf("Uart_Ack error!!!\n");
}





