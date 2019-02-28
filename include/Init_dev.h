#ifndef _INIT_DEV_H_
#define _INIT_DEV_H_

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

int Uart_Init(int* UART_fd, const char *file_name, unsigned int BaudRate); //串口初始化


void Uart_Ack(int fd, char *pBuf, int pBufLen);


#endif







