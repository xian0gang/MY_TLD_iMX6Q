#ifndef _FILE_QUEUE_H_
#define _FILE_QUEUE_H_

#include <stdio.h>
#include <signal.h>

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <malloc.h>
#include <asm/types.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>


#define H264 1
#define JPG  2

typedef struct
{
	char* base;
	short front;	
	short rear;		
	short maxLength;
}SqQueue;


void create_file(const char *filename);
int trave_dir(char* path, int depth);
int EmptyDir(const char *destDir) ;
char *new_264(int type);


int QueueInit(SqQueue* Q, int maxLength);
int QueueLength(SqQueue *Q);
int QueuePopFront(SqQueue *Q,  char *pDst,  int num, int bEraseFlag);
int QueuePushBack(SqQueue *Q,  char *pSrc,  int num);


int QueueCompare(SqQueue *Q,  int pos, char *pDst,  int num);

//比较数据的头部 和 尾部 数据是否一致***********
int FrameCompare(SqQueue *Q, char *pHeadSrc, int headnum, char *pTailSrc, int tailnum, int framelength);


#endif
