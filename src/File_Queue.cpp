#include "File_Queue.h"


char wenjianxinname[256];
char filename[256][256];
short len_findinfo ;

void  create_file(const char *filename)
{
	int fd;
	fd = open(filename, O_CREAT | O_RDWR | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	close(fd);
}  

int trave_dir(char* path, int depth)
{
    DIR *d;
    struct dirent *file;
    struct stat sb;    
    if(!(d = opendir(path)))
    {
        printf("error opendir %s!!!/n",path);
        return -1;
    }
    while((file = readdir(d)) != NULL)
    {
        if(strncmp(file->d_name, ".", 1) == 0)
            continue;
        strcpy(filename[len_findinfo++], file->d_name);
        if(stat(file->d_name, &sb) >= 0 && S_ISDIR(sb.st_mode) && depth <= 1)
        {
            trave_dir(file->d_name, depth + 1);
        }
    }
    closedir(d);
    return 0;
}


int EmptyDir(const char *destDir)  
{  
    DIR *dp;  
    struct dirent *entry;  
    struct stat statbuf;  
    if ((dp = opendir(destDir)) == NULL)  
    {  
        fprintf(stderr, "cannot open directory: %s\n", destDir);  
        return -1;  
    }  
    chdir (destDir);  
    while ((entry = readdir(dp)) != NULL)  
    {  
        lstat(entry->d_name, &statbuf);  
        if(S_ISREG(statbuf.st_mode))  
        {  
            remove(entry->d_name);  
        }  
    }  
    return 0;  
} 

//创建保存文件类型   H264 = 1;JPG = 2
//#define H264 1
//#define JPG 2
char *new_264 (int TYPE)
{
	char pinjie264[200];
	char pinjie2[200];
	int file_exist=0;
	char shanroot[200]="/camera_insmod/";
	char shanroot3[200]="/camera_insmod/";
	char shanroot1[200]="/root/";
	DIR *directory_pointer;
	struct dirent *entry;
	if((directory_pointer=opendir("/camera_insmod/"))==NULL)
	{  
		printf("Error open\n");  
	}
	else
	{  
		while((entry=readdir(directory_pointer))!=NULL)
		{  
			if(entry->d_name[0]=='.')  
				continue;  	
			int size = strlen(entry->d_name);  
			if(strcmp(( entry->d_name+ (size - 4) ) , ".txt")!= 0)
			{
				continue;
			}
			file_exist = 1;
	
			//printf("%s  size=%d\n",entry->d_name,size);  
			memset(pinjie264,0,sizeof(pinjie264)); 
			
			sprintf(pinjie264, "%d", atoi(entry->d_name));
			//printf("entry->d_name = %s \n", entry->d_name);
			if(TYPE == H264)
				strcat(pinjie264, ".264");
			else if(TYPE == JPG)
				strcat(pinjie264, ".jpg");           //修改前.jpg
			else
			{
				printf(" File Type error !\n");
				return NULL;
			}
			strcat(shanroot1, pinjie264);
			strncpy(wenjianxinname,shanroot1,sizeof(shanroot1));
			
			strcat(shanroot, entry->d_name);
			remove(shanroot);
			sprintf(pinjie2, "%d", atoi(entry->d_name)+1);
			strcat(pinjie2, ".txt");
			strcat(shanroot3, pinjie2);
			create_file(shanroot3);		
			closedir(directory_pointer);        //导致打开文件太多超过系统预置的1024   ，必须时刻关闭
		}
	}
	if(file_exist==0)
	{
		create_file("0.txt");
		EmptyDir("/root/"); 
		sprintf(pinjie264, "%d", 0);
		if(TYPE==1)
		{
			strcat(pinjie264, ".264");
		}
		else if(TYPE==2)
		{
			strcat(pinjie264, ".jpg");
		}
		strcat(shanroot1, pinjie264);
		strncpy(wenjianxinname,shanroot1,sizeof(shanroot1));
	
		strcat(shanroot, "0.txt");
		remove(shanroot);
		
		sprintf(pinjie2, "%d", 1);
		strcat(pinjie2, ".txt");
		strcat(shanroot3, pinjie2);
		create_file(shanroot3);
	}
	return wenjianxinname;
}


int QueueInit(SqQueue* Q, int maxLength)
{
	if (!Q->base)
	{
		return 0;
	}
	Q->front = Q->rear = 0;
	Q->maxLength = maxLength;
	return 1;
}

int QueueLength(SqQueue *Q)
{
	if((Q->maxLength) >0)
	{
		return (Q->rear - Q->front + Q->maxLength)%(Q->maxLength);
	}
	return 1;
}


int QueuePopFront(SqQueue *Q, char *pDst, int num, int bEraseFlag)
{
	int i;
	if (QueueLength(Q) < num)
	{
		return 0;
	}
	if((Q->maxLength) >0)
	{
		if(pDst)
		{
			for (i = 0; i < num; i++)
			{
				pDst[i] = Q->base[(Q->front+i)% (Q->maxLength)];
			}	
		}
		if(bEraseFlag) //重新定义环形队列头部，即擦除之前的数据
		{
			Q->front = (Q->front + num) % (Q->maxLength);
		}
	}
	return 1;
}


int QueuePushBack(SqQueue *Q, char *pSrc, int num)
{
	int i;
	if((Q->maxLength) >0)
	{
		if ((Q->rear + num)%(Q->maxLength) == Q->front)
		{
			return 0;
		}
		for (i = 0; i < num; i++)
		{
			Q->base[Q->rear] = (pSrc[i] & 0x00FF);     //remove the highbyte
			Q->rear = (Q->rear + 1) %(Q->maxLength);
		}
	}
	return 1;
}


//比较队列pos位置开始的num个数据是否一致
int QueueCompare(SqQueue *Q, int pos, char *pDst, int num)
{
	int i;
	if (QueueLength(Q) < (pos + num))
	{
		return 0;
	}
	if (!pDst)
	{
		return 0;
	}
	if((Q->maxLength) >0)
	{
		for (i = 0; i < num; i++)
		{
			if (Q->base[(Q->front + pos + i) % (Q->maxLength)] != pDst[i])
			{
				return 0;
			}
		}
	}
	return 1;
}


//比较数据的头部 和 尾部 数据是否一致***********
int FrameCompare(SqQueue *Q, char *pHeadSrc, int headnum, char *pTailSrc, int tailnum, int framelength)
{
	if (QueueLength(Q) < framelength)
	{
		return 0;
	}
	if ((QueueCompare(Q, 0, pHeadSrc, headnum)) && (QueueCompare(Q, (framelength - tailnum), pTailSrc, tailnum)))
	{
		return 1;
	}
	return 0;
}

