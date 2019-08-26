#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <netinet/in.h> 
#include <sys/types.h>    
#include <sys/socket.h>   
#include <semaphore.h>
#include <errno.h>
#include "axidma_chrdev.h"
#include "tx_10m_i.h"
#include "tx_10m_q.h"


#define PAGE 4096
#define BUF_SIZE 512*PAGE
#define OFFSET 2048*PAGE
#define SEND_LEN (5120*sizeof(int))

#define ARRAY_LEN(x) sizeof(x) / sizeof(x[0])

static int fd;
static char *send_buf=NULL,*recv_buf=NULL;
static int pipe_fd[2];

#define RX_SERVER_PORT    5555 
#define LENGTH_OF_LISTEN_QUEUE 20
#define BUFFER_SIZE 1024
#define FILE_NAME_MAX_SIZE 512
#if 1

int sock_init()
{
    struct sockaddr_in server_addr;
    bzero(&server_addr,sizeof(server_addr)); 
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htons(INADDR_ANY);
    server_addr.sin_port = htons(RX_SERVER_PORT);
	int ret=0;

    int server_socket = socket(PF_INET,SOCK_STREAM,0);
    if( server_socket < 0)
    {
        printf("Create Socket Failed!");
		return server_socket;        
    }
    int opt =1;
    setsockopt(server_socket,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof(opt));

    if( ret=bind(server_socket,(struct sockaddr*)&server_addr,sizeof(server_addr)))
    {
        printf("Server Bind Port : %d Failed!", RX_SERVER_PORT); 
		return ret;        
    }

    if ( ret=listen(server_socket, LENGTH_OF_LISTEN_QUEUE) )
    {
        printf("Server Listen Failed!"); 
		return ret;         
    }
	return server_socket;
}

struct queue_data
{
	sem_t empty_sem;
	sem_t full_sem;
	int empty_offset;
	int  full_offset;
	pthread_mutex_t queue_mutex;
	int queue_size;
};

static struct queue_data rx_queue;
static struct queue_data tx_queue;

#define QUEUE_SIZE 5

void init_queue(struct queue_data *queue ,int   queue_size)
{
	int ret;
	queue->queue_size=queue_size;
	queue->empty_offset=0;
	queue->full_offset=0;
	ret=pthread_mutex_init(&queue->queue_mutex,NULL);
	if(ret)
	{
		printf("pthread_mutex_init error\n");
	}
	
	ret=sem_init(&queue->empty_sem, 0, (unsigned int)queue->queue_size-1);	
	if(ret)
	{
		printf("sem_init empty_sem error\n");
	}
	
	ret=sem_init(&queue->full_sem, 0, 0);	
	if(ret)
	{
		printf("sem_init full_sem error\n");
	}
}

void dma_buf_init()
{	

	fd=open("/dev/axidma0",O_RDWR);
	if(fd<0)
	{
		printf("open axidma0 error\n");
		return -1;
	}

	send_buf=mmap(NULL, BUF_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
	recv_buf=mmap(NULL, BUF_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, OFFSET);

	memset(send_buf,'\0',BUF_SIZE);
	memset(recv_buf,'\0',BUF_SIZE);

	init_queue(&rx_queue,QUEUE_SIZE);
	init_queue(&tx_queue,QUEUE_SIZE);
}

void * dma_tx_buf(void *arg)
{	
	int rc=0;
	struct axidma_transaction tx_trans;
    while(1)
    {
		if(sem_wait(&tx_queue.full_sem)!=0)
		{
			if(errno == EINTR){
				printf("sem_wait interrupted \n");
				continue;
			}
			printf("sem_wait error\n");
			continue;
		}
		
        tx_trans.buf_offset=tx_queue.full_offset*SEND_LEN;
        tx_trans.buf_len=SEND_LEN;
        rc = ioctl(fd, AXIDMA_DMA_WRITE, &tx_trans);
        if(rc)
            printf("dma write error rc=%d\n",rc);

//		memset(buf,'\0',SEND_LEN);
		tx_queue.full_offset=(tx_queue.full_offset+1)%tx_queue.queue_size;
		sem_post(&tx_queue.empty_sem);
    }
}

static int get_bits_data(FILE *fp,char *intputBit,int size)
{
    int i, ret;
    ret=fread(intputBit, 1, size, fp);
	if(ret<size)
	{
		fseek(fp,0L,SEEK_SET)	;
		return -1;
	}
    for(i=0;i<size;i++)
    {
        intputBit[i]-=48;
    }
	return ret;
}

static void save_bits_data(char *filename,char outputBit[],int size)
{
    FILE *fp;
    int i, d;
    fp=fopen(filename,"w");
    if(fp==NULL)
    {
        printf("%s cannot open!\n ",filename );
        exit(0);
    }

    for(i=0;i<size;i++)
    {
        fprintf(fp,"%d",outputBit[i]);
    }
    fclose(fp);
}

extern int encode(char *inputBit,int len ,int *output);

#define CODE_LEN 1024

void * socket_recv_thread(void *arg)
{
	int ret=0;
//	int server_socket=sock_init();
	char *buf=NULL;
	FILE *fstream=fopen("/nfs/input_bits.txt","r");
	if(fd<0)
	{
		printf("open rx.txt error\n");
		return -1;
	}
	char inputBit[CODE_LEN];	
    while (1)
    {
		
	    ret=get_bits_data(fstream,inputBit,CODE_LEN);
		if(ret<0)
		{

			continue;
		}		
//		printf("ret=%d\n",ret);
#if 1    
		if(sem_wait(&tx_queue.empty_sem)!=0)
		{
			if(errno == EINTR){
				printf("sem_wait interrupted \n");
				continue;
			}
			printf("sem_wait error\n");
			continue;
		}
		
		buf=send_buf+tx_queue.empty_offset*SEND_LEN;
		encode(inputBit,CODE_LEN ,(int*)buf);
#if 0		
		fread(buf,SEND_LEN,1,fstream);
//#else
		int i=0;
//		memset(buf,'\0',SEND_LEN);		
		for(i=0;i<5120;i++){
//			((int*)buf)[i]=(tx_10m_i[i]<<16)|tx_10m_q[i];
				printf("%8.x,",((int*)buf)[i]);
		}
		printf("\n");
//		return NULL;
#endif
		tx_queue.empty_offset=(tx_queue.empty_offset+1)%tx_queue.queue_size;
		sem_post(&tx_queue.full_sem);
#endif		
    	}
}


void * socket_send_thread(void *arg)
{
	int ret=0;
//	int server_socket=sock_init();
	char *buf=NULL;
#if 1
//	mkfifo("/tmp/fifo", S_IFIFO|0666);
	FILE *fstream=fopen("/nfs/rx.txt","w");
	if(fstream==NULL)
	{
		printf("open rx.txt error\n");
	//	return -1;
	}
#endif

	bool start=true;	
	struct timeval tv0,tv1;
	long long ts0=0,ts1=0;
	int count=0;
    while (1)
    {
		
#if 1    
		if(sem_wait(&rx_queue.full_sem)!=0)
		{
			if(errno == EINTR){
				printf("sem_wait interrupted \n");
				continue;
			}
			printf("sem_wait error\n");
			continue;
		}
		buf=recv_buf+rx_queue.full_offset*SEND_LEN;
//		printf("process rx buf\n");
//		fwrite(buf,SEND_LEN,1,fstream);
#if 0		
		int i;
		printf("\n");
		for(i=0;i<1024;i++)
			printf("%x,",*(buf+i));
//			printf("%.8x,",*(int*)(buf+i));
		printf("\n");
#endif		
//		ret=send(server_socket,buf,SEND_LEN,0);
//		printf("process rx buf\n");
//		memset(buf,'\0',SEND_LEN);
		rx_queue.full_offset=(rx_queue.full_offset+1)%rx_queue.queue_size;
		sem_post(&rx_queue.empty_sem);
#endif		


#if 0	
		if(send(server_socket,buf,SEND_LEN,0)<0)
		{
			printf("Send Failed\n");
			break;
		}

#endif		
#if 0    
        struct sockaddr_in client_addr;
        int length = sizeof(client_addr);

        int new_server_socket = accept(server_socket,(struct sockaddr*)&client_addr,&length);
        if ( new_server_socket < 0)
        {
            printf("Server Accept Failed!\n");
            break;
        }
		
        char buffer[BUFFER_SIZE];
        bzero(buffer, BUFFER_SIZE);
		while(1){
	        bzero(buffer, BUFFER_SIZE);
	        if(send(new_server_socket,buffer,BUFFER_SIZE,0)<0)
	        {
	            printf("Send Failed\n");
	            break;
	        }
		}
        close(new_server_socket);
#endif			
    }
    //关闭监听用的socket
    fclose(fstream);
//    close(server_socket);
    return ret;
}
#endif




void * dma_rx_buf(void *arg)
{
	int ret=0;
	char *put_buf=NULL;
	struct axidma_transaction rx_trans;
	
	bool start=true;	
	struct timeval tv0,tv1;
	long long ts0=0,ts1=0;
	int count=0;
    while(1)
    {
#if 1    
   		if(sem_wait(&rx_queue.empty_sem)!=0)
		{
			if(errno == EINTR){
				printf("sem_wait interrupted \n");
				continue;
			}
			printf("sem_wait error\n");
			continue;
   		}
#endif		
		put_buf=recv_buf+rx_queue.empty_offset*SEND_LEN;
		memset(put_buf,'\0',SEND_LEN);		
        rx_trans.buf_offset=rx_queue.empty_offset*SEND_LEN;
        rx_trans.buf_len=SEND_LEN;
//		printf("AXIDMA_DMA_READ\n");
        ret = ioctl(fd, AXIDMA_DMA_READ, &rx_trans);
        if(ret)
            printf("dma rx error ret=%d\n",ret);
		rx_queue.empty_offset=(rx_queue.empty_offset+1)%rx_queue.queue_size;
#if 1		
		sem_post(&rx_queue.full_sem);
#endif

#if 1		
		if(start==true)
		{
			gettimeofday(&tv0, NULL);
			ts0 = (long long)tv0.tv_sec*1000 + tv0.tv_usec/1000;
			start=false;
		}
		count++;
		gettimeofday(&tv1, NULL);
		ts1 = (long long)tv1.tv_sec*1000 + tv1.tv_usec/1000;

		if((ts1-ts0)>=1000){
			printf("%dkB/s\n",(count*1024)/(1024));
//			break;
			start=true;
			count=0;
		}
#endif		
    }
	return NULL;
}


void dma_tx_rx()
{
	dma_buf_init();	 
	pipe(pipe_fd);
	pthread_t tx_thread;
	pthread_create(&tx_thread, NULL,dma_tx_buf,NULL);

	pthread_t rx_thread;
	pthread_create(&rx_thread, NULL,dma_rx_buf,NULL);

	pthread_t socket_send_thread_t;
	pthread_create(&socket_send_thread_t, NULL,socket_send_thread,NULL);

	pthread_t socket_recv_thread_t;
	pthread_create(&socket_recv_thread_t, NULL,socket_recv_thread,NULL);

    return 0;
}
