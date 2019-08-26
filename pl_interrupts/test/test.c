#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include "axidma_chrdev.h"
#include "tx_10m_iq.h"

#define PAGE 4096
#define BUF_SIZE 512*PAGE
#define OFFSET 2048*PAGE
//#define SEND_LEN (16*PAGE-1)
//#define SEND_LEN 5120
#define SEND_LEN 512

#define LEN(x) sizeof(x) / sizeof(x[0])

int main()
{
    int fd;
    int rc=0;
    char *send=NULL,*recv=NULL;

    //printf("tx_10m len=%d\n",LEN(tx_10m));
    struct axidma_transaction tx_trans;
    struct axidma_transaction rx_trans;

    fd=open("/dev/axidma0",O_RDWR|O_EXCL);
    if(fd<0)
    {
        printf("open axidma0 error\n");
        return -1;
    }

    send=mmap(NULL, BUF_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    recv=mmap(NULL, BUF_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, OFFSET);
    memcpy(send,tx_10m,SEND_LEN);
    printf("send:%x\n",send);
    printf("recv:%x\n",recv);
    while(1)
    {
        //memset(send,'a',BUF_SIZE);
        memset(recv,'b',BUF_SIZE);
        send[BUF_SIZE-1]='\0';
        recv[BUF_SIZE-1]='\0';

        tx_trans.buf=send;
        tx_trans.buf_len=SEND_LEN;
        rc = ioctl(fd, AXIDMA_DMA_WRITE, &tx_trans);
        if(rc)
            printf("dma write rc=%d\n",rc);

        rx_trans.buf=recv;
        rx_trans.buf_len=SEND_LEN;
        rc = ioctl(fd, AXIDMA_DMA_READ, &rx_trans);
        if(rc)
            printf("dma read rc=%d\n",rc);
        //printf("%s\n",send);
        printf("--------------------------------------------------------------------\n");
        if(!strncmp(send,recv,SEND_LEN))
        {
            printf("same!\n");
        }else
        {
            printf("diff!\n");
        }
//        return 0;
        //break;
    }

    int i;
    for(i=0;i<32;i++)
        putchar(recv[i]);
    printf("\n");
    // printf("%s\n",recv);
    return 0;
}
