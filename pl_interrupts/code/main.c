/*
 * main.c

 *
 *  Created on: 2019年6月25日
 *      Author: zhou
 */

#include "encoder.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "axidma_chrdev.h"
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

void get_bits_data(char *intputBit,int size)
{
    FILE *fp;
    int i, d;
    fp=fopen("input_bits.txt","rt");
    if(fp==NULL)
    {
        printf("input_bits.txt cannot open! " );
        exit(0);
    }
    fread(intputBit, 1, size, fp);
    
    for(i=0;i<size;i++)
    {
        intputBit[i]-=48;
    }
    fclose(fp);
}

void save_bits_data(char outputBit[],int size)
{
    FILE *fp;
    int i, d;
    fp=fopen("out_bits.txt","wt");
    if(fp==NULL)
    {
        printf("out_bits.txt cannot open! " );
        exit(0);
    }

    for(i=0;i<size;i++)
    {
        fprintf(fp,"%d",outputBit[i]);
    }
    fclose(fp);
}
#define TX_LEN 5120
#define PAGE_SIZE 4096
#define CODE_LEN PAGE_SIZE*4
#define OFFSET PAGE_SIZE*2048
static struct axidma_transaction trans;
static struct axidma_transaction rx_trans;
static struct gpio_data gpio;
int code_len=TX_LEN;
//char outputBit[CODE_LEN*3];
char inputBit[CODE_LEN];
short interleaver_tab[CODE_LEN];

int main(void)
{
    int rc;
    get_bits_data(inputBit,code_len);
    int dma_fd=open("/dev/axidma0",O_RDWR|O_EXCL);
    if(dma_fd<0)
    {
        printf("open axidma error\n");
        return -1;
    }

    char *outputBit;
    printf("sent mmap\n");
    outputBit = mmap(NULL, CODE_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, dma_fd, 0);
    if (outputBit == MAP_FAILED) {
        return -1;
    }

    char *recv;
    printf("recv mmap\n");
    recv = mmap(NULL, CODE_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, dma_fd, OFFSET);
    if (recv == MAP_FAILED) {
        return -1;
    }

    printf("outputBit:%x\n",outputBit);
    printf("recv:%x\n",recv);
    static int i=0;
    while(1){
        memset(outputBit,0x0,CODE_LEN);
        memset(recv,0xff,CODE_LEN);
        genTurboInterleaver(code_len,interleaver_tab);
        encoder(inputBit,outputBit,interleaver_tab,code_len,1); // /1-代表1/3 0--1/2
        trans.buf=outputBit;
        trans.buf_len=TX_LEN*3;
        printf("a\n");
        rc = ioctl(dma_fd, AXIDMA_DMA_WRITE, &trans);
        printf("dma write rc=%d\n",rc);
        gpio.num=913;
        gpio.value=i%2;
        rc = ioctl(dma_fd, AXIDMA_GPIO_WRITE, &gpio);
        i++;
        rx_trans.buf=recv;
        rx_trans.buf_len=TX_LEN*3;
        printf("dma write rc=%d\n",rc);
        rc = ioctl(dma_fd, AXIDMA_DMA_READ, &rx_trans);
        sleep(2);
        //if(!strncmp(outputBit,recv,16384))
        printf("recv:\n");
        int j=0;
        for(j=0;j<TX_LEN;j++)
        {
            printf("%x,",recv[j]);
        }
        printf("\n");

        return 0;
        if(!memcmp(outputBit,recv,15360))
        {
            printf("same!!\n");
            printf("outputBit:\n");
            for(j=0;j<CODE_LEN;j++)
            {
                printf("%x,",outputBit[j]);
            }
            printf("\n");
            return 0;
        }else
        {
            printf("diff!!\n");
        }
//       printf("gpio wrte rc=%d\n",rc);
    }
    save_bits_data(outputBit,code_len*3); 
    printf("aaaaaa\n");

    return 0;
}
