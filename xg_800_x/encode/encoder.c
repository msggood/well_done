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
#include <complex.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "channel_esti_preamble.h"
#include "syn_pn.h"

#define CODE_LEN 1024
#define MODULATION_TYPE "QPSK"
float code_rate = 1/2;
int spread_num = 1;
#define INTER_LEAVER_LEN (CODE_LEN * spread_num / code_rate)
#define SYS_NUM (INTER_LEAVER_LEN/2/1024)
int interleaver_on=0;
#define CRC_LEN 256

#define ARRAY_LEN(x) sizeof(x) / sizeof(x[0])
#define FRAME_LEN 5120
static complex double  frame_sym[FRAME_LEN];
//static int  frame_sym_i[FRAME_LEN];
//static int  frame_sym_q[FRAME_LEN];
static int frame_sym_iq[FRAME_LEN];
#define APM 512

static void tx_modulate(complex double* mod_symbols,char*out_code,int code_len,char *type)
{
    int i=0;
    int offset=0;
    if(strncmp("QPSK",type,strlen(type))==0)
    {	
        complex  double table[]={cexp(I*-3/4*M_PI),cexp(I*3/4*M_PI),cexp(I*-1/4*M_PI),cexp(I*1/4*M_PI)};        
        for(i=0;i<(code_len/2);i++)
        {   
            offset=2*out_code[i*2]+1*out_code[i*2+1];
            mod_symbols[i]=table[offset];
        }
    }else{
        printf("tpye error\n");
    }
}

int encode(char *inputBit,int len ,int *output)
{
    char outputBit[CODE_LEN*2];
    memset(outputBit,'\0',CODE_LEN*2);
    complex double  mod_symbols[CODE_LEN];
    complex double  packet_sym[CODE_LEN+CRC_LEN];

    short interleaver_tab[CODE_LEN];
    genTurboInterleaver(CODE_LEN,interleaver_tab);
    encoder(inputBit,outputBit,interleaver_tab,CODE_LEN,0); // /1-代表1/3 0--1/2
//    save_bits_data("spread_code_out.txt",outputBit,CODE_LEN*2); 

    tx_modulate(packet_sym+CRC_LEN,outputBit,CODE_LEN*2,MODULATION_TYPE);    

    int i=0;
    for(i=0;i<CRC_LEN;i++)
    {
        packet_sym[i]=packet_sym[CODE_LEN+i];
    }

    int syn_pn_len=ARRAY_LEN(syn_pn);
//    printf("syn_pn_len=%d\n",syn_pn_len);
    for(i=0;i<syn_pn_len;i++)
    {
        frame_sym[i]=syn_pn[i];
    }
    int channel_esti_preamble_len=ARRAY_LEN(channel_esti_preamble);
//    printf("channel_esti_preamble_len=%d\n",channel_esti_preamble_len);
    for(i=0;i<channel_esti_preamble_len;i++)
    {
        frame_sym[syn_pn_len+i]=channel_esti_preamble[i];
    }
    int packet_sym_len=ARRAY_LEN(packet_sym);
//    printf("packet_sym_len=%d\n",packet_sym_len);
    for(i=0;i<packet_sym_len;i++)
    {
        frame_sym[syn_pn_len+channel_esti_preamble_len+i]=packet_sym[i];
    }

    int fram_len=ARRAY_LEN(syn_pn)+ARRAY_LEN(channel_esti_preamble)+ARRAY_LEN(packet_sym);

    double tmp;
    int  frame_sym_i;
    int  frame_sym_q;

    for(i=0;i<fram_len;i++)
    {
        tmp=creal(frame_sym[i]);
        tmp=tmp*APM;
        frame_sym_i=round(tmp);
        if(frame_sym_i<0)
        {
            frame_sym_i=frame_sym_i&0xfff;
        }

        tmp=cimag(frame_sym[i]);
        tmp=tmp*APM;
        frame_sym_q=round(tmp);
        if(frame_sym_q<0)
        {

           frame_sym_q=frame_sym_q&0xfff;
        }
        output[i]=frame_sym_i<<16|frame_sym_q;
    } 
    return 0;
}


