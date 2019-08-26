%   创  建：   周志强
%   版  本：    V 1.0
%   创建日期： 2018年09月19日
%   完成日期： 2018年09月19日
%--------------------------------------------------------------------------
%  功能说明：
%       1. 发送端程序
%--------------------------------------------------------------------------
%  修改说明：
%       1. 
%       2. 
%       3. 
%--------------------------------------------------------------------------
clear all;
% close all;
clc;
%%
load('pn_1024.mat','pn_1024');

NumFrame = 100;  %帧数一般情况下仿10000帧,时变信道下仿5000帧

SNR = -10:2:10;  %信噪比dB
 SNR = -1;  %信噪比dB

Nsamples = 2;
data_type = '3M';%packet_head  packet_head40 10M 5M 3M 1.5M 750K 
ch_type = 'fadawgn';% awgn fad fadawgn 
fad_type = 'exp';%randn exp
trms = 1;
channel_len = 20;
smooth_window = 16;
syn_sample_offset = 20;
add_res_fre_offset = 0;
res_fre_offset = 50;
apm = 2^9;

filter_delay = 6;
alpha = 0.35;
fs = 12.8e6;         %采样速率
ts = 1/fs;           %抽样间隔
fft_len = 1024;      %载波
crc_len = 256;

half_win = smooth_window/2;



modulation_type = 'BPSK';
switch(data_type)
    case 'packet_head'
        code_len = 336;
        spread_num = 3;
        inter_leaver_len = 1024 * spread_num;
        syms_num = spread_num;
        code_rate = 1/3;
    case 'packet_head40'
        code_len = 40;
        spread_num = 8;
        inter_leaver_len = 1024;
        code_rate = 1/3;
        syms_num = 1;
    case '10M'
        code_len = 5120;
        modulation_type = 'QPSK';
        code_rate = 1/2;
        spread_num = 1;
        inter_leaver_len = code_len * spread_num / code_rate;
        syms_num = inter_leaver_len/2/1024;
    case '5M'
        code_len = 5120;
        modulation_type = 'BPSK';
        spread_num = 1;
        code_rate = 1/2;
        inter_leaver_len = code_len * spread_num / code_rate;
        syms_num = inter_leaver_len/1024;
    case '3M'
        code_len = 5120;
        code_rate = 1/3;
        spread_num = 1;
        inter_leaver_len = code_len * spread_num / code_rate;
        syms_num = inter_leaver_len/1024;
    case '1.5M'
        code_len = 5120;
        code_rate = 1/3;
        spread_num = 2;
        inter_leaver_len = code_len * spread_num / code_rate;
        syms_num = inter_leaver_len/1024;
    case '750K'
        code_len = 5120;
        code_rate = 1/3;
        spread_num = 4;
        inter_leaver_len = code_len * spread_num / code_rate;
        syms_num = inter_leaver_len/1024;
    otherwise
        sprintf('error data type!');
end

switch(modulation_type)
    case 'BPSK'
        bps = 1;
    case 'QPSK'
        bps = 2;
end

packet_num_in_fram = 1;

bit_num = code_len;

pn_1024_time = ifft(pn_1024)*sqrt(1024);
% 生成长同步头序列
channel_esti_preamble = [pn_1024_time(769:1024) pn_1024_time];
channel_esti_preamble = channel_esti_preamble.';

%第二级交织器还需要研究一下
intlvr_patt = gen_intlvr(inter_leaver_len,64);
% //addpath(genpath('E:\matlab work\shengzheng201809\work code\ad hoc\test_different_pn'));
[f1,f2]     = getf1f2(code_len);
InterTbl    = zeros(1,code_len);
for ii = 0:code_len-1
    InterTbl(ii+1) = mod(f1*ii+f2*ii^2,code_len);
end
hTEnc           = comm.TurboEncoder(poly2trellis(4, [13 15], 13), InterTbl + 1); 
hTDec           = comm.TurboDecoder(poly2trellis(4, [13 15], 13), InterTbl + 1, 10,'Algorithm','Max');  

%不同信噪比下发送NumFrame帧数据统计相应的误码率
for n = 1:length(SNR)
    
    tic  %SC-FDE计时开始
    
    %误码数量
    numberErrors = 0;
    Errcount1_ZF = 0;
    Errcount1_MMSE = 0;
    Errcount0 = 0;
    
    for k = 1:NumFrame
       
        
        %随机生成data
%         Inputsymbols = randint(1,(Ldata-LUW)*index);
         input_bits = randi([0 1],bit_num,1);%输入数据源
%         code_bits = lteTurboEncode(input_bits);
        code_bits    = step(hTEnc, input_bits);%编码
        if(strcmp(data_type,'packet_head'))
            code_out_p = [code_bits; zeros(4,1)];
        elseif(strcmp(data_type,'packet_head40'))
            code_out_p = repmat(code_bits(1:end-12),8,1);%去掉尾部12bit,扩频8次
            code_out_p = [code_out_p;zeros(64,1)];%填充64个0，成为一个OFDM符号
        else
            if(code_rate == 1/2)
                % 打孔生成1/2的Turbo码，并丢弃尾部的12个比特
                code_out_p = zeros(2*code_len,1);
                code_out_p(1:2:2*code_len) = code_bits(1:3:3*code_len);
                code_out_p(2:4:2*code_len) = code_bits(2:6:3*code_len);
                code_out_p(4:4:2*code_len) = code_bits(6:6:3*code_len);
            elseif(code_rate == 1/3)
                code_out_p = code_bits(1:end-12);
            end
        end
        
        %扩频处理
        spread_code_out = repmat(code_out_p,spread_num,1);
        
        %交织
%         interlved_bits = spread_code_out(intlvr_patt);
        interlved_bits = spread_code_out;
    
    end
 
    toc;
    
end

% fid1 = fopen('rx_real_with_freshift1800_noise_5144.txt','wt');
% fid2 = fopen('rx_imag_with_freshift1800_noise_5144.txt','wt');
% fprintf(fid1,'%x\n',out_unsigned_i);
% fprintf(fid2,'%x\n',out_unsigned_q);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
