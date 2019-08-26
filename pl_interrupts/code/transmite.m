%   ��  ����   ��־ǿ
%   ��  ����    V 1.0
%   �������ڣ� 2018��09��19��
%   ������ڣ� 2018��09��19��
%--------------------------------------------------------------------------
%  ����˵����
%       1. ���Ͷ˳���
%--------------------------------------------------------------------------
%  �޸�˵����
%       1. 
%       2. 
%       3. 
%--------------------------------------------------------------------------
clear all;
% close all;
clc;
%%
load('pn_1024.mat','pn_1024');

NumFrame = 100;  %֡��һ������·�10000֡,ʱ���ŵ��·�5000֡

SNR = -10:2:10;  %�����dB
 SNR = -1;  %�����dB

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
fs = 12.8e6;         %��������
ts = 1/fs;           %�������
fft_len = 1024;      %�ز�
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
% ���ɳ�ͬ��ͷ����
channel_esti_preamble = [pn_1024_time(769:1024) pn_1024_time];
channel_esti_preamble = channel_esti_preamble.';

%�ڶ�����֯������Ҫ�о�һ��
intlvr_patt = gen_intlvr(inter_leaver_len,64);
% //addpath(genpath('E:\matlab work\shengzheng201809\work code\ad hoc\test_different_pn'));
[f1,f2]     = getf1f2(code_len);
InterTbl    = zeros(1,code_len);
for ii = 0:code_len-1
    InterTbl(ii+1) = mod(f1*ii+f2*ii^2,code_len);
end
hTEnc           = comm.TurboEncoder(poly2trellis(4, [13 15], 13), InterTbl + 1); 
hTDec           = comm.TurboDecoder(poly2trellis(4, [13 15], 13), InterTbl + 1, 10,'Algorithm','Max');  

%��ͬ������·���NumFrame֡����ͳ����Ӧ��������
for n = 1:length(SNR)
    
    tic  %SC-FDE��ʱ��ʼ
    
    %��������
    numberErrors = 0;
    Errcount1_ZF = 0;
    Errcount1_MMSE = 0;
    Errcount0 = 0;
    
    for k = 1:NumFrame
       
        
        %�������data
%         Inputsymbols = randint(1,(Ldata-LUW)*index);
         input_bits = randi([0 1],bit_num,1);%��������Դ
%         code_bits = lteTurboEncode(input_bits);
        code_bits    = step(hTEnc, input_bits);%����
        if(strcmp(data_type,'packet_head'))
            code_out_p = [code_bits; zeros(4,1)];
        elseif(strcmp(data_type,'packet_head40'))
            code_out_p = repmat(code_bits(1:end-12),8,1);%ȥ��β��12bit,��Ƶ8��
            code_out_p = [code_out_p;zeros(64,1)];%���64��0����Ϊһ��OFDM����
        else
            if(code_rate == 1/2)
                % �������1/2��Turbo�룬������β����12������
                code_out_p = zeros(2*code_len,1);
                code_out_p(1:2:2*code_len) = code_bits(1:3:3*code_len);
                code_out_p(2:4:2*code_len) = code_bits(2:6:3*code_len);
                code_out_p(4:4:2*code_len) = code_bits(6:6:3*code_len);
            elseif(code_rate == 1/3)
                code_out_p = code_bits(1:end-12);
            end
        end
        
        %��Ƶ����
        spread_code_out = repmat(code_out_p,spread_num,1);
        
        %��֯
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
