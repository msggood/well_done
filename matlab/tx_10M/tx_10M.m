%   创  建：   周志强
%   tx 10M
%--------------------------------------------------------------------------
%  功能说明：
%
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

NumFrame = 1000;  %帧数一般情况下仿10000帧,时变信道下仿5000帧

% SNR = -10:1:10;  %信噪比dB
SNR = 10;  %信噪比dB
saved_file = 1;
Nsamples = 2;
data_type = '10M';%packet_head  packet_head40 10M 5M 3M 1.5M 750K
ch_type = 'fadawgn';% awgn fad fadawgn
fad_type = 'exp';%randn exp
trms = 10;
interleaver_on = 0;
channel_len = 20;
smooth_window = 16;
syn_sample_offset = 0;
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
        code_len = 1024;
        modulation_type = 'QPSK';
        code_rate = 1/2;
        spread_num = 1;
        inter_leaver_len = code_len * spread_num / code_rate;
        syms_num = inter_leaver_len/2/1024;
    case '5M'
        code_len = 512;
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
% intlvr_patt = gen_intlvr(inter_leaver_len,64);
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
    
    
    
    %随机生成data
    %         Inputsymbols = randint(1,(Ldata-LUW)*index);
    %         input_bits = randi([0 1],bit_num,1);
    input_bits = zeros(bit_num,1);
    input_bits(1:3:end) = 1;
    fd = fopen('input_bits.txt','wt');
    fprintf(fd,'%d',input_bits);
    fclose(fd);
    %         code_bits = lteTurboEncode(input_bits);
    code_bits    = step(hTEnc, input_bits);
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
    if(interleaver_on)
        interlved_bits = spread_code_out(intlvr_patt);
    else
        interlved_bits = spread_code_out;
    end

    fd = fopen('spread_code_out.txt','wt');
    fprintf(fd,'%d',spread_code_out);
    fclose(fd);

    
    %调制
    tx_sym = tx_modulate(interlved_bits,modulation_type);
    
    tx_sym_p = reshape(tx_sym,1024,syms_num);

%     a=[1 2 3 4];
%     b=reshape(a,4,syms_num);
%     c=b(end - 3+1 : end,:);
%     d=[c;b];
    %add crc
    packet_sym = [tx_sym_p(end - crc_len+1 : end,:);tx_sym_p];
    
    packet_sym_s = packet_sym(:);
    
    %调制
    pn = load('pn_512.mat');
    pn_512 = pn.bestPn1;
    syn_pn = [pn_512,pn_512,pn_512,pn_512,pn_512];
    %         fram_sym = [channel_esti_preamble; packet_sym_s];
%     fram_sym = packet_sym_s;
%     a=fram_sym*apm;
%     b=round(a);
%     c=real(b);
%     d=signed2unsigned(c,12);
    fram_sym = [syn_pn.';channel_esti_preamble; packet_sym_s];
    if(saved_file)
        [ out_unsigned_i,out_unsigned_q ] = signed2unsigned( round(fram_sym*apm),12);
%        [ out_unsigned_i,out_unsigned_q ] = signed2unsigned( fram_sym*apm,12);
        
        fid = fopen('packet_sym_i.h','wt');
        fprintf(fid,'complex double packet_sym_i[]={\n');
        fprintf(fid,'%f,\n',real(packet_sym));
        %fprintf(fid,'%f,',real(packet_sym_i),imag(channel_esti_preamble));
        fprintf(fid,'};');
        
        fid0 = fopen('syn_pn.h','wt');
        fprintf(fid0,'char syn_pn[]={');
        fprintf(fid0,'%d,',syn_pn);
        fprintf(fid0,'};');
        
        
        fid1 = fopen('tx_10m_i.h','wt');
        fprintf(fid1,'int tx_10m_i[]={\n');
        fprintf(fid1,'%d,\n',out_unsigned_i);
        fprintf(fid1,'};');
        
        fid2 = fopen('tx_10m_q.h','wt');
        fprintf(fid2,'int tx_10m_q[]={\n');
        fprintf(fid2,'%d,\n',out_unsigned_q);
        fprintf(fid2,'};');
        
    end
end
fclose(fid);
fclose(fid0);
fclose(fid1);
fclose(fid2);

