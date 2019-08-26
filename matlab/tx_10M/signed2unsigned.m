function [ out_unsigned_i,out_unsigned_q ] = signed2unsigned( in_data,bit_width )

for ii=1:length(in_data)
    if (real(in_data(ii))<0)
        out_unsigned_i(ii)=real(in_data(ii))+2^bit_width;
    else
        out_unsigned_i(ii)=real(in_data(ii));
    end
    if (imag(in_data(ii))<0)
        out_unsigned_q(ii)=imag(in_data(ii))+2^bit_width;
    else
        out_unsigned_q(ii)=imag(in_data(ii));
    end   
end

end