
function mod_symbols = tx_modulate(bits_in, modulation)

full_len = length(bits_in);

% BPSK modulation
if ~isempty(findstr(modulation, 'BPSK'))
   % Angle [pi/4 -3*pi/4] corresponds to 
   % Gray code vector [0 1], respectively.
   table=exp(j*[0 -pi]);  % generates BPSK symbols
   table=table([1 0]+1); % Gray code mapping pattern for BPSK symbols
   inp=bits_in;
   mod_symbols=table(inp+1);  % maps transmitted bits into BPSK symbols
   
   % QPSK modulation
elseif ~isempty(findstr(modulation, 'QPSK'))
   % Angle [pi/4 3*pi/4 -3*pi/4 -pi/4] corresponds to 
   % Gray code vector [00 10 11 01], respectively.
   table=exp(j*[-3/4*pi 3/4*pi 1/4*pi -1/4*pi]);  % generates QPSK symbols
   table=table([0 1 3 2]+1); % Gray code mapping pattern for QPSK symbols
   inp=reshape(bits_in,2,full_len/2);
   mod_symbols=table([2 1]*inp+1);  % maps transmitted bits into QPSK symbols
   
else
   error('Unimplemented modulation');
end


